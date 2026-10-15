#include "world.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "cast.h"

namespace orblit {
namespace {

Vec3 vectorOf(const float v[3]) { return {v[0], v[1], v[2]}; }

/// A rotation from four floats, reading all zeroes as identity so a caller
/// that does not care about orientation can leave the field alone.
Quat rotationOf(const float r[4]) {
  const Quat q{r[0], r[1], r[2], r[3]};
  if (lengthSquared(Vec3{q.x, q.y, q.z}) < kTiny && std::fabs(q.w) < kTiny) {
    return Quat{};
  }
  return normalised(q);
}

Motion motionOf(uint32_t motion) {
  switch (motion) {
    case ORBLIT_PHYSICS_STATIC: return Motion::fixed;
    case ORBLIT_PHYSICS_KINEMATIC: return Motion::driven;
    default: return Motion::free;
  }
}

/// Shed as a fraction per second, implicitly, so a large delta slows a body
/// rather than reversing it.
float damped(float damping, float delta) {
  return 1.0f / (1.0f + std::fmax(damping, 0.0f) * delta);
}

/// The steepest slope a character may be told it can stand on, in radians:
/// about 87 degrees. Short of vertical, because a character that may stand
/// on a wall stands on every wall, and short of it by enough that "is this a
/// floor" never turns on the last bit of a float.
constexpr float kSteepest = 1.52f;

/// The thinnest gap a character may keep, in metres. Casts stop a tenth of a
/// millimetre short of what they meet, so a skin thinner than a few of those
/// is a character that is always touching something.
constexpr float kThinnest = 1.0e-3f;

/// How nearly two contact normals must agree, as a cosine, for last step's
/// push along one to be a fair start for this step's along the other: about
/// 25 degrees. Loose enough that a crate rocking on a slope keeps its warm
/// start, tight enough that a corner that has slid across a crease does not.
constexpr float kSameWay = 0.9f;

/// How near a body must be to a surface to count as resting on it, in metres.
/// A little over the solver's slop, which is as far into it as a body is let
/// in.
constexpr float kResting = 0.05f;

/// How far two bodies must close in one step, in metres, before the pair is
/// held apart by a speculative contact. Slower pairs are left to the exact
/// contact. Speculating for every pair doubles the manifolds, and each near
/// miss ties a neighbour into its island, which keeps a heap from sleeping.
/// It is about 3 m/s at 60 steps a second.
constexpr float kFast = 0.05f;

/// A move this short, in metres, cannot take a body through anything worth
/// asking about, so a continuous body that is this slow is not cast.
constexpr float kStill = 1.0e-4f;

/// How far past the first touch, in metres, a continuous body is carried. The
/// cast calls a gap of 0.1 mm a touch, and a body stopped short of the surface
/// has no contact to push it, so a slow one would hang there with its velocity
/// still pointing in. Half a millimetre is overlap enough for the narrowphase.
constexpr float kPastTouch = 5.0e-4f;

/// Carries last step's impulse onto whichever of this step's points is
/// nearest. No distance limit, deliberately: an old manifold only exists
/// because the pair was already touching, so the worst a bad match can do is
/// start a pass from a number that is merely close, and the passes then
/// correct it within the same step.
///
/// A limit on direction, though. Against ground the points of one pair can
/// face different ways, and the push that held a corner against one slope is
/// no start at all for a corner now against the other.
void carryImpulses(const Manifold &old, Manifold &manifold) {
  // Which body is named first can change between steps, and a pair that slept
  // keeps its manifold while rows move, so it is the ids that say whether the
  // impulse is along a normal that flipped.
  const float sign = old.reversed != manifold.reversed ? -1.0f : 1.0f;
  for (uint32_t c = 0; c < manifold.count; ++c) {
    uint32_t nearest = Bodies::kNone;
    float best = 3.0e38f;
    for (uint32_t o = 0; o < old.count; ++o) {
      if (dot(old.points[o].normal, manifold.points[c].normal) * sign < kSameWay) {
        continue;
      }
      const float away = lengthSquared(old.points[o].at - manifold.points[c].at);
      if (away < best) {
        best = away;
        nearest = o;
      }
    }
    if (nearest == Bodies::kNone) continue;
    manifold.points[c].normalImpulse = old.points[nearest].normalImpulse;
    manifold.points[c].frictionImpulse[0] =
        old.points[nearest].frictionImpulse[0] * sign;
    manifold.points[c].frictionImpulse[1] =
        old.points[nearest].frictionImpulse[1] * sign;
  }
}

/// Puts the events from `first` on in the order of their pairs. What ended or
/// was left is found by walking a hash table, whose order is whatever its
/// buckets happen to hold, and a world restored from a snapshot has its own
/// buckets. Sorted, the same step reports the same events in the same order
/// however the table was built.
void sortByPair(std::vector<OrblitPhysicsEvent> &events, size_t first) {
  std::sort(events.begin() + static_cast<std::ptrdiff_t>(first), events.end(),
            [](const OrblitPhysicsEvent &l, const OrblitPhysicsEvent &r) {
              return PairKey{l.a, l.b} < PairKey{r.a, r.b};
            });
}

/// The path `shape` takes under a query. A ray is a sphere of no size, which is
/// what it is, and means one routine in the cast rather than two that differ by
/// a radius of zero.
Journey journeyOf(const Shape &shape, const OrblitPhysicsCast &query,
                  const Vec3 &direction, float distance) {
  Sieve sieve;
  sieve.layerIs = query.layerIs;
  sieve.layerCares = query.layerCares;
  sieve.ignore = query.ignore;
  sieve.triggers = query.triggers;

  return Journey{Placed{shape, vectorOf(query.from), rotationOf(query.rotation)},
                 direction, distance, sieve};
}

OrblitPhysicsHit hitOf(OrblitPhysicsId body, const Impact &impact) {
  OrblitPhysicsHit hit{};
  hit.body = body;
  hit.at[0] = impact.at.x;
  hit.at[1] = impact.at.y;
  hit.at[2] = impact.at.z;
  hit.normal[0] = impact.normal.x;
  hit.normal[1] = impact.normal.y;
  hit.normal[2] = impact.normal.z;
  hit.distance = impact.distance;
  hit.started = impact.started;
  return hit;
}

/// Whether a query goes anywhere. A direction of no length goes nowhere.
bool castable(const OrblitPhysicsCast &query) {
  return lengthSquared(normalised(vectorOf(query.direction))) >= kTiny;
}

/// The journey a castable query describes, for `shape`.
Journey castOf(const Shape &shape, const OrblitPhysicsCast &query) {
  return journeyOf(shape, query, normalised(vectorOf(query.direction)),
                   std::fmax(query.distance, 0.0f));
}

/// Puts `hit` into the `written` hits in `out`, which are nearest first and
/// which `out` has room for `capacity` of, and drops the furthest if that
/// leaves too many. Returns how many are there now.
uint32_t keepNearest(OrblitPhysicsHit *out, uint32_t written, uint32_t capacity,
                     const OrblitPhysicsHit &hit) {
  uint32_t at = written;
  while (at > 0 && out[at - 1].distance > hit.distance) --at;
  if (at == capacity) return written;

  const uint32_t length = std::min(written + 1, capacity);
  for (uint32_t i = length - 1; i > at; --i) out[i] = out[i - 1];
  out[at] = hit;
  return length;
}

bool anyNan(std::initializer_list<float> values) {
  return std::any_of(values.begin(), values.end(),
                     [](float v) { return std::isnan(v); });
}

constexpr uint32_t kZoneFields = ORBLIT_PHYSICS_ZONE_GRAVITY |
                                 ORBLIT_PHYSICS_ZONE_LINEAR_DAMPING |
                                 ORBLIT_PHYSICS_ZONE_ANGULAR_DAMPING;

constexpr uint32_t kRuleFields = ORBLIT_PHYSICS_RULE_FRICTION |
                                 ORBLIT_PHYSICS_RULE_RESTITUTION |
                                 ORBLIT_PHYSICS_RULE_MOVE_SCALE |
                                 ORBLIT_PHYSICS_RULE_IGNORE;

constexpr uint32_t kLockBits =
    ORBLIT_PHYSICS_LOCK_MOVE_X | ORBLIT_PHYSICS_LOCK_MOVE_Y |
    ORBLIT_PHYSICS_LOCK_MOVE_Z | ORBLIT_PHYSICS_LOCK_TURN_X |
    ORBLIT_PHYSICS_LOCK_TURN_Y | ORBLIT_PHYSICS_LOCK_TURN_Z;

bool allFinite(std::initializer_list<float> values) {
  return std::all_of(values.begin(), values.end(),
                     [](float v) { return std::isfinite(v); });
}

/// Shortens `v` to `most` if it is longer. Nought is no limit.
Vec3 capped(const Vec3 &v, float most) {
  if (!(most > 0.0f)) return v;
  const float speed = length(v);
  return speed > most ? v * (most / speed) : v;
}

/// Whether the numbers of a set of controls mean anything. The caps and the
/// inertia are magnitudes, so a negative one is a mistake rather than a wish.
bool meaningful(const OrblitPhysicsControls &from) {
  return allFinite({from.gravityScale, from.maxSpeed, from.maxSpin, from.centre[0],
                    from.centre[1], from.centre[2], from.inertia[0],
                    from.inertia[1], from.inertia[2]}) &&
         (from.locks & ~kLockBits) == 0 && from.maxSpeed >= 0.0f &&
         from.maxSpin >= 0.0f && from.inertia[0] >= 0.0f &&
         from.inertia[1] >= 0.0f && from.inertia[2] >= 0.0f;
}

/// Planes, terrain and triangle meshes are fixed: how they move cannot be
/// changed, because they do not.
bool isGround(const Shape &shape) {
  return shape.kind == ShapeKind::plane || shape.kind == ShapeKind::heightField ||
         shape.kind == ShapeKind::mesh;
}

Controls controlsOf(const OrblitPhysicsControls &from) {
  Controls controls;
  controls.locks = from.locks;
  controls.gravityScale = from.gravityScale;
  controls.maxSpeed = from.maxSpeed;
  controls.maxSpin = from.maxSpin;
  controls.centre = vectorOf(from.centre);
  controls.inertia = vectorOf(from.inertia);
  controls.continuous = from.continuous;
  return controls;
}

Zone zoneOf(const OrblitPhysicsZone &from) {
  Zone zone;
  zone.overrides = from.overrides & kZoneFields;
  zone.priority = from.priority;
  zone.gravity = vectorOf(from.gravity);
  zone.linearDamping = from.damping[0];
  zone.angularDamping = from.damping[1];
  return zone;
}

/// Whether the numbers of a rule mean anything. A move scale that is NaN fails
/// the comparison, so only the two that clamp need asking about.
bool meaningful(const OrblitPhysicsRule &from) {
  return !anyNan({from.friction, from.restitution}) && from.moveScale[0] >= 0.0f &&
         from.moveScale[1] >= 0.0f;
}

/// The rule with its move scales in the key's order, and the two values a body
/// is made with held to the range a body is made with.
Rule ruleOf(const OrblitPhysicsRule &from, const PairKey &key) {
  Rule rule;
  rule.overrides = from.overrides & kRuleFields;
  rule.friction = std::fmax(from.friction, 0.0f);
  rule.restitution = clamped(from.restitution, 0.0f, 1.0f);
  rule.move[0] = from.moveScale[0];
  rule.move[1] = from.moveScale[1];
  return key.a == from.a ? rule : rule.turned();
}

} // namespace

// --------------------------------------------------------------------------

void World::defaults(OrblitPhysicsSettings *out) {
  if (out == nullptr) return;
  *out = OrblitPhysicsSettings{};
  out->gravity[0] = 0.0f;
  out->gravity[1] = -9.81f;
  out->gravity[2] = 0.0f;
  out->subSteps = 6;
  out->velocitySteps = 1;
  out->positionSteps = 2;
  out->jointSteps = 8;
  out->slop = 0.02f;
  out->contactHertz = 50.0f;
  out->bounceThreshold = 1.0f;
  out->sleepSpeed = 0.03f;
  out->sleepAfter = 0.5f;
  out->sleeping = true;
}

World::World(const OrblitPhysicsSettings &settings)
    : settings_(settings), solving_(solvingFor(settings)) {
  gravity_ = vectorOf(settings_.gravity);
}

SolverSettings World::solvingFor(const OrblitPhysicsSettings &settings) {
  SolverSettings solving;
  // A step with no sub-steps, or a sub-step with no pass in it, is a world
  // that never moves or never holds anything up. One of each is the least that
  // is a solver at all.
  solving.subSteps = std::max(settings.subSteps, 1u);
  solving.velocitySteps = std::max(settings.velocitySteps, 1u);
  solving.positionSteps = settings.positionSteps;
  solving.jointSteps = std::max(settings.jointSteps, 1u);
  solving.slop = std::fmax(settings.slop, 0.0f);
  solving.contactHertz = std::fmax(settings.contactHertz, 0.0f);
  solving.bounceThreshold = std::fmax(settings.bounceThreshold, 0.0f);
  return solving;
}

// -------------------------------------------------------------- commands ---

void World::submit(const OrblitPhysicsCommand *commands, uint32_t count) {
  if (commands == nullptr) return;
  for (uint32_t i = 0; i < count; ++i) apply(commands[i]);
}

void World::apply(const OrblitPhysicsCommand &command) {
  if (command.kind == ORBLIT_PHYSICS_CREATE) {
    BodyDescription made;
    // Ground has its own call, because a command has nowhere to put a grid.
    if (!shapeFor(command.shape, command.size, command.hull, made.shape)) return;
    made.motion = motionOf(command.motion);
    if (made.shape.kind == ShapeKind::mesh && made.motion != Motion::fixed) return;
    made.at = vectorOf(command.at);
    made.rotation = rotationOf(command.rotation);
    made.mass = command.mass;
    made.friction = std::fmax(command.friction, 0.0f);
    // Above one a collision hands back more than it was given, and a ball
    // dropped from a metre ends up on the ceiling.
    made.restitution = clamped(command.restitution, 0.0f, 1.0f);
    made.linearDamping = command.damping[0];
    made.angularDamping = command.damping[1];
    made.layerIs = command.layerIs;
    made.layerCares = command.layerCares;
    made.asleep = command.asleep;
    // A body the solver moves is pushed by what it touches, which is the one
    // thing a trigger is not.
    made.sensor = command.sensor && made.motion != Motion::free;
    made.stay = command.stay;
    bodies_.add(command.id, made);
    return;
  }

  const uint32_t row = bodies_.rowOf(command.id);
  if (row == Bodies::kNone) return;

  switch (command.kind) {
    case ORBLIT_PHYSICS_DESTROY:
      destroy(command.id);
      return;

    case ORBLIT_PHYSICS_PLACE:
      bodies_.place(row, vectorOf(command.at), rotationOf(command.rotation));
      if (Character *character = characterOf(command.id)) character->forget();
      wake(row);
      return;

    case ORBLIT_PHYSICS_VELOCITY:
      // A character's velocity is a request, not a fact. The column keeps
      // what it actually did last step, which is what the solver needs to
      // know about it when a crate is up against it.
      if (Character *character = characterOf(command.id)) {
        character->wanted = vectorOf(command.vector);
        return;
      }
      bodies_.velocity(row) = vectorOf(command.vector);
      bodies_.spin(row) = vectorOf(command.spin);
      bodies_.hold(row);
      bodies_.refresh(row);
      wake(row);
      return;

    case ORBLIT_PHYSICS_MOTION:
      switchMotion(row, command.motion);
      return;

    case ORBLIT_PHYSICS_IMPULSE: {
      // A point that is not a number is a caller with no point in mind, which
      // means through the centre of mass. Only this side knows where that is.
      const Vec3 point = vectorOf(command.spin);
      push(row, vectorOf(command.vector),
           std::isnan(point.x) ? bodies_.centre(row) : point);
      return;
    }

    case ORBLIT_PHYSICS_WAKE:
      wake(row);
      return;

    case ORBLIT_PHYSICS_CHARACTER: {
      const Motion motion = bodies_.motion(row);
      if (motion != Motion::driven && motion != Motion::character) return;
      if (bodies_.shape(row).kind == ShapeKind::plane) return;

      Character *character = characterOf(command.id);
      if (character == nullptr) {
        characters_.emplace_back();
        character = &characters_.back();
        character->id = command.id;
        // Whatever it was being driven at, it now asks for, so turning a
        // moving platform into a character does not stop it dead.
        character->wanted = bodies_.velocity(row);
      }
      // fmax and fmin rather than a comparison, so a NaN from the caller
      // becomes the edge of the range rather than a character that is never
      // on any floor at all.
      character->stepHeight = std::fmax(command.size[0], 0.0f);
      character->slopeCos =
          std::cos(std::fmin(std::fmax(command.size[1], 0.0f), kSteepest));
      character->skin = std::fmax(command.size[2], kThinnest);
      character->strength = std::fmax(command.size[3], 0.0f);

      bodies_.steer(row, Motion::character);
      bodies_.spin(row) = {};
      return;
    }

    case ORBLIT_PHYSICS_SURFACE:
      // Kept whole. Which part of it runs along a surface depends on the
      // surface a contact is with, which is only known when there is one.
      bodies_.surface(row) = vectorOf(command.vector);
      // What rests on it, and the surface's own row, which wakes whatever is
      // joined to a belt that is itself being driven.
      wakeWithin(bodies_.bounds(row).grown(kResting));
      return;

    default:
      return;
  }
}

void World::destroy(OrblitPhysicsId id) {
  // A zone that goes leaves what was asleep in it at rest in air it no longer
  // feels.
  const uint32_t row = bodies_.rowOf(id);
  if (row != Bodies::kNone && zones_.count(id) != 0) {
    wakeWithin(bodies_.bounds(row));
  }

  // What it was joined to is let go of, not left holding a joint to nothing,
  // and woken, because what was hanging from it should now fall.
  partners_.clear();
  joints_.drop(id, partners_);
  unmake(id);
  for (const OrblitPhysicsId partner : partners_) {
    const uint32_t row = bodies_.rowOf(partner);
    if (row != Bodies::kNone) wake(row);
  }

  // A rule is about two particular bodies. A new body that is given a dead
  // one's id is not them and must not inherit what was said about them.
  for (auto rule = rules_.begin(); rule != rules_.end();) {
    rule = (rule->first.a == id || rule->first.b == id) ? rules_.erase(rule)
                                                        : std::next(rule);
  }
}

void World::unmake(OrblitPhysicsId id) {
  leaveIsland(id);
  bodies_.remove(id);
  // Erased in place rather than swapped out, because the list's order is the
  // order characters move in, and two characters that swap turns swap which
  // of them gets to a doorway first.
  characters_.erase(std::remove_if(characters_.begin(), characters_.end(),
                                   [&](const Character &c) { return c.id == id; }),
                    characters_.end());
  // After the body, which points into it.
  fields_.erase(id);
  zones_.erase(id);
}

void World::leaveIsland(OrblitPhysicsId id) {
  const uint32_t row = bodies_.rowOf(id);
  if (row == Bodies::kNone) return;
  const auto found = islands_.find(bodies_.island(row));
  if (found == islands_.end()) return;

  Island &island = found->second;
  island.bodies.erase(std::remove(island.bodies.begin(), island.bodies.end(), id),
                      island.bodies.end());
  // Its pairs are given back to the last step's, so they end on the next one,
  // as they would have if it had been destroyed awake.
  for (auto pair = island.pairs.begin(); pair != island.pairs.end();) {
    if (pair->first.a == id || pair->first.b == id) {
      wasTouching_.emplace(*pair);
      pair = island.pairs.erase(pair);
    } else {
      ++pair;
    }
  }
  if (island.bodies.empty()) islands_.erase(found);
}

bool World::ground(const OrblitPhysicsGround &from) {
  const uint32_t least = from.margin ? 4u : 2u;
  if (from.id == 0 || from.heights == nullptr) return false;
  if (from.columns < least || from.rows < least) return false;
  if (!(from.spacing > 0.0f) || !std::isfinite(from.spacing)) return false;

  auto field = std::make_shared<HeightField>();
  field->lay(from.columns, from.rows, from.spacing, from.heights, from.margin);

  // Replaced rather than changed in place: a body's shape is fixed once it is
  // made, and the pair it was in is keyed by id, so what was touching it
  // before is still warm started against the new one. Where it was is kept,
  // because what was on it may be above where it is now.
  const uint32_t was = bodies_.rowOf(from.id);
  const bool replacing = was != Bodies::kNone;
  const Bounds before = replacing ? bodies_.bounds(was) : Bounds{};
  unmake(from.id);

  BodyDescription made;
  made.shape = Shape::ground(field.get());
  made.motion = Motion::fixed;
  made.at = vectorOf(from.at);
  made.rotation = Quat{};
  made.friction = std::fmax(from.friction, 0.0f);
  made.restitution = clamped(from.restitution, 0.0f, 1.0f);
  made.layerIs = from.layerIs;
  made.layerCares = from.layerCares;
  const uint32_t row = bodies_.add(from.id, made);
  if (row == Bodies::kNone) return false;
  fields_[from.id] = std::move(field);

  // Ground that has moved under a sleeper must wake it, or a crate on a hill
  // that was just lowered hangs in the air where the hill was.
  const Bounds &after = bodies_.bounds(row);
  const Bounds area =
      replacing ? Bounds{minPerAxis(before.low, after.low),
                         maxPerAxis(before.high, after.high)}
                : after;
  wakeWithin(area);
  return true;
}

bool World::hull(OrblitPhysicsId id, const float *xyz, uint32_t count) {
  if (id == 0 || hulls_.count(id) != 0) return false;

  std::shared_ptr<const Hull> cooked = Hull::cook(xyz, count);
  if (cooked == nullptr) return false;
  hulls_[id] = std::move(cooked);
  return true;
}

bool World::dropHull(OrblitPhysicsId id) {
  const auto found = hulls_.find(id);
  if (found == hulls_.end()) return false;

  const Hull *dropped = found->second.get();
  for (uint32_t row = 0; row < bodies_.count(); ++row) {
    if (bodies_.shape(row).cooked == dropped) return false;
  }
  for (const auto &entry : compounds_) {
    if (entry.second->uses(dropped)) return false;
  }
  hulls_.erase(found);
  return true;
}

bool World::shapeFor(uint32_t kind, const float size[4], OrblitPhysicsId hull,
                     Shape &out) const {
  if (kind == ORBLIT_PHYSICS_HEIGHT_FIELD) return false;

  if (kind == ORBLIT_PHYSICS_MESH) {
    const auto found = meshes_.find(hull);
    if (found == meshes_.end()) return false;
    out = Shape::triangles(found->second.get());
    return true;
  }
  if (kind == ORBLIT_PHYSICS_COMPOUND) {
    const auto found = compounds_.find(hull);
    if (found == compounds_.end()) return false;
    out = Shape::compound(found->second.get());
    return true;
  }
  if (kind == ORBLIT_PHYSICS_HULL) {
    const auto found = hulls_.find(hull);
    if (found == hulls_.end()) return false;
    out = Shape::hull(found->second.get());
    return true;
  }

  // A cylinder with no radius or no length has no flat ends to build, and the
  // support routines divide by what they find there.
  if (kind == ORBLIT_PHYSICS_CYLINDER &&
      !(allFinite({size[0], size[1]}) && size[0] > 0.0f && size[1] > 0.0f)) {
    return false;
  }
  out = Shape::fromCommand(kind, size);
  return true;
}

bool World::shapeOf(const OrblitPhysicsCast &query, Shape &out) const {
  if (query.shape == 0) {
    out = Shape::sphere(0.0f);
    return true;
  }
  return shapeFor(query.shape, query.size, query.hull, out);
}

bool World::zone(const OrblitPhysicsZone &from) {
  const uint32_t row = bodies_.rowOf(from.body);
  if (row == Bodies::kNone || !bodies_.sensor(row)) return false;
  if (anyNan({from.gravity[0], from.gravity[1], from.gravity[2], from.damping[0],
              from.damping[1]})) {
    return false;
  }

  if ((from.overrides & kZoneFields) == 0) {
    if (zones_.erase(from.body) == 0) return false;
  } else {
    zones_[from.body] = zoneOf(from);
  }
  // What is in it now feels something else, so what was asleep in it is not
  // at rest any more.
  wakeWithin(bodies_.bounds(row));
  return true;
}

bool World::rule(const OrblitPhysicsRule &from) {
  if (from.a == 0 || from.b == 0 || from.a == from.b) return false;
  if (bodies_.rowOf(from.a) == Bodies::kNone ||
      bodies_.rowOf(from.b) == Bodies::kNone) {
    return false;
  }
  if (!meaningful(from)) return false;

  const PairKey key = PairKey::of(from.a, from.b);
  if ((from.overrides & kRuleFields) == 0) {
    if (rules_.erase(key) == 0) return false;
  } else {
    rules_[key] = ruleOf(from, key);
  }
  wakeEnds(from.a, from.b);
  return true;
}

bool World::ignores(OrblitPhysicsId a, OrblitPhysicsId b) const {
  if (rules_.empty()) return false;
  const auto found = rules_.find(PairKey::of(a, b));
  return found != rules_.end() &&
         (found->second.overrides & ORBLIT_PHYSICS_RULE_IGNORE) != 0;
}

bool World::controls(const OrblitPhysicsControls &from) {
  const uint32_t row = bodies_.rowOf(from.body);
  if (row == Bodies::kNone || isGround(bodies_.shape(row)) || !meaningful(from)) {
    return false;
  }

  bodies_.control(row, controlsOf(from));
  // How a body moves is part of whether it is at rest, and so is it for what
  // rests on it. A body just made asleep is at rest with the controls it was
  // made with, so its caller says not to wake it.
  if (!from.quiet) wakeWithin(bodies_.bounds(row).grown(kResting));
  return true;
}

bool World::gravity(const float to[3]) {
  if (!allFinite({to[0], to[1], to[2]})) return false;

  gravity_ = vectorOf(to);
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (bodies_.movable(row)) wake(row);
  }
  return true;
}

bool World::stance(OrblitPhysicsId id, uint32_t kind, const float size[4]) {
  const uint32_t row = bodies_.rowOf(id);
  if (row == Bodies::kNone || characterOf(id) == nullptr) return false;
  const bool rigid = kind == ORBLIT_PHYSICS_SPHERE || kind == ORBLIT_PHYSICS_BOX ||
                     kind == ORBLIT_PHYSICS_CAPSULE ||
                     kind == ORBLIT_PHYSICS_CYLINDER;
  if (!rigid) return false;
  Shape next;
  if (!shapeFor(kind, size, 0, next)) return false;

  // Feet stay where they are, so a crouch is the head coming down and not the
  // character sinking into the floor.
  const Vec3 up = upwards();
  const Quat &rotation = bodies_.rotation(row);
  const Vec3 was = bodies_.at(row);
  const float sole = dot(support(Placed{bodies_.shape(row), was, rotation}, -up), up);
  const float lowest = dot(support(Placed{next, was, rotation}, -up), up);
  const Vec3 at = was + up * (sole - lowest);

  Sieve sieve;
  sieve.layerIs = bodies_.layerIs(row);
  sieve.layerCares = bodies_.layerCares(row);
  sieve.ignore = id;
  Impact hit;
  if (nearest(bodies_, Journey{Placed{next, at, rotation}, up, 0.0f, sieve}, hit) !=
      Bodies::kNone) {
    return false;
  }

  bodies_.at(row) = at;
  bodies_.reshape(row, next);
  return true;
}

Vec3 World::upwards() const {
  const float fall = length(gravity_);
  return fall > kTiny ? gravity_ * (-1.0f / fall) : Vec3{0.0f, 1.0f, 0.0f};
}

void World::switchMotion(uint32_t row, uint32_t to) {
  if (to > ORBLIT_PHYSICS_DYNAMIC) return;

  const Motion next = motionOf(to);
  const Motion now = bodies_.motion(row);
  // A trigger is never pushed, a character has its own way of moving, and
  // ground has no mass to be made free with.
  if (next == now || now == Motion::character || bodies_.sensor(row) ||
      isGround(bodies_.shape(row))) {
    return;
  }

  bodies_.convert(row, next);
  // What rested on it is not held up any more, and what it now rests on may be
  // held up by it.
  wakeWithin(bodies_.bounds(row).grown(kResting));
}

void World::wakeWithin(const Bounds &area) {
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (bodies_.bounds(row).overlaps(area)) wake(row);
  }
}

void World::push(uint32_t row, const Vec3 &impulse, const Vec3 &point) {
  if (!bodies_.movable(row)) return;
  wake(row);
  const Vec3 lever = point - bodies_.centre(row);
  bodies_.velocity(row) += mulPerAxis(impulse, bodies_.gain(row));
  bodies_.spin(row) += bodies_.inverseInertia(row) * cross(lever, impulse);
}

Character *World::characterOf(OrblitPhysicsId id) {
  for (Character &character : characters_) {
    if (character.id == id) return &character;
  }
  return nullptr;
}

const Character *World::characterOf(OrblitPhysicsId id) const {
  for (const Character &character : characters_) {
    if (character.id == id) return &character;
  }
  return nullptr;
}

void World::wake(uint32_t row) {
  const bool sleeper = bodies_.movable(row) && bodies_.asleep(row);
  if (sleeper) wakeIsland(row);
  // An awake body that the solver moves is in a group that is awake already,
  // because joined bodies only ever sleep together.
  if (sleeper || !bodies_.movable(row)) wakeJoined(row);
}

void World::wakeIsland(uint32_t row) {
  const uint32_t name = bodies_.island(row);
  const auto found = islands_.find(name);
  if (name == 0 || found == islands_.end()) {
    // Made asleep, so it has no fellows.
    waken(row);
    return;
  }

  const Island island = std::move(found->second);
  islands_.erase(found);
  for (const OrblitPhysicsId id : island.bodies) {
    // A body that was destroyed has left the island, and its id may belong to
    // another body by now, which is in another island or none.
    const uint32_t member = bodies_.rowOf(id);
    if (member != Bodies::kNone && bodies_.island(member) == name) waken(member);
  }

  // What was touching goes back to being what the last step found, so the
  // pair is warm started if it is still there and ended if it is not.
  for (const auto &pair : island.pairs) wasTouching_.emplace(pair);
}

void World::waken(uint32_t row) {
  bodies_.wake(row);
  note(ORBLIT_PHYSICS_WOKE, row);
  woke_ = true;
}

void World::wakeJoined(uint32_t row) {
  if (!joints_.on(bodies_.id(row))) return;
  waking_.clear();
  waking_.push_back(bodies_.id(row));
  while (!waking_.empty()) {
    const OrblitPhysicsId from = waking_.back();
    waking_.pop_back();
    joints_.eachPartner(from, [this](OrblitPhysicsId partner) {
      const uint32_t other = bodies_.rowOf(partner);
      if (!bodies_.movable(other) || !bodies_.asleep(other)) return;
      wakeIsland(other);
      waking_.push_back(partner);
    });
  }
}

// ---------------------------------------------------------------- joints ---

namespace {

JointDescription describe(const OrblitPhysicsJoint &from) {
  JointDescription made;
  made.id = from.id;
  made.a = from.a;
  made.b = from.b;
  made.kind = from.kind;
  made.limited = from.limited;
  made.at = vectorOf(from.at);
  made.rotation = rotationOf(from.rotation);
  made.to = vectorOf(from.to);
  for (int which = 0; which < 6; ++which) {
    made.low[which] = from.low[which];
    made.high[which] = from.high[which];
    made.springHertz[which] = from.springHertz[which];
    made.springRatio[which] = from.springRatio[which];
    made.springRest[which] = from.springRest[which];
  }
  made.swing = from.swing;
  made.speed = from.speed;
  made.strength = from.strength;
  made.goal = from.goal;
  made.goalHertz = from.goalHertz;
  made.goalRatio = from.goalRatio;
  made.limitHertz = from.limitHertz;
  made.limitRatio = from.limitRatio;
  made.breakingForce = from.breakingForce;
  made.breakingTorque = from.breakingTorque;
  made.collide = from.collide;
  return made;
}

/// Whether every number in `from` is one a joint may be made from. A NaN
/// anywhere is a joint that puts NaN into every body it touches on its first
/// step, so it is never made. Where it stands must be somewhere as well; a
/// limit or a spring's rest may be as far as it likes.
bool isMakeable(const OrblitPhysicsJoint &from) {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(from.at[i]) || !std::isfinite(from.to[i])) return false;
  }
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(from.rotation[i])) return false;
  }
  for (int i = 0; i < 6; ++i) {
    for (const float v : {from.low[i], from.high[i], from.springHertz[i],
                          from.springRatio[i], from.springRest[i]}) {
      if (std::isnan(v)) return false;
    }
  }
  for (const float v : {from.swing, from.speed, from.strength, from.goal,
                        from.goalHertz, from.goalRatio, from.limitHertz,
                        from.limitRatio, from.breakingForce,
                        from.breakingTorque}) {
    if (std::isnan(v)) return false;
  }
  return true;
}

} // namespace

bool World::join(const OrblitPhysicsJoint &from) {
  if (!isMakeable(from)) return false;
  const JointDescription made = describe(from);
  if (!joints_.add(made, bodies_)) return false;

  wakeEnds(made.a, made.b);
  return true;
}

bool World::unjoin(OrblitPhysicsId id) {
  const Joint *joint = joints_.find(id);
  if (joint == nullptr) return false;
  const OrblitPhysicsId a = joint->a;
  const OrblitPhysicsId b = joint->b;
  joints_.remove(id);

  // Woken after it is gone, so waking does not reach across it.
  wakeEnds(a, b);
  return true;
}

bool World::joint(OrblitPhysicsId id, OrblitPhysicsJointState &out) const {
  return joints_.state(bodies_, id, out);
}

void World::breakJoints(float delta) {
  if (joints_.empty()) return;
  broken_.clear();
  joints_.strain(bodies_, delta, broken_);
  for (const Broken &gave : broken_) {
    const Joint *joint = joints_.find(gave.id);
    const OrblitPhysicsId a = joint->a;
    const OrblitPhysicsId b = joint->b;
    joints_.remove(gave.id);

    OrblitPhysicsEvent event{};
    event.kind = ORBLIT_PHYSICS_BROKE;
    event.a = gave.id;
    event.at[0] = gave.at.x;
    event.at[1] = gave.at.y;
    event.at[2] = gave.at.z;
    event.force = gave.force;
    events_.push_back(event);

    // Both were being solved to have broken it, so both are awake; this is
    // for a partner on the far side that the joint no longer reaches.
    wakeEnds(a, b);
  }
}

void World::wakeEnds(OrblitPhysicsId a, OrblitPhysicsId b) {
  for (const OrblitPhysicsId end : {a, b}) {
    if (end != 0) wake(bodies_.rowOf(end));
  }
}

void World::note(uint32_t kind, uint32_t row) {
  OrblitPhysicsEvent event{};
  event.kind = kind;
  event.a = bodies_.id(row);
  events_.push_back(event);
}

// ------------------------------------------------------------------ step ---

void World::step(float delta) {
  events_.clear();
  if (!(delta > 0.0f)) return;

  rememberPositions();
  findContacts(delta);
  feelZones();
  solveStep(delta);

  bodies_.refreshAll();
  moveCharacters(delta);
  reportTouches();
  reportSensing();
  updateSleep(delta);

  wasTouching_.swap(touching_);
  wasSensing_.swap(sensing_);
}

void World::solveStep(float delta) {
  const float h = delta / static_cast<float>(solving_.subSteps);

  solver_.prepare(bodies_, manifolds_.data(),
                  static_cast<uint32_t>(manifolds_.size()), solving_);
  for (uint32_t i = 0; i < solving_.subSteps; ++i) {
    integrateVelocities(h);
    solver_.solve(bodies_, joints_, h, solving_);
    capSpeeds();
    integratePositions(h);
    solver_.relax(bodies_, joints_, h, solving_);
  }
  solver_.finish(bodies_, manifolds_.data(), joints_, solving_);

  if (solver_.rebounds()) {
    solver_.rebound(bodies_);
    // A bounce may be faster than the cap, which holds a body at the end of
    // every step.
    capSpeeds();
  }
  breakJoints(delta);
}

void World::findContacts(float delta) {
  manifolds_.clear();
  keys_.clear();

  // Fixed bodies are not swept. They never move, so the tree in `bodies_` has
  // them, and each body that moves asks it what is near. A plane has no
  // bounds to overlap, being half the world, so it is paired against
  // everything else. There are only ever a few.
  planes_.clear();
  sorted_.clear();
  for (const uint32_t row : bodies_.scanned()) {
    if (bodies_.shape(row).kind == ShapeKind::plane) {
      planes_.push_back(row);
    } else {
      sorted_.push_back(row);
    }
  }

  // Swept along x: once two bodies are further apart along one axis than
  // either reaches, nothing after them in the order can touch the first one
  // either, so the inner loop stops rather than running to the end.
  std::sort(sorted_.begin(), sorted_.end(), [this](uint32_t l, uint32_t r) {
    return bodies_.bounds(l).low.x < bodies_.bounds(r).low.x;
  });

  reach_.clear();
  longest_ = 0.0f;
  for (const uint32_t row : sorted_) {
    reach_.push_back(reachOf(row, delta));
    longest_ = std::max(longest_, reach_.back());
  }

  // A pair that is entirely asleep is left out of the list, which is why a
  // settled stack costs nothing. Touching one of them wakes the whole island,
  // and the pairs inside it were left out, so the list is made again until a
  // pass wakes nothing. That is one pass when nothing sleeping is touched.
  do {
    woke_ = false;
    pairUp();
    wakeTouched(delta);
  } while (woke_);
  pairs_ = static_cast<uint32_t>(candidates_.size());

  Manifold manifold;
  for (const auto &candidate : candidates_) {
    const uint32_t a = candidate.first;
    const uint32_t b = candidate.second;
    // Still asleep after the pass above means it did not touch.
    if (bodies_.asleep(a) || bodies_.asleep(b)) continue;
    if (!meet(a, b, delta, manifold)) continue;

    manifold.a = a;
    manifold.b = b;

    const PairKey key = PairKey::of(bodies_.id(a), bodies_.id(b));
    manifold.reversed = bodies_.id(a) != key.a;
    manifold.rule = ruleFor(key, bodies_.id(a));

    const auto before = wasTouching_.find(key);
    if (before != wasTouching_.end() && before->second.count > 0) {
      carryImpulses(before->second, manifold);
    }

    manifolds_.push_back(manifold);
    keys_.push_back(key);
  }
}

void World::pairUp() {
  candidates_.clear();

  const auto consider = [this](uint32_t a, uint32_t b) {
    // A trigger is somewhere to be, not something to hit. It is read once the
    // step is over, in `reportSensing`.
    if (bodies_.sensor(a) || bodies_.sensor(b)) return;
    // Nothing the solver would move means nothing worth looking at. That is
    // static against static, and it is also a pair that is entirely asleep,
    // which is the whole point of sleeping.
    if (!bodies_.solved(a) && !bodies_.solved(b)) return;
    if (joints_.apart(bodies_.id(a), bodies_.id(b))) return;
    if (ignores(bodies_.id(a), bodies_.id(b))) return;
    if (!interact(bodies_.layerIs(a), bodies_.layerCares(a), bodies_.layerIs(b),
                  bodies_.layerCares(b))) {
      return;
    }
    candidates_.emplace_back(a, b);
  };

  for (size_t i = 0; i < sorted_.size(); ++i) {
    const uint32_t a = sorted_[i];
    const Bounds own = bodies_.bounds(a);
    // Two bodies are near enough to be held apart when the distance between
    // them is under what they can close in a step, which is what each one
    // can travel. The sweep reads the longest reach there is, since it does
    // not know yet which body it will meet.
    const float stop = own.high.x + reach_[i] + longest_;
    for (size_t j = i + 1; j < sorted_.size(); ++j) {
      const uint32_t b = sorted_[j];
      if (bodies_.bounds(b).low.x > stop) break;
      const float near = reach_[i] + reach_[j];
      if (!own.grown(near).overlaps(bodies_.bounds(b))) continue;
      consider(a, b);
    }
    for (const uint32_t plane : planes_) consider(a, plane);
    bodies_.visitStill(own.grown(reach_[i]),
                       [&](uint32_t still) { consider(a, still); });
  }
}

void World::wakeTouched(float delta) {
  Manifold scratch;
  for (const auto &candidate : candidates_) {
    const uint32_t a = candidate.first;
    const uint32_t b = candidate.second;
    // Woken by an earlier pair in this loop, or awake to begin with.
    if (!bodies_.asleep(a) && !bodies_.asleep(b)) continue;
    // A slow pair has no margin, so a sleeper is woken by touching and not by
    // a neighbour a centimetre away, which would wake that neighbour back
    // when it slept and keep a heap from ever settling. A fast one is woken
    // a step early, so that what hits it is stopped there.
    if (!meet(a, b, delta, scratch)) continue;
    wake(a);
    wake(b);
  }
}

float World::extentOf(uint32_t row) const {
  // A field's bounds go down as far as a float does, so squaring them for
  // the length overflows, and nought spin times infinity is not a number. It
  // never moves, so it has no edge to be fast.
  if (bodies_.shape(row).kind == ShapeKind::heightField) return 0.0f;
  const Bounds around = bodies_.bounds(row);
  return 0.5f * length(around.high - around.low);
}

float World::reachOf(uint32_t row, float delta) const {
  if (bodies_.asleep(row)) return 0.0f;
  return (length(bodies_.velocity(row)) +
          length(bodies_.spin(row)) * extentOf(row)) *
         delta;
}

bool World::beyondPlane(uint32_t body, uint32_t plane, float margin) const {
  const Shape &shape = bodies_.shape(plane);
  if (shape.kind != ShapeKind::plane) return false;

  Vec3 normal;
  float surface = 0.0f;
  planeInWorld(shape, bodies_.at(plane), bodies_.rotation(plane), normal,
               surface);
  const Bounds around = bodies_.bounds(body);
  const Vec3 centre = (around.low + around.high) * 0.5f;
  const Vec3 half = (around.high - around.low) * 0.5f;
  const float lowest = dot(normal, centre) - dot(absPerAxis(normal), half);
  return lowest - surface > margin;
}

bool World::meet(uint32_t a, uint32_t b, float delta, Manifold &out) const {
  // How far the two can close on each other this step. Two bodies falling
  // together close on nothing, which is why this is not the sum of reaches.
  const Vec3 apart = bodies_.velocity(a) - bodies_.velocity(b);
  const float closing = (length(apart) + length(bodies_.spin(a)) * extentOf(a) +
                         length(bodies_.spin(b)) * extentOf(b)) *
                        delta;
  const float margin = closing < kFast ? 0.0f : closing;
  // Every body is paired with a plane, there being no bounds to sweep, and
  // most are nowhere near it.
  if (beyondPlane(a, b, margin) || beyondPlane(b, a, margin)) return false;
  return collideNear(bodies_.shape(a), bodies_.at(a), bodies_.rotation(a),
                     bodies_.shape(b), bodies_.at(b), bodies_.rotation(b),
                     margin, out);
}

Rule World::ruleFor(const PairKey &key, OrblitPhysicsId first) const {
  const auto found = rules_.find(key);
  if (found == rules_.end()) return Rule{};
  return first == key.a ? found->second : found->second.turned();
}

void World::feelZones() {
  felt_.clear();
  if (zones_.empty()) return;

  const uint32_t count = bodies_.count();
  felt_.resize(count);
  Manifold inside;
  for (const auto &entry : zones_) {
    const uint32_t trigger = bodies_.rowOf(entry.first);
    for (uint32_t row = 0; row < count; ++row) {
      if (bodies_.solved(row) && within(trigger, row, inside)) {
        felt_[row].add(entry.first, entry.second);
      }
    }
  }
}

void World::integrateVelocities(float delta) {
  const Felt unfelt;
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row)) continue;
    // No zones means nothing was built to read, and every body feels only
    // the world.
    const Felt &felt = felt_.empty() ? unfelt : felt_[row];
    bodies_.velocity(row) +=
        felt.gravity(gravity_) * (bodies_.controls(row).gravityScale * delta);
    bodies_.velocity(row) *=
        damped(felt.linearDamping(bodies_.linearDamping(row)), delta);
    bodies_.spin(row) *=
        damped(felt.angularDamping(bodies_.angularDamping(row)), delta);
    // Gravity, or a zone's, may point along an axis the body is locked on.
    bodies_.hold(row);
  }
}

void World::capSpeeds() {
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row)) continue;
    const Controls &controls = bodies_.controls(row);
    bodies_.velocity(row) = capped(bodies_.velocity(row), controls.maxSpeed);
    bodies_.spin(row) = capped(bodies_.spin(row), controls.maxSpin);
  }
}

void World::integratePositions(float delta) {
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    // A kinematic body moves too: it is driven rather than solved, and a
    // platform that never went anywhere would not be much of a platform. A
    // character is moved later, by looking before it goes.
    if (bodies_.motion(row) == Motion::fixed) continue;
    if (bodies_.motion(row) == Motion::character) continue;
    if (bodies_.movable(row) && bodies_.asleep(row)) continue;
    // The turn is cut by the same share as the move, so a body that stops at
    // a wall has not also swung into it.
    const bool continuous =
        bodies_.solved(row) && bodies_.controls(row).continuous;
    const float run = continuous ? freeRun(row, delta) : 1.0f;
    bodies_.at(row) += bodies_.velocity(row) * (delta * run);
    bodies_.turn(row, bodies_.spin(row), delta * run);
  }
}

float World::freeRun(uint32_t row, float delta) const {
  const Vec3 velocity = bodies_.velocity(row);
  const float travel = length(velocity) * delta;
  if (!(travel > kStill)) return 1.0f;

  Sieve sieve;
  sieve.layerIs = bodies_.layerIs(row);
  sieve.layerCares = bodies_.layerCares(row);
  sieve.ignore = bodies_.id(row);
  const Placed moving{bodies_.shape(row), bodies_.at(row), bodies_.rotation(row)};
  const Journey journey(moving, velocity * (1.0f / length(velocity)), travel, sieve);

  // A body it is already inside belongs to the contacts, and counting it
  // would hold a ball back for resting a few millimetres into the floor. One
  // it reached in an earlier sub-step has no contact yet, because the
  // contacts were found where the bodies began the step, and is stopped
  // where it is until the next step finds the overlap.
  float nearest = travel;
  const auto ask = [&](uint32_t other) {
    Impact impact;
    if (ignores(bodies_.id(row), bodies_.id(other))) return;
    if (joints_.apart(bodies_.id(row), bodies_.id(other))) return;
    if (!meets(bodies_, other, journey, impact)) return;
    if (!impact.started) {
      nearest = std::min(nearest, impact.distance + kPastTouch);
    } else if (!holds(row, other)) {
      nearest = 0.0f;
    }
  };
  bodies_.visitStill(journey.reach, ask);
  for (const uint32_t plane : planes_) ask(plane);
  return nearest / travel;
}

bool World::holds(uint32_t a, uint32_t b) const {
  return std::any_of(manifolds_.begin(), manifolds_.end(),
                     [a, b](const Manifold &manifold) {
                       const bool pair = (manifold.a == a && manifold.b == b) ||
                                         (manifold.a == b && manifold.b == a);
                       return pair && manifold.count > 0;
                     });
}

void World::rememberPositions() {
  const uint32_t count = bodies_.count();
  began_.resize(count);
  for (uint32_t row = 0; row < count; ++row) began_[row] = bodies_.at(row);
}

void World::updateSleep(float delta) {
  if (!settings_.sleeping) return;

  keepDragged();
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row)) continue;

    // Measured at the fastest-moving point rather than the centre, so a body
    // spinning on the spot is not mistaken for a body at rest. The velocity
    // is what the step left, which a contact that is pushing the body out of
    // an overlap has already taken back. How far it went is not, so a body
    // that is climbing out of the ground is not at rest.
    const float walked = length(bodies_.at(row) - began_[row]) / delta;
    const float speed = std::fmax(length(bodies_.velocity(row)), walked) +
                        length(bodies_.spin(row)) * bodies_.reach(row);
    if (speed > settings_.sleepSpeed) {
      bodies_.still(row) = 0.0f;
    } else {
      bodies_.still(row) += delta;
    }
  }

  sleepIslands();
}

void World::keepDragged() {
  for (const Manifold &manifold : manifolds_) {
    const bool moving = lengthSquared(bodies_.surface(manifold.a)) > 0.0f ||
                        lengthSquared(bodies_.surface(manifold.b)) > 0.0f;
    if (!moving) continue;
    bodies_.still(manifold.a) = 0.0f;
    bodies_.still(manifold.b) = 0.0f;
  }
}

void World::sleepIslands() {
  // An island is found by following this step's contacts and the joints
  // between bodies the solver moves, and only those: a wall does not tie
  // together the two chains hanging from it, and the ground does not tie
  // together everything lying on it.
  const uint32_t count = bodies_.count();
  group_.resize(count);
  for (uint32_t row = 0; row < count; ++row) group_[row] = row;
  const auto root = [this](uint32_t row) {
    while (group_[row] != row) {
      group_[row] = group_[group_[row]];
      row = group_[row];
    }
    return row;
  };

  for (const Manifold &manifold : manifolds_) {
    if (bodies_.solved(manifold.a) && bodies_.solved(manifold.b)) {
      group_[root(manifold.a)] = root(manifold.b);
    }
  }
  for (const Joint &joint : joints_.all()) {
    if (joint.a == 0 || joint.b == 0) continue;
    const uint32_t a = bodies_.rowOf(joint.a);
    const uint32_t b = bodies_.rowOf(joint.b);
    if (!bodies_.movable(a) || !bodies_.movable(b)) continue;
    group_[root(a)] = root(b);
  }

  // An island sleeps when every body in it has been still long enough, and
  // none of it hangs from something moving. A rope tied to a rising lift is
  // still relative to the lift, and would otherwise go to sleep in mid-air.
  // A body asleep already is not in any island, so it counts for nothing.
  ready_.assign(count, 1);
  for (uint32_t row = 0; row < count; ++row) {
    if (bodies_.solved(row) && bodies_.still(row) < settings_.sleepAfter) {
      ready_[root(row)] = 0;
    }
  }
  for (const Joint &joint : joints_.all()) {
    const uint32_t a = joint.a == 0 ? Bodies::kNone : bodies_.rowOf(joint.a);
    const uint32_t b = joint.b == 0 ? Bodies::kNone : bodies_.rowOf(joint.b);
    if (a == Bodies::kNone || b == Bodies::kNone ||
        bodies_.movable(a) == bodies_.movable(b)) {
      continue;
    }
    const uint32_t held = bodies_.movable(a) ? a : b;
    const uint32_t holder = bodies_.movable(a) ? b : a;
    if (lengthSquared(bodies_.velocity(holder)) > 0.0f ||
        lengthSquared(bodies_.spin(holder)) > 0.0f) {
      ready_[root(held)] = 0;
    }
  }

  named_.assign(count, 0);
  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row)) continue;
    const uint32_t head = root(row);
    if (!ready_[head]) continue;

    if (named_[head] == 0) named_[head] = ++lastIsland_;
    islands_[named_[head]].bodies.push_back(bodies_.id(row));
    bodies_.sleep(row, named_[head]);
    note(ORBLIT_PHYSICS_SLEPT, row);
  }

  keepResting();
}

void World::keepResting() {
  // A pair that is entirely at rest is not looked at again until one of its
  // bodies wakes, so it is not ended by that. It goes with its island, and is
  // put back when the island wakes. A driven body or a character moves without
  // waking what it touches, so a pair with one of those is left to end.
  const auto holds = [this](uint32_t row) {
    return bodies_.motion(row) == Motion::fixed || bodies_.asleep(row);
  };
  for (size_t i = 0; i < manifolds_.size(); ++i) {
    const Manifold &manifold = manifolds_[i];
    if (!holds(manifold.a) || !holds(manifold.b)) continue;

    const uint32_t name = bodies_.island(manifold.a) != 0
                              ? bodies_.island(manifold.a)
                              : bodies_.island(manifold.b);
    const auto touch = touching_.find(keys_[i]);
    if (name == 0 || touch == touching_.end()) continue;
    islands_[name].pairs.push_back(*touch);
    touching_.erase(touch);
  }
}

void World::reportTouches() {
  touching_.clear();

  for (size_t i = 0; i < manifolds_.size(); ++i) {
    const Manifold &manifold = manifolds_[i];
    const PairKey &key = keys_[i];
    touching_.emplace(key, manifold);

    if (wasTouching_.find(key) == wasTouching_.end()) {
      events_.push_back(pairEvent(ORBLIT_PHYSICS_TOUCH_BEGAN, key, manifold));
    } else if (bodies_.stay(manifold.a) || bodies_.stay(manifold.b)) {
      events_.push_back(pairEvent(ORBLIT_PHYSICS_TOUCH_STAY, key, manifold));
    }
  }

  const size_t ended = events_.size();
  for (const auto &was : wasTouching_) {
    if (touching_.find(was.first) != touching_.end()) continue;
    OrblitPhysicsEvent event{};
    event.kind = ORBLIT_PHYSICS_TOUCH_ENDED;
    event.a = was.first.a;
    event.b = was.first.b;
    events_.push_back(event);
  }
  sortByPair(events_, ended);
}

OrblitPhysicsEvent World::pairEvent(uint32_t kind, const PairKey &key,
                                    const Manifold &manifold) const {
  // How hard, in newton-seconds along the normal, the number a collision sound
  // scales with. Summed over the points, because a box landing flat on four
  // corners hit as hard as the four of them together.
  float force = 0.0f;
  Vec3 at;
  float deepest = -1.0f;
  for (uint32_t c = 0; c < manifold.count; ++c) {
    force += manifold.points[c].normalImpulse;
    if (manifold.points[c].depth > deepest) {
      deepest = manifold.points[c].depth;
      at = manifold.points[c].at;
    }
  }

  OrblitPhysicsEvent event{};
  event.kind = kind;
  event.a = key.a;
  event.b = key.b;
  event.at[0] = at.x;
  event.at[1] = at.y;
  event.at[2] = at.z;
  // The manifold's normal points out of its own `b`, which is not always the
  // key's. Turn it so the event means what it says.
  const Vec3 normal =
      bodies_.id(manifold.b) == key.b ? manifold.normal : -manifold.normal;
  event.normal[0] = normal.x;
  event.normal[1] = normal.y;
  event.normal[2] = normal.z;
  event.force = force;
  return event;
}

void World::reportSensing() {
  sensing_.clear();
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (bodies_.sensor(row)) senseFrom(row);
  }

  // Found by what is missing rather than by anything the body did, so a body
  // that was destroyed, or a trigger that was, leaves in the same way as one
  // that walked out.
  const size_t exited = events_.size();
  for (const PairKey &was : wasSensing_) {
    if (sensing_.find(was) != sensing_.end()) continue;
    OrblitPhysicsEvent event{};
    event.kind = ORBLIT_PHYSICS_EXITED;
    event.a = was.a;
    event.b = was.b;
    events_.push_back(event);
  }
  sortByPair(events_, exited);
}

void World::senseFrom(uint32_t trigger) {
  const OrblitPhysicsId id = bodies_.id(trigger);
  const uint32_t count = bodies_.count();
  Manifold inside;
  for (uint32_t body = 0; body < count; ++body) {
    if (!within(trigger, body, inside)) continue;

    const PairKey key{id, bodies_.id(body)};
    sensing_.insert(key);
    const bool began = wasSensing_.find(key) == wasSensing_.end();
    if (began || bodies_.stay(trigger) || bodies_.stay(body)) {
      events_.push_back(pairEvent(
          began ? ORBLIT_PHYSICS_ENTERED : ORBLIT_PHYSICS_INSIDE, key, inside));
    }
  }
}

bool World::within(uint32_t trigger, uint32_t body, Manifold &inside) const {
  // Neither of two triggers sees the other, and a static body is where it was
  // made for ever, so what it says about a trigger is never news.
  if (bodies_.sensor(body) || bodies_.motion(body) == Motion::fixed) return false;
  if (!interact(bodies_.layerIs(trigger), bodies_.layerCares(trigger),
                bodies_.layerIs(body), bodies_.layerCares(body))) {
    return false;
  }
  if (ignores(bodies_.id(trigger), bodies_.id(body))) return false;

  // A plane has no bounds worth asking, being half the world.
  const bool unbounded = bodies_.shape(trigger).kind == ShapeKind::plane ||
                         bodies_.shape(body).kind == ShapeKind::plane;
  if (!unbounded && !bodies_.bounds(trigger).overlaps(bodies_.bounds(body))) {
    return false;
  }

  // Asleep or not: a trigger is a fact about where things are.
  if (!collide(bodies_.shape(trigger), bodies_.at(trigger),
               bodies_.rotation(trigger), bodies_.shape(body), bodies_.at(body),
               bodies_.rotation(body), inside) ||
      inside.count == 0) {
    return false;
  }
  // Named the way round a pair event reads them: out of the body, towards the
  // trigger.
  inside.a = trigger;
  inside.b = body;
  return true;
}

// --------------------------------------------------------------- reading ---

uint32_t World::read(const OrblitPhysicsId *ids, uint32_t count, float *out,
                     uint32_t stride, uint32_t offset) const {
  if (ids == nullptr || out == nullptr || stride < 7) return 0;

  uint32_t written = 0;
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t row = bodies_.rowOf(ids[i]);
    if (row == Bodies::kNone) continue;

    float *into = out + static_cast<size_t>(i) * stride + offset;
    const Vec3 &at = bodies_.at(row);
    const Quat &rotation = bodies_.rotation(row);
    into[0] = at.x;
    into[1] = at.y;
    into[2] = at.z;
    into[3] = rotation.x;
    into[4] = rotation.y;
    into[5] = rotation.z;
    into[6] = rotation.w;
    ++written;
  }
  return written;
}

bool World::cast(const OrblitPhysicsCast &query, OrblitPhysicsHit &out) const {
  Shape shape;
  if (!castable(query) || !shapeOf(query, shape)) return false;

  Impact impact;
  const uint32_t hit = nearest(bodies_, castOf(shape, query), impact);
  if (hit == Bodies::kNone) return false;

  out = hitOf(bodies_.id(hit), impact);
  return true;
}

uint32_t World::castAll(const OrblitPhysicsCast &query, OrblitPhysicsHit *out,
                        uint32_t capacity) const {
  Shape shape;
  if (!castable(query) || out == nullptr || capacity == 0) return 0;
  if (!shapeOf(query, shape)) return 0;

  uint32_t written = 0;
  forEachMeeting(bodies_, castOf(shape, query), [&](uint32_t row, const Impact &impact) {
    written = keepNearest(out, written, capacity, hitOf(bodies_.id(row), impact));
    return true;
  });
  return written;
}

bool World::castAny(const OrblitPhysicsCast &query) const {
  Shape shape;
  if (!castable(query) || !shapeOf(query, shape)) return false;

  bool found = false;
  forEachMeeting(bodies_, castOf(shape, query), [&](uint32_t, const Impact &) {
    found = true;
    return false;
  });
  return found;
}

uint32_t World::overlap(const OrblitPhysicsCast &query, OrblitPhysicsId *out,
                        uint32_t capacity) const {
  Shape shape;
  if (out == nullptr || capacity == 0 || !shapeOf(query, shape)) return 0;

  // A cast that goes nowhere. Only what it already overlaps can be met.
  const Journey stay = journeyOf(shape, query, Vec3{}, 0.0f);
  uint32_t written = 0;
  forEachMeeting(bodies_, stay, [&](uint32_t row, const Impact &) {
    out[written++] = bodies_.id(row);
    return written < capacity;
  });
  return written;
}

uint32_t World::footing(const OrblitPhysicsId *ids, uint32_t count,
                        OrblitPhysicsFooting *out) const {
  if (ids == nullptr || out == nullptr) return 0;

  uint32_t written = 0;
  for (uint32_t i = 0; i < count; ++i) {
    const Character *character = characterOf(ids[i]);
    if (character == nullptr) continue;

    OrblitPhysicsFooting &into = out[i];
    into = OrblitPhysicsFooting{};
    into.ground = character->ground;
    into.normal[0] = character->normal.x;
    into.normal[1] = character->normal.y;
    into.normal[2] = character->normal.z;
    into.velocity[0] = character->wanted.x;
    into.velocity[1] = character->wanted.y;
    into.velocity[2] = character->wanted.z;
    into.carried[0] = character->carried.x;
    into.carried[1] = character->carried.y;
    into.carried[2] = character->carried.z;
    into.turning = character->turning;
    into.grounded = character->grounded;
    into.wall = character->wall;
    into.ceiling = character->ceiling;
    ++written;
  }
  return written;
}

} // namespace orblit
