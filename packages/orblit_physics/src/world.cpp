#include "world.h"

#include <algorithm>
#include <cmath>

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

} // namespace

// --------------------------------------------------------------------------

void World::defaults(OrblitPhysicsSettings *out) {
  if (out == nullptr) return;
  *out = OrblitPhysicsSettings{};
  out->gravity[0] = 0.0f;
  out->gravity[1] = -9.81f;
  out->gravity[2] = 0.0f;
  out->velocitySteps = 10;
  out->positionSteps = 2;
  out->slop = 0.02f;
  out->stiffness = 0.2f;
  out->bounceThreshold = 1.0f;
  out->sleepSpeed = 0.03f;
  out->sleepAfter = 0.5f;
  out->sleeping = true;
}

World::World(const OrblitPhysicsSettings &settings) : settings_(settings) {
  gravity_ = vectorOf(settings_.gravity);
  // One velocity pass cannot produce friction, so asking for one is asking for
  // ice. Take the two it needs rather than silently giving no grip.
  solving_.velocitySteps = std::max(settings_.velocitySteps, 2u);
  solving_.positionSteps = settings_.positionSteps;
  solving_.slop = std::fmax(settings_.slop, 0.0f);
  solving_.stiffness = clamped(settings_.stiffness, 0.0f, 1.0f);
  solving_.bounceThreshold = std::fmax(settings_.bounceThreshold, 0.0f);
}

// -------------------------------------------------------------- commands ---

void World::submit(const OrblitPhysicsCommand *commands, uint32_t count) {
  if (commands == nullptr) return;
  for (uint32_t i = 0; i < count; ++i) apply(commands[i]);
}

void World::apply(const OrblitPhysicsCommand &command) {
  if (command.kind == ORBLIT_PHYSICS_CREATE) {
    // Ground has its own call, because a command has nowhere to put a grid.
    if (command.shape == ORBLIT_PHYSICS_HEIGHT_FIELD) return;
    BodyDescription made;
    made.shape = Shape::fromCommand(command.shape, command.size);
    made.motion = motionOf(command.motion);
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
      bodies_.refresh(row);
      wake(row);
      return;

    case ORBLIT_PHYSICS_IMPULSE:
      push(row, vectorOf(command.vector), vectorOf(command.spin));
      return;

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

    default:
      return;
  }
}

void World::destroy(OrblitPhysicsId id) {
  // What it was joined to is let go of, not left holding a joint to nothing,
  // and woken, because what was hanging from it should now fall.
  partners_.clear();
  joints_.drop(id, partners_);
  unmake(id);
  for (const OrblitPhysicsId partner : partners_) {
    const uint32_t row = bodies_.rowOf(partner);
    if (row != Bodies::kNone) wake(row);
  }
}

void World::unmake(OrblitPhysicsId id) {
  bodies_.remove(id);
  // Erased in place rather than swapped out, because the list's order is the
  // order characters move in, and two characters that swap turns swap which
  // of them gets to a doorway first.
  characters_.erase(std::remove_if(characters_.begin(), characters_.end(),
                                   [&](const Character &c) { return c.id == id; }),
                    characters_.end());
  // After the body, which points into it.
  fields_.erase(id);
}

bool World::ground(const OrblitPhysicsGround &from) {
  const uint32_t least = from.margin ? 4u : 2u;
  if (from.id == 0 || from.heights == nullptr) return false;
  if (from.columns < least || from.rows < least) return false;
  if (!(from.spacing > 0.0f) || !std::isfinite(from.spacing)) return false;

  auto field = std::make_unique<HeightField>();
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
  const uint32_t count = bodies_.count();
  for (uint32_t other = 0; other < count; ++other) {
    if (bodies_.bounds(other).overlaps(area)) wake(other);
  }
  return true;
}

void World::push(uint32_t row, const Vec3 &impulse, const Vec3 &point) {
  if (!bodies_.movable(row)) return;
  wake(row);
  const Vec3 lever = point - bodies_.at(row);
  bodies_.velocity(row) += impulse * bodies_.inverseMass(row);
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
  if (sleeper) {
    bodies_.wake(row);
    note(ORBLIT_PHYSICS_WOKE, row);
  }
  // An awake body that the solver moves is in a group that is awake already,
  // because joined bodies only ever sleep together.
  if (sleeper || !bodies_.movable(row)) wakeJoined(row);
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
      bodies_.wake(other);
      note(ORBLIT_PHYSICS_WOKE, other);
      waking_.push_back(partner);
    });
  }
}

// ---------------------------------------------------------------- joints ---

bool World::join(const OrblitPhysicsJoint &from) {
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
  }
  made.swing = from.swing;
  made.speed = from.speed;
  made.strength = from.strength;
  made.breakingForce = from.breakingForce;
  made.breakingTorque = from.breakingTorque;
  made.collide = from.collide;
  // A NaN anywhere is a joint that puts NaN into every body it touches on
  // its first step, so it is never made. Where it stands must be somewhere
  // as well; a limit may be as wide as it likes.
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(from.at[i]) || !std::isfinite(from.to[i])) return false;
  }
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(from.rotation[i])) return false;
  }
  for (int i = 0; i < 6; ++i) {
    if (std::isnan(from.low[i]) || std::isnan(from.high[i])) return false;
  }
  for (const float v : {from.swing, from.speed, from.strength,
                        from.breakingForce, from.breakingTorque}) {
    if (std::isnan(v)) return false;
  }
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

  findContacts();
  integrateVelocities(delta);
  solver_.solveVelocities(bodies_, manifolds_.data(),
                          static_cast<uint32_t>(manifolds_.size()), joints_,
                          delta, solving_);
  integratePositions(delta);
  solver_.solvePositions(bodies_, manifolds_.data(), joints_, solving_);
  breakJoints(delta);

  bodies_.refreshAll();
  moveCharacters(delta);
  reportTouches();
  updateSleep(delta);

  wasTouching_.swap(touching_);
}

void World::findContacts() {
  manifolds_.clear();
  keys_.clear();
  candidates_.clear();

  const uint32_t count = bodies_.count();

  // A plane has no bounds to overlap — it is half the world — so it is paired
  // against everything else rather than swept. There are only ever a few.
  planes_.clear();
  sorted_.clear();
  for (uint32_t row = 0; row < count; ++row) {
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

  const auto consider = [this](uint32_t a, uint32_t b) {
    // Nothing the solver would move means nothing worth looking at. That is
    // static against static, and it is also a pair that is entirely asleep —
    // which is the whole point of sleeping, and why a settled stack costs
    // nothing until something disturbs it.
    if (!bodies_.solved(a) && !bodies_.solved(b)) return;
    if (joints_.apart(bodies_.id(a), bodies_.id(b))) return;
    if (!interact(bodies_.layerIs(a), bodies_.layerCares(a), bodies_.layerIs(b),
                  bodies_.layerCares(b))) {
      return;
    }
    candidates_.emplace_back(a, b);
  };

  for (size_t i = 0; i < sorted_.size(); ++i) {
    const uint32_t a = sorted_[i];
    const float reach = bodies_.bounds(a).high.x;
    for (size_t j = i + 1; j < sorted_.size(); ++j) {
      const uint32_t b = sorted_[j];
      if (bodies_.bounds(b).low.x > reach) break;
      if (!bodies_.bounds(a).overlaps(bodies_.bounds(b))) continue;
      consider(a, b);
    }
    for (const uint32_t plane : planes_) consider(a, plane);
  }

  Manifold manifold;
  for (const auto &candidate : candidates_) {
    const uint32_t a = candidate.first;
    const uint32_t b = candidate.second;
    if (!collide(bodies_.shape(a), bodies_.at(a), bodies_.rotation(a),
                 bodies_.shape(b), bodies_.at(b), bodies_.rotation(b),
                 manifold)) {
      continue;
    }

    // Something touching a sleeper wakes it. It is too late for this step's
    // solve to know that, so the pair is dropped and picked up next step,
    // which costs one frame of overlap and no correctness.
    if (bodies_.asleep(a) || bodies_.asleep(b)) {
      wake(a);
      wake(b);
      continue;
    }

    manifold.a = a;
    manifold.b = b;

    // Warm starting: carry last step's impulse onto whichever of this step's
    // points is nearest. No distance limit, deliberately — an old manifold
    // only exists because the pair was already touching, so the worst a bad
    // match can do is start a pass from a number that is merely close, and
    // the passes then correct it within the same step.
    //
    // A limit on direction, though. Against ground the points of one pair
    // can face different ways, and the push that held a corner against one
    // slope is no start at all for a corner now against the other.
    const PairKey key = PairKey::of(bodies_.id(a), bodies_.id(b));
    const auto before = wasTouching_.find(key);
    if (before != wasTouching_.end() && before->second.count > 0) {
      const Manifold &old = before->second;
      // The rows a manifold names can swap between steps; the impulse is
      // along a normal that flips with them.
      const float sign = old.a != a ? -1.0f : 1.0f;
      for (uint32_t c = 0; c < manifold.count; ++c) {
        uint32_t nearest = Bodies::kNone;
        float best = 3.0e38f;
        for (uint32_t o = 0; o < old.count; ++o) {
          if (dot(old.points[o].normal, manifold.points[c].normal) * sign <
              kSameWay) {
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

    manifolds_.push_back(manifold);
    keys_.push_back(key);
  }
}

void World::integrateVelocities(float delta) {
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row)) continue;
    bodies_.velocity(row) += gravity_ * delta;
    bodies_.velocity(row) *= damped(bodies_.linearDamping(row), delta);
    bodies_.spin(row) *= damped(bodies_.angularDamping(row), delta);
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
    bodies_.at(row) += bodies_.velocity(row) * delta;
    bodies_.rotation(row) =
        integrate(bodies_.rotation(row), bodies_.spin(row), delta);
  }
}

void World::updateSleep(float delta) {
  if (!settings_.sleeping) return;

  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row)) continue;

    // Measured at the fastest-moving point rather than the centre, so a body
    // spinning on the spot is not mistaken for a body at rest.
    const float speed = length(bodies_.velocity(row)) +
                        length(bodies_.spin(row)) * bodies_.shape(row).reach();
    if (speed > settings_.sleepSpeed) {
      bodies_.still(row) = 0.0f;
      continue;
    }

    bodies_.still(row) += delta;
    if (bodies_.still(row) < settings_.sleepAfter) continue;

    // A joined body waits for the rest of what it is joined to.
    if (joints_.on(bodies_.id(row))) continue;
    bodies_.sleep(row);
    note(ORBLIT_PHYSICS_SLEPT, row);
  }

  sleepJoined();
}

void World::sleepJoined() {
  if (joints_.empty()) return;

  // Joined bodies are grouped by following joints between bodies the solver
  // moves, and only those: a wall does not tie together the two chains
  // hanging from it.
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

  for (const Joint &joint : joints_.all()) {
    if (joint.a == 0 || joint.b == 0) continue;
    const uint32_t a = bodies_.rowOf(joint.a);
    const uint32_t b = bodies_.rowOf(joint.b);
    if (!bodies_.movable(a) || !bodies_.movable(b)) continue;
    group_[root(a)] = root(b);
  }

  // A group sleeps when every body in it has been still long enough — one
  // already asleep counts, since it is as still as a body gets — and none of
  // it hangs from something moving. A rope tied to a rising lift is still
  // relative to the lift, and would otherwise go to sleep in mid-air.
  ready_.assign(count, 1);
  for (const Joint &joint : joints_.all()) {
    const uint32_t a = joint.a == 0 ? Bodies::kNone : bodies_.rowOf(joint.a);
    const uint32_t b = joint.b == 0 ? Bodies::kNone : bodies_.rowOf(joint.b);
    for (const uint32_t row : {a, b}) {
      if (row == Bodies::kNone || !bodies_.movable(row)) continue;
      if (bodies_.solved(row) && bodies_.still(row) < settings_.sleepAfter) {
        ready_[root(row)] = 0;
      }
    }
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

  for (uint32_t row = 0; row < count; ++row) {
    if (!bodies_.solved(row) || !ready_[root(row)]) continue;
    if (!joints_.on(bodies_.id(row))) continue;
    bodies_.sleep(row);
    note(ORBLIT_PHYSICS_SLEPT, row);
  }
}

void World::reportTouches() {
  touching_.clear();

  for (size_t i = 0; i < manifolds_.size(); ++i) {
    const Manifold &manifold = manifolds_[i];
    const PairKey &key = keys_[i];
    touching_.emplace(key, manifold);
    if (wasTouching_.find(key) != wasTouching_.end()) continue;

    // How hard, in newton-seconds along the normal — the number a collision
    // sound scales with. Summed over the points, because a box landing flat
    // on four corners hit as hard as the four of them together.
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
    event.kind = ORBLIT_PHYSICS_TOUCH_BEGAN;
    event.a = key.a;
    event.b = key.b;
    event.at[0] = at.x;
    event.at[1] = at.y;
    event.at[2] = at.z;
    // The manifold's normal points out of its own `b`, which is not always
    // the key's. Turn it so the event means what it says.
    const Vec3 normal =
        bodies_.id(manifold.b) == key.b ? manifold.normal : -manifold.normal;
    event.normal[0] = normal.x;
    event.normal[1] = normal.y;
    event.normal[2] = normal.z;
    event.force = force;
    events_.push_back(event);
  }

  for (const auto &was : wasTouching_) {
    if (touching_.find(was.first) != touching_.end()) continue;
    OrblitPhysicsEvent event{};
    event.kind = ORBLIT_PHYSICS_TOUCH_ENDED;
    event.a = was.first.a;
    event.b = was.first.b;
    events_.push_back(event);
  }
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
  if (query.shape == ORBLIT_PHYSICS_HEIGHT_FIELD) return false;
  const Vec3 direction = normalised(vectorOf(query.direction));
  if (lengthSquared(direction) < kTiny) return false;

  Placed moving;
  // A ray is a sphere of no size, which is what it is, and means one routine
  // below rather than two that differ by a radius of zero.
  moving.shape = query.shape == 0 ? Shape::sphere(0.0f)
                                  : Shape::fromCommand(query.shape, query.size);
  moving.at = vectorOf(query.from);
  moving.rotation = rotationOf(query.rotation);

  Sieve sieve;
  sieve.layerIs = query.layerIs;
  sieve.layerCares = query.layerCares;
  sieve.ignore = query.ignore;

  Impact impact;
  const uint32_t hit = nearest(bodies_, moving, direction,
                               std::fmax(query.distance, 0.0f), sieve, impact);
  if (hit == Bodies::kNone) return false;

  out = OrblitPhysicsHit{};
  out.body = bodies_.id(hit);
  out.at[0] = impact.at.x;
  out.at[1] = impact.at.y;
  out.at[2] = impact.at.z;
  out.normal[0] = impact.normal.x;
  out.normal[1] = impact.normal.y;
  out.normal[2] = impact.normal.z;
  out.distance = impact.distance;
  out.started = impact.started;
  return true;
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
    ++written;
  }
  return written;
}

} // namespace orblit
