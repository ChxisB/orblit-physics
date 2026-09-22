#include "world.h"

#include <algorithm>

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

/// Orblit's `Layers` rule, unchanged: a pair interacts when either cares about
/// the other, not when both do.
bool interact(uint32_t isA, uint32_t caresA, uint32_t isB, uint32_t caresB) {
  return (caresA & isB) != 0 || (caresB & isA) != 0;
}

PairKey keyOf(OrblitPhysicsId a, OrblitPhysicsId b) {
  return a < b ? PairKey{a, b} : PairKey{b, a};
}

/// Shed as a fraction per second, implicitly, so a large delta slows a body
/// rather than reversing it.
float damped(float damping, float delta) {
  return 1.0f / (1.0f + std::fmax(damping, 0.0f) * delta);
}

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
      bodies_.remove(command.id);
      return;

    case ORBLIT_PHYSICS_PLACE:
      bodies_.place(row, vectorOf(command.at), rotationOf(command.rotation));
      wake(row);
      return;

    case ORBLIT_PHYSICS_VELOCITY:
      bodies_.velocity(row) = vectorOf(command.vector);
      bodies_.spin(row) = vectorOf(command.spin);
      bodies_.refresh(row);
      wake(row);
      return;

    case ORBLIT_PHYSICS_IMPULSE: {
      if (!bodies_.movable(row)) return;
      wake(row);
      const Vec3 impulse = vectorOf(command.vector);
      const Vec3 lever = vectorOf(command.spin) - bodies_.at(row);
      bodies_.velocity(row) += impulse * bodies_.inverseMass(row);
      bodies_.spin(row) += bodies_.inverseInertia(row) * cross(lever, impulse);
      return;
    }

    case ORBLIT_PHYSICS_WAKE:
      wake(row);
      return;

    default:
      return;
  }
}

void World::wake(uint32_t row) {
  if (!bodies_.movable(row) || !bodies_.asleep(row)) return;
  bodies_.wake(row);
  note(ORBLIT_PHYSICS_WOKE, row);
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
                          static_cast<uint32_t>(manifolds_.size()), solving_);
  integratePositions(delta);
  solver_.solvePositions(bodies_, manifolds_.data(), solving_);

  bodies_.refreshAll();
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
    const PairKey key = keyOf(bodies_.id(a), bodies_.id(b));
    const auto before = wasTouching_.find(key);
    if (before != wasTouching_.end() && before->second.count > 0) {
      const Manifold &old = before->second;
      const bool flipped = old.a != a;
      for (uint32_t c = 0; c < manifold.count; ++c) {
        uint32_t nearest = 0;
        float best = 3.0e38f;
        for (uint32_t o = 0; o < old.count; ++o) {
          const float away = lengthSquared(old.points[o].at - manifold.points[c].at);
          if (away < best) {
            best = away;
            nearest = o;
          }
        }
        // The rows a manifold names can swap between steps; the impulse is
        // along a normal that flips with them.
        const float sign = flipped ? -1.0f : 1.0f;
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
    // platform that never went anywhere would not be much of a platform.
    if (bodies_.motion(row) == Motion::fixed) continue;
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
  const Vec3 direction = normalised(vectorOf(query.direction));
  if (lengthSquared(direction) < kTiny) return false;

  Placed moving;
  // A ray is a sphere of no size, which is what it is, and means one routine
  // below rather than two that differ by a radius of zero.
  moving.shape = query.shape == 0 ? Shape::sphere(0.0f)
                                  : Shape::fromCommand(query.shape, query.size);
  moving.at = vectorOf(query.from);
  moving.rotation = rotationOf(query.rotation);
  if (moving.shape.kind == ShapeKind::plane) return false;

  const float distance = std::fmax(query.distance, 0.0f);
  const Vec3 finish = moving.at + direction * distance;

  // Where the cast could possibly reach, as one box: the shape at each end of
  // its travel and everything between. Grown by a hair so a body it meets
  // exactly edge on is not filtered out before it is looked at properly.
  const Bounds begins = moving.shape.boundsAt(moving.at, moving.rotation);
  const Bounds ends = moving.shape.boundsAt(finish, moving.rotation);
  const Bounds swept = Bounds{minPerAxis(begins.low, ends.low),
                              maxPerAxis(begins.high, ends.high)}
                           .grown(1.0e-3f);

  Impact nearest{};
  uint32_t hit = Bodies::kNone;
  const uint32_t count = bodies_.count();
  for (uint32_t row = 0; row < count; ++row) {
    if (bodies_.id(row) == query.ignore) continue;
    if (!interact(query.layerIs, query.layerCares, bodies_.layerIs(row),
                  bodies_.layerCares(row))) {
      continue;
    }
    // A half-space has no bounds to test against — it is half the world — so
    // it is always asked properly, the way the broadphase treats it.
    const bool half = bodies_.shape(row).kind == ShapeKind::plane;
    if (!half && !swept.overlaps(bodies_.bounds(row))) continue;

    const Placed fixed{bodies_.shape(row), bodies_.at(row),
                       bodies_.rotation(row)};
    Impact impact;
    if (!sweep(moving, direction, distance, fixed, impact)) continue;
    if (hit != Bodies::kNone && impact.distance >= nearest.distance) continue;
    nearest = impact;
    hit = row;
  }
  if (hit == Bodies::kNone) return false;

  out = OrblitPhysicsHit{};
  out.body = bodies_.id(hit);
  out.at[0] = nearest.at.x;
  out.at[1] = nearest.at.y;
  out.at[2] = nearest.at.z;
  out.normal[0] = nearest.normal.x;
  out.normal[1] = nearest.normal.y;
  out.normal[2] = nearest.normal.z;
  out.distance = nearest.distance;
  out.started = nearest.started;
  return true;
}

} // namespace orblit
