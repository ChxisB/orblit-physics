#include "solver.h"

#include <algorithm>

namespace orblit {
namespace {

/// The most speed a soft contact pushes two bodies apart with, in metres per
/// second.
///
/// Deep overlap happens: a body spawned inside a wall, a teleport. Without a
/// ceiling the push out of it is a shove big enough to put the body somewhere
/// else entirely. Capped, it takes a few frames to climb out, which looks like
/// nothing at all.
constexpr float kMaxPush = 3.0f;

/// How far into the slop a body that was stopped short of a surface comes to
/// rest, as a fraction of it. Stopped exactly at the surface it would be on
/// the edge of touching it, and the contact would come and go with the last
/// digit of its height. A quarter in is well outside the rounding, and the
/// push, which starts at the surface, takes it back out to where it holds.
constexpr float kSettle = 0.25f;

/// How much a contact's spring is damped, as a multiple of critical damping.
/// Far past critical, so that a stack of springs settles without a beat
/// running up and down it.
constexpr float kDamping = 10.0f;

/// The fastest a contact's spring may turn, as a share of the sub-step rate.
/// A spring that is stiffer than the step can follow goes unstable. At a
/// quarter, a wall stepped at 30 a second threw boxes off at 13 metres a
/// second. At this share a wall of ten by ten settles there, and one of 25 by
/// 20 still comes down. With the default settings it binds below about 55
/// steps a second and is not reached at 60.
constexpr float kMostOfRate = 0.15f;

/// How many times the points of one manifold are solved against each other
/// before the bodies take the result. A face has four points that push on one
/// another, and one sweep leaves the last of them over-corrected. Eight is
/// where the residual stops changing what a box face does.
constexpr uint32_t kBlockSweeps = 8;

/// What a soft contact does to the impulse a pass would have made, and how
/// fast it pushes out of an overlap, for a sub-step of length `h`.
Softness softnessFor(float h, float hertz) {
  return softnessOf(h, std::fmin(hertz, kMostOfRate / h), kDamping);
}

} // namespace

void Solver::solve(Bodies &bodies, Joints &joints, float h,
                   const SolverSettings &settings) {
  joints.prepare(bodies, h);
  if (idle(joints)) return;

  aim(bodies, h, settings, true);
  joints.warmStart(bodies);
  warmStart(bodies);
  solveVelocities(bodies, joints, settings.velocitySteps, settings.jointSteps);
}

void Solver::relax(Bodies &bodies, Joints &joints, float h,
                   const SolverSettings &settings) {
  if (idle(joints)) return;

  aim(bodies, h, settings, false);
  solveVelocities(bodies, joints, 1, settings.jointSteps);
  joints.accumulate();
  accumulate();
}

void Solver::finish(Bodies &bodies, Manifold *manifolds, Joints &joints,
                    const SolverSettings &settings) {
  for (uint32_t i = 0; i < settings.positionSteps; ++i) {
    joints.correctPositions(bodies);
  }

  for (const Pair &pair : pairs_) {
    Manifold &manifold = manifolds[pair.manifold];
    for (uint32_t c = 0; c < pair.count; ++c) {
      const Row &row = rows_[pair.first + c];
      manifold.points[c].normalImpulse = row.total;
      manifold.points[c].frictionImpulse[0] = row.frictionTotal[0];
      manifold.points[c].frictionImpulse[1] = row.frictionTotal[1];
    }
  }
}

Solver::Pair Solver::pairOf(const Bodies &bodies, const Manifold &manifold) {
  Pair pair;
  pair.a = manifold.a;
  pair.b = manifold.b;

  const Rule &rule = manifold.rule;
  // Two slippery things together are slipperier than either against
  // something grippy, which a geometric mean gives and an average does not.
  pair.friction =
      (rule.overrides & ORBLIT_PHYSICS_RULE_FRICTION) != 0
          ? rule.friction
          : std::sqrt(bodies.friction(pair.a) * bodies.friction(pair.b));
  pair.bounciness =
      (rule.overrides & ORBLIT_PHYSICS_RULE_RESTITUTION) != 0
          ? rule.restitution
          : std::fmax(bodies.restitution(pair.a), bodies.restitution(pair.b));
  pair.belt = bodies.surface(pair.a) - bodies.surface(pair.b);

  // A body the solver will not move contributes nothing, whether that is
  // because it is static or because it is asleep. A rule scales what is left,
  // which makes a body heavier or lighter for this contact and no other.
  const float moveA = bodies.solved(pair.a) ? rule.move[0] : 0.0f;
  const float moveB = bodies.solved(pair.b) ? rule.move[1] : 0.0f;
  pair.gainA = bodies.gain(pair.a) * moveA;
  pair.gainB = bodies.gain(pair.b) * moveB;
  pair.inertiaA = bodies.inverseInertia(pair.a) * moveA;
  pair.inertiaB = bodies.inverseInertia(pair.b) * moveB;
  return pair;
}

void Solver::prepare(const Bodies &bodies, const Manifold *manifolds,
                     uint32_t count, const SolverSettings &settings) {
  pairs_.clear();
  rows_.clear();
  const float share = 1.0f / static_cast<float>(settings.subSteps);

  for (uint32_t m = 0; m < count; ++m) {
    const Manifold &manifold = manifolds[m];
    if (manifold.count == 0) continue;

    Pair pair = pairOf(bodies, manifold);
    pair.manifold = m;
    pair.first = static_cast<uint32_t>(rows_.size());
    pair.count = manifold.count;

    const uint32_t index = static_cast<uint32_t>(pairs_.size());
    for (uint32_t c = 0; c < manifold.count; ++c) {
      const Contact &contact = manifold.points[c];

      Row row;
      row.pair = index;
      // `at` is midway between the surfaces, which are `depth` apart along the
      // normal. A lever to it is too long on a body that has a gap to its
      // neighbour and too short on one that overlaps, and friction turns the
      // body by the difference. Each body's arm runs to its own surface.
      const Vec3 half = contact.normal * (contact.depth * 0.5f);
      const Vec3 onA = contact.at - half;
      const Vec3 onB = contact.at + half;
      row.leverA = onA - bodies.centre(pair.a);
      row.leverB = onB - bodies.centre(pair.b);
      // From the origin, not the centre: that is what the point and the
      // rotation carry it along with.
      row.localA = unrotate(bodies.rotation(pair.a), onA - bodies.at(pair.a));
      row.localB = unrotate(bodies.rotation(pair.b), onB - bodies.at(pair.b));
      row.normal = contact.normal;
      perpendiculars(row.normal, row.tangent[0], row.tangent[1]);
      row.normalImpulse = contact.normalImpulse * share;
      row.frictionImpulse[0] = contact.frictionImpulse[0] * share;
      row.frictionImpulse[1] = contact.frictionImpulse[1] * share;
      row.normalMass = effectiveMass(pair, row, row.normal);
      for (int t = 0; t < 2; ++t) {
        row.tangentMass[t] = effectiveMass(pair, row, row.tangent[t]);
      }

      // Restitution is decided here, from the speed the bodies met at, not
      // during the passes: by then the solver has already taken that speed
      // away, and a bounce computed from what is left is no bounce at all.
      const Vec3 relativeSpeed = bodies.velocity(pair.a) +
                                 cross(bodies.spin(pair.a), row.leverA) -
                                 bodies.velocity(pair.b) -
                                 cross(bodies.spin(pair.b), row.leverB);
      const float closing = dot(relativeSpeed, row.normal);
      row.rebound =
          closing < -settings.bounceThreshold ? -pair.bounciness * closing : 0.0f;

      rows_.push_back(row);
    }

    couple(pair);
    pairs_.push_back(pair);
  }
}

void Solver::couple(Pair &pair) const {
  const Vec3 together = pair.gainA + pair.gainB;
  for (uint32_t i = 0; i < pair.count; ++i) {
    const Row &from = rows_[pair.first + i];
    const Vec3 turnA = cross(from.leverA, from.normal);
    const Vec3 turnB = cross(from.leverB, from.normal);
    for (uint32_t j = 0; j < pair.count; ++j) {
      const Row &to = rows_[pair.first + j];
      pair.coupling[i][j] =
          dot(from.normal, mulPerAxis(together, to.normal)) +
          dot(turnA, pair.inertiaA * cross(to.leverA, to.normal)) +
          dot(turnB, pair.inertiaB * cross(to.leverB, to.normal));
    }
  }
}

void Solver::aim(const Bodies &bodies, float h, const SolverSettings &settings,
                 bool pushes) {
  const Softness soft = softnessFor(h, settings.contactHertz);
  const float settle = settings.slop * kSettle;

  for (Row &row : rows_) {
    const Pair &pair = pairs_[row.pair];

    // Where the two surface points have moved to since the narrowphase ran, and
    // so how far they overlap along the normal now. Following them is not the
    // same as running it again, but it is right for the small moves of a
    // sub-step and it costs two rotations.
    const Vec3 worldA = bodies.at(pair.a) + rotate(bodies.rotation(pair.a), row.localA);
    const Vec3 worldB = bodies.at(pair.b) + rotate(bodies.rotation(pair.b), row.localB);
    const float depth = -dot(worldA - worldB, row.normal);

    row.massScale = 1.0f;
    row.impulseScale = 0.0f;
    row.allowed = 0.0f;
    if (depth < 0.0f) {
      // Apart. The two may close by the gap and a little more, so that they
      // come to rest in each other and not on the edge of it.
      row.allowed = (settle - depth) / h;
    } else if (pushes) {
      row.massScale = soft.massScale;
      row.impulseScale = soft.impulseScale;
      // The push starts at the surface. A spring that is slack for the first
      // centimetres still leaks the load it carries, and a stack sinks through
      // that slack a contact at a time.
      row.allowed = -std::fmin(depth * soft.pushRate, kMaxPush);
    }
  }
}

bool Solver::bounces(const Row &row) {
  // A contact that held nothing this step was not reached, and a body that has
  // not reached a surface has nothing to bounce off.
  return row.rebound > 0.0f && row.total > 0.0f;
}

bool Solver::rebounds() const {
  return std::any_of(rows_.begin(), rows_.end(), bounces);
}

void Solver::rebound(Bodies &bodies) {
  for (const Row &row : rows_) {
    if (!bounces(row)) continue;

    const Pair &pair = pairs_[row.pair];
    const float closing = dot(relative(bodies, pair, row), row.normal);
    if (closing >= row.rebound) continue;
    apply(bodies, pair, row, row.normal * (row.normalMass * (row.rebound - closing)));
  }
}

void Solver::warmStart(Bodies &bodies) {
  for (const Row &row : rows_) {
    const Pair &pair = pairs_[row.pair];
    apply(bodies, pair, row,
          row.normal * row.normalImpulse +
              row.tangent[0] * row.frictionImpulse[0] +
              row.tangent[1] * row.frictionImpulse[1]);
  }
}

void Solver::solveVelocities(Bodies &bodies, Joints &joints,
                             uint32_t contactPasses, uint32_t jointPasses) {
  // Counted back from the end, so the last round is the one from the leaves
  // in and the one that solves every contact, whatever the numbers are.
  const uint32_t rounds =
      joints.idle() ? contactPasses : std::max(contactPasses, jointPasses);
  for (uint32_t left = rounds; left-- > 0;) {
    joints.correctVelocities(bodies, left % 2 == 0);
    correctVelocities(bodies, joints, left < contactPasses);
  }
}

void Solver::correctVelocities(Bodies &bodies, const Joints &joints,
                               bool all) {
  for (const Pair &pair : pairs_) {
    if (!all && !joints.carries(pair.a) && !joints.carries(pair.b)) continue;
    for (uint32_t c = 0; c < pair.count; ++c) {
      correctFriction(bodies, pair, rows_[pair.first + c]);
    }
    correctNormals(bodies, pair);
  }
}

void Solver::correctFriction(Bodies &bodies, const Pair &pair, Row &row) {
  // Bounded by the normal impulse the pass before settled on. The first pass
  // of a contact that is new has no bound and so no friction. The pass that
  // relaxes it is the one that gives it some.
  const float bound = pair.friction * row.normalImpulse;
  for (int t = 0; t < 2; ++t) {
    const float sliding =
        dot(relative(bodies, pair, row) + pair.belt, row.tangent[t]);
    const float was = row.frictionImpulse[t];
    row.frictionImpulse[t] =
        clamped(was - row.tangentMass[t] * sliding, -bound, bound);
    apply(bodies, pair, row, row.tangent[t] * (row.frictionImpulse[t] - was));
  }
}

void Solver::correctNormals(Bodies &bodies, const Pair &pair) {
  // The points of one manifold share two bodies, so each one's impulse moves
  // the others. Solving them one after another in the bodies, one pass each,
  // leaves the first point having taken most of a hard stop and the last
  // having taken the rest from a body already turned, and a box that meets a
  // wall flat leaves it turning. The points are few, so they are solved
  // against one another here, in a small matrix, and the bodies are changed
  // once at the end.
  float residual[kMaxContacts];
  float held[kMaxContacts];
  for (uint32_t i = 0; i < pair.count; ++i) {
    const Row &row = rows_[pair.first + i];
    residual[i] = dot(relative(bodies, pair, row), row.normal) + row.allowed;
    held[i] = row.normalImpulse;
  }

  for (uint32_t sweep = 0; sweep < kBlockSweeps; ++sweep) {
    for (uint32_t i = 0; i < pair.count; ++i) {
      Row &row = rows_[pair.first + i];
      // May push and never pull: a contact that would need to hold the
      // bodies together is a contact that has ended. A soft one takes only
      // part of the push, and gives back part of what it held.
      const float next = std::fmax(
          row.normalImpulse -
              row.massScale * row.normalMass * residual[i] -
              row.impulseScale * row.normalImpulse,
          0.0f);
      const float change = next - row.normalImpulse;
      row.normalImpulse = next;
      for (uint32_t j = 0; j < pair.count; ++j) {
        residual[j] += pair.coupling[i][j] * change;
      }
    }
  }

  for (uint32_t i = 0; i < pair.count; ++i) {
    const Row &row = rows_[pair.first + i];
    apply(bodies, pair, row, row.normal * (row.normalImpulse - held[i]));
  }
}

void Solver::accumulate() {
  for (Row &row : rows_) {
    row.total += row.normalImpulse;
    row.frictionTotal[0] += row.frictionImpulse[0];
    row.frictionTotal[1] += row.frictionImpulse[1];
  }
}

float Solver::effectiveMass(const Pair &pair, const Row &row, const Vec3 &along) {
  const Vec3 turnA = cross(row.leverA, along);
  const Vec3 turnB = cross(row.leverB, along);
  const float k = dot(along, mulPerAxis(pair.gainA, along)) +
                  dot(along, mulPerAxis(pair.gainB, along)) +
                  dot(cross(pair.inertiaA * turnA, row.leverA), along) +
                  dot(cross(pair.inertiaB * turnB, row.leverB), along);
  return k > kTiny ? 1.0f / k : 0.0f;
}

Vec3 Solver::relative(const Bodies &bodies, const Pair &pair, const Row &row) {
  return bodies.velocity(pair.a) + cross(bodies.spin(pair.a), row.leverA) -
         bodies.velocity(pair.b) - cross(bodies.spin(pair.b), row.leverB);
}

void Solver::apply(Bodies &bodies, const Pair &pair, const Row &row,
                   const Vec3 &impulse) {
  bodies.velocity(pair.a) += mulPerAxis(impulse, pair.gainA);
  bodies.spin(pair.a) += pair.inertiaA * cross(row.leverA, impulse);
  bodies.velocity(pair.b) -= mulPerAxis(impulse, pair.gainB);
  bodies.spin(pair.b) -= pair.inertiaB * cross(row.leverB, impulse);
}

} // namespace orblit
