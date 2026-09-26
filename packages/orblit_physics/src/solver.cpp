#include "solver.h"

namespace orblit {
namespace {

/// The most one position pass will move a pair apart, in metres.
///
/// Deep overlap happens — a body spawned inside a wall, a teleport — and
/// without a ceiling the correction for it is a shove big enough to put the
/// body somewhere else entirely. Capped, it takes a few frames to climb out,
/// which looks like nothing at all.
constexpr float kMaxPush = 0.2f;

/// How hard it is to change the velocity of the contact point along `along`,
/// inverted. Zero when neither body can move, which makes the impulse zero
/// without a branch anywhere else.
float effectiveMass(float inverseMassA, float inverseMassB, const Mat3 &inertiaA,
                    const Mat3 &inertiaB, const Vec3 &leverA, const Vec3 &leverB,
                    const Vec3 &along) {
  const Vec3 turnA = cross(leverA, along);
  const Vec3 turnB = cross(leverB, along);
  const float k = inverseMassA + inverseMassB +
                  dot(cross(inertiaA * turnA, leverA), along) +
                  dot(cross(inertiaB * turnB, leverB), along);
  return k > kTiny ? 1.0f / k : 0.0f;
}

} // namespace

void Solver::solveVelocities(Bodies &bodies, const Manifold *manifolds,
                             uint32_t count, Joints &joints, float delta,
                             const SolverSettings &settings) {
  prepare(bodies, manifolds, count, settings);
  joints.prepare(bodies, delta);
  if (rows_.empty() && joints.idle()) return;

  joints.warmStart(bodies);
  warmStart(bodies);
  for (uint32_t i = 0; i < settings.velocitySteps; ++i) {
    joints.correctVelocities(bodies);
    correctVelocities(bodies);
  }
}

void Solver::solvePositions(Bodies &bodies, Manifold *manifolds, Joints &joints,
                            const SolverSettings &settings) {
  for (uint32_t i = 0; i < settings.positionSteps; ++i) {
    joints.correctPositions(bodies);
    correctPositions(bodies, settings);
  }

  for (const Pair &pair : pairs_) {
    Manifold &manifold = manifolds[pair.manifold];
    for (uint32_t c = 0; c < pair.count; ++c) {
      const Row &row = rows_[pair.first + c];
      manifold.points[c].normalImpulse = row.normalImpulse;
      manifold.points[c].frictionImpulse[0] = row.frictionImpulse[0];
      manifold.points[c].frictionImpulse[1] = row.frictionImpulse[1];
    }
  }
}

void Solver::prepare(Bodies &bodies, const Manifold *manifolds,
                     uint32_t count,
                     const SolverSettings &settings) {
  pairs_.clear();
  rows_.clear();

  for (uint32_t m = 0; m < count; ++m) {
    const Manifold &manifold = manifolds[m];
    if (manifold.count == 0) continue;

    Pair pair;
    pair.manifold = m;
    pair.a = manifold.a;
    pair.b = manifold.b;
    // Two slippery things together are slipperier than either against
    // something grippy, which a geometric mean gives and an average does not.
    pair.friction =
        std::sqrt(bodies.friction(pair.a) * bodies.friction(pair.b));
    // A body the solver will not move contributes nothing, whether that is
    // because it is static or because it is asleep.
    pair.inverseMassA = bodies.solved(pair.a) ? bodies.inverseMass(pair.a) : 0.0f;
    pair.inverseMassB = bodies.solved(pair.b) ? bodies.inverseMass(pair.b) : 0.0f;
    pair.inertiaA =
        bodies.solved(pair.a) ? bodies.inverseInertia(pair.a) : Mat3::zero();
    pair.inertiaB =
        bodies.solved(pair.b) ? bodies.inverseInertia(pair.b) : Mat3::zero();
    pair.first = static_cast<uint32_t>(rows_.size());
    pair.count = manifold.count;

    const float bounciness =
        std::fmax(bodies.restitution(pair.a), bodies.restitution(pair.b));

    const uint32_t index = static_cast<uint32_t>(pairs_.size());
    for (uint32_t c = 0; c < manifold.count; ++c) {
      const Contact &contact = manifold.points[c];

      Row row;
      row.pair = index;
      row.leverA = contact.at - bodies.at(pair.a);
      row.leverB = contact.at - bodies.at(pair.b);
      row.localA = unrotate(bodies.rotation(pair.a), row.leverA);
      row.localB = unrotate(bodies.rotation(pair.b), row.leverB);
      row.normal = contact.normal;
      perpendiculars(row.normal, row.tangent[0], row.tangent[1]);
      row.depth = contact.depth;
      row.normalImpulse = contact.normalImpulse;
      row.frictionImpulse[0] = contact.frictionImpulse[0];
      row.frictionImpulse[1] = contact.frictionImpulse[1];
      row.normalMass =
          effectiveMass(pair.inverseMassA, pair.inverseMassB, pair.inertiaA,
                        pair.inertiaB, row.leverA, row.leverB, row.normal);
      for (int t = 0; t < 2; ++t) {
        row.tangentMass[t] =
            effectiveMass(pair.inverseMassA, pair.inverseMassB, pair.inertiaA,
                          pair.inertiaB, row.leverA, row.leverB, row.tangent[t]);
      }

      // Restitution is decided here, from the speed the bodies met at, not
      // during the passes: by then the solver has already taken that speed
      // away, and a bounce computed from what is left is no bounce at all.
      const Vec3 relative = bodies.velocity(pair.a) +
                            cross(bodies.spin(pair.a), row.leverA) -
                            bodies.velocity(pair.b) -
                            cross(bodies.spin(pair.b), row.leverB);
      const float closing = dot(relative, row.normal);
      row.bounce = closing < -settings.bounceThreshold ? -bounciness * closing : 0.0f;

      rows_.push_back(row);
    }

    pairs_.push_back(pair);
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

void Solver::correctVelocities(Bodies &bodies) {
  for (Row &row : rows_) {
    const Pair &pair = pairs_[row.pair];

    // Friction first, bounded by the normal impulse the pass before settled
    // on. On the very first pass there is no bound and so no friction, which
    // is why one velocity step is not enough for anything to grip.
    const float bound = pair.friction * row.normalImpulse;
    for (int t = 0; t < 2; ++t) {
      const float sliding = dot(relative(bodies, pair, row), row.tangent[t]);
      const float was = row.frictionImpulse[t];
      row.frictionImpulse[t] =
          clamped(was - row.tangentMass[t] * sliding, -bound, bound);
      apply(bodies, pair, row, row.tangent[t] * (row.frictionImpulse[t] - was));
    }

    // Then the normal, which may push and never pull: a contact that would
    // need to hold the bodies together is a contact that has ended.
    const float closing = dot(relative(bodies, pair, row), row.normal);
    const float was = row.normalImpulse;
    row.normalImpulse =
        std::fmax(was - row.normalMass * (closing - row.bounce), 0.0f);
    apply(bodies, pair, row, row.normal * (row.normalImpulse - was));
  }
}

void Solver::correctPositions(Bodies &bodies, const SolverSettings &settings) {
  for (const Row &row : rows_) {
    const Pair &pair = pairs_[row.pair];

    // Where the two contact points have moved to since the narrowphase ran.
    // Following them is not the same as re-running it, but it is right for
    // the small corrections a pass makes and it costs two rotations.
    const Vec3 worldA = bodies.at(pair.a) + rotate(bodies.rotation(pair.a), row.localA);
    const Vec3 worldB = bodies.at(pair.b) + rotate(bodies.rotation(pair.b), row.localB);
    const float depth = row.depth - dot(worldA - worldB, row.normal);

    const float over = depth - settings.slop;
    if (over <= 0.0f) continue;

    const Vec3 leverA = worldA - bodies.at(pair.a);
    const Vec3 leverB = worldB - bodies.at(pair.b);
    const float push = clamped(over * settings.stiffness, 0.0f, kMaxPush);
    const Vec3 impulse = row.normal * (row.normalMass * push);

    bodies.at(pair.a) += impulse * pair.inverseMassA;
    bodies.rotation(pair.a) = integrate(
        bodies.rotation(pair.a), pair.inertiaA * cross(leverA, impulse), 1.0f);
    bodies.at(pair.b) -= impulse * pair.inverseMassB;
    bodies.rotation(pair.b) = integrate(
        bodies.rotation(pair.b), -(pair.inertiaB * cross(leverB, impulse)), 1.0f);
  }
}

Vec3 Solver::relative(const Bodies &bodies, const Pair &pair, const Row &row) {
  return bodies.velocity(pair.a) + cross(bodies.spin(pair.a), row.leverA) -
         bodies.velocity(pair.b) - cross(bodies.spin(pair.b), row.leverB);
}

void Solver::apply(Bodies &bodies, const Pair &pair, const Row &row,
                   const Vec3 &impulse) {
  bodies.velocity(pair.a) += impulse * pair.inverseMassA;
  bodies.spin(pair.a) += pair.inertiaA * cross(row.leverA, impulse);
  bodies.velocity(pair.b) -= impulse * pair.inverseMassB;
  bodies.spin(pair.b) -= pair.inertiaB * cross(row.leverB, impulse);
}

} // namespace orblit
