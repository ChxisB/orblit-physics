// Turning a list of contacts into velocities that do not violate them.
//
// Sequential impulse. Each contact is corrected in turn, using the velocities
// the contacts before it just changed, and the whole list is walked several
// times. It is an iterative answer to a problem with an exact one, and it is
// the right trade: the exact answer is a matrix the size of the pile of
// crates, and nobody can see the difference between a crate held exactly and
// a crate held to within a millimetre.
//
// Two things make it hold up under a stack rather than sagging:
//
// Warm starting. The impulse that held a contact last tick is applied before
// the first pass this tick, so the passes start from nearly the answer and
// spend their budget on what changed. Without it a tall stack visibly
// compresses on every frame and springs back.
//
// Separating the overlap from the velocity. Velocity passes stop bodies
// approaching; a later position pass moves them apart. Folding the overlap
// into the velocity solve instead adds energy that was never there, and a
// stack fed energy by its own contacts is a stack that eventually jumps.

#ifndef ORBLIT_PHYSICS_SOLVER_H
#define ORBLIT_PHYSICS_SOLVER_H

#include <vector>

#include "body.h"
#include "collide.h"

namespace orblit {

struct SolverSettings {
  uint32_t velocitySteps = 10;
  uint32_t positionSteps = 2;
  float slop = 0.02f;
  float stiffness = 0.2f;
  float bounceThreshold = 1.0f;
};

/// Holds its scratch between steps so a step does not allocate.
class Solver {
 public:
  /// Stops the bodies in `count` manifolds approaching one another.
  ///
  /// Reads each contact's impulses from the last tick to start from, and
  /// leaves the scratch it built for the position half, so the two must be
  /// called in order and with the same manifolds.
  void solveVelocities(Bodies &bodies, const Manifold *manifolds, uint32_t count,
                       const SolverSettings &settings);

  /// Moves apart whatever overlap is left, and writes each contact's
  /// accumulated impulses back into the manifold — both for the next tick to
  /// warm start from and so the caller can tell how hard the two bodies hit.
  ///
  /// Called after the caller has integrated positions, because the overlap
  /// worth correcting is the one the bodies have now, not the one they had
  /// before they moved.
  void solvePositions(Bodies &bodies, Manifold *manifolds,
                      const SolverSettings &settings);

 private:
  /// Everything about one manifold the passes need, read once.
  struct Pair {
    uint32_t manifold = 0;
    uint32_t a = 0;
    uint32_t b = 0;
    float friction = 0.0f;
    float inverseMassA = 0.0f;
    float inverseMassB = 0.0f;
    Mat3 inertiaA;
    Mat3 inertiaB;
    uint32_t first = 0;
    uint32_t count = 0;
  };

  /// Everything about one contact point, likewise.
  struct Row {
    uint32_t pair = 0;
    Vec3 leverA;
    Vec3 leverB;
    /// The same two points in each body's own frame, so a position pass can
    /// find where they have moved to without running the narrowphase again.
    Vec3 localA;
    Vec3 localB;
    /// Per point rather than per pair, because a pair touching ground can
    /// be pushed out of two slopes at once.
    Vec3 normal;
    Vec3 tangent[2];
    float normalMass = 0.0f;
    float tangentMass[2] = {0.0f, 0.0f};
    float bounce = 0.0f;
    float depth = 0.0f;
    float normalImpulse = 0.0f;
    float frictionImpulse[2] = {0.0f, 0.0f};
  };

  void prepare(Bodies &bodies, const Manifold *manifolds, uint32_t count,
               const SolverSettings &settings);
  void warmStart(Bodies &bodies);
  void correctVelocities(Bodies &bodies);
  void correctPositions(Bodies &bodies, const SolverSettings &settings);

  /// The velocity of one body's contact point relative to the other's.
  static Vec3 relative(const Bodies &bodies, const Pair &pair, const Row &row);

  /// Pushes `a` along `impulse` and `b` the other way, turning both.
  static void apply(Bodies &bodies, const Pair &pair, const Row &row,
                    const Vec3 &impulse);

  std::vector<Pair> pairs_;
  std::vector<Row> rows_;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_SOLVER_H
