// Turning a list of contacts into velocities that do not violate them.
//
// Sequential impulse. Each contact is corrected in turn, using the velocities
// the contacts before it just changed, and the whole list is walked several
// times. It is an iterative answer to a problem with an exact one, and it is
// the right trade: the exact answer is a matrix the size of the pile of
// crates, and nobody can see the difference between a crate held exactly and
// a crate held to within a millimetre.
//
// Three things make it hold up under a stack rather than sagging:
//
// Sub-steps. A step is cut into several short ones, each with one pass over
// the contacts and one move. Contact geometry is read again in each, from
// where the bodies have got to. A short step has a small overlap to correct,
// and a small correction puts little energy into a stack.
//
// Warm starting. The impulse that held a contact last tick is applied before
// the first pass of each sub-step, so the pass starts from nearly the answer
// and spends its budget on what changed. Without it a tall stack visibly
// compresses on every frame and springs back.
//
// Soft contacts and a relax pass. A contact that overlaps is a damped spring,
// not a wall, and the speed it pushes with is capped. After the move, a second
// pass solves the same contacts with no push in them. That takes back the
// speed the spring added, and what is left is the speed the bodies need.
//
// Joints are solved in the same passes, before the contacts in each. A joint
// is a row like a contact's, and solving the two apart, all the joints to
// convergence and then all the contacts, would have each undo the other: a
// ragdoll's arm held to its shoulder and then pushed out of the floor, with
// nothing left to put it back.

#ifndef ORBLIT_PHYSICS_SOLVER_H
#define ORBLIT_PHYSICS_SOLVER_H

#include <vector>

#include "body.h"
#include "collide.h"
#include "joint.h"

namespace orblit {

struct SolverSettings {
  uint32_t subSteps = 6;
  uint32_t velocitySteps = 1;
  uint32_t positionSteps = 2;
  float slop = 0.02f;
  float contactHertz = 50.0f;
  float bounceThreshold = 1.0f;
};

/// Holds its scratch between steps so a step does not allocate.
class Solver {
 public:
  /// Builds a row for each contact in `count` manifolds, from the bodies as
  /// they are before the step moves them.
  ///
  /// Each contact's impulses from the last tick are the start of the first
  /// sub-step, which they are shared out over, and the rows are kept until
  /// `finish`.
  void prepare(const Bodies &bodies, const Manifold *manifolds, uint32_t count,
               const SolverSettings &settings);

  /// The first half of a sub-step of length `h`: warm starts the contacts and
  /// the joints of `joints`, and corrects velocity so that bodies neither
  /// approach nor drift apart faster than the contacts allow. The caller
  /// moves the bodies afterwards.
  void solve(Bodies &bodies, Joints &joints, float h,
             const SolverSettings &settings);

  /// The second half, called after the move: one pass with no push in it, and
  /// the sub-step's impulses added to what the step has held with so far.
  void relax(Bodies &bodies, Joints &joints, float h,
             const SolverSettings &settings);

  /// Puts back the overlap a joint has left, and writes each contact's
  /// impulses over the whole step into the manifold. Those are both for the
  /// next tick to warm start from and so the caller can tell how hard the
  /// two bodies hit.
  void finish(Bodies &bodies, Manifold *manifolds, Joints &joints,
              const SolverSettings &settings);

  /// Whether `rebound` has anything to do: a contact that bounces and held
  /// something this step.
  bool rebounds() const;

  /// Turns round the bodies that met this step, at the speed they met at
  /// times their restitution. It is called after the move, so they have got
  /// to the surface before they leave it, and not before they had.
  void rebound(Bodies &bodies);

 private:
  /// Everything about one manifold the passes need, read once.
  struct Pair {
    uint32_t manifold = 0;
    uint32_t a = 0;
    uint32_t b = 0;
    float friction = 0.0f;
    float bounciness = 0.0f;
    /// How much faster `a`'s surface moves than `b`'s, in the world. Friction
    /// pulls the two bodies towards this rather than towards standing still
    /// against each other, which is all a conveyor is.
    Vec3 belt;
    /// Inverse mass on each world axis, zero on a locked one.
    Vec3 gainA;
    Vec3 gainB;
    Mat3 inertiaA;
    Mat3 inertiaB;
    uint32_t first = 0;
    uint32_t count = 0;
    /// How much a unit impulse along one point's normal changes how fast the
    /// bodies close along another's. The diagonal is the inverse of that
    /// point's `normalMass`.
    float coupling[kMaxContacts][kMaxContacts] = {};
  };

  /// Everything about one contact point, likewise.
  struct Row {
    uint32_t pair = 0;
    Vec3 leverA;
    Vec3 leverB;
    /// The same two points in each body's own frame, so a sub-step can find
    /// where they have moved to without running the narrowphase again.
    Vec3 localA;
    Vec3 localB;
    /// Per point rather than per pair, because a pair touching ground can
    /// be pushed out of two slopes at once.
    Vec3 normal;
    Vec3 tangent[2];
    float normalMass = 0.0f;
    float tangentMass[2] = {0.0f, 0.0f};
    /// How far the points overlapped when the step began. Negative for two
    /// that were apart.
    float depth = 0.0f;
    /// How fast the two may close along the normal this pass. Negative for a
    /// push. Set from where the bodies are, once per pass.
    float allowed = 0.0f;
    /// How much of the impulse the pass would have made it takes, and how much
    /// of what it already held it keeps, for a soft contact. One and zero for
    /// a rigid one.
    float massScale = 1.0f;
    float impulseScale = 0.0f;
    /// The speed to part at once the bodies have met, for a contact that
    /// bounces. Zero for every other.
    float rebound = 0.0f;
    float normalImpulse = 0.0f;
    float frictionImpulse[2] = {0.0f, 0.0f};
    /// The impulses of the sub-steps so far, which are what the manifold
    /// keeps. The ones above are over one sub-step.
    float total = 0.0f;
    float frictionTotal[2] = {0.0f, 0.0f};
  };

  /// What one manifold means for the passes, without its rows.
  static Pair pairOf(const Bodies &bodies, const Manifold &manifold);

  /// Whether a contact bounces and held something this step.
  static bool bounces(const Row &row);

  /// Whether a pass would have nothing to do.
  bool idle(const Joints &joints) const {
    return rows_.empty() && joints.idle();
  }

  /// Works out what each row may do this pass from where its bodies are. A
  /// pass that `pushes` lets an overlap soften the contact and push the two
  /// apart. One that does not holds them where they are.
  void aim(const Bodies &bodies, float h, const SolverSettings &settings,
           bool pushes);
  void warmStart(Bodies &bodies);
  void correctVelocities(Bodies &bodies);
  void correctFriction(Bodies &bodies, const Pair &pair, Row &row);
  void correctNormals(Bodies &bodies, const Pair &pair);
  void accumulate();

  /// Fills in `pair.coupling` from the rows already built for it.
  void couple(Pair &pair) const;

  /// How hard it is to change the velocity of the contact point along `along`,
  /// inverted. Zero when neither body can move, which makes the impulse zero
  /// without a branch anywhere else.
  static float effectiveMass(const Pair &pair, const Row &row, const Vec3 &along);

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
