// Joints: what holds two bodies together, and how the solver keeps them held.
//
// Every kind is the same thing underneath. There are six ways `b` can move
// relative to `a` — along the three axes of the joint's frame and about them
// — and a joint holds some of them, each within a range. Fixed holds all six
// at nought. A hinge holds five at nought and the sixth within its limits, if
// it has any. A kind is only which ways it holds and how far, decided once
// when the joint is made, so the solver has one kind of row to solve rather
// than seven kinds of joint.
//
// Two kinds bend that. A distance joint's one row is along the line between
// its points rather than along an axis of its frame, and a cone's swing row is
// along whichever way it is swinging, because a cone limits how far it swings
// rather than which way.
//
// A row is a lever, an effective mass and an accumulated impulse, the same as
// a contact's, solved in the same passes: velocity passes that stop it moving
// the wrong way, then position passes that put back what drifted. A joint has
// no slop, unlike a contact. A contact a little apart is a crate a millimetre
// off the floor, which nobody sees; a joint a little apart is a chain that
// stretches every time it swings.

#ifndef ORBLIT_PHYSICS_JOINT_H
#define ORBLIT_PHYSICS_JOINT_H

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "body.h"
#include "maths.h"
#include "orblit_physics.h"

namespace orblit {

/// The six ways `b` can move, in the order the header names them, then the
/// motor, which drives the first or the fourth.
constexpr int kJointRows = 7;
constexpr int kMotorRow = 6;

/// How a joint was asked for, in the engine's own numbers.
struct JointDescription {
  OrblitPhysicsId id = 0;
  OrblitPhysicsId a = 0;
  OrblitPhysicsId b = 0;
  uint32_t kind = 0;
  uint32_t limited = 0;
  Vec3 at;
  Quat rotation;
  Vec3 to;
  float low[6] = {};
  float high[6] = {};
  float swing = 0.0f;
  float speed = 0.0f;
  float strength = 0.0f;
  float breakingForce = 0.0f;
  float breakingTorque = 0.0f;
  bool collide = false;
};

struct Joint {
  OrblitPhysicsId id = 0;

  /// Either may be zero, for the world: a body at the origin that never
  /// moves. Never both.
  OrblitPhysicsId a = 0;
  OrblitPhysicsId b = 0;
  uint32_t kind = 0;

  /// The joint's point and frame on each body, in that body's own frame —
  /// or, for the world, in the world.
  Vec3 anchorA;
  Vec3 anchorB;
  Quat frameA;
  Quat frameB;

  /// Which of the six ways it holds, one bit each, and the range each is held
  /// within. A range whose least is its most is locked there.
  uint32_t holds = 0;
  float least[6] = {};
  float most[6] = {};

  /// The first row is the distance between the points, not along x.
  bool distance = false;

  /// The fifth row is the swing, whichever way it goes, not the turn about y.
  bool cone = false;

  /// Which row the motor drives, 0 or 3, or -1 for no motor.
  int motor = -1;
  float speed = 0.0f;
  float strength = 0.0f;

  float breakingForce = 0.0f;
  float breakingTorque = 0.0f;
  bool collide = false;

  /// Each row's accumulated impulse, kept from one step to the next to warm
  /// start from, as a contact's is.
  float impulse[kJointRows] = {};

  /// What it held with on the last step it was solved.
  float force = 0.0f;
  float torque = 0.0f;
};

/// A joint that gave way this step, and what it was doing when it did.
struct Broken {
  OrblitPhysicsId id = 0;
  Vec3 at;
  float force = 0.0f;
};

/// Every joint in a world, in the order they were made — which is the order
/// they are solved in, so a chain built from its fixed end outwards is solved
/// that way too, and settles in fewer passes than one solved from its tip.
class Joints {
 public:
  bool empty() const { return joints_.empty(); }
  const std::vector<Joint> &all() const { return joints_; }

  /// Makes a joint where the bodies stand. False, and nothing changed, if it
  /// cannot be made: see `orblit_physics_join` for what that means.
  bool add(const JointDescription &from, const Bodies &bodies);

  /// Removes joint `id`. False if there was none.
  bool remove(OrblitPhysicsId id);

  const Joint *find(OrblitPhysicsId id) const;

  /// Removes every joint on `body`, adding the other body of each to
  /// `partners`.
  void drop(OrblitPhysicsId body, std::vector<OrblitPhysicsId> &partners);

  /// Whether any joint is on `body`. One lookup, so a body with none pays
  /// nothing for joints existing.
  bool on(OrblitPhysicsId body) const {
    return !counts_.empty() && counts_.find(body) != counts_.end();
  }

  /// Whether `x` and `y` are joined by a joint that keeps them from colliding.
  bool apart(OrblitPhysicsId x, OrblitPhysicsId y) const {
    return !apart_.empty() && apart_.find(PairKey::of(x, y)) != apart_.end();
  }

  /// Calls `visit` with the other body of every joint on `body`. The world is
  /// not a body and is never visited.
  template <typename Visit>
  void eachPartner(OrblitPhysicsId body, Visit &&visit) const {
    if (!on(body)) return;
    for (const Joint &joint : joints_) {
      if (joint.a == body && joint.b != 0) visit(joint.b);
      if (joint.b == body && joint.a != 0) visit(joint.a);
    }
  }

  /// How joint `id` stands now. False if there is no such joint.
  bool state(const Bodies &bodies, OrblitPhysicsId id,
             OrblitPhysicsJointState &out) const;

  // The solver's half, called in this order within a step. `prepare` builds
  // the rows from where the bodies are before they move, and the position
  // passes work from where they are after.

  void prepare(Bodies &bodies, float delta);
  bool idle() const { return rows_.empty(); }
  void warmStart(Bodies &bodies);
  void correctVelocities(Bodies &bodies);
  void correctPositions(Bodies &bodies);

  /// Works out what each joint solved this step held with, and adds every
  /// one that held with more than it could take to `broken`. Removes none:
  /// the caller does that, and says so.
  void strain(const Bodies &bodies, float delta, std::vector<Broken> &broken);

 private:
  /// A joint being solved this step, and its bodies as the solver sees them.
  struct Held {
    uint32_t joint = 0;
    uint32_t a = 0;
    uint32_t b = 0;
    float inverseMassA = 0.0f;
    float inverseMassB = 0.0f;
    Mat3 inertiaA;
    Mat3 inertiaB;
  };

  /// One row of one joint, for the velocity passes.
  struct Row {
    uint32_t held = 0;
    int which = 0;

    /// Pushing along `linear` at `b`'s point moves `b` along it and `a` the
    /// other way; `turnA` and `turnB` are how that turns each. A row about
    /// an axis has no `linear`, and both turns are the axis.
    Vec3 linear;
    Vec3 turnA;
    Vec3 turnB;
    float mass = 0.0f;

    /// The speed the passes aim for along the row, and the least and most
    /// its accumulated impulse may be. A locked row aims for nought either
    /// way; a limit may only push; a motor aims for its speed, no harder
    /// than its strength.
    float target = 0.0f;
    float least = 0.0f;
    float most = 0.0f;
  };

  Held heldFor(const Bodies &bodies, uint32_t index) const;

  static float speedAlong(const Bodies &bodies, const Held &held,
                          const Row &row);
  static void apply(Bodies &bodies, const Held &held, const Row &row,
                    float impulse);

  std::vector<Joint> joints_;
  std::unordered_map<OrblitPhysicsId, uint32_t> index_;

  /// How many joints each body is on, so a body on none is one lookup.
  std::unordered_map<OrblitPhysicsId, uint32_t> counts_;

  /// How many joints keep each pair from colliding.
  std::unordered_map<PairKey, uint32_t, PairKeyHash> apart_;

  /// Scratch for a step, kept so a step does not allocate.
  std::vector<Held> held_;
  std::vector<Row> rows_;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_JOINT_H
