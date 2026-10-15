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
// Four kinds bend that. A distance joint's one row is along the line between
// its points rather than along an axis of its frame, and a cone's swing row is
// along whichever way it is swinging, because a cone limits how far it swings
// rather than which way. A path joint's frame is wherever the path is, so its
// rows are measured along the path at the point nearest `b`. A coupling holds
// one row only, whose coordinate is a sum of the two bodies' own: a gear's
// turns, a rack's slide and a pinion's turn, a rope's two lengths.
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
/// motor, which drives the first or the fourth, then a spring on each of the
/// six. A way may have a spring, and a limit or a lock besides, so it may have
/// two rows.
constexpr int kMotorRow = 6;
constexpr int kSpringRow = 7;
constexpr int kJointRows = 13;

/// The most points a path joint is given.
constexpr uint32_t kMostPathPoints = 10000;

/// What a coupling ties together, for the kinds that tie a coordinate of `a` to
/// one of `b` and do not hold one body's place against the other's.
enum class Tied : uint8_t { None, Gear, Rack, Pulley };

/// A coupling's one coordinate, and the sum it keeps.
///
/// The coordinate is made of turns, and a turn is not a thing a pose holds: a
/// body that has gone round three times is in the pose it began in. So it is
/// summed, a sub-step at a time, from how far each body turned since it was
/// last looked at.
struct Tie {
  Tied kind = Tied::None;

  /// Gear: how far `b` turns for each turn of `a`, the same way about their
  /// own x axes. Rack: how far `a` moves along `axis` for each radian `b` turns.
  /// Pulley: how much rope comes off `b`'s side for one off `a`'s.
  float ratio = 0.0f;

  /// Rack: the way `a` slides, in the world, as it was when the joint was made.
  Vec3 axis;

  /// Pulley: the two points the rope runs over, in the world.
  Vec3 over[2];

  /// The coordinate when the joint was made, which is what it keeps.
  float constant = 0.0f;

  /// Gear and rack: how far `a` and `b` have turned about their axes, and the
  /// rotation each had when that was last added up.
  float turned[2] = {};
  Quat last[2];
};

/// A path for a joint's point on `b` to follow, in `a`'s frame, or in the world
/// when `a` is the world.
struct Track {
  std::vector<Vec3> points;

  /// How far along the path each point is, in metres, from the first.
  std::vector<float> arc;

  /// Whether `b`'s x axis is kept along the path as well as its point on it.
  bool follow = false;
};

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
  float goal = 0.0f;
  float goalHertz = 0.0f;
  float goalRatio = 0.0f;
  float springHertz[6] = {};
  float springRatio[6] = {};
  float springRest[6] = {};
  float limitHertz = 0.0f;
  float limitRatio = 0.0f;
  float ratio = 0.0f;
  Vec3 axisB;
  Vec3 over[2];
  std::vector<Vec3> path;
  bool follow = false;
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

  /// A motor with a `goalHertz` holds its way at `goal`, as a spring of that
  /// frequency and damping ratio would, and never pushes harder than
  /// `strength`. It has no use for `speed`, which a motor of that kind leaves.
  float goal = 0.0f;
  float goalHertz = 0.0f;
  float goalRatio = 0.0f;

  /// Which ways have a spring, one bit each, and its frequency, damping ratio
  /// and the place it holds the way at. A spring in the place of a lock is
  /// what makes it soft, so the lock's bit is gone from `holds`.
  uint32_t sprung = 0;
  float springHertz[6] = {};
  float springRatio[6] = {};
  float springRest[6] = {};

  /// With a frequency, a limit is a spring that takes over once the way is
  /// past its end, rather than a wall. Locks are not limits and stay hard.
  float limitHertz = 0.0f;
  float limitRatio = 0.0f;

  /// A coupling holds its coordinate as way 0, and has no frame to hold the
  /// others by. A path joint's point follows `track`, which gives way 0 its
  /// arc length and the rest their frame.
  Tie tie;
  Track track;
  bool onPath() const { return !track.points.empty(); }

  float breakingForce = 0.0f;
  float breakingTorque = 0.0f;
  bool collide = false;

  /// Each row's accumulated impulse over one sub-step, kept from one to the
  /// next to warm start from, as a contact's is.
  float impulse[kJointRows] = {};

  /// What the sub-steps of this step have held with so far, in the same rows.
  /// Strain is read from these, because the last sub-step alone can miss a
  /// yank that the first one took.
  float total[kJointRows] = {};

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

/// Every joint in a world, in the order they were made.
///
/// They are not solved in that order. A joint is as far from the world, or
/// from anything that does not move, as the joints between them, and a chain
/// is solved from its fixed end outwards and back again, a few times over,
/// ending at the fixed end. The last joint solved is the one that holds
/// exactly, so it is the one that holds the rest up. See `arrange`.
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

  // The solver's half, called in this order within each sub-step of a step.
  // `prepare` builds the rows from where the bodies are before they move, and
  // `correctPositions` works from where they are after the last of them.

  /// `h` is the length of the sub-step, which the limits and the motor turn
  /// into speeds and forces with.
  void prepare(Bodies &bodies, float h);
  bool idle() const { return rows_.empty(); }
  void warmStart(Bodies &bodies);

  /// Solves every row once. The joints nearest the world go first, or last
  /// when `leavesFirst`. A caller that makes several passes turns every other
  /// one round and ends on `leavesFirst`, so a chain's root has the last word.
  void correctVelocities(Bodies &bodies, bool leavesFirst);

  /// Whether a joint is holding the body in row `row` this step.
  bool carries(uint32_t row) const {
    return row < carried_.size() && carried_[row] != 0;
  }
  /// Adds the sub-step's impulses to the step's.
  void accumulate();
  void correctPositions(Bodies &bodies);

  /// Works out what each joint solved this step held with, over the `delta`
  /// of the whole step, and adds every one that held with more than it could
  /// take to `broken`. Removes none: the caller does that, and says so. The
  /// next step's sum starts from nothing.
  void strain(const Bodies &bodies, float delta, std::vector<Broken> &broken);

 private:
  /// A joint being solved this step, and its bodies as the solver sees them.
  struct Held {
    uint32_t joint = 0;
    uint32_t a = 0;
    uint32_t b = 0;
    /// Whether the solver will move the body: free and awake. The world, a
    /// fixed or driven body and a sleeper have infinite mass.
    bool movesA = false;
    bool movesB = false;
    /// Inverse mass on each world axis, zero on a locked one.
    Vec3 gainA;
    Vec3 gainB;
    Mat3 inertiaA;
    Mat3 inertiaB;

    /// This joint's rows in `rows_`, which are kept together.
    uint32_t firstRow = 0;
    uint32_t rowCount = 0;
  };

  /// One row of one joint, for the velocity passes.
  struct Row {
    uint32_t held = 0;
    int which = 0;

    /// Pushing along the row moves `b` along `linearB` and `a` along
    /// `linearA` the other way; `turnA` and `turnB` are how that turns each.
    /// Most rows have one `linear` for both. A row about an axis has none, and
    /// both turns are the axis. A coupling has a different row for each body.
    Vec3 linearA;
    Vec3 linearB;
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

    /// A hard row takes all of what a pass finds and none of what it held. A
    /// soft one takes part and gives part back. See `Softness`.
    float massScale = 1.0f;
    float impulseScale = 0.0f;
  };

  Held heldFor(const Bodies &bodies, uint32_t index) const;

  /// Puts the joints in `order_`: nearest the world, or to a body that does
  /// not move, first. A joint's distance is how many joints are between it
  /// and one of those, and a structure that touches none counts from the
  /// joint made first. Ties stay in the order they were made.
  void arrange(const Bodies &bodies);

  /// Builds every row `joint` has this sub-step, of length `h`, onto
  /// `rows_`, and gives `held` the range they are in.
  void buildRows(const Bodies &bodies, Held &held, Joint &joint, float h);

  /// Solves the rows of one held joint, once, in the order they were built.
  void solveHeld(Bodies &bodies, const Held &held);

  /// How hard it is to change the speed along a row, inverted. Zero when
  /// neither body can move, which makes every impulse along it zero.
  static float massAlong(const Held &held, const Vec3 &linearA,
                         const Vec3 &linearB, const Vec3 &turnA,
                         const Vec3 &turnB);

  static float speedAlong(const Bodies &bodies, const Held &held,
                          const Row &row);
  static void apply(Bodies &bodies, const Held &held, const Row &row,
                    float impulse);

  std::vector<Joint> joints_;
  std::unordered_map<OrblitPhysicsId, uint32_t> index_;

  /// Indices into `joints_`, in the order they are solved. Made again when a
  /// joint is added or removed. A body that stops or starts moving does not
  /// remake it, which leaves a worse order but never a wrong one.
  std::vector<uint32_t> order_;
  bool arranged_ = false;

  /// One flag per body row, set for each body a joint solved this step moves.
  std::vector<uint8_t> carried_;

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
