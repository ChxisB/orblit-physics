#include "joint.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace orblit {
namespace {

constexpr float kFar = std::numeric_limits<float>::infinity();

/// What fraction of a joint's error one position pass puts back.
///
/// More than a contact's fifth. A contact has slop to rest in and the rest of
/// a stack to share its push with; a joint has neither, and one that puts back
/// only a fifth of what it drifted is a chain that sags while it swings. Less
/// than all of it, because a joint's rows are corrected one after another, and
/// each full correction would undo part of the one before.
constexpr float kPull = 0.5f;

/// The most one position pass moves a joint's point back, in metres, or turns
/// it back, in radians. For the same reason a contact has a ceiling: a body
/// placed far from where its joint wants it should be walked back over a few
/// frames, not thrown there in one.
constexpr float kMaxPull = 0.2f;
constexpr float kMaxTurn = 0.25f;

/// Below this a distance joint's two points are one point, and have no line
/// between them to hold along.
constexpr float kTouching = 1.0e-5f;

const Vec3 kAlongX{1.0f, 0.0f, 0.0f};
const Vec3 kAlongY{0.0f, 1.0f, 0.0f};
const Vec3 kAlongZ{0.0f, 0.0f, 1.0f};

/// Where a joint stands this instant, from where its bodies are.
struct Pose {
  Vec3 centreA;
  Vec3 centreB;
  Vec3 pointA;
  Vec3 pointB;

  /// The axes of `a`'s frame, in the world. The rows that move are along
  /// these, and the swing rows about them.
  Vec3 axis[3];

  /// The axis the twist is measured about: halfway between the two frames'
  /// x axes, so it is the same axis seen from either body.
  Vec3 twistAxis;
  float twist = 0.0f;

  /// The swing, as a turn about an axis in `a`'s frame, and how far it is.
  /// It never has an x part: turning about x is the twist.
  Vec3 swing;
  float swung = 0.0f;
};

/// Where a joint stands this instant. Either row may be `Bodies::kNone`, the
/// world: a body at the origin, never turned, whose anchor and frame were
/// kept in the world to begin with.
Pose poseOf(const Bodies &bodies, uint32_t rowA, uint32_t rowB,
            const Joint &joint) {
  const bool worldA = rowA == Bodies::kNone;
  const bool worldB = rowB == Bodies::kNone;
  const Quat bodyA = worldA ? Quat{} : bodies.rotation(rowA);
  const Quat bodyB = worldB ? Quat{} : bodies.rotation(rowB);

  Pose pose;
  pose.centreA = worldA ? Vec3{} : bodies.at(rowA);
  pose.centreB = worldB ? Vec3{} : bodies.at(rowB);
  pose.pointA = pose.centreA + rotate(bodyA, joint.anchorA);
  pose.pointB = pose.centreB + rotate(bodyB, joint.anchorB);

  const Quat frameA = bodyA * joint.frameA;
  const Quat frameB = bodyB * joint.frameB;
  pose.axis[0] = rotate(frameA, kAlongX);
  pose.axis[1] = rotate(frameA, kAlongY);
  pose.axis[2] = rotate(frameA, kAlongZ);
  const Vec3 between = normalised(pose.axis[0] + rotate(frameB, kAlongX));
  pose.twistAxis = lengthSquared(between) > 0.0f ? between : pose.axis[0];

  // `b`'s frame seen from `a`'s, split into a twist about x and then a swing
  // that tips x over. The shorter way round, so a small turn reads as small.
  Quat turn = conjugate(frameA) * frameB;
  if (turn.w < 0.0f) turn = Quat{-turn.x, -turn.y, -turn.z, -turn.w};
  pose.twist = 2.0f * std::atan2(turn.x, turn.w);

  // With x tipped all the way over there is no twist to take out, and what
  // is left is all swing.
  const float size = std::sqrt(turn.x * turn.x + turn.w * turn.w);
  const Quat twist = size > kTiny ? Quat{turn.x / size, 0.0f, 0.0f, turn.w / size}
                                  : Quat{};
  const Quat swing = turn * conjugate(twist);
  const float sine = std::sqrt(swing.y * swing.y + swing.z * swing.z);
  pose.swung = 2.0f * std::atan2(sine, swing.w);
  pose.swing = sine > kTiny
                   ? Vec3{0.0f, swing.y, swing.z} * (pose.swung / sine)
                   : Vec3{0.0f, 2.0f * swing.y, 2.0f * swing.z};
  return pose;
}

/// Row `which` of `joint` at `pose`: the direction it holds along, and where
/// it stands on it. False if it has no direction this instant, which is a
/// cone not swinging at all.
bool rowAt(const Joint &joint, const Pose &pose, int which, Vec3 &linear,
           Vec3 &turnA, Vec3 &turnB, float &at) {
  if (which == kMotorRow) which = joint.motor;

  if (which == 0 && joint.distance) {
    const Vec3 between = pose.pointB - pose.pointA;
    at = length(between);
    // Two points on top of each other have no line between them. Any
    // direction will do to push them apart along, and the frame's own x is
    // at least the same one every step.
    linear = at > kTouching ? between * (1.0f / at) : pose.axis[0];
    turnA = cross(pose.pointA - pose.centreA, linear);
    turnB = cross(pose.pointB - pose.centreB, linear);
    return true;
  }

  if (which < 3) {
    // Along an axis of `a`'s frame, which turns with `a`. Levering `a` from
    // `b`'s point rather than its own is what accounts for that: a slider's
    // rail swinging round drags what is on it along, however far out it is.
    linear = pose.axis[which];
    at = dot(pose.pointB - pose.pointA, linear);
    turnA = cross(pose.pointB - pose.centreA, linear);
    turnB = cross(pose.pointB - pose.centreB, linear);
    return true;
  }

  linear = Vec3{};
  if (which == 3) {
    turnA = turnB = pose.twistAxis;
    at = pose.twist;
    return true;
  }
  if (joint.cone) {
    const Vec3 way = normalised(pose.axis[1] * pose.swing.y + pose.axis[2] * pose.swing.z);
    if (lengthSquared(way) == 0.0f) return false;
    turnA = turnB = way;
    at = pose.swung;
    return true;
  }
  turnA = turnB = pose.axis[which - 3];
  at = which == 4 ? pose.swing.y : pose.swing.z;
  return true;
}

/// How hard it is to change the speed along a row, inverted. Zero when
/// neither body can move, which makes every impulse along it zero.
float massAlong(float inverseMassA, float inverseMassB, const Mat3 &inertiaA,
                const Mat3 &inertiaB, const Vec3 &linear, const Vec3 &turnA,
                const Vec3 &turnB) {
  const float k = (inverseMassA + inverseMassB) * lengthSquared(linear) +
                  dot(turnA, inertiaA * turnA) + dot(turnB, inertiaB * turnB);
  return k > kTiny ? 1.0f / k : 0.0f;
}

/// A turn kept within a half turn either way, which is all an angle measured
/// from a rotation can say.
float withinHalfTurn(float radians) { return clamped(radians, -kPi, kPi); }

} // namespace

// ------------------------------------------------------------- keeping ---

bool Joints::add(const JointDescription &from, const Bodies &bodies) {
  if (from.id == 0 || index_.find(from.id) != index_.end()) return false;
  // Both zero is the world joined to itself, which is the same refusal.
  if (from.a == from.b) return false;
  if (from.kind < ORBLIT_PHYSICS_JOINT_FIXED ||
      from.kind > ORBLIT_PHYSICS_JOINT_SIX_AXIS) {
    return false;
  }
  const uint32_t rowA = from.a == 0 ? Bodies::kNone : bodies.rowOf(from.a);
  if (from.a != 0 && rowA == Bodies::kNone) return false;
  const uint32_t rowB = from.b == 0 ? Bodies::kNone : bodies.rowOf(from.b);
  if (from.b != 0 && rowB == Bodies::kNone) return false;

  Joint joint;
  joint.id = from.id;
  joint.a = from.a;
  joint.b = from.b;
  joint.kind = from.kind;
  joint.collide = from.collide;
  joint.breakingForce = std::fmax(from.breakingForce, 0.0f);
  joint.breakingTorque = std::fmax(from.breakingTorque, 0.0f);

  // Kept on each body as each body sees it, so that from here on the joint
  // goes where the bodies go. The world sees it as it is.
  const auto keep = [&bodies, &from](uint32_t row, const Vec3 &point,
                                     Vec3 &anchor, Quat &frame) {
    if (row == Bodies::kNone) {
      anchor = point;
      frame = normalised(from.rotation);
      return;
    }
    const Quat body = bodies.rotation(row);
    anchor = unrotate(body, point - bodies.at(row));
    frame = normalised(conjugate(body) * from.rotation);
  };
  keep(rowA, from.at, joint.anchorA, joint.frameA);
  keep(rowB, from.kind == ORBLIT_PHYSICS_JOINT_DISTANCE ? from.to : from.at,
       joint.anchorB, joint.frameB);

  const auto lock = [&joint](int which) {
    joint.holds |= 1u << which;
    joint.least[which] = 0.0f;
    joint.most[which] = 0.0f;
  };
  const auto limit = [&joint, &from](int which) {
    if ((from.limited & (1u << which)) == 0) return;
    joint.holds |= 1u << which;
    joint.least[which] = std::fmin(from.low[which], from.high[which]);
    joint.most[which] = std::fmax(from.low[which], from.high[which]);
    if (which >= 3) {
      joint.least[which] = withinHalfTurn(joint.least[which]);
      joint.most[which] = withinHalfTurn(joint.most[which]);
    }
  };
  const auto drive = [&joint, &from](int which) {
    if (!(from.strength > 0.0f)) return;
    joint.motor = which;
    joint.speed = from.speed;
    joint.strength = from.strength;
  };

  switch (from.kind) {
    case ORBLIT_PHYSICS_JOINT_FIXED:
      for (int which = 0; which < 6; ++which) lock(which);
      break;
    case ORBLIT_PHYSICS_JOINT_POINT:
      for (int which = 0; which < 3; ++which) lock(which);
      break;
    case ORBLIT_PHYSICS_JOINT_HINGE:
      for (int which = 0; which < 3; ++which) lock(which);
      lock(4);
      lock(5);
      limit(3);
      drive(3);
      break;
    case ORBLIT_PHYSICS_JOINT_SLIDER:
      for (int which = 1; which < 6; ++which) lock(which);
      limit(0);
      drive(0);
      break;
    case ORBLIT_PHYSICS_JOINT_DISTANCE:
      joint.distance = true;
      if ((from.limited & 1u) != 0) {
        limit(0);
        joint.least[0] = std::fmax(joint.least[0], 0.0f);
        joint.most[0] = std::fmax(joint.most[0], 0.0f);
      } else {
        // A rod, as long as it was made.
        joint.holds |= 1u;
        joint.least[0] = joint.most[0] = length(from.to - from.at);
      }
      break;
    case ORBLIT_PHYSICS_JOINT_CONE:
      for (int which = 0; which < 3; ++which) lock(which);
      joint.cone = true;
      joint.holds |= 1u << 4;
      joint.least[4] = -kFar;
      joint.most[4] = clamped(from.swing, 0.0f, kPi);
      limit(3);
      break;
    default: // ORBLIT_PHYSICS_JOINT_SIX_AXIS
      for (int which = 0; which < 6; ++which) limit(which);
      break;
  }

  index_.emplace(joint.id, static_cast<uint32_t>(joints_.size()));
  if (joint.a != 0) ++counts_[joint.a];
  if (joint.b != 0) ++counts_[joint.b];
  if (joint.a != 0 && joint.b != 0 && !joint.collide) {
    ++apart_[PairKey::of(joint.a, joint.b)];
  }
  joints_.push_back(joint);
  return true;
}

bool Joints::remove(OrblitPhysicsId id) {
  const auto found = index_.find(id);
  if (found == index_.end()) return false;
  const uint32_t at = found->second;
  const Joint &joint = joints_[at];

  const auto release = [this](OrblitPhysicsId body) {
    const auto count = counts_.find(body);
    if (count != counts_.end() && --count->second == 0) counts_.erase(count);
  };
  if (joint.a != 0) release(joint.a);
  if (joint.b != 0) release(joint.b);
  if (joint.a != 0 && joint.b != 0 && !joint.collide) {
    const auto pair = apart_.find(PairKey::of(joint.a, joint.b));
    if (pair != apart_.end() && --pair->second == 0) apart_.erase(pair);
  }

  // Erased in place rather than swapped out, so what is left is still solved
  // in the order it was made.
  index_.erase(found);
  joints_.erase(joints_.begin() + at);
  for (uint32_t later = at; later < joints_.size(); ++later) {
    index_[joints_[later].id] = later;
  }
  return true;
}

const Joint *Joints::find(OrblitPhysicsId id) const {
  const auto found = index_.find(id);
  return found == index_.end() ? nullptr : &joints_[found->second];
}

void Joints::drop(OrblitPhysicsId body, std::vector<OrblitPhysicsId> &partners) {
  if (!on(body)) return;
  std::vector<OrblitPhysicsId> going;
  for (const Joint &joint : joints_) {
    if (joint.a != body && joint.b != body) continue;
    going.push_back(joint.id);
    const OrblitPhysicsId other = joint.a == body ? joint.b : joint.a;
    if (other != 0) partners.push_back(other);
  }
  for (const OrblitPhysicsId id : going) remove(id);
}

bool Joints::state(const Bodies &bodies, OrblitPhysicsId id,
                   OrblitPhysicsJointState &out) const {
  const Joint *joint = find(id);
  if (joint == nullptr) return false;
  const uint32_t rowA = joint->a == 0 ? Bodies::kNone : bodies.rowOf(joint->a);
  const uint32_t rowB = joint->b == 0 ? Bodies::kNone : bodies.rowOf(joint->b);
  const Pose pose = poseOf(bodies, rowA, rowB, *joint);

  out = OrblitPhysicsJointState{};
  const Vec3 between = pose.pointB - pose.pointA;
  if (joint->distance) {
    out.offset[0] = length(between);
  } else {
    for (int i = 0; i < 3; ++i) out.offset[i] = dot(between, pose.axis[i]);
  }
  out.angles[0] = pose.twist;
  out.angles[1] = pose.swing.y;
  out.angles[2] = pose.swing.z;
  out.force = joint->force;
  out.torque = joint->torque;
  return true;
}

// ------------------------------------------------------------- solving ---

Joints::Held Joints::heldFor(const Bodies &bodies, uint32_t index) const {
  const Joint &joint = joints_[index];
  Held held;
  held.joint = index;
  held.a = joint.a == 0 ? Bodies::kNone : bodies.rowOf(joint.a);
  held.b = joint.b == 0 ? Bodies::kNone : bodies.rowOf(joint.b);
  // As for a contact: a body the solver will not move is a body of infinite
  // mass, whether it is static, driven, asleep, or the world.
  if (held.a != Bodies::kNone && bodies.solved(held.a)) {
    held.inverseMassA = bodies.inverseMass(held.a);
    held.inertiaA = bodies.inverseInertia(held.a);
  } else {
    held.inertiaA = Mat3::zero();
  }
  if (held.b != Bodies::kNone && bodies.solved(held.b)) {
    held.inverseMassB = bodies.inverseMass(held.b);
    held.inertiaB = bodies.inverseInertia(held.b);
  } else {
    held.inertiaB = Mat3::zero();
  }
  return held;
}

void Joints::prepare(Bodies &bodies, float delta) {
  held_.clear();
  rows_.clear();

  for (uint32_t index = 0; index < joints_.size(); ++index) {
    Joint &joint = joints_[index];
    const Held held = heldFor(bodies, index);
    // Nothing to move. Its impulses are kept, so a joint that went to sleep
    // holding something up starts holding it up again the moment it wakes.
    if (held.inverseMassA == 0.0f && held.inverseMassB == 0.0f) continue;

    const uint32_t heldAt = static_cast<uint32_t>(held_.size());
    held_.push_back(held);
    const Pose pose = poseOf(bodies, held.a, held.b, joint);

    uint32_t built = 0;
    const auto build = [&](int which) {
      Row row;
      row.held = heldAt;
      row.which = which;
      float at = 0.0f;
      if (!rowAt(joint, pose, which, row.linear, row.turnA, row.turnB, at)) return;
      row.mass = massAlong(held.inverseMassA, held.inverseMassB, held.inertiaA,
                           held.inertiaB, row.linear, row.turnA, row.turnB);

      if (which == kMotorRow) {
        row.target = joint.speed;
        row.least = -joint.strength * delta;
        row.most = joint.strength * delta;
      } else if (joint.least[which] == joint.most[which]) {
        // Locked. Only the velocity is held here; where it has drifted to is
        // put back by the position passes, which is what keeps a held joint
        // from adding energy to what it holds.
        row.target = 0.0f;
        row.least = -kFar;
        row.most = kFar;
      } else {
        // A limit, and only its nearer end. It lets the joint close on that
        // end as fast as it likes so long as it will not pass it this step,
        // so it catches a fast swing on the step it arrives rather than the
        // step after, and never pulls it back from anywhere short of it.
        const float low = joint.least[which];
        const float high = joint.most[which];
        if (at - low < high - at) {
          row.target = at > low ? (low - at) / delta : 0.0f;
          row.least = 0.0f;
          row.most = kFar;
        } else {
          row.target = at < high ? (high - at) / delta : 0.0f;
          row.least = -kFar;
          row.most = 0.0f;
        }
      }

      // Last step's impulse, if it still points the way this row may push.
      // A limit that has swapped ends starts from nothing.
      joint.impulse[which] = clamped(joint.impulse[which], row.least, row.most);
      built |= 1u << which;
      rows_.push_back(row);
    };

    // The motor first, so a limit on the same axis has the last word.
    if (joint.motor >= 0) build(kMotorRow);
    for (int which = 0; which < 6; ++which) {
      if ((joint.holds & (1u << which)) != 0) build(which);
    }
    for (int which = 0; which < kJointRows; ++which) {
      if ((built & (1u << which)) == 0) joint.impulse[which] = 0.0f;
    }
  }
}

void Joints::warmStart(Bodies &bodies) {
  for (const Row &row : rows_) {
    const Held &held = held_[row.held];
    apply(bodies, held, row, joints_[held.joint].impulse[row.which]);
  }
}

void Joints::correctVelocities(Bodies &bodies) {
  for (const Row &row : rows_) {
    const Held &held = held_[row.held];
    float &impulse = joints_[held.joint].impulse[row.which];
    const float was = impulse;
    impulse = clamped(
        was - row.mass * (speedAlong(bodies, held, row) - row.target),
        row.least, row.most);
    apply(bodies, held, row, impulse - was);
  }
}

void Joints::correctPositions(Bodies &bodies) {
  for (const Held &held : held_) {
    const Joint &joint = joints_[held.joint];
    for (int which = 0; which < 6; ++which) {
      if ((joint.holds & (1u << which)) == 0) continue;

      // Measured again for every row, because the row before has just moved
      // the bodies, and a correction worked out from where they were is a
      // correction that overshoots.
      const Pose pose = poseOf(bodies, held.a, held.b, joint);
      Vec3 linear;
      Vec3 turnA;
      Vec3 turnB;
      float at = 0.0f;
      if (!rowAt(joint, pose, which, linear, turnA, turnB, at)) continue;

      float error = 0.0f;
      if (at < joint.least[which]) {
        error = at - joint.least[which];
      } else if (at > joint.most[which]) {
        error = at - joint.most[which];
      } else {
        continue;
      }

      const float most = which < 3 ? kMaxPull : kMaxTurn;
      const float correction = clamped(error * kPull, -most, most);
      const float pull =
          -correction * massAlong(held.inverseMassA, held.inverseMassB,
                                  held.inertiaA, held.inertiaB, linear, turnA,
                                  turnB);

      if (held.a != Bodies::kNone) {
        bodies.at(held.a) -= linear * (pull * held.inverseMassA);
        bodies.rotation(held.a) = integrate(
            bodies.rotation(held.a), -(held.inertiaA * turnA) * pull, 1.0f);
      }
      if (held.b == Bodies::kNone) continue;
      bodies.at(held.b) += linear * (pull * held.inverseMassB);
      bodies.rotation(held.b) = integrate(bodies.rotation(held.b),
                                          (held.inertiaB * turnB) * pull, 1.0f);
    }
  }
}

void Joints::strain(const Bodies &bodies, float delta,
                    std::vector<Broken> &broken) {
  if (!(delta > 0.0f)) return;
  for (const Held &held : held_) {
    Joint &joint = joints_[held.joint];

    // The rows that move are at right angles to one another, and so are the
    // rows that turn, so each set adds up as a vector. The motor pushes along
    // the row it drives.
    float along[6];
    for (int which = 0; which < 6; ++which) along[which] = joint.impulse[which];
    if (joint.motor >= 0) along[joint.motor] += joint.impulse[kMotorRow];
    joint.force = std::sqrt(along[0] * along[0] + along[1] * along[1] +
                            along[2] * along[2]) / delta;
    joint.torque = std::sqrt(along[3] * along[3] + along[4] * along[4] +
                             along[5] * along[5]) / delta;

    const bool pulled = joint.breakingForce > 0.0f && joint.force > joint.breakingForce;
    const bool twisted =
        joint.breakingTorque > 0.0f && joint.torque > joint.breakingTorque;
    if (!pulled && !twisted) continue;

    Broken gave;
    gave.id = joint.id;
    gave.at = poseOf(bodies, held.a, held.b, joint).pointA;
    gave.force = pulled ? joint.force : joint.torque;
    broken.push_back(gave);
  }
}

float Joints::speedAlong(const Bodies &bodies, const Held &held,
                         const Row &row) {
  float speed = 0.0f;
  if (held.a != Bodies::kNone) {
    speed -= dot(row.linear, bodies.velocity(held.a)) +
             dot(row.turnA, bodies.spin(held.a));
  }
  if (held.b != Bodies::kNone) {
    speed += dot(row.linear, bodies.velocity(held.b)) +
             dot(row.turnB, bodies.spin(held.b));
  }
  return speed;
}

void Joints::apply(Bodies &bodies, const Held &held, const Row &row,
                   float impulse) {
  if (held.a != Bodies::kNone) {
    bodies.velocity(held.a) -= row.linear * (impulse * held.inverseMassA);
    bodies.spin(held.a) -= (held.inertiaA * row.turnA) * impulse;
  }
  if (held.b == Bodies::kNone) return;
  bodies.velocity(held.b) += row.linear * (impulse * held.inverseMassB);
  bodies.spin(held.b) += (held.inertiaB * row.turnB) * impulse;
}

} // namespace orblit
