#include "joint.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace orblit {
namespace {

constexpr float kFar = std::numeric_limits<float>::infinity();

/// What fraction of a joint's error one position pass puts back.
///
/// More than a contact's fifth. A contact has a soft spring and the rest of a
/// stack to share its push with; a joint has neither, and one that puts back
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

/// The stiffest a spring on a joint is let be, as a share of the sub-step
/// rate. Past a quarter of it a spring is as stiff as the step can follow, and
/// a stiffer one is only a hard row with a bigger number written on it.
constexpr float kMostOfRate = 0.25f;

const Vec3 kAlongX{1.0f, 0.0f, 0.0f};
const Vec3 kAlongY{0.0f, 1.0f, 0.0f};
const Vec3 kAlongZ{0.0f, 0.0f, 1.0f};

/// Where a joint stands this instant, from where its bodies are.
struct Pose {
  /// Where each body is placed from, which its anchors are measured from.
  Vec3 originA;
  Vec3 originB;

  /// Where each body turns about, which the levers are measured from. The same
  /// point as the origin unless a free body's weight is moved.
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

  /// `b`'s own x axis, in the world: what a gear or a pinion turns about.
  Vec3 axisB;

  /// On a path, how far along it `pointA` is.
  float arc = 0.0f;

  /// The swing, as a turn about an axis in `a`'s frame, and how far it is.
  /// It never has an x part: turning about x is the twist.
  Vec3 swing;
  float swung = 0.0f;
};

/// The nearest place on a path to a point, which way the path runs there, and
/// how far along it that is.
struct Nearest {
  Vec3 point;
  Vec3 tangent;
  float arc = 0.0f;
};

Nearest nearestOnPath(const Track &track, const Vec3 &to) {
  Nearest best;
  float least = kFar;
  for (size_t i = 0; i + 1 < track.points.size(); ++i) {
    const Vec3 &from = track.points[i];
    const Vec3 run = track.points[i + 1] - from;
    const float size = length(run);
    // Points that touch were dropped when the path was laid, so a run has a
    // direction and a length.
    const float raw = dot(to - from, run) / (size * size);
    const float along = clamped(raw, 0.0f, 1.0f);
    const Vec3 at = from + run * along;
    const float gap = lengthSquared(to - at);
    if (gap >= least) continue;
    least = gap;
    best.point = at;
    best.tangent = run * (1.0f / size);

    // The line goes on past each end, and how far along it keeps counting
    // there. Held at the end instead, a limit at the end of a rail would
    // never be passed, so it would never stop what runs off.
    const bool first = i == 0;
    const bool last = i + 2 == track.points.size();
    const bool past = (first && raw < 0.0f) || (last && raw > 1.0f);
    best.arc = track.arc[i] + (past ? raw : along) * size;
  }
  return best;
}

/// Puts `pose`'s point on `a` where the path is nearest `b`'s, and returns a
/// frame there: x along the path, y as near `frameA`'s y as it can be.
Quat layOnPath(Pose &pose, const Quat &bodyA, const Quat &frameA,
               const Track &track) {
  const Nearest near =
      nearestOnPath(track, unrotate(bodyA, pose.pointB - pose.originA));
  pose.pointA = pose.originA + rotate(bodyA, near.point);
  pose.arc = near.arc;

  const Vec3 along = rotate(bodyA, near.tangent);
  const auto across = [&along](const Vec3 &v) { return v - along * dot(v, along); };
  Vec3 up = across(rotate(frameA, kAlongY));
  // A path running straight along `a`'s y has no y across it. Its z has.
  if (lengthSquared(up) < 0.25f) up = across(rotate(frameA, kAlongZ));
  up = normalised(up);
  return fromAxes(along, up, cross(along, up));
}

/// `b`'s frame seen from `a`'s, split into a twist about x and then a swing
/// that tips x over. The shorter way round, so a small turn reads as small.
void measureTurn(Pose &pose, const Quat &frameA, const Quat &frameB) {
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
}

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
  pose.originA = worldA ? Vec3{} : bodies.at(rowA);
  pose.originB = worldB ? Vec3{} : bodies.at(rowB);
  pose.centreA = worldA ? Vec3{} : bodies.centre(rowA);
  pose.centreB = worldB ? Vec3{} : bodies.centre(rowB);
  pose.pointA = pose.originA + rotate(bodyA, joint.anchorA);
  pose.pointB = pose.originB + rotate(bodyB, joint.anchorB);

  Quat frameA = bodyA * joint.frameA;
  const Quat frameB = bodyB * joint.frameB;
  if (joint.onPath()) frameA = layOnPath(pose, bodyA, frameA, joint.track);

  pose.axis[0] = rotate(frameA, kAlongX);
  pose.axis[1] = rotate(frameA, kAlongY);
  pose.axis[2] = rotate(frameA, kAlongZ);
  pose.axisB = rotate(frameB, kAlongX);
  const Vec3 between = normalised(pose.axis[0] + pose.axisB);
  pose.twistAxis = lengthSquared(between) > 0.0f ? between : pose.axis[0];
  measureTurn(pose, frameA, frameB);
  return pose;
}

/// How far `after` has turned from `before`, as a vector along the axis of the
/// turn, the shorter way round.
Vec3 turnBetween(const Quat &before, const Quat &after) {
  Quat turn = after * conjugate(before);
  if (turn.w < 0.0f) turn = Quat{-turn.x, -turn.y, -turn.z, -turn.w};
  const Vec3 part{turn.x, turn.y, turn.z};
  const float sine = length(part);
  if (sine < kTiny) return part * 2.0f;
  return part * (2.0f * std::atan2(sine, turn.w) / sine);
}

/// Adds to `tie` how far each body has turned about its axis since the last
/// time this looked. Called often enough that no look sees a turn of half a
/// revolution or more, which is every sub-step.
void sumTurns(const Bodies &bodies, uint32_t rowA, uint32_t rowB,
              const Joint &joint, Tie &tie) {
  if (tie.kind != Tied::Gear && tie.kind != Tied::Rack) return;
  const uint32_t rows[2] = {rowA, rowB};
  const Quat frames[2] = {joint.frameA, joint.frameB};
  for (int side = 0; side < 2; ++side) {
    if (rows[side] == Bodies::kNone) continue;
    const Quat now = bodies.rotation(rows[side]);
    const Vec3 axis = rotate(now * frames[side], kAlongX);
    tie.turned[side] += dot(turnBetween(tie.last[side], now), axis);
    tie.last[side] = now;
  }
}

/// The coordinate a coupling keeps. It is what the joint's row is the
/// derivative of, so it is zero-speed along the row when it holds.
float tiedCoordinate(const Tie &tie, const Pose &pose) {
  switch (tie.kind) {
    case Tied::Gear:
      return tie.turned[1] - tie.ratio * tie.turned[0];
    case Tied::Rack:
      return tie.ratio * tie.turned[1] - dot(pose.pointA, tie.axis);
    default: // Tied::Pulley
      return length(pose.pointA - tie.over[0]) +
             tie.ratio * length(pose.pointB - tie.over[1]);
  }
}

/// Which way a row pushes each body, and where the way stands on it.
struct Direction {
  Vec3 linearA;
  Vec3 linearB;
  Vec3 turnA;
  Vec3 turnB;
  float at = 0.0f;
};

/// The one row of a coupling. Pushing along it moves each body along what the
/// coordinate is made of, a turn or a slide or a length of rope, so each body
/// has a row of its own.
Direction tiedAt(const Joint &joint, const Pose &pose) {
  const Tie &tie = joint.tie;
  Direction row;
  row.at = tiedCoordinate(tie, pose) - tie.constant;
  switch (tie.kind) {
    case Tied::Gear:
      row.turnA = pose.axis[0] * tie.ratio;
      row.turnB = pose.axisB;
      break;
    case Tied::Rack:
      row.linearA = tie.axis;
      row.turnA = cross(pose.pointA - pose.centreA, tie.axis);
      row.turnB = pose.axisB * tie.ratio;
      break;
    default: // Tied::Pulley
      row.linearA = normalised(pose.pointA - tie.over[0]) * -1.0f;
      row.linearB = normalised(pose.pointB - tie.over[1]) * tie.ratio;
      row.turnA = cross(pose.pointA - pose.centreA, row.linearA);
      row.turnB = cross(pose.pointB - pose.centreB, row.linearB);
      break;
  }
  return row;
}

/// Row `which` of `joint` at `pose`: the direction it holds along, and where
/// it stands on it. False if it has no direction this instant, which is a
/// cone not swinging at all.
bool rowAt(const Joint &joint, const Pose &pose, int which, Direction &out) {
  if (which == kMotorRow) which = joint.motor;
  if (which >= kSpringRow) which -= kSpringRow;

  if (joint.tie.kind != Tied::None) {
    out = tiedAt(joint, pose);
    return true;
  }

  out = Direction{};
  if (which == 0 && joint.distance) {
    const Vec3 between = pose.pointB - pose.pointA;
    out.at = length(between);
    // Two points on top of each other have no line between them. Any
    // direction will do to push them apart along, and the frame's own x is
    // at least the same one every step.
    const Vec3 linear = out.at > kTouching ? between * (1.0f / out.at) : pose.axis[0];
    out.linearA = out.linearB = linear;
    out.turnA = cross(pose.pointA - pose.centreA, linear);
    out.turnB = cross(pose.pointB - pose.centreB, linear);
    return true;
  }

  if (which < 3) {
    // Along an axis of `a`'s frame, which turns with `a`. Levering `a` from
    // `b`'s point rather than its own is what accounts for that: a slider's
    // rail swinging round drags what is on it along, however far out it is.
    const Vec3 linear = pose.axis[which];
    out.linearA = out.linearB = linear;
    out.at = dot(pose.pointB - pose.pointA, linear);
    // On a path the first way is how far along it, not how far off the
    // nearest point, which is nought by the way that point was chosen.
    if (which == 0 && joint.onPath()) out.at = pose.arc;
    out.turnA = cross(pose.pointB - pose.centreA, linear);
    out.turnB = cross(pose.pointB - pose.centreB, linear);
    return true;
  }

  if (which == 3) {
    out.turnA = out.turnB = pose.twistAxis;
    out.at = pose.twist;
    return true;
  }
  if (joint.cone) {
    const Vec3 way = normalised(pose.axis[1] * pose.swing.y + pose.axis[2] * pose.swing.z);
    if (lengthSquared(way) == 0.0f) return false;
    out.turnA = out.turnB = way;
    out.at = pose.swung;
    return true;
  }
  out.turnA = out.turnB = pose.axis[which - 3];
  out.at = which == 4 ? pose.swing.y : pose.swing.z;
  return true;
}

/// A turn kept within a half turn either way, which is all an angle measured
/// from a rotation can say.
float withinHalfTurn(float radians) { return clamped(radians, -kPi, kPi); }

/// How far `at` is from `place` on way `which`, the short way round for an
/// angle: a hinge at 3 radians is a quarter turn from one at -3, not six.
float gapTo(int which, float at, float place) {
  const float gap = at - place;
  if (which < 3) return gap;
  return gap - 2.0f * kPi * std::round(gap / (2.0f * kPi));
}

/// Where a way stands for one sub-step of length `h`, and how much stiffer than
/// its own row a spring on it must be to be the spring its owner asked for.
struct Standing {
  float at = 0.0f;
  float h = 0.0f;
  float reach = 1.0f;
};

/// A spring of `hertz` and damping `ratio`, as a row of the stiffness a step
/// can follow. `reach` makes a row that turns a body about a point away from
/// its centre as stiff as that body is to turn about the point, and not about
/// itself, which is what a spring on a hinge has to move.
Softness softAt(float hertz, float ratio, const Standing &standing) {
  const float limit = kMostOfRate / standing.h;
  return softnessOf(standing.h, std::fmin(hertz * standing.reach, limit),
                    ratio * standing.reach);
}

/// What a row aims for this sub-step: the speed along it, how much impulse it
/// may carry either way, and how soft it is.
struct Aim {
  float target = 0.0f;
  float least = -kFar;
  float most = kFar;
  Softness soft{1.0f, 0.0f, 0.0f};
};

/// A spring on way `which`: the speed that closes its gap to where it rests,
/// as fast as its frequency and damping ratio say.
Aim springAim(const Joint &joint, int which, const Standing &standing) {
  Aim aim;
  aim.soft = softAt(joint.springHertz[which], joint.springRatio[which], standing);
  aim.target = -aim.soft.pushRate *
               gapTo(which, standing.at, joint.springRest[which]);
  return aim;
}

/// The motor: its speed, or with a frequency the speed that closes its gap to
/// the goal as a spring would, and no harder than its strength either way.
Aim motorAim(const Joint &joint, const Standing &standing) {
  Aim aim;
  aim.target = joint.speed;
  aim.least = -joint.strength * standing.h;
  aim.most = joint.strength * standing.h;
  if (!(joint.goalHertz > 0.0f)) return aim;

  aim.soft = softAt(joint.goalHertz, joint.goalRatio, standing);
  aim.target = -aim.soft.pushRate *
               gapTo(joint.motor, standing.at, joint.goal);
  return aim;
}

/// A limit that gives: nothing while inside the range, and past an end a spring
/// about that end. It pushes and pulls, so it can hold what it has stopped and
/// leave it at the end, not rebound it off.
Aim softLimitAim(const Joint &joint, float end, float room,
                 const Standing &standing) {
  Aim aim;
  if (room > 0.0f) {
    aim.least = 0.0f;
    aim.most = 0.0f;
    return aim;
  }
  aim.soft = softAt(joint.limitHertz, joint.limitRatio, standing);
  aim.target = aim.soft.pushRate * (end - standing.at);
  return aim;
}

/// The nearer end of a range, and only that end. A hard limit lets the joint
/// close on the end as fast as it likes so long as it will not pass it this
/// sub-step, so it catches a fast swing on the sub-step it arrives rather than
/// the one after, and never pulls it back from anywhere short of it. Past the
/// end it holds still and the position passes put the overshoot back.
Aim limitAim(const Joint &joint, int which, const Standing &standing) {
  const float low = joint.least[which];
  const float high = joint.most[which];
  const bool lower = standing.at - low < high - standing.at;
  const float end = lower ? low : high;
  const float room = lower ? standing.at - low : high - standing.at;
  if (joint.limitHertz > 0.0f) return softLimitAim(joint, end, room, standing);

  Aim aim;
  if (lower) {
    aim.least = 0.0f;
  } else {
    aim.most = 0.0f;
  }
  if (room > 0.0f) aim.target = (end - standing.at) / standing.h;
  return aim;
}

/// Row `slot` of `joint`.
Aim aimFor(const Joint &joint, int slot, const Standing &standing) {
  if (slot == kMotorRow) return motorAim(joint, standing);
  if (slot >= kSpringRow) return springAim(joint, slot - kSpringRow, standing);
  // Locked. Only the velocity is held here; where it has drifted to is put
  // back by the position passes, which is what keeps a held joint from adding
  // energy to what it holds.
  if (joint.least[slot] == joint.most[slot]) return Aim{};
  return limitAim(joint, slot, standing);
}

/// One over the inertia of a body about `axis` through its joint point, `lever`
/// from its centre: its own spin about the axis plus its mass swung on the
/// lever. Zero for a body that cannot move that way.
float inverseInertiaAbout(const Mat3 &inverse, const Vec3 &gain,
                          const Vec3 &lever, const Vec3 &axis) {
  const float spin = dot(axis, inverse * axis);
  if (!(spin > kTiny)) return 0.0f;

  const Vec3 arm = cross(axis, lever);
  const float swung[3] = {arm.x, arm.y, arm.z};
  const float give[3] = {gain.x, gain.y, gain.z};
  float inertia = 1.0f / spin;
  for (int i = 0; i < 3; ++i) {
    if (give[i] > kTiny) {
      inertia += swung[i] * swung[i] / give[i];
    } else if (std::fabs(swung[i]) > kTiny) {
      return 0.0f;
    }
  }
  return 1.0f / inertia;
}

/// Whether the position passes leave way `which` to its own soft limit.
bool softLimited(const Joint &joint, int which) {
  return joint.limitHertz > 0.0f && joint.least[which] != joint.most[which];
}

/// Gives `joint` the springs and the soft limits `from` asks for, once its
/// kind has said which ways it holds. A spring takes the place of a lock on
/// its way, which is what makes a lock soft, and sits beside a limit.
void keepSprings(const JointDescription &from, Joint &joint) {
  joint.limitHertz = std::fmax(from.limitHertz, 0.0f);
  joint.limitRatio = std::fmax(from.limitRatio, 0.0f);

  for (int which = 0; which < 6; ++which) {
    if (!(from.springHertz[which] > 0.0f)) continue;
    // A cone swings as one, and has no second way to hold apart from it. A
    // coupling has the one.
    if (joint.cone && which == 5) continue;
    if (joint.tie.kind != Tied::None && which != 0) continue;
    joint.sprung |= 1u << which;
    joint.springHertz[which] = from.springHertz[which];
    joint.springRatio[which] = std::fmax(from.springRatio[which], 0.0f);
    joint.springRest[which] = from.springRest[which];
    if (joint.least[which] == joint.most[which]) joint.holds &= ~(1u << which);
  }
}

/// Where on a body a joint's point and frame are, as the body sees them, so
/// that from here on the joint goes where the body goes. The world sees them as
/// they are.
void anchorOn(const Bodies &bodies, uint32_t row, const Vec3 &point,
              const Quat &rotation, Vec3 &anchor, Quat &frame) {
  if (row == Bodies::kNone) {
    anchor = point;
    frame = normalised(rotation);
    return;
  }
  const Quat body = bodies.rotation(row);
  anchor = unrotate(body, point - bodies.at(row));
  frame = normalised(conjugate(body) * rotation);
}

/// A joint being made, and the ways its kind holds one at a time.
struct Making {
  const JointDescription &from;
  Joint &joint;

  void lock(int which) {
    joint.holds |= 1u << which;
    joint.least[which] = 0.0f;
    joint.most[which] = 0.0f;
  }

  void limit(int which) {
    if ((from.limited & (1u << which)) == 0) return;
    joint.holds |= 1u << which;
    joint.least[which] = std::fmin(from.low[which], from.high[which]);
    joint.most[which] = std::fmax(from.low[which], from.high[which]);
    if (which >= 3) {
      joint.least[which] = withinHalfTurn(joint.least[which]);
      joint.most[which] = withinHalfTurn(joint.most[which]);
    }
  }

  void drive(int which) {
    if (!(from.strength > 0.0f)) return;
    joint.motor = which;
    joint.speed = from.speed;
    joint.strength = from.strength;
    joint.goal = from.goal;
    joint.goalHertz = std::fmax(from.goalHertz, 0.0f);
    joint.goalRatio = std::fmax(from.goalRatio, 0.0f);
  }
};

/// Which ways each of the kinds that hold bodies against each other holds.
void holdWays(Making &making) {
  const JointDescription &from = making.from;
  Joint &joint = making.joint;
  switch (from.kind) {
    case ORBLIT_PHYSICS_JOINT_FIXED:
      for (int which = 0; which < 6; ++which) making.lock(which);
      break;
    case ORBLIT_PHYSICS_JOINT_POINT:
      for (int which = 0; which < 3; ++which) making.lock(which);
      break;
    case ORBLIT_PHYSICS_JOINT_HINGE:
      for (int which = 0; which < 3; ++which) making.lock(which);
      making.lock(4);
      making.lock(5);
      making.limit(3);
      making.drive(3);
      break;
    case ORBLIT_PHYSICS_JOINT_SLIDER:
      for (int which = 1; which < 6; ++which) making.lock(which);
      making.limit(0);
      making.drive(0);
      break;
    case ORBLIT_PHYSICS_JOINT_DISTANCE:
      joint.distance = true;
      if ((from.limited & 1u) != 0) {
        making.limit(0);
        joint.least[0] = std::fmax(joint.least[0], 0.0f);
        joint.most[0] = std::fmax(joint.most[0], 0.0f);
      } else {
        // A rod, as long as it was made.
        joint.holds |= 1u;
        joint.least[0] = joint.most[0] = length(from.to - from.at);
      }
      break;
    case ORBLIT_PHYSICS_JOINT_CONE:
      for (int which = 0; which < 3; ++which) making.lock(which);
      joint.cone = true;
      joint.holds |= 1u << 4;
      joint.least[4] = -kFar;
      joint.most[4] = clamped(from.swing, 0.0f, kPi);
      making.limit(3);
      break;
    case ORBLIT_PHYSICS_JOINT_PATH:
      // The point stays on the path and slides along it, within a range of
      // how far along, and with a motor. Following also keeps x along it.
      making.lock(1);
      making.lock(2);
      if (from.follow) {
        making.lock(4);
        making.lock(5);
      }
      making.limit(0);
      making.drive(0);
      break;
    default: // ORBLIT_PHYSICS_JOINT_SIX_AXIS
      for (int which = 0; which < 6; ++which) making.limit(which);
      break;
  }
}

/// The path a joint was asked to follow, in `a`'s frame. False when it is too
/// short to run along: under two points once those that touch are one.
bool laidOut(const JointDescription &from, const Bodies &bodies, uint32_t rowA,
             Track &track) {
  const bool world = rowA == Bodies::kNone;
  const Vec3 origin = world ? Vec3{} : bodies.at(rowA);
  const Quat turn = world ? Quat{} : bodies.rotation(rowA);

  float run = 0.0f;
  for (const Vec3 &point : from.path) {
    const Vec3 local = unrotate(turn, point - origin);
    if (!track.points.empty()) {
      const float step = length(local - track.points.back());
      if (!(step > kTouching)) continue;
      run += step;
    }
    track.points.push_back(local);
    track.arc.push_back(run);
  }
  track.follow = from.follow;
  return track.points.size() >= 2;
}

/// Makes `joint` the coupling `from` is, which holds the one coordinate its
/// kind sums at what it was when made.
void tieWays(const JointDescription &from, const Bodies &bodies, uint32_t rowA,
             uint32_t rowB, Joint &joint) {
  Tie &tie = joint.tie;
  tie.ratio = from.ratio;
  tie.axis = rotate(normalised(from.rotation), kAlongX);
  tie.over[0] = from.over[0];
  tie.over[1] = from.over[1];
  switch (from.kind) {
    case ORBLIT_PHYSICS_JOINT_GEAR:
      tie.kind = Tied::Gear;
      break;
    case ORBLIT_PHYSICS_JOINT_RACK:
      tie.kind = Tied::Rack;
      break;
    default: // ORBLIT_PHYSICS_JOINT_PULLEY
      tie.kind = Tied::Pulley;
      break;
  }
  joint.holds = 1u;
  joint.least[0] = joint.most[0] = 0.0f;

  tie.last[0] = rowA == Bodies::kNone ? Quat{} : bodies.rotation(rowA);
  tie.last[1] = rowB == Bodies::kNone ? Quat{} : bodies.rotation(rowB);
  tie.constant = tiedCoordinate(tie, poseOf(bodies, rowA, rowB, joint));
}

/// A gear or a pinion turns about its own x. Where that was asked to be a
/// different way from `a`'s, the frame on `b` is tipped onto it.
Quat frameForB(const JointDescription &from) {
  const Quat frame = normalised(from.rotation);
  if (!(lengthSquared(from.axisB) > 0.0f)) return frame;
  const Quat tip = turnOnto(rotate(frame, kAlongX), normalised(from.axisB));
  return normalised(tip * frame);
}

bool isCoupling(uint32_t kind) {
  return kind == ORBLIT_PHYSICS_JOINT_GEAR ||
         kind == ORBLIT_PHYSICS_JOINT_RACK ||
         kind == ORBLIT_PHYSICS_JOINT_PULLEY;
}

} // namespace

// ------------------------------------------------------------- keeping ---

bool Joints::add(const JointDescription &from, const Bodies &bodies) {
  if (from.id == 0 || index_.find(from.id) != index_.end()) return false;
  // Both zero is the world joined to itself, which is the same refusal.
  if (from.a == from.b) return false;
  if (from.kind < ORBLIT_PHYSICS_JOINT_FIXED ||
      from.kind > ORBLIT_PHYSICS_JOINT_PATH) {
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

  // Two kinds have a point of their own on each body.
  const bool apart = from.kind == ORBLIT_PHYSICS_JOINT_DISTANCE ||
                     from.kind == ORBLIT_PHYSICS_JOINT_PULLEY;
  anchorOn(bodies, rowA, from.at, from.rotation, joint.anchorA, joint.frameA);
  anchorOn(bodies, rowB, apart ? from.to : from.at, frameForB(from),
           joint.anchorB, joint.frameB);

  if (from.kind == ORBLIT_PHYSICS_JOINT_PATH &&
      !laidOut(from, bodies, rowA, joint.track)) {
    return false;
  }
  if (isCoupling(from.kind)) {
    tieWays(from, bodies, rowA, rowB, joint);
  } else {
    Making making{from, joint};
    holdWays(making);
  }
  keepSprings(from, joint);

  index_.emplace(joint.id, static_cast<uint32_t>(joints_.size()));
  if (joint.a != 0) ++counts_[joint.a];
  if (joint.b != 0) ++counts_[joint.b];
  if (joint.a != 0 && joint.b != 0 && !joint.collide) {
    ++apart_[PairKey::of(joint.a, joint.b)];
  }
  joints_.push_back(std::move(joint));
  arranged_ = false;
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
  arranged_ = false;
  return true;
}

const Joint *Joints::find(OrblitPhysicsId id) const {
  const auto found = index_.find(id);
  return found == index_.end() ? nullptr : &joints_[found->second];
}

const Joint *Joints::aim(OrblitPhysicsId id, const float *rest, float goal) {
  const auto found = index_.find(id);
  if (found == index_.end()) return nullptr;
  Joint &joint = joints_[found->second];

  bool moved = false;
  // A way is moved only where it has a spring, so a rest given for a way that
  // holds by a lock is as ignored as it is when the joint is made. Equal
  // values are not a move, which is what lets a held pose sleep.
  for (int which = 0; rest != nullptr && which < 6; ++which) {
    const bool sprung = (joint.sprung & (1u << which)) != 0;
    if (!sprung || std::isnan(rest[which])) continue;
    if (joint.springRest[which] == rest[which]) continue;
    joint.springRest[which] = rest[which];
    moved = true;
  }

  const bool goaled = joint.motor >= 0 && joint.goalHertz > 0.0f;
  if (goaled && !std::isnan(goal) && joint.goal != goal) {
    joint.goal = goal;
    moved = true;
  }
  return moved ? &joint : nullptr;
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
  if (joint->onPath()) out.offset[0] = pose.arc;
  if (joint->tie.kind != Tied::None) {
    // How far the coordinate it keeps has drifted. The turns are brought up to
    // date on a copy, because asking must not change anything.
    Tie tie = joint->tie;
    sumTurns(bodies, rowA, rowB, *joint, tie);
    out.offset[0] = tiedCoordinate(tie, pose) - tie.constant;
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
  held.movesA = held.a != Bodies::kNone && bodies.solved(held.a);
  held.movesB = held.b != Bodies::kNone && bodies.solved(held.b);
  if (held.movesA) {
    held.gainA = bodies.gain(held.a);
    held.inertiaA = bodies.inverseInertia(held.a);
  }
  if (held.movesB) {
    held.gainB = bodies.gain(held.b);
    held.inertiaB = bodies.inverseInertia(held.b);
  }
  return held;
}

float Joints::massAlong(const Held &held, const Vec3 &linearA,
                        const Vec3 &linearB, const Vec3 &turnA,
                        const Vec3 &turnB) {
  const float k = dot(linearA, mulPerAxis(held.gainA, linearA)) +
                  dot(linearB, mulPerAxis(held.gainB, linearB)) +
                  dot(turnA, held.inertiaA * turnA) +
                  dot(turnB, held.inertiaB * turnB);
  return k > kTiny ? 1.0f / k : 0.0f;
}

void Joints::arrange(const Bodies &bodies) {
  const uint32_t count = static_cast<uint32_t>(joints_.size());
  constexpr uint32_t kUnreached = 0xFFFFFFFFu;

  // The joints on each body, so that a walk outwards from the world visits
  // each joint once. Joints are few and this is made only when they change.
  std::unordered_map<OrblitPhysicsId, std::vector<uint32_t>> on;
  for (uint32_t i = 0; i < count; ++i) {
    if (joints_[i].a != 0) on[joints_[i].a].push_back(i);
    if (joints_[i].b != 0) on[joints_[i].b].push_back(i);
  }
  const auto anchored = [&bodies](OrblitPhysicsId id) {
    return id == 0 || !bodies.movable(bodies.rowOf(id));
  };

  std::vector<uint32_t> depth(count, kUnreached);
  std::vector<uint32_t> queue;
  const auto walk = [&]() {
    for (size_t head = 0; head < queue.size(); ++head) {
      const uint32_t at = queue[head];
      for (const OrblitPhysicsId end : {joints_[at].a, joints_[at].b}) {
        if (anchored(end)) continue;
        for (const uint32_t next : on[end]) {
          if (depth[next] != kUnreached) continue;
          depth[next] = depth[at] + 1;
          queue.push_back(next);
        }
      }
    }
  };

  for (uint32_t i = 0; i < count; ++i) {
    if (anchored(joints_[i].a) || anchored(joints_[i].b)) {
      depth[i] = 0;
      queue.push_back(i);
    }
  }
  walk();
  // What is left holds nothing to the world: a chain in the air, or two
  // bodies hung from each other. The first made is its root.
  for (uint32_t i = 0; i < count; ++i) {
    if (depth[i] != kUnreached) continue;
    depth[i] = 0;
    queue.assign(1, i);
    walk();
  }

  order_.resize(count);
  for (uint32_t i = 0; i < count; ++i) order_[i] = i;
  std::stable_sort(order_.begin(), order_.end(),
                   [&depth](uint32_t x, uint32_t y) { return depth[x] < depth[y]; });
  arranged_ = true;
}

void Joints::prepare(Bodies &bodies, float h) {
  held_.clear();
  rows_.clear();
  carried_.clear();
  if (joints_.empty()) return;
  if (!arranged_) arrange(bodies);
  carried_.assign(bodies.count(), 0);

  for (const uint32_t index : order_) {
    Joint &joint = joints_[index];
    Held held = heldFor(bodies, index);
    // Nothing to move. Its impulses are kept, so a joint that went to sleep
    // holding something up starts holding it up again the moment it wakes.
    if (!held.movesA && !held.movesB) continue;
    if (held.movesA) carried_[held.a] = 1;
    if (held.movesB) carried_[held.b] = 1;

    held.firstRow = static_cast<uint32_t>(rows_.size());
    buildRows(bodies, held, joint, h);
    held.rowCount = static_cast<uint32_t>(rows_.size()) - held.firstRow;
    held_.push_back(held);
  }
}

void Joints::buildRows(const Bodies &bodies, Held &held, Joint &joint,
                       float h) {
  const uint32_t heldAt = static_cast<uint32_t>(held_.size());
  sumTurns(bodies, held.a, held.b, joint, joint.tie);
  const Pose pose = poseOf(bodies, held.a, held.b, joint);

  // How much stiffer a spring on a way that turns a body about its joint
  // point has to be, for the frequency asked for to be the frequency it has.
  const auto reachOf = [&held, &pose, &joint](const Row &row) {
    // A coupling's turns are scaled by its ratio, which is no axis to measure
    // an inertia about.
    if (joint.tie.kind != Tied::None || !(row.mass > 0.0f)) return 1.0f;
    if (lengthSquared(row.linearA) > 0.0f) return 1.0f;
    float inverse = 0.0f;
    if (held.movesA) {
      inverse += inverseInertiaAbout(held.inertiaA, held.gainA,
                                     pose.pointA - pose.centreA, row.turnA);
    }
    if (held.movesB) {
      inverse += inverseInertiaAbout(held.inertiaB, held.gainB,
                                     pose.pointB - pose.centreB, row.turnB);
    }
    if (!(inverse > kTiny)) return 1.0f;
    return std::sqrt(std::fmax(1.0f / (inverse * row.mass), 1.0f));
  };

  uint32_t built = 0;
  const auto build = [&](int slot) {
    Row row;
    row.held = heldAt;
    row.which = slot;
    Standing standing;
    standing.h = h;
    Direction way;
    if (!rowAt(joint, pose, slot, way)) return;
    row.linearA = way.linearA;
    row.linearB = way.linearB;
    row.turnA = way.turnA;
    row.turnB = way.turnB;
    standing.at = way.at;
    row.mass = massAlong(held, row.linearA, row.linearB, row.turnA, row.turnB);
    standing.reach = reachOf(row);

    const Aim aim = aimFor(joint, slot, standing);
    row.target = aim.target;
    row.least = aim.least;
    row.most = aim.most;
    row.massScale = aim.soft.massScale;
    row.impulseScale = aim.soft.impulseScale;

    // Last step's impulse, if it still points the way this row may push.
    // A limit that has swapped ends starts from nothing.
    joint.impulse[slot] = clamped(joint.impulse[slot], row.least, row.most);
    built |= 1u << slot;
    rows_.push_back(row);
  };

  // The motor and the springs first, so a limit or a lock on the same way has
  // the last word.
  if (joint.motor >= 0) build(kMotorRow);
  for (int which = 0; which < 6; ++which) {
    if ((joint.sprung & (1u << which)) != 0) build(kSpringRow + which);
  }
  for (int which = 0; which < 6; ++which) {
    if ((joint.holds & (1u << which)) != 0) build(which);
  }
  for (int slot = 0; slot < kJointRows; ++slot) {
    if ((built & (1u << slot)) == 0) joint.impulse[slot] = 0.0f;
  }
}

void Joints::warmStart(Bodies &bodies) {
  for (const Row &row : rows_) {
    const Held &held = held_[row.held];
    apply(bodies, held, row, joints_[held.joint].impulse[row.which]);
  }
}

void Joints::solveHeld(Bodies &bodies, const Held &held) {
  for (uint32_t i = held.firstRow; i < held.firstRow + held.rowCount; ++i) {
    const Row &row = rows_[i];
    float &impulse = joints_[held.joint].impulse[row.which];
    const float was = impulse;
    impulse = clamped(
        was - row.massScale * row.mass *
                  (speedAlong(bodies, held, row) - row.target) -
            row.impulseScale * was,
        row.least, row.most);
    apply(bodies, held, row, impulse - was);
  }
}

void Joints::correctVelocities(Bodies &bodies, bool leavesFirst) {
  // Whole joints turn round and their rows do not, because a limit on the
  // motor's axis is meant to have the last word.
  if (leavesFirst) {
    for (size_t i = held_.size(); i-- > 0;) solveHeld(bodies, held_[i]);
  } else {
    for (const Held &held : held_) solveHeld(bodies, held);
  }
}

void Joints::accumulate() {
  for (const Held &held : held_) {
    Joint &joint = joints_[held.joint];
    for (int which = 0; which < kJointRows; ++which) {
      joint.total[which] += joint.impulse[which];
    }
  }
}

void Joints::correctPositions(Bodies &bodies) {
  // Leaves first, so what is left at the root is the least out.
  for (size_t at = held_.size(); at-- > 0;) {
    const Held &held = held_[at];
    Joint &joint = joints_[held.joint];
    for (int which = 0; which < 6; ++which) {
      if ((joint.holds & (1u << which)) == 0) continue;
      if (softLimited(joint, which)) continue;

      // Measured again for every row, because the row before has just moved
      // the bodies, and a correction worked out from where they were is a
      // correction that overshoots.
      sumTurns(bodies, held.a, held.b, joint, joint.tie);
      const Pose pose = poseOf(bodies, held.a, held.b, joint);
      Direction way;
      if (!rowAt(joint, pose, which, way)) continue;

      float error = 0.0f;
      if (way.at < joint.least[which]) {
        error = way.at - joint.least[which];
      } else if (way.at > joint.most[which]) {
        error = way.at - joint.most[which];
      } else {
        continue;
      }

      const float most = which < 3 ? kMaxPull : kMaxTurn;
      const float correction = clamped(error * kPull, -most, most);
      const float pull =
          -correction *
          massAlong(held, way.linearA, way.linearB, way.turnA, way.turnB);

      if (held.a != Bodies::kNone) {
        bodies.at(held.a) -= mulPerAxis(way.linearA, held.gainA) * pull;
        bodies.turn(held.a, -(held.inertiaA * way.turnA) * pull, 1.0f);
      }
      if (held.b == Bodies::kNone) continue;
      bodies.at(held.b) += mulPerAxis(way.linearB, held.gainB) * pull;
      bodies.turn(held.b, (held.inertiaB * way.turnB) * pull, 1.0f);
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
    for (int which = 0; which < 6; ++which) {
      along[which] = joint.total[which] + joint.total[kSpringRow + which];
    }
    if (joint.motor >= 0) along[joint.motor] += joint.total[kMotorRow];
    // A gear's one row is a turn, so what it holds is a torque.
    if (joint.tie.kind == Tied::Gear) std::swap(along[0], along[3]);
    for (float &sum : joint.total) sum = 0.0f;
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
    speed -= dot(row.linearA, bodies.velocity(held.a)) +
             dot(row.turnA, bodies.spin(held.a));
  }
  if (held.b != Bodies::kNone) {
    speed += dot(row.linearB, bodies.velocity(held.b)) +
             dot(row.turnB, bodies.spin(held.b));
  }
  return speed;
}

void Joints::apply(Bodies &bodies, const Held &held, const Row &row,
                   float impulse) {
  if (held.a != Bodies::kNone) {
    bodies.velocity(held.a) -= mulPerAxis(row.linearA, held.gainA) * impulse;
    bodies.spin(held.a) -= (held.inertiaA * row.turnA) * impulse;
  }
  if (held.b == Bodies::kNone) return;
  bodies.velocity(held.b) += mulPerAxis(row.linearB, held.gainB) * impulse;
  bodies.spin(held.b) += (held.inertiaB * row.turnB) * impulse;
}

} // namespace orblit
