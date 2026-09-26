// Every body in a world, stored as columns.
//
// One vector per property rather than one struct per body, for the same reason
// the entity store is built that way: a step touches every body's velocity,
// then every body's position, then every body's bounds, and a column keeps
// each of those passes reading one run of memory instead of striding over the
// twenty other fields a body happens to have.
//
// Rows are dense. Removing a body moves the last row into its place, so there
// are never holes to skip, and the index is fixed up as part of the move. A
// row index is therefore only good until the next removal, which is why
// nothing outside this file keeps one across a step.

#ifndef ORBLIT_PHYSICS_BODY_H
#define ORBLIT_PHYSICS_BODY_H

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "maths.h"
#include "orblit_physics.h"
#include "shape.h"

namespace orblit {

enum class Motion : uint8_t {
  fixed = ORBLIT_PHYSICS_STATIC,
  driven = ORBLIT_PHYSICS_KINEMATIC,
  free = ORBLIT_PHYSICS_DYNAMIC,

  /// A driven body the character pass moves by sweeping, rather than the
  /// integrator by velocity. Not in the header: a caller makes one by turning
  /// a driven body into a character, never by creating one, so there is no
  /// such thing as a character without its step height and slope.
  character = 3,
};

/// Orblit's `Layers` rule, unchanged: a pair interacts when either cares about
/// the other, not when both do.
inline bool interact(uint32_t isA, uint32_t caresA, uint32_t isB, uint32_t caresB) {
  return (caresA & isB) != 0 || (caresB & isA) != 0;
}

/// Two bodies, by the caller's ids, smaller first.
struct PairKey {
  OrblitPhysicsId a = 0;
  OrblitPhysicsId b = 0;

  /// The key for `x` and `y`, whichever way round they come.
  static PairKey of(OrblitPhysicsId x, OrblitPhysicsId y) {
    return x < y ? PairKey{x, y} : PairKey{y, x};
  }

  bool operator==(const PairKey &o) const { return a == o.a && b == o.b; }
};

struct PairKeyHash {
  size_t operator()(const PairKey &k) const {
    // Two ids into one hash. The shift stops a pair and its reverse landing
    // on the same bucket, which an xor alone would do.
    const uint64_t mixed = k.a * 0x9E3779B97F4A7C15ull ^ (k.b + 0x165667B1ull);
    return static_cast<size_t>(mixed ^ (mixed >> 29));
  }
};

/// How a body was made. Everything that does not change after a create.
struct BodyDescription {
  Shape shape;
  Motion motion = Motion::free;
  Vec3 at;
  Quat rotation;
  float mass = 1.0f;
  float friction = 0.5f;
  float restitution = 0.0f;
  float linearDamping = 0.05f;
  float angularDamping = 0.05f;
  uint32_t layerIs = 1;
  uint32_t layerCares = 0xFFFFFFFFu;
  bool asleep = false;
};

class Bodies {
 public:
  static constexpr uint32_t kNone = 0xFFFFFFFFu;

  uint32_t count() const { return static_cast<uint32_t>(id_.size()); }

  /// The row holding `id`, or kNone.
  uint32_t rowOf(OrblitPhysicsId id) const {
    const auto found = index_.find(id);
    return found == index_.end() ? kNone : found->second;
  }

  /// Appends a row, or returns kNone if `id` is zero or already here.
  ///
  /// Already here is refused rather than replaced: a caller that creates the
  /// same body twice has a bug, and quietly rebuilding it would hide the
  /// frame on which the first one's velocity disappeared.
  uint32_t add(OrblitPhysicsId id, const BodyDescription &from);

  /// Removes the row holding `id`, if any. The last row moves into its place.
  void remove(OrblitPhysicsId id);

  // --- columns -------------------------------------------------------------

  const std::vector<OrblitPhysicsId> &ids() const { return id_; }

  OrblitPhysicsId id(uint32_t row) const { return id_[row]; }
  Motion motion(uint32_t row) const { return motion_[row]; }
  const Shape &shape(uint32_t row) const { return shape_[row]; }

  Vec3 &at(uint32_t row) { return at_[row]; }
  const Vec3 &at(uint32_t row) const { return at_[row]; }
  Quat &rotation(uint32_t row) { return rotation_[row]; }
  const Quat &rotation(uint32_t row) const { return rotation_[row]; }
  Vec3 &velocity(uint32_t row) { return velocity_[row]; }
  const Vec3 &velocity(uint32_t row) const { return velocity_[row]; }
  Vec3 &spin(uint32_t row) { return spin_[row]; }
  const Vec3 &spin(uint32_t row) const { return spin_[row]; }

  float friction(uint32_t row) const { return friction_[row]; }
  float restitution(uint32_t row) const { return restitution_[row]; }
  float linearDamping(uint32_t row) const { return linearDamping_[row]; }
  float angularDamping(uint32_t row) const { return angularDamping_[row]; }
  uint32_t layerIs(uint32_t row) const { return layerIs_[row]; }
  uint32_t layerCares(uint32_t row) const { return layerCares_[row]; }

  /// Zero for anything the solver does not move, which is what turns "is this
  /// body static" from a branch in every line of the solver into a multiply
  /// by nothing.
  float inverseMass(uint32_t row) const { return inverseMass_[row]; }
  const Mat3 &inverseInertia(uint32_t row) const { return inverseInertia_[row]; }

  bool asleep(uint32_t row) const { return asleep_[row] != 0; }
  bool movable(uint32_t row) const { return motion_[row] == Motion::free; }

  /// Whether the solver will move it this step: free, awake, and with mass.
  bool solved(uint32_t row) const {
    return motion_[row] == Motion::free && asleep_[row] == 0;
  }

  float &still(uint32_t row) { return still_[row]; }
  const Bounds &bounds(uint32_t row) const { return bounds_[row]; }

  /// Puts it to sleep, stopping it dead. Velocity is cleared rather than kept
  /// for waking, because a body that resumes at the speed it fell asleep at is
  /// a body that twitches when something brushes past it.
  void sleep(uint32_t row) {
    asleep_[row] = 1;
    velocity_[row] = {};
    spin_[row] = {};
    still_[row] = 0.0f;
  }

  void wake(uint32_t row) {
    asleep_[row] = 0;
    still_[row] = 0.0f;
  }

  /// Changes how it moves after it was made. Only ever driven to character
  /// and back: neither has mass, so nothing derived from mass goes stale.
  void steer(uint32_t row, Motion motion) { motion_[row] = motion; }

  void place(uint32_t row, const Vec3 &at, const Quat &rotation) {
    at_[row] = at;
    rotation_[row] = normalised(rotation);
    velocity_[row] = {};
    spin_[row] = {};
    refresh(row);
  }

  /// Recomputes what the position and rotation imply: world inertia and
  /// bounds. Called after anything moves a body outside a step.
  void refresh(uint32_t row) {
    inverseInertia_[row] =
        rotatedInverseInertia(rotation_[row], localInverseInertia_[row]);
    bounds_[row] = shape_[row].boundsAt(at_[row], rotation_[row]);
  }

  /// Recomputes every row's derived state. Once per step, over columns.
  void refreshAll() {
    for (uint32_t row = 0; row < count(); ++row) refresh(row);
  }

 private:
  std::vector<OrblitPhysicsId> id_;
  std::vector<Vec3> at_;
  std::vector<Quat> rotation_;
  std::vector<Vec3> velocity_;
  std::vector<Vec3> spin_;
  std::vector<Shape> shape_;
  std::vector<Motion> motion_;
  std::vector<float> inverseMass_;
  std::vector<Vec3> localInverseInertia_;
  std::vector<Mat3> inverseInertia_;
  std::vector<float> friction_;
  std::vector<float> restitution_;
  std::vector<float> linearDamping_;
  std::vector<float> angularDamping_;
  std::vector<uint32_t> layerIs_;
  std::vector<uint32_t> layerCares_;
  std::vector<uint8_t> asleep_;
  std::vector<float> still_;
  std::vector<Bounds> bounds_;

  std::unordered_map<OrblitPhysicsId, uint32_t> index_;
};

inline uint32_t Bodies::add(OrblitPhysicsId id, const BodyDescription &from) {
  if (id == 0 || index_.find(id) != index_.end()) return kNone;

  const uint32_t row = count();
  const bool free = from.motion == Motion::free;
  const float mass = from.mass > kTiny ? from.mass : 1.0f;

  id_.push_back(id);
  at_.push_back(from.at);
  rotation_.push_back(normalised(from.rotation));
  velocity_.push_back({});
  spin_.push_back({});
  shape_.push_back(from.shape);
  motion_.push_back(from.motion);
  inverseMass_.push_back(free ? 1.0f / mass : 0.0f);
  localInverseInertia_.push_back(free ? from.shape.inverseInertia(mass) : Vec3{});
  inverseInertia_.push_back(Mat3::zero());
  friction_.push_back(from.friction);
  restitution_.push_back(from.restitution);
  linearDamping_.push_back(from.linearDamping);
  angularDamping_.push_back(from.angularDamping);
  layerIs_.push_back(from.layerIs);
  layerCares_.push_back(from.layerCares);
  asleep_.push_back(from.asleep && free ? 1 : 0);
  still_.push_back(0.0f);
  bounds_.push_back({});

  index_.emplace(id, row);
  refresh(row);
  return row;
}

inline void Bodies::remove(OrblitPhysicsId id) {
  const auto found = index_.find(id);
  if (found == index_.end()) return;

  const uint32_t row = found->second;
  const uint32_t last = count() - 1;
  index_.erase(found);

  if (row != last) {
    id_[row] = id_[last];
    at_[row] = at_[last];
    rotation_[row] = rotation_[last];
    velocity_[row] = velocity_[last];
    spin_[row] = spin_[last];
    shape_[row] = shape_[last];
    motion_[row] = motion_[last];
    inverseMass_[row] = inverseMass_[last];
    localInverseInertia_[row] = localInverseInertia_[last];
    inverseInertia_[row] = inverseInertia_[last];
    friction_[row] = friction_[last];
    restitution_[row] = restitution_[last];
    linearDamping_[row] = linearDamping_[last];
    angularDamping_[row] = angularDamping_[last];
    layerIs_[row] = layerIs_[last];
    layerCares_[row] = layerCares_[last];
    asleep_[row] = asleep_[last];
    still_[row] = still_[last];
    bounds_[row] = bounds_[last];
    index_[id_[row]] = row;
  }

  id_.pop_back();
  at_.pop_back();
  rotation_.pop_back();
  velocity_.pop_back();
  spin_.pop_back();
  shape_.pop_back();
  motion_.pop_back();
  inverseMass_.pop_back();
  localInverseInertia_.pop_back();
  inverseInertia_.pop_back();
  friction_.pop_back();
  restitution_.pop_back();
  linearDamping_.pop_back();
  angularDamping_.pop_back();
  layerIs_.pop_back();
  layerCares_.pop_back();
  asleep_.pop_back();
  still_.pop_back();
  bounds_.pop_back();
}

} // namespace orblit

#endif // ORBLIT_PHYSICS_BODY_H
