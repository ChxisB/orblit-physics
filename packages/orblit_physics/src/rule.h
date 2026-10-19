// What a game says about one pair of bodies, or about one region of space.
//
// The contact hook is data rather than a callback. A callback into the caller
// on every contact of every step would put a call across the C boundary, and a
// garbage-collected runtime behind it, inside the solver's inner loop. A game
// that wants this crate on that ice to slide says so once, here, and the solver
// reads the answer from a table it already owns.

#ifndef ORBLIT_PHYSICS_RULE_H
#define ORBLIT_PHYSICS_RULE_H

#include <cstdint>

#include "maths.h"
#include "orblit_physics.h"

namespace orblit {

/// How fast a body may go up through a one-way platform and still be held by
/// it. A body standing on one is nudged up a little by the solver's own push
/// out of the overlap, and that is not a body passing through.
constexpr float kOneWayRise = 0.25f;

/// How squarely a contact must face up out of a one-way platform to hold. The
/// side of a platform has none of it, so a body beside one is not held.
constexpr float kOneWayFacing = 0.25f;

/// A rule between two bodies. Nothing set is an ordinary contact.
struct Rule {
  /// OrblitPhysicsRuleField bits for the two values that replace the bodies'
  /// own. The move scales are not one of them: one is already "leave it".
  uint32_t overrides = 0;
  float friction = 0.0f;
  float restitution = 0.0f;

  /// For the first and second body of whatever it is attached to.
  float move[2] = {1.0f, 1.0f};

  /// For ORBLIT_PHYSICS_RULE_ONE_WAY: the unit direction the passer goes up
  /// through the platform, and which of the two bodies, 0 or 1, the platform is.
  Vec3 through;
  uint32_t platform = 0;

  /// The same rule with the two bodies named the other way round.
  Rule turned() const {
    Rule other = *this;
    other.move[0] = move[1];
    other.move[1] = move[0];
    other.platform = 1 - platform;
    return other;
  }

  bool oneWay() const { return (overrides & ORBLIT_PHYSICS_RULE_ONE_WAY) != 0; }

  /// How fast the passer goes up through the platform, given the first body's
  /// velocity minus the second's. Negative when it comes down on it.
  float rise(const Vec3 &firstMinusSecond) const {
    const float along = dot(firstMinusSecond, through);
    return platform == 0 ? -along : along;
  }

  /// Whether a contact normal, which points out of the second body towards the
  /// first, faces the passer up out of the platform's top squarely enough to
  /// hold it.
  bool faces(const Vec3 &normal) const {
    const float along = dot(normal, through);
    return (platform == 1 ? along : -along) > kOneWayFacing;
  }
};

/// A region that changes how the bodies in it move.
struct Zone {
  /// OrblitPhysicsZoneField bits.
  uint32_t overrides = 0;
  int32_t priority = 0;
  Vec3 gravity;
  float linearDamping = 0.0f;
  float angularDamping = 0.0f;
  float density = 0.0f;
  float drag = 0.0f;
};

/// What one body feels from every zone it is in, settled field by field.
///
/// Each field goes to the zone that claims it with the highest priority, and at
/// a tie to the lower id. That is a total order, so the answer does not depend
/// on which zone the body happened to be found in first.
class Felt {
 public:
  /// Lets `zone`, which is the body `by`, speak for whichever fields it claims.
  /// `surface` is the height of its fluid, which only a fluid zone is asked.
  void add(OrblitPhysicsId by, const Zone &zone, float surface) {
    if (claim(kGravity, zone.overrides & ORBLIT_PHYSICS_ZONE_GRAVITY, by,
              zone.priority)) {
      gravity_ = zone.gravity;
    }
    if (claim(kLinear, zone.overrides & ORBLIT_PHYSICS_ZONE_LINEAR_DAMPING, by,
              zone.priority)) {
      linear_ = zone.linearDamping;
    }
    if (claim(kAngular, zone.overrides & ORBLIT_PHYSICS_ZONE_ANGULAR_DAMPING, by,
              zone.priority)) {
      angular_ = zone.angularDamping;
    }
    if (claim(kFluid, zone.overrides & ORBLIT_PHYSICS_ZONE_FLUID, by,
              zone.priority)) {
      density_ = zone.density;
      drag_ = zone.drag;
      surface_ = surface;
    }
  }

  /// What the body's own answer becomes: the zone's if one spoke, else itself.
  Vec3 gravity(const Vec3 &world) const {
    return claims_[kGravity].by != 0 ? gravity_ : world;
  }
  float linearDamping(float own) const {
    return claims_[kLinear].by != 0 ? linear_ : own;
  }
  float angularDamping(float own) const {
    return claims_[kAngular].by != 0 ? angular_ : own;
  }

  /// Whether a fluid zone spoke, and if so what it said.
  bool fluid() const { return claims_[kFluid].by != 0; }
  float density() const { return density_; }
  float drag() const { return drag_; }
  float surface() const { return surface_; }

 private:
  enum Field { kGravity, kLinear, kAngular, kFluid };

  struct Claim {
    /// Zero for nobody, which no body can be: ids start at one.
    OrblitPhysicsId by = 0;
    int32_t priority = 0;
  };

  /// Whether `by` takes the field, which it does by holding it afterwards.
  bool claim(Field field, uint32_t speaks, OrblitPhysicsId by, int32_t priority) {
    if (speaks == 0) return false;
    Claim &held = claims_[field];
    const bool outbids = held.by == 0 || priority > held.priority ||
                         (priority == held.priority && by < held.by);
    if (outbids) held = {by, priority};
    return outbids;
  }

  Claim claims_[4];
  Vec3 gravity_;
  float linear_ = 0.0f;
  float angular_ = 0.0f;
  float density_ = 0.0f;
  float drag_ = 0.0f;
  float surface_ = 0.0f;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_RULE_H
