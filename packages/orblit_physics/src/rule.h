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

/// A rule between two bodies. Nothing set is an ordinary contact.
struct Rule {
  /// OrblitPhysicsRuleField bits for the two values that replace the bodies'
  /// own. The move scales are not one of them: one is already "leave it".
  uint32_t overrides = 0;
  float friction = 0.0f;
  float restitution = 0.0f;

  /// For the first and second body of whatever it is attached to.
  float move[2] = {1.0f, 1.0f};

  /// The same rule with the two bodies named the other way round.
  Rule turned() const {
    Rule other = *this;
    other.move[0] = move[1];
    other.move[1] = move[0];
    return other;
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
};

/// What one body feels from every zone it is in, settled field by field.
///
/// Each field goes to the zone that claims it with the highest priority, and at
/// a tie to the lower id. That is a total order, so the answer does not depend
/// on which zone the body happened to be found in first.
class Felt {
 public:
  /// Lets `zone`, which is the body `by`, speak for whichever fields it claims.
  void add(OrblitPhysicsId by, const Zone &zone) {
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

 private:
  enum Field { kGravity, kLinear, kAngular };

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

  Claim claims_[3];
  Vec3 gravity_;
  float linear_ = 0.0f;
  float angular_ = 0.0f;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_RULE_H
