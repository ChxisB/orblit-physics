// A world: the bodies, the order a step does things in, and what happened.
//
// The order is the part worth stating, because every other choice follows
// from it. One step is:
//
//   find contacts -> integrate velocity -> solve velocity
//                 -> integrate position -> solve position -> sleep
//
// Contacts are found on the positions the last step left, so a caller reading
// transforms and a caller reading events are looking at the same instant.
// Velocity is solved before anything moves, so the move already has the
// collision in it rather than needing to be undone. Position is solved last,
// on the overlap that survived, so the correction is never spent on overlap
// that integration was about to remove anyway.

#ifndef ORBLIT_PHYSICS_WORLD_H
#define ORBLIT_PHYSICS_WORLD_H

#include <unordered_map>
#include <utility>
#include <vector>

#include "body.h"
#include "collide.h"
#include "orblit_physics.h"
#include "solver.h"

namespace orblit {

/// Two bodies, by the caller's ids, smaller first.
struct PairKey {
  OrblitPhysicsId a = 0;
  OrblitPhysicsId b = 0;

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

class World {
 public:
  explicit World(const OrblitPhysicsSettings &settings);

  /// Fills `out` with the defaults every field is documented to have.
  static void defaults(OrblitPhysicsSettings *out);

  void submit(const OrblitPhysicsCommand *commands, uint32_t count);
  void step(float delta);

  uint32_t read(const OrblitPhysicsId *ids, uint32_t count, float *out,
                uint32_t stride, uint32_t offset) const;

  const Bodies &bodies() const { return bodies_; }
  const std::vector<OrblitPhysicsEvent> &events() const { return events_; }

  /// Wakes a body and says so, if it was asleep.
  void wake(uint32_t row);

 private:
  void apply(const OrblitPhysicsCommand &command);
  void findContacts();
  void integrateVelocities(float delta);
  void integratePositions(float delta);
  void updateSleep(float delta);
  void reportTouches();
  void note(uint32_t kind, uint32_t row);

  OrblitPhysicsSettings settings_;
  SolverSettings solving_;
  Vec3 gravity_;

  Bodies bodies_;
  Solver solver_;

  std::vector<Manifold> manifolds_;
  std::vector<PairKey> keys_;

  /// The contacts of this step and of the one before, so a pair that appears
  /// or disappears can be reported and a pair that persists can be warm
  /// started. Swapped rather than copied at the end of a step.
  std::unordered_map<PairKey, Manifold, PairKeyHash> touching_;
  std::unordered_map<PairKey, Manifold, PairKeyHash> wasTouching_;

  std::vector<OrblitPhysicsEvent> events_;

  /// Scratch for the broadphase, kept so a step does not allocate.
  std::vector<uint32_t> sorted_;
  std::vector<uint32_t> planes_;
  std::vector<std::pair<uint32_t, uint32_t>> candidates_;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_WORLD_H
