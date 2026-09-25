// A world: the bodies, the order a step does things in, and what happened.
//
// The order is the part worth stating, because every other choice follows
// from it. One step is:
//
//   find contacts -> integrate velocity -> solve velocity
//                 -> integrate position -> solve position
//                 -> move characters -> sleep
//
// Contacts are found on the positions the last step left, so a caller reading
// transforms and a caller reading events are looking at the same instant.
// Velocity is solved before anything moves, so the move already has the
// collision in it rather than needing to be undone. Position is solved last,
// on the overlap that survived, so the correction is never spent on overlap
// that integration was about to remove anyway.
//
// Characters move after everything else has, because they are the one kind
// of body that looks at the world before moving through it. A lift has
// already risen and a crate already settled by the time a character asks
// where it can go, so it stands on the lift where the lift now is rather than
// where it was.

#ifndef ORBLIT_PHYSICS_WORLD_H
#define ORBLIT_PHYSICS_WORLD_H

#include <unordered_map>
#include <utility>
#include <vector>

#include "body.h"
#include "character.h"
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

  /// The nearest body `query` would meet, and whether it met one. Changes
  /// nothing: a world is the same after a cast as it was before.
  bool cast(const OrblitPhysicsCast &query, OrblitPhysicsHit &out) const;

  const Bodies &bodies() const { return bodies_; }
  const std::vector<OrblitPhysicsEvent> &events() const { return events_; }

  /// What each character in `ids` stood on at the end of the last step.
  /// Anything that is not a character is skipped and its slot left alone.
  uint32_t footing(const OrblitPhysicsId *ids, uint32_t count,
                   OrblitPhysicsFooting *out) const;

  /// Wakes a body and says so, if it was asleep.
  void wake(uint32_t row);

 private:
  void apply(const OrblitPhysicsCommand &command);
  void findContacts();
  void integrateVelocities(float delta);
  void integratePositions(float delta);
  void moveCharacters(float delta);
  void moveCharacter(Character &character, uint32_t row, const Vec3 &up,
                     float delta);
  void updateSleep(float delta);
  void reportTouches();
  void note(uint32_t kind, uint32_t row);

  /// Adds an impulse to a free body at a world point, waking it. Anything
  /// else is left alone: only the solver's bodies have a mass to divide by.
  void push(uint32_t row, const Vec3 &impulse, const Vec3 &point);

  Character *characterOf(OrblitPhysicsId id);
  const Character *characterOf(OrblitPhysicsId id) const;

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

  /// In the order they were made, which is the order they move in. A list
  /// rather than a column of the bodies because there are a handful of them
  /// among thousands of bodies, and every column pays for every row.
  std::vector<Character> characters_;

  /// The pushes a character's move gave, and the ones a second try at it
  /// gave, so the try that is thrown away pushes nothing.
  std::vector<Shove> shoves_;
  std::vector<Shove> trial_;

  /// Scratch for the broadphase, kept so a step does not allocate.
  std::vector<uint32_t> sorted_;
  std::vector<uint32_t> planes_;
  std::vector<std::pair<uint32_t, uint32_t>> candidates_;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_WORLD_H
