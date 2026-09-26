// A world: the bodies, the order a step does things in, and what happened.
//
// The order is the part worth stating, because every other choice follows
// from it. One step is:
//
//   find contacts -> integrate velocity -> solve velocity
//                 -> integrate position -> solve position -> break joints
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
//
// Joints break after the position passes, on what they held with in the
// velocity passes. A joint that gave way this step still held for all of it,
// so what it held moves off from where it was held rather than from a pose
// half-corrected by a joint that had already gone.

#ifndef ORBLIT_PHYSICS_WORLD_H
#define ORBLIT_PHYSICS_WORLD_H

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "body.h"
#include "character.h"
#include "collide.h"
#include "heightfield.h"
#include "joint.h"
#include "orblit_physics.h"
#include "solver.h"

namespace orblit {

class World {
 public:
  explicit World(const OrblitPhysicsSettings &settings);

  /// Fills `out` with the defaults every field is documented to have.
  static void defaults(OrblitPhysicsSettings *out);

  void submit(const OrblitPhysicsCommand *commands, uint32_t count);
  void step(float delta);

  /// Lays ground as a static body, replacing whatever had its id, and wakes
  /// everything over it. False, and nothing changed, if it cannot be laid.
  bool ground(const OrblitPhysicsGround &from);

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

  /// Wakes a body and says so, if it was asleep, and everything joined to it
  /// that is asleep with it.
  ///
  /// A body the solver does not move is never asleep, but waking one — by
  /// placing it or setting it going — still wakes what hangs from it: a lift
  /// that starts to rise wakes the rope tied to it.
  void wake(uint32_t row);

  /// Makes a joint where the bodies stand, waking both. False, and nothing
  /// changed, if it cannot be made.
  bool join(const OrblitPhysicsJoint &from);

  /// Removes a joint, waking both its bodies. False if there was none.
  bool unjoin(OrblitPhysicsId id);

  /// How joint `id` stands. False if there is no such joint.
  bool joint(OrblitPhysicsId id, OrblitPhysicsJointState &out) const;

 private:
  void apply(const OrblitPhysicsCommand &command);

  /// Removes a body and everything this world keeps beside it, its joints
  /// included, and wakes what it was joined to.
  void destroy(OrblitPhysicsId id);

  /// Removes a body and what this world keeps beside it, but not its joints:
  /// ground laid again under the same id is still what they were tied to.
  void unmake(OrblitPhysicsId id);
  void findContacts();
  void integrateVelocities(float delta);
  void integratePositions(float delta);
  void moveCharacters(float delta);
  void moveCharacter(Character &character, uint32_t row, const Vec3 &up,
                     float delta);
  void updateSleep(float delta);

  /// Puts every group of joined bodies to sleep that is still enough as a
  /// whole, and none that is not.
  void sleepJoined();

  /// Wakes everything joined to `row`, and everything joined to that, but
  /// not through a body the solver does not move: two chains hung from the
  /// same wall are two chains, and touching one leaves the other asleep.
  void wakeJoined(uint32_t row);

  /// Removes every joint that held with more than it could take this step,
  /// saying so.
  void breakJoints(float delta);

  /// Wakes the two ends of a joint, either of which may be the world.
  void wakeEnds(OrblitPhysicsId a, OrblitPhysicsId b);
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
  Joints joints_;

  /// The heights of every ground body, by its id. A body's shape points into
  /// one of these, so one is only ever let go of after its body is.
  std::unordered_map<OrblitPhysicsId, std::unique_ptr<HeightField>> fields_;

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

  /// Scratch for joints, likewise: which group each row is in, whether each
  /// group may sleep, the bodies left to wake, and the joints that broke.
  std::vector<uint32_t> group_;
  std::vector<uint8_t> ready_;
  std::vector<OrblitPhysicsId> waking_;
  std::vector<OrblitPhysicsId> partners_;
  std::vector<Broken> broken_;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_WORLD_H
