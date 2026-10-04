// A world: the bodies, the order a step does things in, and what happened.
//
// The order is the part worth stating, because every other choice follows
// from it. One step is:
//
//   find contacts -> feel zones -> integrate velocity -> solve velocity
//                 -> cap speeds -> integrate position -> solve position
//                 -> break joints -> move characters
//                 -> report touches and triggers -> sleep
//
// Contacts are found on the positions the last step left, so a caller reading
// transforms and a caller reading events are looking at the same instant.
// Velocity is solved before anything moves, so the move already has the
// collision in it rather than needing to be undone. Position is solved last,
// on the overlap that survived, so the correction is never spent on overlap
// that integration was about to remove anyway.
//
// A zone is read on those same positions, before velocity is integrated,
// because it changes what integration does. A trigger is read after everything
// has moved, because it only reports where things ended up, and a body that
// came to rest inside one should be in it by the time the caller looks.
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
#include <unordered_set>
#include <utility>
#include <vector>

#include "body.h"
#include "character.h"
#include "collide.h"
#include "compound.h"
#include "heightfield.h"
#include "hull.h"
#include "joint.h"
#include "orblit_physics.h"
#include "rule.h"
#include "solver.h"

namespace orblit {

/// Everything a world carries from one step to the next, and nothing it
/// rebuilds inside one.
///
/// A copy of the bodies, joints, characters, zones, rules and ground, and of
/// what the last step left for the next to read: which pairs touched, with the
/// impulses that warm start them, which bodies were in which triggers, and the
/// events. Restoring it gives back a world that steps exactly as the one it
/// was taken from would have, because a step reads nothing else.
///
/// Ground and hulls are shared rather than copied. A height field or a hull
/// never changes once it is laid, so a snapshot and the world it came from hold
/// the same one, and laying it again makes a new one and lets go of the old.
///
/// The fields here are the members of `World` that are not scratch. A member
/// added to the world that a step reads from the step before belongs here too,
/// and the check that restores step 50 into a scene with every feature in it is
/// the thing that notices when it is not.
struct Snapshot {
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const Compound>> compounds;
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const TriangleMesh>> meshes;
  OrblitPhysicsSettings settings;
  Vec3 gravity;
  Bodies bodies;
  Joints joints;
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const HeightField>> fields;
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const Hull>> hulls;
  std::unordered_map<PairKey, Manifold, PairKeyHash> touching;
  std::unordered_set<PairKey, PairKeyHash> sensing;
  std::unordered_map<OrblitPhysicsId, Zone> zones;
  std::unordered_map<PairKey, Rule, PairKeyHash> rules;
  std::vector<Character> characters;
  std::vector<OrblitPhysicsEvent> events;
  uint32_t pairs = 0;
};

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

  /// Cooks `count` points into a hull and keeps it under `id`. False, and
  /// nothing laid, if the id is zero or taken or the points enclose no volume.
  bool hull(OrblitPhysicsId id, const float *xyz, uint32_t count);

  /// Lets go of a hull. False, and nothing changed, if there is none under
  /// `id` or a body is still made from it.
  bool dropHull(OrblitPhysicsId id);
  bool compound(OrblitPhysicsId id, const OrblitPhysicsPart *parts, uint32_t count);
  bool dropCompound(OrblitPhysicsId id);
  bool mesh(OrblitPhysicsId id, const float *xyz, uint32_t vertices,
            const uint32_t *indices, uint32_t count);
  bool dropMesh(OrblitPhysicsId id);

  uint32_t read(const OrblitPhysicsId *ids, uint32_t count, float *out,
                uint32_t stride, uint32_t offset) const;

  /// The nearest body `query` would meet, and whether it met one. Changes
  /// nothing: a world is the same after a cast as it was before.
  bool cast(const OrblitPhysicsCast &query, OrblitPhysicsHit &out) const;

  /// Every body `query` would meet, nearest first, in `out` up to `capacity`.
  /// Returns how many. A full list keeps the nearest, not the first found.
  uint32_t castAll(const OrblitPhysicsCast &query, OrblitPhysicsHit *out,
                   uint32_t capacity) const;

  /// Whether `query` would meet anything, without finding what was nearest.
  bool castAny(const OrblitPhysicsCast &query) const;

  /// The bodies the query's shape overlaps where it stands, in no order, up to
  /// `capacity`. Returns how many.
  uint32_t overlap(const OrblitPhysicsCast &query, OrblitPhysicsId *out,
                   uint32_t capacity) const;

  /// Sets or removes the zone over a trigger, waking what is inside it. False,
  /// and nothing changed, if it cannot be set or there was none to remove.
  bool zone(const OrblitPhysicsZone &from);

  /// Sets or removes the rule between two bodies, waking both. False, and
  /// nothing changed, if it cannot be set or there was none to remove.
  bool rule(const OrblitPhysicsRule &from);

  /// Sets how a body moves, waking it and what rests on it. False, and nothing
  /// changed, if the body is not there, is ground, or the numbers mean nothing.
  bool controls(const OrblitPhysicsControls &from);

  /// Sets the world's gravity, waking every body the solver moves. False, and
  /// nothing changed, if any axis is not finite.
  bool gravity(const float to[3]);

  /// What the world is now, to be given back by `restore`.
  Snapshot snapshot() const;

  /// Makes this world the one `from` was taken from, settings and all, so it
  /// can be given a snapshot of another world and become that one. Anything
  /// that happened since is gone.
  void restore(const Snapshot &from);

  /// The contact points of the last step, in the order of the pairs' ids, up to
  /// `capacity`, and how many there were in all. With no room it counts.
  uint32_t contacts(OrblitPhysicsContact *out, uint32_t capacity) const;

  /// How much the world holds and how much the last step looked at.
  void stats(OrblitPhysicsStats &out) const;

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

  /// The shape a command or a cast names, and whether it names one. Not a
  /// height field, which has its own call; not a hull that was never laid; not
  /// a cylinder whose size is not a positive number.
  bool partFor(const OrblitPhysicsPart &from, Part &out) const;

  bool shapeFor(uint32_t kind, const float size[4], OrblitPhysicsId hull,
                Shape &out) const;

  /// The shape a cast moves, as `shapeFor` reads it, except that shape zero is
  /// a ray, which is a sphere of no size.
  bool shapeOf(const OrblitPhysicsCast &query, Shape &out) const;

  /// Removes a body and everything this world keeps beside it, its joints
  /// included, and wakes what it was joined to.
  void destroy(OrblitPhysicsId id);

  /// Removes a body and what this world keeps beside it, but not its joints:
  /// ground laid again under the same id is still what they were tied to.
  void unmake(OrblitPhysicsId id);
  void findContacts();

  /// What `rules_` says about the pair, named the way round `first` and the
  /// other body are, which is how a manifold names them.
  Rule ruleFor(const PairKey &key, OrblitPhysicsId first) const;

  /// Whether a rule says the two bodies pass through each other.
  bool ignores(OrblitPhysicsId a, OrblitPhysicsId b) const;

  /// Makes a body fixed, driven or free, if it can be. A body already what it
  /// is asked to be, a trigger, a character and ground are left alone.
  void switchMotion(uint32_t row, uint32_t to);

  /// Settles, for every body the solver moves, what each zone it is in says.
  void feelZones();
  void integrateVelocities(float delta);

  /// Holds each body the solver moves to its caps, after the contacts have had
  /// their say, so a cap limits where the body goes and not what it hits.
  void capSpeeds();
  void integratePositions(float delta);
  void moveCharacters(float delta);
  void moveCharacter(Character &character, uint32_t row, const Vec3 &up,
                     float delta);
  void updateSleep(float delta);

  /// Stops a body that a moving surface is holding still from counting that as
  /// rest. It is at rest against the surface and not against the world, and
  /// asleep it would stay put when whatever held it in place was taken away.
  void keepDragged();

  /// Wakes every body whose bounds overlap `area`.
  void wakeWithin(const Bounds &area);

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

  /// Reports which bodies came into and left each trigger since the step
  /// before, and what is still in one that asked to hear about it.
  void reportSensing();
  void senseFrom(uint32_t trigger);

  /// Whether `body` is in `trigger`, and the overlap if it is. Not a trigger
  /// against a trigger and not a static body, which nothing changes about.
  bool within(uint32_t trigger, uint32_t body, Manifold &inside) const;

  /// The event a manifold amounts to, its normal turned to come out of the
  /// key's second body towards its first.
  OrblitPhysicsEvent pairEvent(uint32_t kind, const PairKey &key,
                               const Manifold &manifold) const;
  void note(uint32_t kind, uint32_t row);

  /// Adds an impulse to a free body at a world point, waking it. Anything
  /// else is left alone: only the solver's bodies have a mass to divide by.
  void push(uint32_t row, const Vec3 &impulse, const Vec3 &point);

  Character *characterOf(OrblitPhysicsId id);
  const Character *characterOf(OrblitPhysicsId id) const;

  /// What the solver runs with, which is the settings with the values it
  /// cannot work with brought into range. Worked out again wherever the
  /// settings are set, so the two cannot disagree.
  static SolverSettings solvingFor(const OrblitPhysicsSettings &settings);

  OrblitPhysicsSettings settings_;
  SolverSettings solving_;
  Vec3 gravity_;

  Bodies bodies_;
  Solver solver_;
  Joints joints_;

  /// The heights of every ground body, by its id. A body's shape points into
  /// one of these, so one is only ever let go of after its body is. Shared with
  /// any snapshot that was taken while it was the ground.
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const HeightField>>
      fields_;

  /// The hulls that have been laid, by id. A body's shape points into one, so
  /// one is only ever let go of when no body is made from it. Shared with any
  /// snapshot, for the same reason ground is.
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const Hull>> hulls_;
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const Compound>> compounds_;
  std::unordered_map<OrblitPhysicsId, std::shared_ptr<const TriangleMesh>> meshes_;

  std::vector<Manifold> manifolds_;
  std::vector<PairKey> keys_;

  /// The contacts of this step and of the one before, so a pair that appears
  /// or disappears can be reported and a pair that persists can be warm
  /// started. Swapped rather than copied at the end of a step.
  std::unordered_map<PairKey, Manifold, PairKeyHash> touching_;
  std::unordered_map<PairKey, Manifold, PairKeyHash> wasTouching_;

  /// Which bodies are in which triggers, this step and the one before, keyed
  /// trigger first rather than smaller id first. Only ever a trigger and a
  /// body, so the two orders never collide.
  std::unordered_set<PairKey, PairKeyHash> sensing_;
  std::unordered_set<PairKey, PairKeyHash> wasSensing_;

  /// What has been said about regions and pairs, by body id, so it survives
  /// the row a body is in moving. A zone's key is the trigger it is over, and
  /// a rule's is its pair, with the move scales in the key's order.
  std::unordered_map<OrblitPhysicsId, Zone> zones_;
  std::unordered_map<PairKey, Rule, PairKeyHash> rules_;

  /// What each row feels from the zones, rebuilt every step there are any.
  std::vector<Felt> felt_;

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

  /// How many of those the narrowphase looked at in the last step. Kept past
  /// the step, unlike the list, so a snapshot can carry it.
  uint32_t pairs_ = 0;

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
