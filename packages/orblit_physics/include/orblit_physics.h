// Orblit physics: a rigid-body solver in C++, behind a C ABI.
//
// This header is the whole contract. Dart binds to it over dart:ffi, a script
// runtime binds to it directly, and a headless test harness with no renderer
// anywhere near it binds to the same functions — which is why nothing here
// mentions any of them.
//
// Three ideas carry the design:
//
//   A tick is three calls, never one per body. Commands go in as one buffer,
//   the step is one call, and what came out is read back as one write into
//   memory the caller already owns. An engine that made you call in once per
//   body would spend more time crossing this boundary than solving anything,
//   and the store this feeds is shaped by exactly the same argument.
//
//   Bodies are keyed by whatever the caller calls them. Orblit passes an
//   entity handle; nothing here interprets the number, only matches it. That
//   is what lets the caller keep its own identity scheme and stops a second
//   one leaking out of this library into everything that touches it.
//
//   Reading writes into the caller's memory, in the caller's layout. Not into
//   a list this library allocates and the caller then walks. When the caller
//   is Orblit's core, that memory is a transform column in an archetype chunk,
//   and the whole of a step's output lands there in one call without this
//   library ever learning what an archetype is.

#ifndef ORBLIT_PHYSICS_H
#define ORBLIT_PHYSICS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Whatever the caller calls a body. Orblit passes an entity handle.
///
/// Never interpreted here, only matched, so a caller with its own generational
/// scheme keeps it. Zero is never a body.
typedef uint64_t OrblitPhysicsId;

typedef struct OrblitPhysics OrblitPhysics;

// ----------------------------------------------------------------- shapes ---

typedef enum {
  /// `size[0]` is the radius.
  ORBLIT_PHYSICS_SPHERE = 1,

  /// `size[0..2]` are the half extents, so a unit cube is 0.5 in each.
  ORBLIT_PHYSICS_BOX = 2,

  /// An infinite half-space: `size[0..2]` is the outward normal and `size[3]`
  /// how far along it the surface sits. Ground and walls, and never dynamic —
  /// a half-space has no centre to rotate about and no mass to give it.
  ORBLIT_PHYSICS_PLANE = 3,
} OrblitPhysicsShapeKind;

typedef enum {
  /// Never moves and is never moved. The cheapest thing to be.
  ORBLIT_PHYSICS_STATIC = 0,

  /// Moved by whoever owns it, and pushes dynamic bodies without being pushed
  /// back. A lift, a door, a moving platform.
  ORBLIT_PHYSICS_KINEMATIC = 1,

  /// Moved by the solver.
  ORBLIT_PHYSICS_DYNAMIC = 2,
} OrblitPhysicsMotion;

// --------------------------------------------------------------- commands ---

typedef enum {
  ORBLIT_PHYSICS_CREATE = 1,
  ORBLIT_PHYSICS_DESTROY = 2,

  /// Put it exactly here, forgetting how it was moving. The teleport, not the
  /// shove: for spawning and for a cut, not for driving a character.
  ORBLIT_PHYSICS_PLACE = 3,

  /// Set linear velocity (`vector`) and angular velocity (`spin`) outright.
  ORBLIT_PHYSICS_VELOCITY = 4,

  /// Add an impulse `vector`, applied at world point `spin`. An impulse away
  /// from the centre of mass spins the body, which is the whole reason the
  /// point is here rather than assumed.
  ORBLIT_PHYSICS_IMPULSE = 5,

  /// Wake it, whether or not anything touched it.
  ORBLIT_PHYSICS_WAKE = 6,
} OrblitPhysicsCommandKind;

/// One thing to do to one body, before the next step.
///
/// Deliberately one size for every kind. A create is the big one and an
/// impulse wastes most of the difference, but a tick submits tens of these
/// rather than thousands, and one stride means a caller writes an array
/// instead of a parser. If that ever stops being true the buffer grows a
/// length per command, and this comment is the note that it was considered.
///
/// Fields a kind does not name are ignored, so zeroing the struct and filling
/// in what matters is always correct.
typedef struct {
  uint32_t kind;  ///< An OrblitPhysicsCommandKind.
  uint32_t shape; ///< CREATE: an OrblitPhysicsShapeKind.
  OrblitPhysicsId id;

  /// CREATE: the shape's dimensions, read as that kind describes.
  float size[4];

  /// CREATE, PLACE: where.
  float at[3];

  /// CREATE, PLACE: the orientation, xyzw. All zeroes is read as identity, so
  /// a caller that does not care about rotation can leave it alone.
  float rotation[4];

  /// VELOCITY: metres per second. IMPULSE: the impulse, newton-seconds.
  float vector[3];

  /// VELOCITY: radians per second. IMPULSE: the world point it acts at.
  float spin[3];

  uint32_t motion;   ///< CREATE: an OrblitPhysicsMotion.
  float mass;        ///< CREATE: kilograms. Ignored unless dynamic.
  float friction;    ///< CREATE: 0 slides for ever, 1 grips. Typically 0.5.
  float restitution; ///< CREATE: 0 lands dead, 1 bounces back to the height it
                     ///< fell from. Above 1 is a body that gains energy out of
                     ///< nowhere, so it is clamped.
  float damping[2];  ///< CREATE: linear, then angular, as a fraction shed per
                     ///< second. Small but not zero: without it a body coasts
                     ///< for ever and nothing ever sleeps.

  /// CREATE: which layers it is in, and which it wants to be told about.
  ///
  /// These are Orblit's `Layers`, unchanged — one vocabulary, whichever
  /// implementation reads it. A pair interacts when either cares about the
  /// other, not when both do, so a bullet that cares about walls hits a wall
  /// that cares about nothing.
  uint32_t layerIs;
  uint32_t layerCares;

  /// CREATE: starts asleep. A level full of crates should not simulate itself
  /// awake on the first frame only to find it was already settled.
  bool asleep;
  bool _reserved[3];
} OrblitPhysicsCommand;

// ----------------------------------------------------------------- events ---

typedef enum {
  ORBLIT_PHYSICS_TOUCH_BEGAN = 1,
  ORBLIT_PHYSICS_TOUCH_ENDED = 2,
  ORBLIT_PHYSICS_SLEPT = 3,
  ORBLIT_PHYSICS_WOKE = 4,
} OrblitPhysicsEventKind;

/// Something that happened during a step and is worth telling the caller.
///
/// Began and ended, never "is touching": a game reacts to the change, and a
/// caller that wants the standing set keeps one, which is cheaper than this
/// library rebuilding that set every tick for the callers that do not.
typedef struct {
  uint32_t kind; ///< An OrblitPhysicsEventKind.
  uint32_t _pad;

  /// The bodies. For SLEPT and WOKE only `a` means anything.
  ///
  /// A pair is always reported with the smaller id first, so a caller keying a
  /// set by the pair does not have to order it.
  OrblitPhysicsId a;
  OrblitPhysicsId b;

  float at[3];     ///< TOUCH_BEGAN: where, in the world.
  float normal[3]; ///< TOUCH_BEGAN: out of `b`, towards `a`.

  /// TOUCH_BEGAN: how hard, in newton-seconds along the normal. This is the
  /// number a collision sound scales with, which is why it is reported rather
  /// than left to be guessed from the velocities.
  float force;
} OrblitPhysicsEvent;

// --------------------------------------------------------------- settings ---

/// How the world behaves. Every field has a default that works.
typedef struct {
  float gravity[3];

  /// How many passes the solver makes over the contacts correcting velocity.
  ///
  /// Below two, friction does not work at all: friction is bounded by the
  /// normal impulse from the pass before it, and on the first pass there is
  /// not one yet.
  uint32_t velocitySteps;

  /// Passes correcting the overlap the velocity passes could not prevent.
  uint32_t positionSteps;

  /// How far bodies may sink into each other before it is worth pushing them
  /// apart, in metres.
  ///
  /// Not zero, and not a tolerance for sloppiness: solving to exactly touching
  /// means a resting stack separates and re-collides every frame, and what
  /// that looks like is a pile of crates buzzing.
  float slop;

  /// What fraction of the remaining overlap one position pass removes, 0 to 1.
  ///
  /// All of it at once turns a deep overlap into a body flung across the
  /// level. A fifth of it, several times, settles.
  float stiffness;

  /// Below this closing speed a collision does not bounce, however elastic it
  /// is, in metres per second.
  ///
  /// Without it a ball with any restitution at all never comes to rest: every
  /// tiny settling contact returns a tiny bounce, for ever.
  float bounceThreshold;

  /// How slowly a body must move to be a candidate for sleep, in metres per
  /// second, measured at whichever point of it is moving fastest.
  ///
  /// At the fastest point rather than at the centre, because a body spinning
  /// on the spot has a centre going nowhere.
  float sleepSpeed;

  /// How long it has to stay that slow, in seconds.
  float sleepAfter;

  /// Whether bodies may sleep at all. Off is how you find out whether a bug is
  /// in the solver or in the sleeping.
  bool sleeping;
  bool _reserved[3];
} OrblitPhysicsSettings;

/// Fills `out` with the defaults. Call this and change what you mean, rather
/// than zeroing the struct: a zeroed one has no gravity and no solver passes.
void orblit_physics_defaults(OrblitPhysicsSettings *out);

// ------------------------------------------------------------------ world ---

/// `settings` may be NULL for the defaults. It is copied, not kept.
OrblitPhysics *orblit_physics_create(const OrblitPhysicsSettings *settings);
void orblit_physics_destroy(OrblitPhysics *physics);

/// Applies `count` commands, in order.
///
/// They take effect immediately rather than at the next step, so a body
/// created and then shoved in one buffer is shoved. A command naming a body
/// that does not exist is ignored, which is what makes a destroy followed by a
/// stale impulse harmless rather than a crash.
void orblit_physics_submit(OrblitPhysics *physics,
                           const OrblitPhysicsCommand *commands,
                           uint32_t count);

/// Advances the world by `delta` seconds.
///
/// One call per tick. The caller owns the frame: if it wants a fixed step it
/// calls this several times with the same delta, because a solver handed a
/// wobbling delta produces a wobbling simulation, and only the caller knows
/// what its frame is doing.
///
/// Events from the previous step are dropped here, so read them before
/// stepping again.
void orblit_physics_step(OrblitPhysics *physics, float delta);

/// Writes where each of `ids` ended up into the caller's own memory.
///
/// Seven floats per body — translation xyz then rotation xyzw, which is the
/// layout `orblit_core`'s local transform begins with — written `stride`
/// floats apart, starting `offset` floats into `out`. Scale is never written:
/// physics does not own it, and a solver that quietly reset it would be a bug
/// nobody could find.
///
/// A `stride` of 7 and an `offset` of 0 is a packed array. A caller writing
/// into an archetype chunk passes that column's stride instead, and a whole
/// step's output lands in the store in one call.
///
/// An id this world does not know is skipped and its place in `out` left
/// exactly as it was. Returns how many were written.
uint32_t orblit_physics_read(OrblitPhysics *physics, const OrblitPhysicsId *ids,
                             uint32_t count, float *out, uint32_t stride,
                             uint32_t offset);

/// Everything that happened in the last step, and how much of it there is.
///
/// The pointer is this world's own memory and is good until the next step or
/// submit. Copy anything worth keeping.
const OrblitPhysicsEvent *orblit_physics_events(const OrblitPhysics *physics,
                                                uint32_t *count);

// ---------------------------------------------------------------- reading ---
//
// One body at a time, for a test or an inspector. Not for a system: crossing
// here once per body per frame is what the buffer above exists to avoid.

uint32_t orblit_physics_count(const OrblitPhysics *physics);

bool orblit_physics_alive(const OrblitPhysics *physics, OrblitPhysicsId id);

bool orblit_physics_asleep(const OrblitPhysics *physics, OrblitPhysicsId id);

/// Writes translation xyz then rotation xyzw into `out`. False if unknown.
bool orblit_physics_transform(const OrblitPhysics *physics, OrblitPhysicsId id,
                              float *out);

/// Writes linear velocity xyz then angular velocity xyz into `out`. False if
/// unknown.
bool orblit_physics_velocity(const OrblitPhysics *physics, OrblitPhysicsId id,
                             float *out);

#ifdef __cplusplus
}
#endif

#endif // ORBLIT_PHYSICS_H
