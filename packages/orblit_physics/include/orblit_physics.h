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

  /// A cylinder capped with a hemisphere at each end, standing along the
  /// body's own y. `size[0]` is the radius and `size[1]` half the straight
  /// part, so the whole thing is `2 * (size[1] + size[0])` tall.
  ///
  /// Half the straight part rather than the total height because that is the
  /// number every routine inside uses — the segment the hemispheres are swept
  /// along — and a shape whose field means one thing to the caller and
  /// another inside is a shape somebody eventually gets wrong by a radius.
  ORBLIT_PHYSICS_CAPSULE = 4,

  /// Ground, as heights on a grid. Not made with CREATE, which has nowhere to
  /// put a grid's worth of heights: laid with `orblit_physics_ground`, and
  /// after that a body like any other, destroyed the same way. Never dynamic,
  /// and never cast.
  ORBLIT_PHYSICS_HEIGHT_FIELD = 5,
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

  /// Take it out of the world, and every joint on it with it: silently,
  /// since nothing broke, and waking whatever those joints held.
  ORBLIT_PHYSICS_DESTROY = 2,

  /// Put it exactly here, forgetting how it was moving. The teleport, not the
  /// shove: for spawning and for a cut, not for driving a character.
  ORBLIT_PHYSICS_PLACE = 3,

  /// Set linear velocity (`vector`) and angular velocity (`spin`) outright.
  ///
  /// For a character, `vector` is instead the velocity it asks for, gravity
  /// included, relative to whatever it stands on, and `spin` is ignored.
  /// Where it actually goes is up to what is in the way.
  ORBLIT_PHYSICS_VELOCITY = 4,

  /// Add an impulse `vector`, applied at world point `spin`. An impulse away
  /// from the centre of mass spins the body, which is the whole reason the
  /// point is here rather than assumed.
  ORBLIT_PHYSICS_IMPULSE = 5,

  /// Wake it, whether or not anything touched it.
  ORBLIT_PHYSICS_WAKE = 6,

  /// Make an existing kinematic body a character, or change one that already
  /// is. `size[0]` is the tallest step it walks up, in metres; `size[1]` the
  /// steepest slope it stands on, in radians; `size[2]` the gap it keeps from
  /// everything, in metres, never less than a millimetre; `size[3]` the most
  /// it pushes with, in newtons.
  ///
  /// A character is a body that walks. It is moved by sweeping its shape
  /// through the world rather than by the solver, so it slides along walls,
  /// climbs steps and stands on slopes it could not climb, rather than
  /// bouncing off them. It stands on whatever is under it and goes where that
  /// goes, pushes dynamic bodies no harder than it is allowed to, and is
  /// pushed only by kinematic ones.
  ///
  /// It does not fall on its own. Gravity is part of what it is asked for, so
  /// a game that stops asking has a character that hangs in the air, and a
  /// game that wants a jump asks for one. Read its footing after each step to
  /// know what survived of what it asked for.
  ///
  /// Ignored for anything but a kinematic body, and for a plane.
  ORBLIT_PHYSICS_CHARACTER = 7,
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
  /// CHARACTER: step height, steepest slope, skin and strength.
  float size[4];

  /// CREATE, PLACE: where.
  float at[3];

  /// CREATE, PLACE: the orientation, xyzw. All zeroes is read as identity, so
  /// a caller that does not care about rotation can leave it alone.
  float rotation[4];

  /// VELOCITY: metres per second, or for a character what it asks for.
  /// IMPULSE: the impulse, newton-seconds.
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

  /// A joint gave way. `a` is the joint, not a body, and `b` is zero; `at` is
  /// where it was, and `force` what broke it: the force in newtons, or the
  /// torque in newton-metres if that was what went past its limit.
  ORBLIT_PHYSICS_BROKE = 5,
} OrblitPhysicsEventKind;

/// Something that happened during a step and is worth telling the caller.
///
/// Began and ended, never "is touching": a game reacts to the change, and a
/// caller that wants the standing set keeps one, which is cheaper than this
/// library rebuilding that set every tick for the callers that do not.
typedef struct {
  uint32_t kind; ///< An OrblitPhysicsEventKind.
  uint32_t _pad;

  /// The bodies. For SLEPT and WOKE only `a` means anything, and for BROKE
  /// `a` is a joint.
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

// ---------------------------------------------------------------- casting ---

/// A shape moved through the world in a straight line, to find out what it
/// would meet first.
///
/// This is how a character asks whether it can take a step, how a camera finds
/// the wall it must not go through, and how a hitscan weapon finds what it
/// hit. It moves nothing and changes nothing: a world is the same after a cast
/// as it was before one.
typedef struct {
  /// An OrblitPhysicsShapeKind, or zero for a ray.
  ///
  /// A ray is a sphere of no size and behaves exactly like one, so a caller
  /// that wants one does not have to invent a radius small enough not to
  /// matter — which is a number that does not exist.
  uint32_t shape;

  /// Which layers the cast is in, and which it wants to hit. The same rule
  /// bodies follow: it meets a body when either cares about the other.
  uint32_t layerIs;
  uint32_t layerCares;
  uint32_t _pad;

  /// The shape's dimensions, read as its kind describes. Ignored for a ray.
  /// A plane cannot be cast: an infinite surface is touching everything
  /// already.
  float size[4];

  /// Where it starts, and how it is turned. All zeroes is read as identity.
  float from[3];
  float rotation[4];

  /// Which way it goes. Normalised here, so a caller may pass a movement
  /// vector as it stands.
  float direction[3];

  /// How far along `direction` to look, in metres.
  float distance;

  /// A body to pass straight through. Zero hits everything.
  ///
  /// This is nearly always the body doing the casting, which would otherwise
  /// hit itself at no distance at all and never see anything else.
  OrblitPhysicsId ignore;
} OrblitPhysicsCast;

typedef struct {
  /// What was hit.
  OrblitPhysicsId body;

  /// Where they met, and which way out of the body that was hit — the same
  /// rule a touch event's normal follows.
  float at[3];
  float normal[3];

  /// How far along the cast's direction, in metres.
  float distance;

  /// The cast overlapped that body before it moved at all. `distance` is zero
  /// and `at` is somewhere inside it rather than where the cast came in,
  /// because for something already inside there is no such place.
  ///
  /// Worth checking rather than ignoring: a character whose capsule starts
  /// inside a wall wants to be pushed out of it, not stopped at a distance of
  /// nought and left there.
  bool started;
  bool _reserved[3];
} OrblitPhysicsHit;

/// Finds the nearest thing `cast` would meet, and returns whether it met
/// anything. `out` is untouched on a miss.
///
/// Nearest, not every: a caller that wants the list can cast again past what
/// it found. Sleeping and static bodies are hit like any other — a query asks
/// where things are, and a crate that has settled is still in the way.
bool orblit_physics_cast(const OrblitPhysics *physics,
                         const OrblitPhysicsCast *cast, OrblitPhysicsHit *out);

// ----------------------------------------------------------------- ground ---

/// Ground as heights on a grid, laid as one static body.
///
/// Sample (c, r) is `heights[r * columns + c]` and sits at
/// `at + (c * spacing, height, r * spacing)`. Each square of four samples is
/// two triangles, split along the diagonal from (c, r) to (c + 1, r + 1).
///
/// Everything under the surface is ground, however far down: something that
/// has sunk in is pushed up and out, never down through, so ground can be
/// raised under a crate that has gone to sleep on it.
///
/// A height that is not a finite number is a hole, and every triangle with
/// that sample as a corner is missing. Things fall through a hole, and roll
/// off its rim.
typedef struct {
  OrblitPhysicsId id;

  /// `columns * rows` heights in metres, row after row. Copied, not kept.
  const float *heights;
  uint32_t columns;
  uint32_t rows;

  /// Metres between neighbouring samples, the same both ways.
  float spacing;

  /// The outermost ring of samples is the neighbours' ground, not this
  /// field's. It is never stood on; it only says how the ground carries on
  /// past this field's edge.
  ///
  /// Ground streamed in pieces is laid one field per piece, side by side, and
  /// those have to behave as one ground. A ball on the ridge of a hill whose
  /// top runs along the line between two pieces is held up by whichever way
  /// both slopes agree on, and without a margin neither piece knows which way
  /// the other slopes. Without one, the edge of a grid is taken to carry on
  /// as it was going, which is right everywhere but along a ridge or a
  /// valley.
  ///
  /// A hole in the margin is no ground past the edge: the edge is a rim.
  bool margin;
  bool _reserved[3];

  /// Where sample (0, 0) is, margin included. Ground is never turned.
  float at[3];

  float friction;    ///< As for CREATE.
  float restitution; ///< As for CREATE.

  /// As for CREATE.
  uint32_t layerIs;
  uint32_t layerCares;
  uint32_t _pad;
} OrblitPhysicsGround;

/// Lays ground, and returns whether it was laid.
///
/// Laying it again under the same id replaces it, which is how an edit to
/// the heights reaches the world: lay the piece again. Anything already under
/// that id is replaced, ground or not, and keeps its joints, so a rope tied to
/// a hill stays tied when the hill is edited. Everything over the field is
/// woken, so a crate on a hill that was just lowered falls with it rather than
/// hanging asleep where the hill was. Destroying it, as destroying any body,
/// wakes nothing it was only touching: a crate asleep on ground streamed out
/// from under it stays put until something wakes it.
///
/// False, and nothing changed, for a zero id, no heights, a spacing that is
/// not a positive number, or a grid with no square of its own: at least two
/// samples each way, four with a margin.
bool orblit_physics_ground(OrblitPhysics *physics,
                           const OrblitPhysicsGround *ground);

// ------------------------------------------------------------- characters ---

/// What a character stood on at the end of a step, and what it was left with.
///
/// This is the half of a character the game reads back. It asked for a
/// velocity; this is what the world made of it, and what the next request
/// should start from.
typedef struct {
  /// What is under it, walkable or not. Zero for nothing within reach.
  OrblitPhysicsId ground;

  /// Which way that surface faces. For a step's edge this is the top of the
  /// step rather than the corner it touched, because "which way is the floor"
  /// is the question a game is asking.
  float normal[3];

  /// What survived of the velocity it asked for, relative to its ground. A
  /// wall takes away the part going into it, and a floor takes away the part
  /// going down, which is why the next request adds gravity to this rather
  /// than to the last one: a character standing still does not accumulate a
  /// fall it is not taking.
  float velocity[3];

  /// How far its ground moved it last step, per second, and how fast its
  /// ground turned it about up, in radians per second. Kept apart from
  /// `velocity` so an animation can play the walk it asked for rather than
  /// the ride it was given, and a camera can turn with a turntable.
  float carried[3];
  float turning;

  /// Standing on something it could stand on. False in the air, on a slope
  /// steeper than it may climb, and on the way up a jump.
  bool grounded;
  bool _reserved[7];
} OrblitPhysicsFooting;

/// Writes the footing of each of `ids` into `out[i]`.
///
/// An id that is not a character is skipped and its slot left exactly as it
/// was, like `orblit_physics_read`. Returns how many were written.
uint32_t orblit_physics_footing(const OrblitPhysics *physics,
                                const OrblitPhysicsId *ids, uint32_t count,
                                OrblitPhysicsFooting *out);

// ----------------------------------------------------------------- joints ---
//
// A joint holds two bodies together, or one body to the world, in some ways
// and not others. A door is a hinge: its edge stays on the frame and it turns
// about one axis. A drawer is a slider. A rope is a distance that may shorten
// and never lengthen. A shoulder is a cone.
//
// Every joint is made where the bodies stand. It is given one frame in the
// world — a point, and a rotation whose x axis is the axis that matters — and
// it keeps that frame on each body from then on. What it holds is how the
// bodies stood when it was made, so a limit is measured from there: a hinge
// made with its door shut has an angle of nought when the door is shut.
//
// Every axis and every measure is `a`'s: how `b` has moved and turned as `a`
// sees it, along axes that turn when `a` does. A slider on a body that is
// spinning slides along a line that spins with it. Either may be the world,
// and which one is the world decides which way everything reads: with the
// world as `a`, a door's angle is the door's; with the world as `b`, it is
// the world's as the door sees it, the other way round.
//
// Joints are solved alongside contacts, in the same passes, so a chain lying
// on the ground is held together and held up at once rather than one fighting
// the other. Two bodies joined together do not collide with each other unless
// the joint asks them to, because a joint's two bodies nearly always overlap
// where it is.
//
// Joined bodies sleep together and wake together. A chain does not go to sleep
// one link at a time, and touching its end wakes all of it.

typedef enum {
  /// Holds `b` exactly where it is relative to `a`: welded.
  ORBLIT_PHYSICS_JOINT_FIXED = 1,

  /// Holds the two points together and lets the bodies turn any way about
  /// them. A pendulum, a ball and socket with no limit.
  ORBLIT_PHYSICS_JOINT_POINT = 2,

  /// Holds the points together and lets `b` turn about the frame's x axis
  /// only. Limited by `low[3]` and `high[3]`, the angle `b` has turned about
  /// x relative to `a`, right-handed, in radians, within a half turn either
  /// way. Takes a motor.
  ORBLIT_PHYSICS_JOINT_HINGE = 3,

  /// Lets `b` move along the frame's x axis only, never turning. Limited by
  /// `low[0]` and `high[0]`, how far `b` has moved along x, in metres. Takes
  /// a motor.
  ORBLIT_PHYSICS_JOINT_SLIDER = 4,

  /// Keeps a point on `a`, `at`, and a point on `b`, `to`, a distance apart,
  /// with the bodies free to turn. Limited, it is a range: `low[0]` to
  /// `high[0]` in metres, and a rope is nought to its length. Unlimited, it is
  /// a rod the length it was made.
  ORBLIT_PHYSICS_JOINT_DISTANCE = 5,

  /// Holds the points together and lets `b`'s x axis swing within `swing`
  /// radians of `a`'s, in any direction. Limited by `low[3]` and `high[3]`,
  /// it also bounds the twist about x. A shoulder or a hip.
  ORBLIT_PHYSICS_JOINT_CONE = 6,

  /// Each of the six ways `b` can move relative to `a` set on its own: free
  /// unless limited, and locked where a limit's low is its high. For whatever
  /// the other kinds do not cover.
  ///
  /// Turning is measured as a twist about x and then a swing, and the swing's
  /// turn about y and about z are limited separately. For a joint that stays
  /// within a quarter turn that is the angle it looks like; further, the
  /// three stop being independent, which is true of any three angles.
  ORBLIT_PHYSICS_JOINT_SIX_AXIS = 7,
} OrblitPhysicsJointKind;

/// How to make a joint.
///
/// Fields a kind does not name are ignored, so zeroing the struct and filling
/// in what matters is always correct: a zeroed limit is no limit, a zeroed
/// motor is no motor, and a zeroed strength never breaks.
typedef struct {
  /// Whatever the caller calls the joint. Joints are named apart from bodies,
  /// so a joint may share a number with a body. Zero is never a joint.
  OrblitPhysicsId id;

  /// The two bodies. Either may be zero, which is the world, but not both.
  OrblitPhysicsId a;
  OrblitPhysicsId b;

  uint32_t kind; ///< An OrblitPhysicsJointKind.

  /// Which of the six axes `low` and `high` apply to, one bit each: bits 0 to
  /// 2 for moving along x, y and z, bits 3 to 5 for turning about them. An
  /// axis whose bit is clear is free, or whatever its kind makes it.
  uint32_t limited;

  /// The joint's point, and on every kind but DISTANCE its point on `b` too,
  /// in the world.
  float at[3];

  /// The joint's frame in the world, xyzw. Its x axis is the one a hinge
  /// turns about, a slider slides along and a cone points along. All zeroes
  /// is identity.
  float rotation[4];

  /// DISTANCE: the point on `b`, in the world.
  float to[3];

  /// The least and most of each axis, in the order `limited` names them:
  /// metres along x, y and z, then radians about them. Kept in order, so a
  /// low above its high is read as the other way round.
  float low[6];
  float high[6];

  /// CONE: the widest `b`'s x axis may swing from `a`'s, in radians.
  float swing;

  /// HINGE and SLIDER: a motor, driving `b` relative to `a` at `speed`, in
  /// radians or metres a second, and never harder than `strength`, in
  /// newton-metres or newtons. A strength of zero is no motor. A speed of
  /// zero with a small strength is friction in the joint.
  float speed;
  float strength;

  /// Past this force in newtons, or this torque in newton-metres, the joint
  /// breaks: it is removed, and a BROKE event says so. Zero never breaks.
  float breakingForce;
  float breakingTorque;

  /// Whether the two bodies still collide with each other.
  bool collide;
  bool _reserved[3];
} OrblitPhysicsJoint;

/// How a joint stands, and how hard it held on the last step.
typedef struct {
  /// Where `b`'s point is from `a`'s, along the axes of `a`'s frame, in
  /// metres. For DISTANCE, `offset[0]` is the distance between them.
  float offset[3];

  /// How far `b`'s frame has turned from `a`'s, in radians: the twist about
  /// x, then the swing's turn about y and about z.
  float angles[3];

  /// The force and torque it held with on the last step, in newtons and
  /// newton-metres. This is the number to read before choosing where it
  /// should break.
  float force;
  float torque;
} OrblitPhysicsJointState;

/// Makes a joint, and returns whether it was made. Both bodies are woken.
///
/// False, and nothing changed, for a zero id or one already a joint, an `a` or
/// `b` that is neither zero nor a body, a joint from a body to itself or from
/// the world to itself, a kind that is not one, a NaN in any field, or a point
/// or rotation that is not finite.
bool orblit_physics_join(OrblitPhysics *physics, const OrblitPhysicsJoint *joint);

/// Removes a joint, waking both its bodies so what it held up falls. False if
/// there was no such joint.
///
/// Destroying a body removes its joints the same way, silently: no BROKE
/// event, because nothing broke.
bool orblit_physics_unjoin(OrblitPhysics *physics, OrblitPhysicsId joint);

/// Writes how joint `joint` stands into `out`. False if there is no such joint.
bool orblit_physics_joint(const OrblitPhysics *physics, OrblitPhysicsId joint,
                          OrblitPhysicsJointState *out);

#ifdef __cplusplus
}
#endif

#endif // ORBLIT_PHYSICS_H
