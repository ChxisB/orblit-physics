// The Dart face of the engine: lifetimes, a command buffer, and turning the
// ABI's flat structs into something worth reading.
//
// Commands are collected rather than sent one at a time. A tick creates a
// handful of bodies and shoves a handful more, and crossing into native code
// once for each of them would spend more time on the crossing than on the
// physics. They are flushed before anything reads, so from the caller's side
// a command still takes effect the moment it is given.

import 'dart:ffi';
import 'dart:math' show pi;
import 'dart:typed_data';

import 'package:ffi/ffi.dart';

import 'bindings.dart' as native;

/// How the solver treats a body.
enum PhysicsMotion {
  /// Never moves. The floor, the walls.
  fixed(0),

  /// Moves exactly where it is put, and pushes anything in the way without
  /// being pushed back. Lifts, doors, turntables. Something that walks wants
  /// `Physics.addCharacter` instead, which is a driven body that looks where
  /// it is going.
  driven(1),

  /// Falls, is pushed, and pushes back.
  free(2);

  const PhysicsMotion(this.code);
  final int code;
}

/// What a body is, geometrically.
final class Shape {
  const Shape._(this.kind, this.x, this.y, this.z, this.w);

  /// A ball of `radius` metres.
  const Shape.sphere(double radius) : this._(1, radius, 0.0, 0.0, 0.0);

  /// A box reaching `halfX` by `halfY` by `halfZ` from its centre — halves,
  /// not widths, because everything the solver does with them is measured
  /// from the middle outwards.
  const Shape.box(double halfX, double halfY, double halfZ)
    : this._(2, halfX, halfY, halfZ, 0.0);

  /// A cylinder of `radius` with `halfHeight` metres of straight either side
  /// of the middle, capped with a hemisphere at each end, standing along the
  /// body's own y. It is `2 * (halfHeight + radius)` tall in all, and it is
  /// what a character or a limb is made of, because it has no corner to catch
  /// on a seam between two floor tiles.
  const Shape.capsule(double radius, double halfHeight)
    : this._(4, radius, halfHeight, 0.0, 0.0);

  /// An endless flat surface: everything behind the normal `nx, ny, nz` at
  /// `offset` along it is solid. Never dynamic — a half-space has no centre
  /// to spin about.
  const Shape.plane(double nx, double ny, double nz, {double offset = 0.0})
    : this._(3, nx, ny, nz, offset);

  /// Which kind it is, as the ABI numbers them.
  final int kind;

  /// The ABI's four size floats, read as the kind describes: a radius, three
  /// half extents, a radius and a half height, or a normal and an offset.
  final double x;
  final double y;
  final double z;
  final double w;
}

/// Which layers a body is in, and which it wants to be told about.
///
/// A pair interacts when either cares about the other, not when both do, so a
/// bullet that cares about walls hits a wall that cares about nothing.
final class Layers {
  const Layers({this.is_ = 1, this.cares = 0xFFFFFFFF});

  final int is_;
  final int cares;

  static const Layers everything = Layers();
}

/// What happened during a step.
enum PhysicsEventKind {
  touchBegan,
  touchEnded,
  slept,
  woke,

  /// A joint gave way and is gone. The event's `a` is the joint, not a body.
  broke,
}

final class PhysicsEvent {
  const PhysicsEvent({
    required this.kind,
    required this.a,
    required this.b,
    required this.at,
    required this.normal,
    required this.force,
  });

  final PhysicsEventKind kind;

  /// The body it happened to, and for a touch the other one. The pair is
  /// always given smaller id first, so a caller can key on it without
  /// sorting. For [PhysicsEventKind.broke], `a` is the joint and `b` is zero.
  final int a;
  final int b;

  /// Where they met, and which way out of `b` towards `a`. Zero for anything
  /// that is not a touch beginning, except that a joint breaking says where
  /// it was.
  final List<double> at;
  final List<double> normal;

  /// How hard, in newton-seconds along the normal. The number a collision
  /// sound scales with. For a joint breaking, what broke it: the force in
  /// newtons, or the torque in newton-metres if that went past its limit.
  final double force;
}

/// What a cast ran into.
final class PhysicsHit {
  const PhysicsHit({
    required this.body,
    required this.at,
    required this.normal,
    required this.distance,
    required this.started,
  });

  /// The body it met.
  final int body;

  /// Where they touched, and which way out of the body towards the cast.
  final List<double> at;
  final List<double> normal;

  /// How far along the cast's direction it got, in metres.
  final double distance;

  /// The cast started inside this body rather than running into it, so
  /// `distance` is zero and `at` is the deepest point of the overlap. Worth
  /// telling apart: a character whose capsule begins inside a wall wants to be
  /// pushed out of it, not stopped where it already is.
  final bool started;
}

/// What a character stood on at the end of the last step, and what survived
/// of what it asked for.
///
/// This is the half of a character a game reads back. It asked for a
/// velocity; this is what the world made of it, and the next request starts
/// from here rather than from the last one — a character standing still does
/// not pile up a fall it is not taking.
final class PhysicsFooting {
  const PhysicsFooting({
    required this.ground,
    required this.normal,
    required this.velocity,
    required this.carried,
    required this.turning,
    required this.grounded,
  });

  /// What is under it, walkable or not, within a step's reach. Zero for
  /// nothing at all.
  final int ground;

  /// Which way that surface faces. On the edge of a step this is the top of
  /// the step rather than the corner it is touching, because "which way is
  /// the floor" is the question being asked.
  final List<double> normal;

  /// What is left of the velocity it asked for, relative to its ground. A wall
  /// takes away the part going into it, and a floor takes away everything up
  /// and down, so a character walking up a ramp reads back a flat speed.
  final List<double> velocity;

  /// How far its ground moved it last step, per second, and how fast its
  /// ground turned it about up, in radians per second. Apart from `velocity`
  /// so an animation plays the walk it asked for rather than the ride it was
  /// given, and a camera can turn with a turntable.
  final List<double> carried;
  final double turning;

  /// Standing on something it may stand on. False in the air, on a slope
  /// steeper than it may climb, and on the way up a jump.
  final bool grounded;
}

/// Which kind of joint, as the ABI numbers them.
///
/// Named for the package, as [PhysicsMotion] is, because a scene document has
/// a `JointKind` of its own and a game imports both.
enum PhysicsJointKind {
  fixed(1),
  point(2),
  hinge(3),
  slider(4),
  distance(5),
  cone(6),
  sixAxis(7);

  const PhysicsJointKind(this.code);
  final int code;
}

/// The least and most a joint lets one of its axes go: metres along it, or
/// radians about it, measured from where the bodies stood when it was made.
final class JointLimit {
  /// Kept in order, so a `low` above its `high` is read the other way round.
  const JointLimit(this.low, this.high);

  /// Held exactly at `at`, which is where it was made unless said otherwise.
  const JointLimit.locked([double at = 0.0]) : this(at, at);

  final double low;
  final double high;
}

/// What a joint holds, apart from where it is.
///
/// Every joint holds some of the six ways its second body can move relative
/// to its first — along the three axes of its frame and about them — each
/// within a range, and a kind is only which ways and how far. The value says
/// nothing about where it goes, so one elbow serves both arms: [Physics.join]
/// says where.
///
/// Every axis and every measure is the first body's, and turns when it does.
/// Joined to the world, it is the world that moves as that body sees it, so a
/// body falling down a slider reads as the world sliding up.
final class Joint {
  const Joint._(
    this.kind, {
    this.alongX,
    this.alongY,
    this.alongZ,
    this.aboutX,
    this.aboutY,
    this.aboutZ,
    this.swing = 0.0,
    this.speed = 0.0,
    this.strength = 0.0,
  });

  /// Holds the second body exactly where it is relative to the first: welded.
  const Joint.fixed() : this._(PhysicsJointKind.fixed);

  /// Holds the two points together and lets the bodies turn any way about
  /// them. A pendulum, or a ball and socket with no limit.
  const Joint.point() : this._(PhysicsJointKind.point);

  /// Holds the points together and lets the second body turn about the
  /// frame's x axis only: a door, a wheel, a knee. `limit` is the angle it
  /// may turn, in radians, within a half turn either way.
  ///
  /// A motor turns it at `speed` radians a second, never harder than
  /// `strength` newton-metres. No strength is no motor, and a speed of nought
  /// with a little strength is friction in the joint.
  const Joint.hinge({
    JointLimit? limit,
    double speed = 0.0,
    double strength = 0.0,
  }) : this._(
         PhysicsJointKind.hinge,
         aboutX: limit,
         speed: speed,
         strength: strength,
       );

  /// Lets the second body move along the frame's x axis only, never turning:
  /// a drawer, a piston, a lift on a rail. `limit` is how far, in metres.
  ///
  /// A motor moves it at `speed` metres a second, never harder than
  /// `strength` newtons.
  const Joint.slider({
    JointLimit? limit,
    double speed = 0.0,
    double strength = 0.0,
  }) : this._(
         PhysicsJointKind.slider,
         alongX: limit,
         speed: speed,
         strength: strength,
       );

  /// Keeps a point on each body a distance apart, with both free to turn.
  ///
  /// With no limit it is a rod as long as the points were apart when it was
  /// made. With one it is a range, in metres: `JointLimit(0, 2)` is a two
  /// metre rope, slack until it is taut.
  const Joint.distance({JointLimit? limit})
    : this._(PhysicsJointKind.distance, alongX: limit);

  /// Holds the points together and lets the second body's x axis swing up to
  /// `swing` radians from the first's, in any direction: a shoulder or a hip.
  /// `twist` bounds how far it turns about that axis as well.
  const Joint.cone({required double swing, JointLimit? twist})
    : this._(PhysicsJointKind.cone, aboutX: twist, swing: swing);

  /// Each of the six ways set on its own: free unless limited, and locked
  /// where a limit's low is its high. For whatever the other kinds do not
  /// cover.
  ///
  /// Turning is measured as a twist about x and then a swing, whose turns
  /// about y and z are limited separately. Within a quarter turn that is the
  /// angle it looks like; further, the three stop being independent, which is
  /// true of any three angles.
  const Joint.sixAxis({
    JointLimit? alongX,
    JointLimit? alongY,
    JointLimit? alongZ,
    JointLimit? aboutX,
    JointLimit? aboutY,
    JointLimit? aboutZ,
  }) : this._(
         PhysicsJointKind.sixAxis,
         alongX: alongX,
         alongY: alongY,
         alongZ: alongZ,
         aboutX: aboutX,
         aboutY: aboutY,
         aboutZ: aboutZ,
       );

  final PhysicsJointKind kind;

  /// The range of each of the six ways, or null where the kind leaves it
  /// alone: metres along the frame's axes, then radians about them.
  final JointLimit? alongX;
  final JointLimit? alongY;
  final JointLimit? alongZ;
  final JointLimit? aboutX;
  final JointLimit? aboutY;
  final JointLimit? aboutZ;

  /// A cone's widest swing, in radians.
  final double swing;

  /// A hinge's or a slider's motor.
  final double speed;
  final double strength;

  /// The six limits, in the order the ABI names them.
  List<JointLimit?> get limits => [
    alongX,
    alongY,
    alongZ,
    aboutX,
    aboutY,
    aboutZ,
  ];
}

/// How a joint stands, and how hard it held on the last step.
final class JointState {
  const JointState({
    required this.offset,
    required this.angles,
    required this.force,
    required this.torque,
  });

  /// Where the second body's point is from the first's, along the first's
  /// axes, in metres. For a distance joint, `offset[0]` is the distance.
  final List<double> offset;

  /// How far the second body has turned from the first, in radians: the
  /// twist about x, then the swing's turn about y and about z.
  final List<double> angles;

  /// The force and torque it held with on the last step, in newtons and
  /// newton-metres. The numbers to read before choosing where it breaks.
  final double force;
  final double torque;
}

/// How a world behaves. Anything left null is the engine's own default, so
/// there is one place the numbers live and it is not this file.
final class PhysicsSettings {
  const PhysicsSettings({
    this.gravity,
    this.velocitySteps,
    this.positionSteps,
    this.slop,
    this.stiffness,
    this.bounceThreshold,
    this.sleepSpeed,
    this.sleepAfter,
    this.sleeping,
  });

  final List<double>? gravity;
  final int? velocitySteps;
  final int? positionSteps;
  final double? slop;
  final double? stiffness;
  final double? bounceThreshold;
  final double? sleepSpeed;
  final double? sleepAfter;
  final bool? sleeping;
}

/// A rigid body world.
///
/// Bodies are named by whatever the caller calls them — an Orblit entity
/// handle, or any number that is not zero. The world never interprets one; it
/// only matches them, which is what lets this package know nothing about the
/// entity store it is usually driven by.
class Physics {
  Physics({PhysicsSettings settings = const PhysicsSettings()}) {
    final configured = calloc<native.OrblitPhysicsSettings>();
    try {
      native.physicsDefaults(configured);
      final it = configured.ref;
      final gravity = settings.gravity;
      if (gravity != null) {
        it.gravity[0] = gravity[0];
        it.gravity[1] = gravity[1];
        it.gravity[2] = gravity[2];
      }
      if (settings.velocitySteps != null) {
        it.velocitySteps = settings.velocitySteps!;
      }
      if (settings.positionSteps != null) {
        it.positionSteps = settings.positionSteps!;
      }
      if (settings.slop != null) it.slop = settings.slop!;
      if (settings.stiffness != null) it.stiffness = settings.stiffness!;
      if (settings.bounceThreshold != null) {
        it.bounceThreshold = settings.bounceThreshold!;
      }
      if (settings.sleepSpeed != null) it.sleepSpeed = settings.sleepSpeed!;
      if (settings.sleepAfter != null) it.sleepAfter = settings.sleepAfter!;
      if (settings.sleeping != null) it.sleeping = settings.sleeping!;

      _physics = native.physicsCreate(configured);
    } finally {
      calloc.free(configured);
    }
    if (_physics == nullptr) {
      throw StateError('Could not create an Orblit physics world.');
    }
    _scratch = calloc<Float>(7);
    _count = calloc<Uint32>();
  }

  Pointer<native.OrblitPhysicsStruct> _physics = nullptr;
  late final Pointer<Float> _scratch;
  late final Pointer<Uint32> _count;

  Pointer<native.OrblitPhysicsCommand> _commands = nullptr;
  int _room = 0;
  int _pending = 0;

  // --- commands ------------------------------------------------------------

  /// Adds a body. Ignored if `id` is zero or already here.
  void add(
    int id, {
    required Shape shape,
    PhysicsMotion motion = PhysicsMotion.free,
    List<double> at = const [0.0, 0.0, 0.0],
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
    double mass = 1.0,
    double friction = 0.5,
    double restitution = 0.0,
    double linearDamping = 0.05,
    double angularDamping = 0.05,
    Layers layers = Layers.everything,
    bool asleep = false,
  }) {
    final command = _next(1, id);
    command.shape = shape.kind;
    command.size[0] = shape.x;
    command.size[1] = shape.y;
    command.size[2] = shape.z;
    command.size[3] = shape.w;
    _write3(command.at, at);
    for (var i = 0; i < 4; i++) {
      command.rotation[i] = rotation[i];
    }
    command.motion = motion.code;
    command.mass = mass;
    command.friction = friction;
    command.restitution = restitution;
    command.damping[0] = linearDamping;
    command.damping[1] = angularDamping;
    command.layerIs = layers.is_;
    command.layerCares = layers.cares;
    command.asleep = asleep;
  }

  /// Adds a character: a body that walks.
  ///
  /// It is moved by sweeping its shape through the world rather than by the
  /// solver, so it slides along walls instead of bouncing off them, climbs
  /// anything up to `stepHeight` metres tall, stands on slopes up to
  /// `steepest` radians and slides down steeper ones, and rides whatever it is
  /// standing on. It pushes free bodies it walks into with at most `strength`
  /// newtons, and is pushed by nothing but driven ones: a door closing on it
  /// shoves it aside, a crate rolling into it stops. It keeps `skin` metres
  /// from everything, which is what stops it snagging on a seam between two
  /// floor tiles.
  ///
  /// `at` is its centre, as it is for every body. The default capsule is 1.8
  /// metres tall, so it stands with its centre 0.9 above its feet, and a skin
  /// above that.
  ///
  /// Tell it where to go with [drive], every step, and read what came of it
  /// with [footingOf]. It does not fall on its own: gravity is part of what it
  /// asks for, which is what lets a game decide what a jump is.
  void addCharacter(
    int id, {
    Shape shape = const Shape.capsule(0.3, 0.6),
    List<double> at = const [0.0, 0.0, 0.0],
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
    double stepHeight = 0.3,
    double steepest = pi / 4,
    double skin = 0.01,
    double strength = 500.0,
    double friction = 0.5,
    Layers layers = Layers.everything,
  }) {
    add(
      id,
      shape: shape,
      motion: PhysicsMotion.driven,
      at: at,
      rotation: rotation,
      friction: friction,
      layers: layers,
    );
    final command = _next(7, id);
    command.size[0] = stepHeight;
    command.size[1] = steepest;
    command.size[2] = skin;
    command.size[3] = strength;
  }

  /// Lays ground as heights on a grid, as one fixed body named `id`, and
  /// answers whether it was laid.
  ///
  /// Sample (c, r) is `heights[r * columns + c]` and sits at
  /// `at + (c * spacing, height, r * spacing)`; each square of four samples is
  /// two triangles, split from (c, r) to (c + 1, r + 1). Everything under the
  /// surface is ground, so something that has sunk in comes up and out rather
  /// than through. A height that is not a finite number is a hole: things fall
  /// through it and roll off its rim.
  ///
  /// Laying it again under the same id replaces it — whatever was there,
  /// ground or not, keeping its joints — and wakes everything over it, so an
  /// edit reaches the world by laying the piece again, and a crate on a hill
  /// that was lowered falls with it. It is taken away with [remove], which, as
  /// for any body, wakes nothing it was only touching: a crate asleep on
  /// ground streamed out from under it stays where it was until something
  /// wakes it, and is still there when the ground comes back.
  ///
  /// Ground streamed in pieces is laid one field per piece, side by side.
  /// With `margin` the outermost ring of `heights` is the neighbouring
  /// pieces' samples, never stood on, which is how a ridge running along the
  /// line between two pieces holds a ball up the way one field would. `at`
  /// is then where the margin's corner is, a spacing out from the field's own.
  ///
  /// False, and nothing changed, for a zero id, a spacing that is not a
  /// positive number, fewer heights than `columns * rows`, or a grid with no
  /// square of its own: at least two samples each way, four with a margin.
  bool layGround(
    int id, {
    required List<double> heights,
    required int columns,
    required int rows,
    double spacing = 1.0,
    bool margin = false,
    List<double> at = const [0.0, 0.0, 0.0],
    double friction = 0.5,
    double restitution = 0.0,
    Layers layers = Layers.everything,
  }) {
    _requireAlive();
    // Sent now rather than queued, so it lands after whatever was queued
    // before it: a body added under this id a moment ago is the one replaced.
    _flush();
    if (columns < 0 || rows < 0 || heights.length < columns * rows) {
      return false;
    }
    final count = columns * rows;
    final samples = calloc<Float>(count == 0 ? 1 : count);
    final ground = calloc<native.OrblitPhysicsGround>();
    try {
      samples.asTypedList(count).setAll(0, heights.take(count));
      final it = ground.ref;
      it.id = id;
      it.heights = samples;
      it.columns = columns;
      it.rows = rows;
      it.spacing = spacing;
      it.margin = margin;
      _write3(it.at, at);
      it.friction = friction;
      it.restitution = restitution;
      it.layerIs = layers.is_;
      it.layerCares = layers.cares;
      return native.physicsGround(_alive, ground);
    } finally {
      calloc.free(samples);
      calloc.free(ground);
    }
  }

  /// Takes a body out of the world. Anything that was touching it is told the
  /// touch has ended on the next step. Its joints go with it, silently —
  /// nothing broke — and whatever they held is woken, so it falls.
  void remove(int id) => _next(2, id);

  /// Puts a body exactly here, forgetting how it was moving. The teleport,
  /// not the shove.
  void place(
    int id, {
    required List<double> at,
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
  }) {
    final command = _next(3, id);
    _write3(command.at, at);
    for (var i = 0; i < 4; i++) {
      command.rotation[i] = rotation[i];
    }
  }

  /// Sets how a body is moving, outright. This is how a driven body is told
  /// where to go.
  ///
  /// For a character it is a request rather than an order: `velocity` is
  /// where it would like to go, relative to whatever it is standing on and
  /// with gravity in it, and the world decides how much of that it gets.
  /// `spin` is ignored: which way a character faces is the game's to keep,
  /// since a standing capsule is the same shape whichever way it looks.
  void drive(
    int id, {
    List<double> velocity = const [0.0, 0.0, 0.0],
    List<double> spin = const [0.0, 0.0, 0.0],
  }) {
    final command = _next(4, id);
    _write3(command.vector, velocity);
    _write3(command.spin, spin);
  }

  /// Hits a body with `impulse` newton-seconds at world point `at`, which
  /// defaults to its own centre. Away from the centre it spins as well as
  /// moves, which is the whole reason the point is a parameter.
  void push(int id, {required List<double> impulse, List<double>? at}) {
    // Found before the command is started: looking a body up flushes, and a
    // flush half way through filling one in would send it unfinished.
    final point = at ?? _centreOf(id);
    final command = _next(5, id);
    _write3(command.vector, impulse);
    _write3(command.spin, point);
  }

  /// Wakes a body, whether or not anything touched it.
  void wake(int id) => _next(6, id);

  // --- joints --------------------------------------------------------------

  /// Joins body `a` to body `b` as `joint` says, and answers whether it was
  /// made. Both bodies are woken.
  ///
  /// Either may be zero, for the world, but not both. Every measure is `b`
  /// as `a` sees it, so which one is the world decides the sense: hang a
  /// door with the world as `a` and its angle is the door's; with the world
  /// as `b` it is the world's as the door sees it, the other way round.
  ///
  /// It is made where the bodies stand: `at` is the joint's point in the
  /// world, and `rotation` its frame, whose x axis is the one a hinge turns
  /// about, a slider slides along and a cone points along. What it holds is
  /// how they stood, so a hinge made with its door shut reads nought when the
  /// door is shut. A distance joint keeps `at` on `a` a distance from `to` on
  /// `b`; every other kind ignores `to`.
  ///
  /// Past `breakingForce` newtons, or `breakingTorque` newton-metres, it
  /// breaks: it is removed, and a [PhysicsEventKind.broke] event says so.
  /// Nought never breaks. The two bodies do not collide with each other
  /// unless `collide` says they do, because a joint's bodies nearly always
  /// overlap where it is.
  ///
  /// Joint ids are apart from body ids, so a joint may share a number with a
  /// body. False, and nothing changed, for a zero id or one already a joint,
  /// an `a` or `b` that is neither zero nor a body, a body or the world joined
  /// to itself, or a number that is not a number.
  bool join(
    int id,
    Joint joint, {
    required int a,
    int b = 0,
    required List<double> at,
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
    List<double>? to,
    double breakingForce = 0.0,
    double breakingTorque = 0.0,
    bool collide = false,
  }) {
    _requireAlive();
    // Sent first so a body added or placed a moment ago is where the joint
    // is made.
    _flush();
    final made = calloc<native.OrblitPhysicsJoint>();
    try {
      final it = made.ref;
      it.id = id;
      it.a = a;
      it.b = b;
      it.kind = joint.kind.code;
      final limits = joint.limits;
      var limited = 0;
      for (var i = 0; i < 6; i++) {
        final limit = limits[i];
        if (limit == null) continue;
        limited |= 1 << i;
        it.low[i] = limit.low;
        it.high[i] = limit.high;
      }
      it.limited = limited;
      _write3(it.at, at);
      for (var i = 0; i < 4; i++) {
        it.rotation[i] = rotation[i];
      }
      _write3(it.to, to ?? at);
      it.swing = joint.swing;
      it.speed = joint.speed;
      it.strength = joint.strength;
      it.breakingForce = breakingForce;
      it.breakingTorque = breakingTorque;
      it.collide = collide;
      return native.physicsJoin(_alive, made);
    } finally {
      calloc.free(made);
    }
  }

  /// Removes joint `id`, waking both its bodies so what it held up falls.
  /// False if there was no such joint.
  bool unjoin(int id) {
    _flush();
    return native.physicsUnjoin(_alive, id);
  }

  /// How joint `id` stands now, or null if there is no such joint.
  JointState? jointStateOf(int id) {
    _flush();
    final state = calloc<native.OrblitPhysicsJointState>();
    try {
      if (!native.physicsJoint(_alive, id, state)) return null;
      final it = state.ref;
      return JointState(
        offset: [it.offset[0], it.offset[1], it.offset[2]],
        angles: [it.angles[0], it.angles[1], it.angles[2]],
        force: it.force,
        torque: it.torque,
      );
    } finally {
      calloc.free(state);
    }
  }

  // --- the step ------------------------------------------------------------

  /// Advances the world by `delta` seconds, applying anything submitted first.
  ///
  /// One call per tick. A caller wanting a fixed step calls it several times
  /// with the same delta rather than once with a varying one: a solver handed
  /// a wobbling delta produces a wobbling simulation.
  void step(double delta) {
    _flush();
    native.physicsStep(_alive, delta);
  }

  // --- reading -------------------------------------------------------------

  /// How many bodies there are.
  int get count {
    _flush();
    return native.physicsCount(_alive);
  }

  bool alive(int id) {
    _flush();
    return native.physicsAlive(_alive, id);
  }

  bool asleep(int id) {
    _flush();
    return native.physicsAsleep(_alive, id);
  }

  /// Seven floats — translation xyz then rotation xyzw — or null if the world
  /// does not know `id`.
  Float32List? transformOf(int id) {
    _flush();
    if (!native.physicsTransform(_alive, id, _scratch)) return null;
    return Float32List.fromList(_scratch.asTypedList(7));
  }

  /// Six floats — linear xyz then angular xyz — or null if the world does not
  /// know `id`.
  Float32List? velocityOf(int id) {
    _flush();
    if (!native.physicsVelocity(_alive, id, _scratch)) return null;
    return Float32List.fromList(_scratch.asTypedList(6));
  }

  /// Writes where each of `ids` ended up into `out`: seven floats per body,
  /// `stride` floats apart, starting `offset` floats in. Returns how many were
  /// written; an id the world does not know is skipped and its place left
  /// exactly as it was.
  ///
  /// This copies, because a Dart list is not native memory. Somewhere with a
  /// pointer already — an entity store's own column — wants
  /// `physicsRead` from `native.dart` instead, which does not.
  int readInto(
    List<int> ids,
    Float32List out, {
    int stride = 7,
    int offset = 0,
  }) {
    _flush();
    if (stride < 7) return 0;

    final keys = calloc<Uint64>(ids.length);
    final into = calloc<Float>(out.length);
    try {
      for (var i = 0; i < ids.length; i++) {
        keys[i] = ids[i];
      }
      into.asTypedList(out.length).setAll(0, out);
      final written = native.physicsRead(
        _alive,
        keys,
        ids.length,
        into,
        stride,
        offset,
      );
      out.setAll(0, into.asTypedList(out.length));
      return written;
    } finally {
      calloc.free(keys);
      calloc.free(into);
    }
  }

  /// Everything that happened in the last step. Dropped by the next one.
  List<PhysicsEvent> get events {
    _flush();
    final events = native.physicsEvents(_alive, _count);
    final length = _count.value;
    if (events == nullptr || length == 0) return const [];

    return List<PhysicsEvent>.generate(length, (index) {
      final it = events[index];
      return PhysicsEvent(
        // The ABI numbers them from one. A kind from a newer engine than this
        // binding knows about is reported as a touch ending, which is the
        // safe thing to mistake something for.
        kind: it.kind >= 1 && it.kind <= PhysicsEventKind.values.length
            ? PhysicsEventKind.values[it.kind - 1]
            : PhysicsEventKind.touchEnded,
        a: it.a,
        b: it.b,
        at: [it.at[0], it.at[1], it.at[2]],
        normal: [it.normal[0], it.normal[1], it.normal[2]],
        force: it.force,
      );
    });
  }

  // --- casting -------------------------------------------------------------

  /// Fires `shape` from `from` along `direction` for `distance` metres and
  /// answers the first body it meets, or null if it meets none. Leaving
  /// `shape` out fires a point, which is the ray cast everything else is named
  /// after and the cheapest of them.
  ///
  /// `direction` need not be a unit vector — it is normalised here — so
  /// `distance` is always metres and never multiples of however long the
  /// direction happened to be. A plane cannot be cast, because a half-space
  /// reaches everywhere along any line and the answer would be meaningless;
  /// asking for one gives null.
  PhysicsHit? cast({
    required List<double> from,
    required List<double> direction,
    required double distance,
    Shape? shape,
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
    Layers layers = Layers.everything,
    int ignore = 0,
  }) {
    _flush();
    final query = calloc<native.OrblitPhysicsCast>();
    final found = calloc<native.OrblitPhysicsHit>();
    try {
      final it = query.ref;
      it.shape = shape?.kind ?? 0;
      if (shape != null) {
        it.size[0] = shape.x;
        it.size[1] = shape.y;
        it.size[2] = shape.z;
        it.size[3] = shape.w;
      }
      _write3(it.from, from);
      for (var i = 0; i < 4; i++) {
        it.rotation[i] = rotation[i];
      }
      _write3(it.direction, direction);
      it.distance = distance;
      it.layerIs = layers.is_;
      it.layerCares = layers.cares;
      it.ignore = ignore;

      if (!native.physicsCast(_alive, query, found)) return null;
      final hit = found.ref;
      return PhysicsHit(
        body: hit.body,
        at: [hit.at[0], hit.at[1], hit.at[2]],
        normal: [hit.normal[0], hit.normal[1], hit.normal[2]],
        distance: hit.distance,
        started: hit.started,
      );
    } finally {
      calloc.free(query);
      calloc.free(found);
    }
  }

  // --- characters ----------------------------------------------------------

  /// What character `id` stood on at the end of the last step, or null if it
  /// is not a character.
  PhysicsFooting? footingOf(int id) => footingsOf([id]).single;

  /// The footing of each of `ids`, in the same order, in one crossing. A slot
  /// is null where that id is not a character, so a game with a crowd reads
  /// all of them at once without keeping its characters in a list of their
  /// own.
  List<PhysicsFooting?> footingsOf(List<int> ids) {
    _flush();
    if (ids.isEmpty) return const [];

    final keys = calloc<Uint64>(ids.length);
    final into = calloc<native.OrblitPhysicsFooting>(ids.length);
    try {
      for (var i = 0; i < ids.length; i++) {
        keys[i] = ids[i];
        // The engine writes every field of a character's slot and none of
        // anyone else's, so a slot still holding NaN afterwards is one it
        // passed over. `turning` is a number for every character there is.
        into[i].turning = double.nan;
      }
      native.physicsFooting(_alive, keys, ids.length, into);

      return List<PhysicsFooting?>.generate(ids.length, (index) {
        final it = into[index];
        if (it.turning.isNaN) return null;
        return PhysicsFooting(
          ground: it.ground,
          normal: [it.normal[0], it.normal[1], it.normal[2]],
          velocity: [it.velocity[0], it.velocity[1], it.velocity[2]],
          carried: [it.carried[0], it.carried[1], it.carried[2]],
          turning: it.turning,
          grounded: it.grounded,
        );
      });
    } finally {
      calloc.free(keys);
      calloc.free(into);
    }
  }

  // --- lifetime ------------------------------------------------------------

  void dispose() {
    if (_physics == nullptr) return;
    native.physicsDestroy(_physics);
    _physics = nullptr;
    calloc.free(_scratch);
    calloc.free(_count);
    if (_commands != nullptr) calloc.free(_commands);
    _commands = nullptr;
    _room = 0;
    _pending = 0;
  }

  // --- the buffer ----------------------------------------------------------

  void _requireAlive() {
    if (_physics == nullptr) {
      throw StateError('This Physics world has been disposed.');
    }
  }

  Pointer<native.OrblitPhysicsStruct> get _alive {
    _requireAlive();
    return _physics;
  }

  /// The next free command, zeroed, with its kind and body already set.
  native.OrblitPhysicsCommand _next(int kind, int id) {
    _requireAlive();
    if (_commands == nullptr) {
      _room = 64;
      _commands = calloc<native.OrblitPhysicsCommand>(_room);
    } else if (_pending == _room) {
      // A full buffer is sent rather than grown. Commands arrive in the same
      // order either way, and one allocation that never moves beats a
      // doubling one that does.
      _flush();
    }

    final command = _commands[_pending];
    _zero(command);
    command.kind = kind;
    command.id = id;
    _pending++;
    return command;
  }

  void _flush() {
    if (_pending == 0) return;
    final sending = _pending;
    // Cleared before the call rather than after, so a command submitted from
    // inside whatever this triggers cannot be sent twice.
    _pending = 0;
    native.physicsSubmit(_alive, _commands, sending);
  }

  List<double> _centreOf(int id) {
    final transform = transformOf(id);
    if (transform == null) return const [0.0, 0.0, 0.0];
    return [transform[0], transform[1], transform[2]];
  }

  static void _write3(Array<Float> into, List<double> from) {
    into[0] = from[0];
    into[1] = from[1];
    into[2] = from[2];
  }

  static void _zero(native.OrblitPhysicsCommand command) {
    command.kind = 0;
    command.shape = 0;
    command.id = 0;
    for (var i = 0; i < 4; i++) {
      command.size[i] = 0;
      command.rotation[i] = 0;
    }
    for (var i = 0; i < 3; i++) {
      command.at[i] = 0;
      command.vector[i] = 0;
      command.spin[i] = 0;
    }
    command.damping[0] = 0;
    command.damping[1] = 0;
    command.motion = 0;
    command.mass = 0;
    command.friction = 0;
    command.restitution = 0;
    command.layerIs = 0;
    command.layerCares = 0;
    command.asleep = false;
  }
}
