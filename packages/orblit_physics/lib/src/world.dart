// The Dart face of the engine: lifetimes, a command buffer, and turning the
// ABI's flat structs into something worth reading.
//
// Commands are collected rather than sent one at a time. A tick creates a
// handful of bodies and shoves a handful more, and crossing into native code
// once for each of them would spend more time on the crossing than on the
// physics. They are flushed before anything reads, so from the caller's side
// a command still takes effect the moment it is given.

import 'dart:ffi';
import 'dart:typed_data';

import 'package:ffi/ffi.dart';

import 'bindings.dart' as native;

/// How the solver treats a body.
enum PhysicsMotion {
  /// Never moves. The floor, the walls.
  fixed(0),

  /// Moves exactly where it is put, and pushes anything in the way without
  /// being pushed back. Lifts, doors, a character controller.
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

  /// An endless flat surface: everything behind the normal `nx, ny, nz` at
  /// `offset` along it is solid. Never dynamic — a half-space has no centre
  /// to spin about.
  const Shape.plane(double nx, double ny, double nz, {double offset = 0.0})
    : this._(3, nx, ny, nz, offset);

  /// Which of the three it is, as the ABI numbers them.
  final int kind;

  /// The ABI's four size floats, read as the kind describes: a radius, three
  /// half extents, or a normal and an offset.
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
enum PhysicsEventKind { touchBegan, touchEnded, slept, woke }

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
  /// sorting.
  final int a;
  final int b;

  /// Where they met, and which way out of `b` towards `a`. Zero for anything
  /// that is not a touch beginning.
  final List<double> at;
  final List<double> normal;

  /// How hard, in newton-seconds along the normal. The number a collision
  /// sound scales with.
  final double force;
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

  /// Takes a body out of the world. Anything that was touching it is told the
  /// touch has ended on the next step.
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
