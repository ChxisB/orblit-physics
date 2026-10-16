import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

import 'spec.dart';

/// The ground under one wheel for one step.
final class Patch {
  const Patch({
    required this.velocity,
    required this.normal,
    required this.heading,
    required this.load,
    required this.surface,
    required this.mass,
  });

  /// How fast the chassis moves at the point the wheel touches.
  final Vector3 velocity;

  /// Unit length, away from the ground.
  final Vector3 normal;

  /// Unit length, the way the wheel rolls. It need not lie in the ground.
  final Vector3 heading;

  /// Newtons the wheel presses on the ground with.
  final double load;

  /// The grip of what it stands on. One is dry ground.
  final double surface;

  /// Kilograms of the car that the wheel carries.
  final double mass;
}

/// The rubber of one wheel and the wheel's spin.
///
/// The patch is held to the ground by bristles. A bristle bends as the patch
/// slips and pulls the patch back, so a car that is parked is held in place,
/// not slowed to a crawl. As the wheel rolls the bristles are carried away and
/// fresh ones take the ground, which is why a tyre turning a corner gives a
/// steady force for a steady slip. The force is limited to the grip.
///
/// [step] solves the bristles and the wheel's spin together, so a wheel that
/// weighs a few kilograms is stable at the step of a game.
final class Tyre {
  Tyre({required this.spec, required this.radius, required this.inertia});

  final TyreSpec spec;
  final double radius;
  final double inertia;

  /// Radians a second the wheel turns at. Positive rolls the car forward.
  double get spin => _spin;

  /// Newtons the patch pushes the chassis with, in the world.
  Vector3 get force => _force;

  double _spin = 0.0;
  Vector3 _force = Vector3.zero();

  /// How far the bristles are bent, in the world, along the ground.
  Vector3 _bent = Vector3.zero();

  /// How fast the last step expected the chassis to move at the end of it,
  /// counting only the tyre's own force, and the share of a step's slip that
  /// bent the bristles. Null when the wheel had no ground.
  Vector3? _expected;
  double _carry = 0.0;

  /// Moves the tyre on by `dt` seconds. `drive` and `brake` are torques in
  /// newton metres: the first turns the wheel forward and the second holds it.
  /// A wheel off the ground or with no load only turns freely and brakes.
  void step(
    Patch? patch, {
    required double dt,
    required double drive,
    required double brake,
  }) {
    final next = _solve(patch, dt, drive, brake);
    _spin = next.spin;
    _force = next.force;
    _bent = next.bent;
    _expected = next.expected;
    _carry = next.carry;
  }

  /// The spin the wheel would end a step at, for a `drive` torque, without
  /// taking the step. The driveline asks it to find how the ground answers the
  /// torque it is about to send.
  double spinAfter(
    Patch? patch, {
    required double dt,
    required double drive,
    required double brake,
  }) => _solve(patch, dt, drive, brake).spin;

  _Step _solve(Patch? patch, double dt, double drive, double brake) {
    final start = _spin + drive * dt / inertia;
    final ground = patch == null ? null : _Ground.of(patch);
    if (patch == null || ground == null || !(patch.load > 0)) {
      return _Step.free(_braked(start, brake * dt));
    }

    // The bristles solved the step as if the tyre were the only force on the
    // chassis. Whatever else pushed it (weight on a slope, a spring) moved the
    // patch further, and the bend has to follow or a tyre holds a car that is
    // still sliding at the speed those forces give it in one step.
    final expected = _expected;
    final from = _Wheel(
      start,
      expected == null ? _bent : _bent + (patch.velocity - expected) * _carry,
    );

    final free = _grip(patch, ground, dt, from, turning: true);
    final turned = start - free.along * radius * dt / inertia;
    final locked = brake * dt >= inertia * turned.abs();
    final held = locked ? _grip(patch, ground, dt, from, turning: false) : free;
    final force = ground.heading * held.along + ground.side * held.across;
    return _Step(
      spin: locked ? 0.0 : _braked(turned, brake * dt),
      force: force,
      bent: ground.heading * held.bentAlong + ground.side * held.bentAcross,
      expected: patch.velocity + force * (dt / patch.mass),
      carry: dt * held.keep,
    );
  }

  double _braked(double spin, double impulse) {
    final slowed = impulse / inertia;
    return spin.abs() <= slowed ? 0.0 : spin - spin.sign * slowed;
  }

  /// What the patch gives for a step with the wheel free to turn, or held.
  _Grip _grip(
    Patch patch,
    _Ground ground,
    double dt,
    _Wheel from, {
    required bool turning,
  }) {
    final spin = turning ? from.spin : 0.0;
    final rolling = spin.abs() * radius;
    final slip = Vector2(ground.along - spin * radius, ground.across);
    // A wheel that turns also takes the force into its spin, which makes the
    // car feel much lighter along the road.
    final mass = Vector2(
      turning ? 1 / (1 / patch.mass + radius * radius / inertia) : patch.mass,
      patch.mass,
    );
    final most = spec.grip * patch.surface * patch.load;
    final bristles = _Bristles(
      stiffness: most / (spec.peakSlip * spec.relaxation),
      keep: 1 / (1 + dt * rolling / spec.relaxation),
      dt: dt,
    );

    final bent = Vector2(
      from.bent.dot(ground.heading),
      from.bent.dot(ground.side),
    );
    final force = Vector2(
      bristles.force(bent.x, slip.x, mass.x),
      bristles.force(bent.y, slip.y, mass.y),
    );
    final reference = math.max(math.max(ground.speed, rolling), 1.0);
    final past = (slip.length / reference / spec.peakSlip - 1).clamp(0.0, 1.0);
    final limit = most * (1 - (1 - spec.sliding) * past);
    if (force.length > limit) force.scale(limit / force.length);

    return _Grip(
      along: force.x,
      across: force.y,
      bentAlong: bristles.bent(bent.x, force.x, mass.x),
      bentAcross: bristles.bent(bent.y, force.y, mass.y),
      keep: bristles.keep,
    );
  }
}

/// A wheel at the start of a step: how fast it turns and how bent its bristles
/// are, in the world.
final class _Wheel {
  const _Wheel(this.spin, this.bent);

  final double spin;
  final Vector3 bent;
}

/// What a step leaves the tyre with.
final class _Step {
  const _Step({
    required this.spin,
    required this.force,
    required this.bent,
    required this.expected,
    required this.carry,
  });

  /// A wheel with no ground turning at `spin`.
  _Step.free(this.spin)
    : force = Vector3.zero(),
      bent = Vector3.zero(),
      expected = null,
      carry = 0.0;

  final double spin;
  final Vector3 force;
  final Vector3 bent;
  final Vector3? expected;
  final double carry;
}

/// A force along the wheel and across it, and the bristles it leaves.
final class _Grip {
  const _Grip({
    required this.along,
    required this.across,
    required this.bentAlong,
    required this.bentAcross,
    required this.keep,
  });

  final double along;
  final double across;
  final double bentAlong;
  final double bentAcross;

  /// The share of the bend that stayed with the wheel through the step.
  final double keep;
}

/// Where the wheel stands on the ground: its two directions along it, and how
/// fast the chassis moves over it that way.
final class _Ground {
  _Ground._(this.heading, this.side, this.along, this.across, this.speed);

  /// Null when the wheel points straight into the ground and has no way along
  /// it.
  static _Ground? of(Patch patch) {
    final normal = patch.normal;
    final heading = patch.heading - normal * patch.heading.dot(normal);
    if (heading.length2 < 1e-6) return null;

    heading.normalize();
    final side = normal.cross(heading);
    final moving = patch.velocity - normal * patch.velocity.dot(normal);
    return _Ground._(
      heading,
      side,
      moving.dot(heading),
      moving.dot(side),
      moving.length,
    );
  }

  final Vector3 heading;
  final Vector3 side;
  final double along;
  final double across;
  final double speed;
}

/// The bristles of one axis as a spring and a damper, taken over a step.
///
/// The damper acts on how fast the bristles bend, not on how fast the patch
/// slips, so it quiets a parked wheel and has nothing to say to one that is
/// rolling. Both are solved at the end of the step. Whatever `mass` the
/// force falls on, the answer is stable.
final class _Bristles {
  const _Bristles({
    required this.stiffness,
    required this.keep,
    required this.dt,
  });

  /// Newtons a metre.
  final double stiffness;

  /// The share of the bend that stays after the wheel has rolled for a step.
  final double keep;

  final double dt;

  /// Critical damping for `mass`, as a force for a bend that goes in one step.
  double _damper(double mass) => 2 * math.sqrt(stiffness * mass) / dt;

  /// The force that holds a patch with `slip` metres a second over bristles
  /// bent by `bent` metres, falling on `mass` kilograms.
  double force(double bent, double slip, double mass) {
    final damper = _damper(mass);
    final both = stiffness + damper;
    return (damper * bent - both * keep * (bent + dt * slip)) /
        (1 + both * keep * dt * dt / mass);
  }

  /// How far the bristles are bent once `force` has acted.
  double bent(double bent, double force, double mass) {
    final damper = _damper(mass);
    return (damper * bent - force) / (stiffness + damper);
  }
}
