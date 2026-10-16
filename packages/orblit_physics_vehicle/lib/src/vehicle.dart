import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'spec.dart';

/// What a wheel found at the last [Vehicle.update].
final class WheelState {
  const WheelState({
    this.grounded = false,
    this.compression = 0.0,
    this.force = 0.0,
  });

  /// It found ground under it. A wheel that did not hangs at full droop and
  /// gives no force.
  final bool grounded;

  /// Metres the spring is squeezed. Past the spring's travel the wheel is
  /// pushed up into the chassis and the chassis is about to rest on the floor.
  final double compression;

  /// Newtons it pushed the chassis with. Never negative, since a spring that
  /// is only held by the ground cannot pull.
  final double force;
}

/// A car: a chassis body whose wheels are casts.
///
/// Call [update] once before each `Physics.step`, with that step's length. It
/// reads the chassis, casts from each wheel, and pushes the chassis with each
/// spring's force as an impulse. The world is never told about the wheels.
final class Vehicle {
  Vehicle({
    required this.physics,
    required this.id,
    required this.spec,
    List<double> at = const [0.0, 0.0, 0.0],
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
  }) : _wheels = List.filled(spec.wheels.length, const WheelState()) {
    physics.add(
      id,
      shape: spec.chassis,
      at: at,
      rotation: rotation,
      mass: spec.mass,
      layers: spec.layers,
    );
    physics.setControls(id, PhysicsControls(centre: spec.centre));
  }

  /// The longest step a spring of [SuspensionSpec.maxFrequency] is stable at.
  static const maxStep = 1 / 30;

  final Physics physics;
  final int id;
  final VehicleSpec spec;

  List<WheelState> _wheels;

  /// Each wheel as the last update left it, in the order of the spec.
  List<WheelState> get wheels => _wheels;

  /// Pushes the chassis with every wheel's force for a step of `dt` seconds.
  void update(double dt) {
    if (!(dt > 0 && dt <= maxStep)) {
      throw ArgumentError.value(
        dt,
        'dt',
        'must be above 0 and at most $maxStep',
      );
    }
    final chassis = _Chassis.read(physics, id);
    if (chassis == null) return;

    final probes = [
      for (var i = 0; i < _wheels.length; i++) _probe(i, chassis, dt),
    ];
    final forces = _forces(probes, dt);
    _wheels = [
      for (var i = 0; i < probes.length; i++)
        WheelState(
          grounded: probes[i] != null,
          compression: probes[i]?.compression ?? 0.0,
          force: forces[i],
        ),
    ];
    for (var i = 0; i < probes.length; i++) {
      final probe = probes[i];
      if (probe == null || forces[i] == 0.0) continue;
      final push = Vector3(probe.normal[0], probe.normal[1], probe.normal[2])
        ..scale(forces[i] * dt);
      physics.push(id, impulse: _list(push), at: probe.at);
    }
  }

  _Probe? _probe(int i, _Chassis chassis, double dt) {
    final wheel = spec.wheels[i];
    final reach = wheel.suspension.travel + wheel.radius;
    final hit = physics.cast(
      from: _list(chassis.toWorld(wheel.mount)),
      direction: _list(chassis.down),
      distance: reach,
      layers: spec.ground,
      ignore: id,
    );
    // A mount already inside the ground has nothing to measure from. The
    // chassis' own contact holds the car up there.
    if (hit == null || hit.started) return null;

    final compression = reach - hit.distance;
    // The rate is the change since the last update, not the chassis' velocity:
    // the solver reports a velocity at the end of its sub-steps, which for a
    // body held by one impulse a step is a sawtooth that is not its motion.
    final before = _wheels[i];
    return _Probe(
      compression: compression,
      rate: before.grounded ? (compression - before.compression) / dt : 0.0,
      at: hit.at,
      normal: hit.normal,
    );
  }

  /// A spring, a damper and half an anti-roll bar for each wheel on the
  /// ground, in newtons.
  List<double> _forces(List<_Probe?> probes, double dt) {
    final sprung = spec.mass / spec.wheels.length;
    final forces = [
      for (var i = 0; i < probes.length; i++)
        probes[i] == null
            ? 0.0
            : _spring(spec.wheels[i].suspension, probes[i]!, sprung, dt),
    ];
    for (final (a, b) in spec.axles) {
      final (pa, pb) = (probes[a], probes[b]);
      final bar =
          spec.antiRoll * ((pa?.compression ?? 0) - (pb?.compression ?? 0));
      if (pa != null) forces[a] += bar;
      if (pb != null) forces[b] -= bar;
    }
    return [for (final force in forces) math.max(0.0, force)];
  }

  double _spring(
    SuspensionSpec spring,
    _Probe probe,
    double sprung,
    double dt,
  ) {
    final omega = 2 * math.pi * spring.frequency;
    final push = sprung * omega * omega * probe.compression;
    final damper = 2 * spring.damping * omega * sprung * probe.rate;
    // The damper may take the compression rate to zero in one step and no
    // further. Past that it pushes the wheel the other way and the car buzzes.
    final most = sprung * probe.rate.abs() / dt;
    return push + damper.clamp(-most, most);
  }
}

/// The ground found under one wheel.
final class _Probe {
  const _Probe({
    required this.compression,
    required this.rate,
    required this.at,
    required this.normal,
  });

  /// Metres squeezed, and how fast it is being squeezed.
  final double compression;
  final double rate;

  final List<double> at;
  final List<double> normal;
}

/// Where the chassis stands at the start of a step.
final class _Chassis {
  _Chassis._(this.origin, this.rotation);

  static _Chassis? read(Physics physics, int id) {
    final t = physics.transformOf(id);
    if (t == null) return null;

    // asRotationMatrix, not Quaternion.rotated, which turns the other way.
    return _Chassis._(
      Vector3(t[0], t[1], t[2]),
      Quaternion(t[3], t[4], t[5], t[6]).asRotationMatrix(),
    );
  }

  final Vector3 origin;
  final Matrix3 rotation;

  /// Its own down, in the world.
  Vector3 get down => rotation.transformed(Vector3(0, -1, 0));

  Vector3 toWorld(Vector3 local) => origin + rotation.transformed(local);
}

List<double> _list(Vector3 v) => [v.x, v.y, v.z];
