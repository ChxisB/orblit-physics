import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'driveline.dart';
import 'input.dart';
import 'spec.dart';
import 'steering.dart';
import 'tyre.dart';

/// What a wheel found at the last [Vehicle.update].
final class WheelState {
  const WheelState({
    this.grounded = false,
    this.compression = 0.0,
    this.force = 0.0,
    this.spin = 0.0,
    this.steer = 0.0,
  });

  /// It found ground under it. A wheel that did not hangs at full droop and
  /// gives no force.
  final bool grounded;

  /// Metres the spring is squeezed. Past the spring's travel the wheel is
  /// pushed up into the chassis and the chassis is about to rest on the floor.
  final double compression;

  /// Newtons it pushed the chassis up with. Never negative, since a spring
  /// that is only held by the ground cannot pull.
  final double force;

  /// Radians a second it turns at. Positive rolls the car forward.
  final double spin;

  /// Radians it is turned from straight ahead. Left is positive.
  final double steer;
}

/// The grip of the ground under a body, as a number to scale a tyre's by. One
/// is dry ground.
typedef SurfaceGrip = double Function(int body);

double _dry(int body) => 1.0;

/// A car: a chassis body whose wheels are casts.
///
/// Call [update] once before each `Physics.step`, with that step's length. It
/// reads the chassis, casts from each wheel, and pushes the chassis with each
/// spring's force and each tyre's grip as an impulse. The world is never told
/// about the wheels.
final class Vehicle {
  Vehicle({
    required this.physics,
    required this.id,
    required this.spec,
    List<double> at = const [0.0, 0.0, 0.0],
    List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
    this.surface = _dry,
  }) : _wheels = List.filled(spec.wheels.length, const WheelState()),
       _tyres = [
         for (final wheel in spec.wheels)
           Tyre(spec: wheel.tyre, radius: wheel.radius, inertia: wheel.inertia),
       ],
       _driveline = spec.drive == null
           ? null
           : Driveline(spec: spec.drive!, wheels: spec.wheels) {
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

  /// How well the tyres hold what a wheel stands on, by the body it stands on.
  final SurfaceGrip surface;

  final List<Tyre> _tyres;
  final Driveline? _driveline;
  List<WheelState> _wheels;
  VehicleInput _input = VehicleInput();

  /// Where the chassis stood at the last update.
  _Chassis? _before;

  /// Each wheel as the last update left it, in the order of the spec.
  List<WheelState> get wheels => _wheels;

  /// What the driver is asking for.
  VehicleInput get input => _input;

  /// Revolutions a minute the engine turns at. Zero for a car with no engine.
  double get rpm => _driveline?.rpm ?? 0.0;

  /// The gear the box is in: reverse is minus one and neutral is zero. Zero for
  /// a car with no engine.
  int get gear => _driveline?.gear ?? 0;

  /// Changes what the driver asks for, from the next update. A gear the car
  /// does not have is refused.
  void setInput(VehicleInput input) {
    final gear = input.gear;
    final top = spec.drive?.ratios.length ?? 0;
    if (gear != null && gear > top) {
      throw ArgumentError.value(gear, 'gear', 'the car has $top');
    }
    _input = input;
  }

  /// Pushes the chassis with every wheel's force for a step of `dt` seconds.
  void update(double dt) {
    if (!(dt > 0 && dt <= maxStep)) {
      throw ArgumentError.value(
        dt,
        'dt',
        'must be above 0 and at most $maxStep',
      );
    }
    final chassis = _Chassis.read(physics, id, spec.centre);
    if (chassis == null) return;

    final steer = steerAngles(spec.wheels, _input.steer * spec.maxSteer);
    final probes = [
      for (var i = 0; i < _wheels.length; i++) _probe(i, chassis, dt),
    ];
    final loads = _forces(probes, dt);
    final patches = [
      for (var i = 0; i < probes.length; i++)
        probes[i] == null
            ? null
            : _patch(
                i,
                probes[i]!,
                chassis,
                dt,
                steer: steer[i],
                load: loads[i],
              ),
    ];
    _driveline?.step(
      [for (final tyre in _tyres) tyre.spin],
      [
        for (var i = 0; i < patches.length; i++)
          (torque) => _tyres[i].spinAfter(
            patches[i],
            dt: dt,
            drive: torque,
            brake: _brake(i),
          ),
      ],
      _input,
      dt,
    );
    _wheels = [
      for (var i = 0; i < probes.length; i++)
        _roll(i, probes[i], patches[i], dt, steer: steer[i], load: loads[i]),
    ];
    _before = chassis;
  }

  /// Turns wheel `i` for a step, pushes the chassis with what it gives, and
  /// reports the wheel.
  WheelState _roll(
    int i,
    _Probe? probe,
    Patch? patch,
    double dt, {
    required double steer,
    required double load,
  }) {
    final tyre = _tyres[i];
    tyre.step(
      patch,
      dt: dt,
      drive: _driveline?.torque(i) ?? 0.0,
      brake: _brake(i),
    );
    if (probe != null) {
      final push = probe.normal * load + tyre.force;
      if (push.length2 > 0) {
        physics.push(id, impulse: _list(push..scale(dt)), at: probe.at);
      }
    }
    return WheelState(
      grounded: probe != null,
      compression: probe?.compression ?? 0.0,
      force: load,
      spin: tyre.spin,
      steer: steer,
    );
  }

  /// Newton metres the brake of wheel `i` gives.
  double _brake(int i) {
    final wheel = spec.wheels[i];
    final held = wheel.handbrake && _input.handbrake;
    return wheel.brakeTorque * (held ? 1.0 : _input.brake);
  }

  Patch _patch(
    int i,
    _Probe probe,
    _Chassis chassis,
    double dt, {
    required double steer,
    required double load,
  }) {
    return Patch(
      velocity: chassis.motionAt(Vector3.array(probe.at), _before, dt),
      normal: probe.normal,
      heading: chassis.direct(Vector3(-math.sin(steer), 0, -math.cos(steer))),
      load: load,
      surface: surface(probe.body),
      mass: spec.sprung[i],
    );
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
      body: hit.body,
      compression: compression,
      rate: before.grounded ? (compression - before.compression) / dt : 0.0,
      at: hit.at,
      normal: Vector3.array(hit.normal),
    );
  }

  /// A spring, a damper and half an anti-roll bar for each wheel on the
  /// ground, in newtons.
  List<double> _forces(List<_Probe?> probes, double dt) {
    final forces = [
      for (var i = 0; i < probes.length; i++)
        probes[i] == null
            ? 0.0
            : _spring(
                spec.wheels[i].suspension,
                probes[i]!,
                spec.sprung[i],
                dt,
              ),
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
    required this.body,
    required this.compression,
    required this.rate,
    required this.at,
    required this.normal,
  });

  final int body;

  /// Metres squeezed, and how fast it is being squeezed.
  final double compression;
  final double rate;

  final List<double> at;
  final Vector3 normal;
}

/// Where the chassis stands, and how it moves, at the start of a step.
final class _Chassis {
  _Chassis._(this.origin, this.rotation, this.centre, this.velocity, this.spin);

  static _Chassis? read(Physics physics, int id, List<double> centre) {
    final t = physics.transformOf(id);
    final v = physics.velocityOf(id);
    if (t == null || v == null) return null;

    // asRotationMatrix, not Quaternion.rotated, which turns the other way.
    final origin = Vector3(t[0], t[1], t[2]);
    final rotation = Quaternion(t[3], t[4], t[5], t[6]).asRotationMatrix();
    return _Chassis._(
      origin,
      rotation,
      origin + rotation.transformed(Vector3(centre[0], centre[1], centre[2])),
      Vector3(v[0], v[1], v[2]),
      Vector3(v[3], v[4], v[5]),
    );
  }

  final Vector3 origin;
  final Matrix3 rotation;

  /// Where its weight is, and how that point moves and turns.
  final Vector3 centre;
  final Vector3 velocity;
  final Vector3 spin;

  /// Its own down, in the world.
  Vector3 get down => direct(Vector3(0, -1, 0));

  Vector3 direct(Vector3 local) => rotation.transformed(local);

  Vector3 toWorld(Vector3 local) => origin + direct(local);

  /// How fast the chassis moves at a point in the world, taken from where that
  /// point of the chassis was at the `before` update, `dt` seconds ago.
  ///
  /// Not from the solver's velocity, which is read at the end of its
  /// sub-steps. For a body held up by one push a step that reading is a
  /// sawtooth about the motion, and a tyre that holds the sawtooth still
  /// lets the car creep. The first update has no past and takes the reading.
  Vector3 motionAt(Vector3 point, _Chassis? before, double dt) {
    if (before == null) return velocity + spin.cross(point - centre);

    final turned = before.rotation.multiplied(rotation.transposed());
    final earlier = before.origin + turned.transformed(point - origin);
    return (point - earlier)..scale(1 / dt);
  }
}

List<double> _list(Vector3 v) => [v.x, v.y, v.z];
