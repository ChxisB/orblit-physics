import 'dart:math' as math;

import 'drive_spec.dart';
import 'input.dart';
import 'spec.dart';

const _rpm = 60 / (2 * math.pi);

/// The engine, the clutch and the gearbox, and the torque they send to each
/// wheel.
///
/// The clutch is a spring on the difference in speed, limited to what it can
/// hold. A wheel on the ground is turned with the whole car behind it and a
/// wheel in the air is not, so the step asks each tyre how it will answer a
/// torque and solves the clutch against that. The engine, the clutch and the
/// wheels are then stable at any gear ratio, spinning or gripping.
final class Driveline {
  Driveline({required this.spec, required this.wheels})
    : _gear = 1,
      _torques = List.filled(wheels.length, 0.0),
      _driven = [
        for (var i = 0; i < wheels.length; i++)
          if (wheels[i].drives) i,
      ] {
    _speed = spec.engine.idle / _rpm;
  }

  /// Radians a second the clutch may slip by while it holds all it can.
  static const _slip = 2.0;

  /// The clutch is open up to this many times idle. The governor holds the
  /// engine just under it, so a car at rest with the pedal up does not creep.
  static const _open = 1.1;

  /// Halvings of the range the clutch torque is found in. Twelve leave it
  /// within a four-thousandth of the clutch's grip.
  static const _passes = 12;

  final DriveSpec spec;
  final List<WheelSpec> wheels;

  final List<int> _driven;
  final List<double> _torques;
  late double _speed;
  int _gear;
  double _shifting = 0.0;

  /// Revolutions a minute the engine turns at.
  double get rpm => _speed * _rpm;

  /// Reverse is minus one, neutral is zero.
  int get gear => _gear;

  /// Newton metres the driveline turns wheel `i` with.
  double torque(int i) => _torques[i];

  /// Moves the engine and the gearbox on by `dt` seconds. `spins` is how fast
  /// each wheel turns, and `spinAfter` gives, for each wheel, the spin it would
  /// end the step at for a drive torque.
  void step(
    List<double> spins,
    List<double Function(double)> spinAfter,
    VehicleInput input,
    double dt,
  ) {
    final shaft =
        _driven.fold(0.0, (sum, i) => sum + spins[i]) / _driven.length;
    _shift(input, shaft, dt);
    final ratio = _ratio();
    final engine = spec.engine;

    final matched = _matched(ratio, shaft, dt);
    if (matched != null) _speed = matched;

    final pedal = math.max(input.throttle, _governor());
    final cut = ((engine.redline * 1.05 - rpm) / (engine.redline * 0.05)).clamp(
      0.0,
      1.0,
    );
    final drag = engine.drag * rpm / engine.redline;
    final push = pedal * engine.full(rpm) * cut - (1 - pedal) * drag;

    final held = _held(ratio, spinAfter, push, dt);
    _speed = math.max(0.0, _speed + (push - held) * dt / engine.inertia);
    _split(ratio * held, spins, dt);
  }

  /// Changes gear, by the driver's word or the box's own, and starts the shift.
  void _shift(VehicleInput input, double shaft, double dt) {
    _shifting = math.max(0.0, _shifting - dt);
    final wanted = input.gear ?? _auto(input.throttle, shaft);
    if (wanted == _gear) return;

    _gear = wanted;
    _shifting = spec.shiftTime;
  }

  /// The gear the box wants for a driveshaft turning at `shaft` radians a
  /// second. It reads the engine speed the wheels would give, not the engine's
  /// own, which flares while the clutch is slipping and would shift again.
  int _auto(double throttle, double shaft) {
    if (_gear < 1) return 1;
    if (_shifting > 0) return _gear;

    final redline = spec.engine.redline;
    final turning = (_ratio() * shaft).abs() * _rpm;
    if (_gear < spec.ratios.length &&
        throttle > 0 &&
        turning > redline * spec.upshift) {
      return _gear + 1;
    }
    if (_gear > 1 && turning < redline * spec.downshift) return _gear - 1;
    return _gear;
  }

  /// Turns of the engine for a turn of the wheels.
  double _ratio() {
    if (_gear == 0) return 0.0;
    final gear = _gear < 0 ? -spec.reverse : spec.ratios[_gear - 1];
    return gear * spec.finalDrive;
  }

  /// While a shift is under way the engine is brought to the speed the new gear
  /// will give, so the clutch comes back on a matched engine and does not
  /// jolt. Null when there is no shift or the gearbox is in neutral.
  double? _matched(double ratio, double shaft, double dt) {
    if (_shifting <= 0 || ratio == 0) return null;

    final target = math.max(spec.engine.idle / _rpm, (ratio * shaft).abs());
    return _speed + (target - _speed) * math.min(1.0, dt / _shifting);
  }

  /// Opens the throttle by itself to keep the engine from stalling at idle.
  double _governor() {
    final idle = spec.engine.idle;
    return ((idle * 1.05 - rpm) / (idle * 0.1)).clamp(0.0, 1.0);
  }

  /// Newton metres the clutch holds between the engine and the driveshaft, for
  /// an engine that gives `push`. Positive drives the car.
  double _held(
    double ratio,
    List<double Function(double)> spinAfter,
    double push,
    double dt,
  ) {
    final open = spec.engine.idle * _open;
    final take = (rpm - open) / (spec.engage - open);
    final shifted = spec.shiftTime == 0 ? 1.0 : 1 - _shifting / spec.shiftTime;
    final most = ratio == 0
        ? 0.0
        : spec.clutch * take.clamp(0.0, 1.0) * shifted;
    if (most == 0) return 0.0;

    // The clutch holds `stiffness` newton metres for each radian a second of
    // slip at the end of the step, and the wheels answer a torque as the tyres
    // say. The residual rises with the torque, since a wheel that is driven
    // harder ends faster, so a bisection finds the one torque that fits both.
    final stiffness = most / _slip;
    final inertia = spec.engine.inertia;
    final share = ratio / _driven.length;
    double residual(double held) {
      final shaft =
          _driven.fold(0.0, (sum, i) => sum + spinAfter[i](share * held)) /
          _driven.length;
      return held -
          stiffness * (_speed + (push - held) * dt / inertia - ratio * shaft);
    }

    if (residual(most) <= 0) return most;
    if (residual(-most) >= 0) return -most;

    var (low, high) = (-most, most);
    for (var pass = 0; pass < _passes; pass++) {
      final mid = (low + high) / 2;
      if (residual(mid) < 0) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return (low + high) / 2;
  }

  /// Shares `total` newton metres out: evenly between the axles, then evenly
  /// between the wheels of an axle, with the differential moving torque to the
  /// slower wheel.
  void _split(double total, List<double> spins, double dt) {
    _torques.fillRange(0, _torques.length, 0.0);
    final axles = <double, List<int>>{};
    for (final i in _driven) {
      (axles[wheels[i].mount.z] ??= []).add(i);
    }
    for (final axle in axles.values) {
      for (final i in axle) {
        _torques[i] = total / axles.length / axle.length;
      }
      if (axle.length != 2) continue;

      final (a, b) = (axle[0], axle[1]);
      final catchUp = wheels[a].inertia * (spins[b] - spins[a]) / (2 * dt);
      final moved = catchUp.clamp(-spec.lock, spec.lock);
      _torques[a] += moved;
      _torques[b] -= moved;
    }
  }
}
