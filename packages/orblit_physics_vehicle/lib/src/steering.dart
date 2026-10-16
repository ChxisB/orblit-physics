import 'dart:math' as math;

import 'spec.dart';

/// The angle each wheel is turned to, in radians and in the order of `wheels`,
/// for a steering angle of `angle` at the middle of the steering axle. Left is
/// positive.
///
/// The wheels that steer turn so that their lines across the road meet on the
/// line through the wheels that do not. The inner wheel turns more than the
/// outer one, and the car follows a circle with no wheel pushed sideways. With
/// no wheel left fixed, or none to steer, they turn together.
List<double> steerAngles(List<WheelSpec> wheels, double angle) {
  final steering = [
    for (final w in wheels)
      if (w.steers) w,
  ];
  final fixed = [
    for (final w in wheels)
      if (!w.steers) w,
  ];
  if (steering.isEmpty) return List.filled(wheels.length, 0.0);

  // Ahead is along the chassis' negative z, left along its negative x.
  final rear = _mean([for (final w in fixed) -w.mount.z]);
  final base = _mean([for (final w in steering) -w.mount.z]) - rear;
  if (fixed.isEmpty || base.abs() < 1e-6) {
    return [for (final w in wheels) w.steers ? angle : 0.0];
  }

  final lean = math.tan(angle);
  return [
    for (final w in wheels)
      w.steers
          ? math.atan((-w.mount.z - rear) * lean / (base + w.mount.x * lean))
          : 0.0,
  ];
}

double _mean(List<double> values) =>
    values.fold(0.0, (sum, v) => sum + v) / values.length;
