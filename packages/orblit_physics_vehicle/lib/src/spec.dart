import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

/// How a wheel is held to the chassis.
///
/// The spring is written as a frequency so that a kart and a truck use the
/// same numbers. The stiffness follows from the weight the wheel carries.
final class SuspensionSpec {
  SuspensionSpec({
    this.travel = 0.3,
    this.frequency = 1.5,
    this.damping = 0.5,
  }) {
    if (!(travel > 0)) {
      throw ArgumentError.value(travel, 'travel', 'must be above zero');
    }
    if (!(frequency > 0 && frequency <= maxFrequency)) {
      throw ArgumentError.value(
        frequency,
        'frequency',
        'must be above zero and at most $maxFrequency Hz',
      );
    }
    if (!(damping >= 0 && damping <= 2)) {
      throw ArgumentError.value(damping, 'damping', 'must be 0 to 2');
    }
  }

  /// The fastest spring a step of [Vehicle.maxStep] holds. Past it the spring
  /// overshoots in a single step and the car buzzes.
  static const maxFrequency = 8.0;

  /// How far the wheel can move, in metres, from hanging free to the mount.
  /// The spring is at rest when the wheel hangs free.
  final double travel;

  /// Cycles a second of the car's weight on this spring.
  final double frequency;

  /// Of critical damping. Zero rings, one settles in a single swing.
  final double damping;
}

/// One wheel. It has no mass of its own.
final class WheelSpec {
  WheelSpec({
    required Vector3 mount,
    required this.radius,
    SuspensionSpec? suspension,
  }) : mount = Vector3.copy(mount),
       suspension = suspension ?? SuspensionSpec() {
    if (!(radius > 0)) {
      throw ArgumentError.value(radius, 'radius', 'must be above zero');
    }
  }

  /// The top of the suspension, in the chassis frame. The wheel hangs below it
  /// along the chassis' own down. Not to be changed.
  final Vector3 mount;

  final double radius;

  final SuspensionSpec suspension;
}

/// A car as data.
///
/// Wheels that share a mount z are an axle. With [antiRoll] above zero every
/// axle has exactly two.
final class VehicleSpec {
  VehicleSpec({
    required this.chassis,
    required this.mass,
    required List<WheelSpec> wheels,
    this.centre = const [0.0, 0.0, 0.0],
    this.antiRoll = 0.0,
    this.layers = Layers.everything,
    this.ground = Layers.everything,
  }) : wheels = List.unmodifiable(wheels),
       axles = _axlesOf(wheels, antiRoll) {
    if (!(mass > 0)) {
      throw ArgumentError.value(mass, 'mass', 'must be above zero');
    }
    if (wheels.isEmpty) {
      throw ArgumentError.value(wheels, 'wheels', 'needs at least one');
    }
  }

  /// What the car collides as. It still hits the floor when the springs bottom
  /// out.
  final Shape chassis;

  /// Kilograms, shared evenly between the wheels.
  final double mass;

  final List<WheelSpec> wheels;

  /// Where its weight is, in the chassis frame.
  final List<double> centre;

  /// Newtons a metre of difference in compression across an axle. Zero is no
  /// bar.
  final double antiRoll;

  /// The chassis' layers.
  final Layers layers;

  /// The layers a wheel sees as ground, so a car does not stand on a
  /// character.
  final Layers ground;

  /// The pairs of wheels, by index, that an anti-roll bar joins.
  final List<(int, int)> axles;

  static List<(int, int)> _axlesOf(List<WheelSpec> wheels, double antiRoll) {
    if (!(antiRoll >= 0)) {
      throw ArgumentError.value(antiRoll, 'antiRoll', 'must not be negative');
    }
    if (antiRoll == 0) return const [];

    final byZ = <double, List<int>>{};
    for (var i = 0; i < wheels.length; i++) {
      (byZ[wheels[i].mount.z] ??= []).add(i);
    }
    if (byZ.values.any((axle) => axle.length != 2)) {
      throw ArgumentError.value(
        wheels,
        'wheels',
        'an anti-roll bar needs exactly two wheels on each axle',
      );
    }
    return [for (final axle in byZ.values) (axle[0], axle[1])];
  }
}
