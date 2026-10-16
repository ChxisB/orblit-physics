import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'drive_spec.dart';

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

  /// Metres the spring is squeezed once it holds the weight it was tuned for,
  /// under `gravity` metres a second squared. The stiffness follows the weight,
  /// so the squeeze does not depend on how heavy the wheel's share is. Never
  /// more than [travel].
  double sag([double gravity = 9.81]) {
    final omega = 2 * math.pi * frequency;
    return math.min(travel, gravity / (omega * omega));
  }
}

/// What a tyre holds the ground with.
///
/// The force a tyre gives rises with slip up to [grip] times the load on it,
/// and falls to [sliding] of that once the patch is slipping twice as much as
/// [peakSlip]. Slip is the speed of the patch over the ground as a share of
/// the speed the wheel is rolling at, or of one metre a second below that.
final class TyreSpec {
  TyreSpec({
    this.grip = 1.0,
    this.peakSlip = 0.1,
    this.sliding = 0.8,
    this.relaxation = 0.2,
  }) {
    if (!(grip > 0)) {
      throw ArgumentError.value(grip, 'grip', 'must be above zero');
    }
    if (!(peakSlip > 0 && peakSlip <= 1)) {
      throw ArgumentError.value(peakSlip, 'peakSlip', 'must be above 0, to 1');
    }
    if (!(sliding > 0 && sliding <= 1)) {
      throw ArgumentError.value(sliding, 'sliding', 'must be above 0, to 1');
    }
    if (!(relaxation > 0)) {
      throw ArgumentError.value(relaxation, 'relaxation', 'must be above zero');
    }
  }

  /// The most force the rubber gives for each newton of load, on a surface
  /// whose grip is one.
  final double grip;

  /// The slip, as a share, at which the force peaks.
  final double peakSlip;

  /// What is left of [grip] when the patch slides, from zero to one. One is a
  /// tyre that does not fall off.
  final double sliding;

  /// Metres the wheel rolls for the force to build to its steady value.
  /// A short one answers quickly, a long one is soft.
  final double relaxation;
}

/// One wheel. It has no mass of its own, but it has a spin to hold.
final class WheelSpec {
  WheelSpec({
    required Vector3 mount,
    required this.radius,
    this.steers = false,
    this.drives = false,
    this.brakeTorque = 0.0,
    this.handbrake = false,
    this.inertia = 1.0,
    SuspensionSpec? suspension,
    TyreSpec? tyre,
  }) : mount = Vector3.copy(mount),
       suspension = suspension ?? SuspensionSpec(),
       tyre = tyre ?? TyreSpec() {
    if (!(radius > 0)) {
      throw ArgumentError.value(radius, 'radius', 'must be above zero');
    }
    if (!(brakeTorque >= 0)) {
      throw ArgumentError.value(
        brakeTorque,
        'brakeTorque',
        'must not be negative',
      );
    }
    if (handbrake && brakeTorque == 0) {
      throw ArgumentError.value(
        brakeTorque,
        'brakeTorque',
        'a wheel on the handbrake needs a brake',
      );
    }
    if (!(inertia > 0)) {
      throw ArgumentError.value(inertia, 'inertia', 'must be above zero');
    }
  }

  /// The top of the suspension, in the chassis frame. The wheel hangs below it
  /// along the chassis' own down. Not to be changed.
  final Vector3 mount;

  final double radius;

  /// It turns with the steering.
  final bool steers;

  /// The engine turns it, through the differential of its axle.
  final bool drives;

  /// Newton metres the brake gives at full pressure. Zero is a wheel with no
  /// brake.
  final double brakeTorque;

  /// The handbrake holds this wheel. It uses the same brake.
  final bool handbrake;

  /// Kilogram square metres the wheel and tyre turn with.
  final double inertia;

  final SuspensionSpec suspension;

  final TyreSpec tyre;
}

/// A car as data.
///
/// The chassis' own front is along its negative z, as a camera's is. Wheels
/// that share a mount z are an axle. With [antiRoll] above zero every axle has
/// exactly two.
final class VehicleSpec {
  VehicleSpec({
    required this.mass,
    required List<WheelSpec> wheels,
    this.centre = const [0.0, 0.0, 0.0],
    this.antiRoll = 0.0,
    this.maxSteer = 0.5,
    this.drive,
    this.ground = Layers.everything,
  }) : wheels = List.unmodifiable(wheels),
       axles = _axlesOf(wheels, antiRoll),
       sprung = _sprungMass(wheels, mass, centre) {
    if (!(mass > 0)) {
      throw ArgumentError.value(mass, 'mass', 'must be above zero');
    }
    if (wheels.isEmpty) {
      throw ArgumentError.value(wheels, 'wheels', 'needs at least one');
    }
    if (drive == null && wheels.any((w) => w.drives)) {
      throw ArgumentError.value(
        wheels,
        'wheels',
        'a driven wheel needs a drive',
      );
    }
    if (drive != null && !wheels.any((w) => w.drives)) {
      throw ArgumentError.value(drive, 'drive', 'needs a wheel that drives');
    }
    if (!(maxSteer >= 0 && maxSteer < math.pi / 2)) {
      throw ArgumentError.value(
        maxSteer,
        'maxSteer',
        'must be 0 to a quarter turn',
      );
    }
  }

  /// Kilograms.
  final double mass;

  final List<WheelSpec> wheels;

  /// Where its weight is, in the chassis frame.
  final List<double> centre;

  /// Newtons a metre of difference in compression across an axle. Zero is no
  /// bar.
  final double antiRoll;

  /// Radians the middle of the steering axle turns at full lock.
  final double maxSteer;

  /// The engine and what joins it to the wheels that drive. Null is a car that
  /// is pushed.
  final DriveSpec? drive;

  /// The layers a wheel sees as ground, so a car does not stand on a
  /// character.
  final Layers ground;

  /// The pairs of wheels, by index, that an anti-roll bar joins.
  final List<(int, int)> axles;

  /// The kilograms each wheel carries at rest, in the order of [wheels].
  final List<double> sprung;

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

  /// Shares the mass out so that the wheels' loads balance about the centre of
  /// mass: an even share, tilted across the wheels as far as the centre is
  /// from their middle. Wheels in a line share it evenly.
  static List<double> _sprungMass(
    List<WheelSpec> wheels,
    double mass,
    List<double> centre,
  ) {
    final count = wheels.length;
    // The constructor refuses these with a clearer message once it is in.
    if (count == 0 || !(mass > 0)) return const [];

    final midX = wheels.fold(0.0, (sum, w) => sum + w.mount.x) / count;
    final midZ = wheels.fold(0.0, (sum, w) => sum + w.mount.z) / count;
    var xx = 0.0, xz = 0.0, zz = 0.0;
    for (final w in wheels) {
      final (x, z) = (w.mount.x - midX, w.mount.z - midZ);
      xx += x * x;
      xz += x * z;
      zz += z * z;
    }
    final det = xx * zz - xz * xz;
    final (wantX, wantZ) = (
      mass * (centre[0] - midX),
      mass * (centre[2] - midZ),
    );
    final (a, b) = det.abs() < 1e-9
        ? (0.0, 0.0)
        : ((wantX * zz - wantZ * xz) / det, (wantZ * xx - wantX * xz) / det);

    final loads = [
      for (final w in wheels)
        mass / count + a * (w.mount.x - midX) + b * (w.mount.z - midZ),
    ];
    if (loads.any((load) => !(load > 0))) {
      throw ArgumentError.value(
        centre,
        'centre',
        'the centre of mass must lie inside the wheels',
      );
    }
    return List.unmodifiable(loads);
  }
}
