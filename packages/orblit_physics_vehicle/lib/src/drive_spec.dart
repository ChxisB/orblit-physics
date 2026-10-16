/// An engine: the torque it gives at each speed with the pedal down.
final class EngineSpec {
  EngineSpec({
    required List<(double, double)> torque,
    this.idle = 900.0,
    this.inertia = 0.25,
    this.drag = 40.0,
  }) : torque = List.unmodifiable(torque) {
    if (torque.length < 2) {
      throw ArgumentError.value(torque, 'torque', 'needs two points at least');
    }
    for (var i = 0; i < torque.length; i++) {
      final (rpm, newtonMetres) = torque[i];
      if (!(rpm > 0 && newtonMetres >= 0)) {
        throw ArgumentError.value(torque, 'torque', 'point $i is out of range');
      }
      if (i > 0 && !(rpm > torque[i - 1].$1)) {
        throw ArgumentError.value(torque, 'torque', 'must rise in rpm');
      }
    }
    if (!(idle > 0 && idle < redline)) {
      throw ArgumentError.value(
        idle,
        'idle',
        'must be above 0, to the redline',
      );
    }
    if (!(inertia > 0)) {
      throw ArgumentError.value(inertia, 'inertia', 'must be above zero');
    }
    if (!(drag >= 0)) {
      throw ArgumentError.value(drag, 'drag', 'must not be negative');
    }
  }

  /// Revolutions a minute and the newton metres at the crank with the pedal
  /// down, in order of speed. Between two points it is a straight line.
  final List<(double, double)> torque;

  /// Revolutions a minute it ticks over at with the pedal up.
  final double idle;

  /// Kilogram square metres the engine turns with.
  final double inertia;

  /// Newton metres the engine drags at the redline with the pedal up. It falls
  /// away in a straight line to nothing at rest.
  final double drag;

  /// The top speed on the curve. The engine gives nothing a little past it.
  double get redline => torque.last.$1;

  /// The most newton metres on the curve.
  double get peak =>
      torque.fold(0.0, (most, point) => point.$2 > most ? point.$2 : most);

  /// Newton metres at `rpm` with the pedal down.
  double full(double rpm) {
    if (rpm <= torque.first.$1) return torque.first.$2;
    for (var i = 1; i < torque.length; i++) {
      final (high, top) = torque[i];
      if (rpm > high) continue;

      final (low, bottom) = torque[i - 1];
      return bottom + (top - bottom) * (rpm - low) / (high - low);
    }
    return torque.last.$2;
  }
}

/// What carries an engine's torque to the driven wheels: a clutch, a gearbox
/// and a differential, shifting by itself unless the driver picks a gear.
///
/// Ratios are turns of the engine for a turn of the wheel. The final drive
/// multiplies every gear. The clutch is a centrifugal one: it holds nothing at
/// idle and grips harder as the engine speeds up, so a car pulls away without
/// stalling.
final class DriveSpec {
  DriveSpec({
    required this.engine,
    required List<double> ratios,
    this.reverse = 3.2,
    this.finalDrive = 3.7,
    double? clutch,
    double? engage,
    this.lock = 0.0,
    this.shiftTime = 0.3,
    this.upshift = 0.85,
    this.downshift = 0.35,
  }) : ratios = List.unmodifiable(ratios),
       clutch = clutch ?? engine.peak * 1.5,
       engage = engage ?? engine.idle + (engine.redline - engine.idle) * 0.25 {
    if (ratios.isEmpty || ratios.any((ratio) => !(ratio > 0))) {
      throw ArgumentError.value(ratios, 'ratios', 'needs gears above zero');
    }
    if (!(reverse > 0)) {
      throw ArgumentError.value(reverse, 'reverse', 'must be above zero');
    }
    if (!(finalDrive > 0)) {
      throw ArgumentError.value(finalDrive, 'finalDrive', 'must be above zero');
    }
    if (!(this.clutch > 0)) {
      throw ArgumentError.value(clutch, 'clutch', 'must be above zero');
    }
    if (!(this.engage > engine.idle * 1.1 && this.engage < engine.redline)) {
      throw ArgumentError.value(
        engage,
        'engage',
        'must be above idle by a tenth, to the redline',
      );
    }
    if (!(lock >= 0)) {
      throw ArgumentError.value(lock, 'lock', 'must not be negative');
    }
    if (!(shiftTime >= 0)) {
      throw ArgumentError.value(shiftTime, 'shiftTime', 'must not be negative');
    }
    if (!(downshift > 0 && downshift < upshift && upshift <= 1)) {
      throw ArgumentError.value(
        upshift,
        'upshift',
        'must be above downshift, and at most one',
      );
    }
    for (var i = 1; i < ratios.length; i++) {
      if (downshift * ratios[i - 1] / ratios[i] >= upshift) {
        throw ArgumentError.value(
          ratios,
          'ratios',
          'a downshift from gear ${i + 1} would land above the upshift',
        );
      }
    }
  }

  final EngineSpec engine;

  /// The forward gears, first to last.
  final List<double> ratios;

  final double reverse;
  final double finalDrive;

  /// The most newton metres the clutch holds once it has taken hold.
  final double clutch;

  /// Revolutions a minute the clutch takes hold at in full. It starts to bite a
  /// tenth above idle.
  final double engage;

  /// Newton metres of difference the differential holds between its two wheels.
  /// Zero is an open one, which gives each wheel the same torque. Infinity is a
  /// locked one. Between is a limited slip.
  final double lock;

  /// Seconds the clutch takes to come back after a shift.
  final double shiftTime;

  /// The shares of the redline at which the box changes up and down.
  final double upshift;
  final double downshift;
}
