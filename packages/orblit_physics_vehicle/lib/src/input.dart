/// What the driver asks of the car for the next step.
final class VehicleInput {
  VehicleInput({
    this.steer = 0.0,
    this.throttle = 0.0,
    this.brake = 0.0,
    this.handbrake = false,
    this.gear,
  }) {
    if (!(steer >= -1 && steer <= 1)) {
      throw ArgumentError.value(steer, 'steer', 'must be -1 to 1');
    }
    if (!(throttle >= 0 && throttle <= 1)) {
      throw ArgumentError.value(throttle, 'throttle', 'must be 0 to 1');
    }
    if (!(brake >= 0 && brake <= 1)) {
      throw ArgumentError.value(brake, 'brake', 'must be 0 to 1');
    }
    final gear = this.gear;
    if (gear != null && gear < -1) {
      throw ArgumentError.value(gear, 'gear', 'must be -1 or more');
    }
  }

  /// Full lock to the left is one and to the right is minus one.
  final double steer;

  /// How far the accelerator is pressed.
  final double throttle;

  /// How hard the brake pedal is pressed.
  final double brake;

  /// Holds the wheels that have a handbrake, with the whole of their brake.
  final bool handbrake;

  /// Reverse is minus one, neutral is zero and the forward gears count up from
  /// one. Null lets the box pick among the forward gears.
  final int? gear;
}
