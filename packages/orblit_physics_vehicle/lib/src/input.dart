/// What the driver asks of the car for the next step.
final class VehicleInput {
  VehicleInput({this.steer = 0.0, this.brake = 0.0, this.handbrake = false}) {
    if (!(steer >= -1 && steer <= 1)) {
      throw ArgumentError.value(steer, 'steer', 'must be -1 to 1');
    }
    if (!(brake >= 0 && brake <= 1)) {
      throw ArgumentError.value(brake, 'brake', 'must be 0 to 1');
    }
  }

  /// Full lock to the left is one and to the right is minus one.
  final double steer;

  /// How hard the brake pedal is pressed.
  final double brake;

  /// Holds the wheels that have a handbrake, with the whole of their brake.
  final bool handbrake;
}
