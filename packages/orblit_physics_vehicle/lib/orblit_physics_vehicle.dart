/// A car for an Orblit physics world.
///
/// A chassis body on wheels that are casts. Each wheel is held up by a spring
/// and a damper, and an anti-roll bar moves load between the two wheels of an
/// axle. The controller sits above the world and does nothing inside the
/// solver.
library;

export 'src/spec.dart' show SuspensionSpec, VehicleSpec, WheelSpec;
export 'src/vehicle.dart' show Vehicle, WheelState;
