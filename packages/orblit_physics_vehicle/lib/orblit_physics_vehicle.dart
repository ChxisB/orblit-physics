/// A car for an Orblit physics world.
///
/// A chassis body on wheels that are casts. Each wheel is held up by a spring
/// and a damper, and an anti-roll bar moves load between the two wheels of an
/// axle. Each tyre grips the ground, steers, and is braked, and an engine can
/// turn the wheels through a clutch, a gearbox and a differential. The
/// controller sits above the world and does nothing inside the solver.
library;

export 'src/drive_spec.dart' show DriveSpec, EngineSpec;
export 'src/input.dart' show VehicleInput;
export 'src/spec.dart' show SuspensionSpec, TyreSpec, VehicleSpec, WheelSpec;
export 'src/vehicle.dart' show SurfaceGrip, Vehicle, WheelState;
