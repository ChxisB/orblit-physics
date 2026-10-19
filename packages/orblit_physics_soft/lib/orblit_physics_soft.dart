/// Cloth, rope and balloons for an Orblit physics world.
///
/// Particles joined by distance links, solved as position constraints in
/// fixed sub-steps. The links are data, so a cloth and a rope differ by which
/// ones they are made of. A balloon adds a constraint on the volume its
/// closed surface encloses. Rigid bodies move particles. The
/// particles do not move rigid bodies.
library;

export 'src/air.dart' show SoftAir;
export 'src/body_point.dart' show bodyPoint;
export 'src/physics_surface.dart' show PhysicsSurface;
export 'src/physics_wind.dart' show PhysicsWind;
export 'src/soft_body.dart' show SoftBody;
export 'src/spec.dart' show SoftSpec;
export 'src/surface.dart' show SoftContact, SoftSurface;
