/// Cloth and rope for an Orblit physics world.
///
/// Particles joined by distance links, solved as position constraints in
/// fixed sub-steps. The links are data, so a cloth, a rope and later a jelly
/// differ by which ones they are made of. Rigid bodies move particles. The
/// particles do not move rigid bodies.
library;

export 'src/physics_surface.dart' show PhysicsSurface;
export 'src/soft_body.dart' show SoftBody;
export 'src/spec.dart' show SoftSpec;
export 'src/surface.dart' show SoftContact, SoftSurface;
