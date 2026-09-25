/// Rigid body physics for Orblit.
///
/// A sequential impulse solver in C++, reached over a C ABI. Dart describes
/// what it wants as a queue of commands and reads a whole world back in one
/// crossing, so a step costs two calls rather than two per body.
///
/// Bodies are named by whatever integer the caller already uses for them — an
/// entity id, usually. The solver never looks inside one.
library;

export 'src/world.dart'
    show
        Layers,
        Physics,
        PhysicsEvent,
        PhysicsEventKind,
        PhysicsFooting,
        PhysicsHit,
        PhysicsMotion,
        PhysicsSettings,
        Shape;
