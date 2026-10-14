/// A scene document's bodies, simulated.
///
/// [ScenePhysics] builds an `orblit_physics` world from the entities in an
/// `orblit_scene` document that have a body, steps it, and answers with the
/// diff that moves those entities to where the simulation put them.
library;

export 'src/character_walk.dart' show CharacterWalk;
export 'src/root_motion_walk.dart' show RootMotionWalk;
export 'src/scene_physics.dart' show ScenePhysics, ScenePhysicsSnapshot;
