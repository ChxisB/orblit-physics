/// A ragdoll for an Orblit rig.
///
/// Bodies and joints are made from a skeleton's bones. Limbs follow an
/// animated pose, go limp from the bone that was hit, and stand up again.
/// The skeleton comes from `orblit_rig` and the bodies from `orblit_physics`.
library;

export 'src/ragdoll.dart' show Ragdoll, RagdollMode, RagdollTuning;
export 'src/ragdoll_character.dart' show RagdollCharacter;
export 'src/ragdoll_pose.dart' show RagdollPose;
export 'src/spec.dart'
    show RagdollBall, RagdollBody, RagdollHinge, RagdollJoint, RagdollSpec;
