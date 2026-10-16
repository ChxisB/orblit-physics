import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_rig/orblit_physics_rig.dart';
import 'package:orblit_rig/orblit_rig.dart';
import 'package:vector_math/vector_math_64.dart';

/// A standing figure with its arms out to the sides, so that an arm which goes
/// limp has somewhere to fall to. Feet are 0.05 above the ground.
Armature humanRig() {
  Bone bone(
    String name,
    List<double> head,
    List<double> tail, {
    String? parent,
    bool connected = false,
  }) => Bone(
    name: name,
    head: Vector3(head[0], head[1], head[2]),
    tail: Vector3(tail[0], tail[1], tail[2]),
    parent: parent,
    connected: connected,
  );

  return Armature([
    bone('hips', [0, 1.0, 0], [0, 1.15, 0]),
    bone('spine', [0, 1.15, 0], [0, 1.45, 0], parent: 'hips', connected: true),
    bone('head', [0, 1.45, 0], [0, 1.75, 0], parent: 'spine', connected: true),
    for (final side in const [('L', 1.0), ('R', -1.0)]) ...[
      bone(
        'upper_arm.${side.$1}',
        [0.15 * side.$2, 1.4, 0],
        [0.45 * side.$2, 1.4, 0],
        parent: 'spine',
      ),
      bone(
        'forearm.${side.$1}',
        [0.45 * side.$2, 1.4, 0],
        [0.75 * side.$2, 1.4, 0],
        parent: 'upper_arm.${side.$1}',
        connected: true,
      ),
      bone(
        'thigh.${side.$1}',
        [0.1 * side.$2, 1.0, 0],
        [0.1 * side.$2, 0.55, 0],
        parent: 'hips',
      ),
      bone(
        'shin.${side.$1}',
        [0.1 * side.$2, 0.55, 0],
        [0.1 * side.$2, 0.05, 0],
        parent: 'thigh.${side.$1}',
        connected: true,
      ),
    ],
  ]);
}

/// A torso running from hips to spine as one body, so the two bones between
/// them are the coarse map, and a body for every limb bone.
RagdollSpec humanSpec() => RagdollSpec([
  RagdollBody(bone: 'spine', from: 'hips', radius: 0.14, mass: 20),
  RagdollBody(
    bone: 'head',
    parent: 'spine',
    joint: RagdollBall(swing: 0.6),
    radius: 0.1,
    mass: 4,
  ),
  for (final side in const ['L', 'R']) ...[
    RagdollBody(
      bone: 'upper_arm.$side',
      parent: 'spine',
      joint: RagdollBall(swing: 1.5),
      radius: 0.05,
      mass: 2,
    ),
    RagdollBody(
      bone: 'forearm.$side',
      parent: 'upper_arm.$side',
      joint: RagdollHinge(),
      radius: 0.04,
      mass: 1.5,
    ),
    RagdollBody(
      bone: 'thigh.$side',
      parent: 'spine',
      joint: RagdollBall(swing: 1.2),
      radius: 0.07,
      mass: 7,
    ),
    RagdollBody(
      bone: 'shin.$side',
      parent: 'thigh.$side',
      joint: RagdollHinge(),
      radius: 0.05,
      mass: 4,
    ),
  ],
]);

/// The layer a character's capsule is in. The ragdoll takes no notice of it.
const characterLayer = 2;

/// A physics world with ground, the figure, and its ragdoll standing in it.
final class Harness {
  Harness({RagdollTuning tuning = const RagdollTuning()}) {
    physics.add(
      900,
      shape: const Shape.plane(0, 1, 0),
      motion: PhysicsMotion.fixed,
    );
    pose.evaluate();
    ragdoll = Ragdoll.build(
      physics: physics,
      pose: pose,
      spec: humanSpec(),
      modelToWorld: model,
      ignore: characterLayer,
      tuning: tuning,
    );
    modifier = RagdollPose(ragdoll);
  }

  static const dt = 1 / 60;

  final Physics physics = Physics();
  final Armature armature = humanRig();
  late final Pose pose = Pose(armature);
  late final Ragdoll ragdoll;
  late final RagdollPose modifier;

  Matrix4 model = Matrix4.identity();

  /// What the clip does this frame, written into a pose that starts at rest.
  void Function(Pose pose)? animate;

  /// One frame as a game runs it: the clip, the modifier, then the step.
  void frame() {
    pose.reset();
    animate?.call(pose);
    pose.evaluate();
    modifier.solve(pose, ModifierFrame(dt: dt, modelToWorld: model));
    physics.step(dt);
  }

  void run(double seconds) {
    for (var i = 0; i < (seconds / dt).round(); i++) {
      frame();
    }
  }

  /// Where a bone's head is in the world after the last frame's output.
  Vector3 headOf(String bone) => model.transformed3(pose.headOf(bone));

  /// The world position of the body that covers [bone].
  Vector3 bodyOf(String bone) {
    final t = physics.transformOf(ragdoll.idOf(bone))!;
    return Vector3(t[0], t[1], t[2]);
  }

  void dispose() => physics.dispose();
}
