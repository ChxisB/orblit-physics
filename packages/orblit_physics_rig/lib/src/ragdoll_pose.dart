import 'dart:math' as math;

import 'package:orblit_rig/orblit_rig.dart';
import 'package:vector_math/vector_math_64.dart';

import 'frame.dart';
import 'ragdoll.dart';

/// Runs a [Ragdoll] inside a [ModifierStack].
///
/// Put it after the modifiers that shape the animated pose and before the
/// ones that should follow what the ragdoll does, such as spring bones. Each
/// frame it sends the animated pose to the bodies, then writes the bones of
/// every free body from where the bodies are.
///
/// Step the physics world once a frame with the same `dt` the modifier frame
/// carries. The bones written are where the last step left the bodies, and
/// the animated bodies are given the velocity that reaches this pose in the
/// step that follows.
///
/// When a character stands up the bones jump from where the ragdoll lay to
/// where the get-up animation starts. [recover] hides that: for the seconds it
/// is given, the output is mixed from the last frame the ragdoll was drawn.
final class RagdollPose extends Modifier {
  RagdollPose(this.ragdoll);

  final Ragdoll ragdoll;

  /// Every bone's world matrix as of the last frame a body was free.
  Map<String, Matrix4> _last = const {};

  /// The matrices a recovery mixes away from.
  Map<String, Matrix4> _from = const {};
  double _total = 0;
  double _left = 0;

  /// Whether the output is still being mixed from a fallen pose.
  bool get recovering => _left > 0;

  /// Mixes the output from the last frame the ragdoll was drawn to the
  /// animated pose over [seconds], easing in and out.
  ///
  /// Call it in the frame the bodies are made animated again, before the
  /// next solve. With nothing drawn yet, or no time, it does nothing.
  void recover(double seconds) {
    if (seconds <= 0 || _last.isEmpty) {
      _left = 0;
      return;
    }
    _from = _last;
    _total = seconds;
    _left = seconds;
  }

  @override
  void check(Armature armature) => ragdoll.spec.check(armature);

  @override
  void solve(Pose pose, ModifierFrame frame) {
    ragdoll.followPose(pose, frame);
    ragdoll.writePose(pose, frame);
    _mixFromFallen(pose, frame);
    _remember(pose, frame);
  }

  @override
  void reset() {
    _last = const {};
    _from = const {};
    _left = 0;
  }

  void _mixFromFallen(Pose pose, ModifierFrame frame) {
    if (_left <= 0) return;
    _left = math.max(0, _left - frame.dt);
    final progress = 1 - _left / _total;
    final amount = progress * progress * (3 - 2 * progress);
    final fallen = _fallenPose(pose.armature, frame);
    for (final bone in pose.armature.bones) {
      final from = fallen[bone.name];
      final now = pose[bone.name];
      now.location.setFrom(
        from.location + (now.location - from.location) * amount,
      );
      now.rotation = slerpBetween(from.rotation, now.rotation, amount);
      now.scale.setFrom(from.scale + (now.scale - from.scale) * amount);
    }
    pose.refresh();
  }

  /// The remembered world matrices as a pose of their own, in the model's
  /// space as it stands now, so a model that has moved is mixed from where
  /// the bones really were.
  Pose _fallenPose(Armature armature, ModifierFrame frame) {
    final fallen = Pose(armature)..evaluate();
    for (final name in armature.evaluationOrder) {
      final world = _from[name];
      if (world == null) continue;
      placeBone(fallen, name, frame.worldToModel.multiplied(world));
    }
    return fallen;
  }

  void _remember(Pose pose, ModifierFrame frame) {
    if (!ragdoll.hasFree) return;
    _last = {
      for (final bone in pose.armature.bones)
        bone.name: frame.modelToWorld.multiplied(pose.worldOf(bone.name)),
    };
  }
}
