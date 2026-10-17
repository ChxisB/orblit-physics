import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

/// The point [local] on physics body [id], in the world, for the `where` of
/// `SoftBody.attach`. It turns with the body.
///
/// Throws once the body is gone, so detach the cloth before removing it.
Vector3 Function() bodyPoint(Physics physics, int id, Vector3 local) {
  final offset = local.clone();
  return () {
    final pose = physics.transformOf(id);
    if (pose == null) throw StateError('Body $id is not in the world');
    // asRotationMatrix, not Quaternion.rotated, which turns the other way.
    final turn = Quaternion(
      pose[3],
      pose[4],
      pose[5],
      pose[6],
    ).asRotationMatrix();
    return turn.transformed(offset)..add(Vector3(pose[0], pose[1], pose[2]));
  };
}
