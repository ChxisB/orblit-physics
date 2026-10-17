import 'package:vector_math/vector_math_64.dart';

/// The moving air a soft body sits in.
///
/// Declared here, next to the body that asks it, so a body can be blown by a
/// physics world, a noise field or a constant without knowing which.
abstract interface class SoftAir {
  /// The velocity of the air at [point], in metres a second. Asked once a
  /// step, at the middle of the body.
  Vector3 windAt(Vector3 point);
}
