import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'surface.dart';

/// The bodies of a [Physics] world, as a soft body meets them.
///
/// A world cannot cast a plane, so a floor a cloth lands on must be a box. A
/// trigger is never met. Step the world before the soft body, so the cast sees
/// where the bodies are now.
final class PhysicsSurface implements SoftSurface {
  PhysicsSurface(
    this.physics, {
    this.layers = Layers.everything,
    this.ignore = 0,
  });

  final Physics physics;

  /// What the cloth collides with.
  final Layers layers;

  /// A body the cloth passes through, such as the one it hangs from.
  final int ignore;

  @override
  SoftContact? sweep(
    Vector3 from,
    Vector3 direction,
    double distance,
    double radius,
  ) {
    final hit = physics.cast(
      from: from.storage,
      direction: direction.storage,
      distance: distance,
      shape: Shape.sphere(radius),
      layers: layers,
      ignore: ignore,
    );
    if (hit == null) return null;
    return SoftContact(
      at: Vector3.array(hit.at),
      normal: Vector3.array(hit.normal),
    );
  }
}
