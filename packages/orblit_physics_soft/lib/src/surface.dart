import 'package:vector_math/vector_math_64.dart';

/// Where a swept ball met the world: a point on the surface and the way out
/// of it.
final class SoftContact {
  const SoftContact({required this.at, required this.normal});

  final Vector3 at;
  final Vector3 normal;
}

/// What a soft body asks of the world around it.
abstract interface class SoftSurface {
  /// Moves a ball of [radius] from [from] along [direction] for [distance]
  /// metres and answers the first surface it meets, or null. [direction] need
  /// not be a unit vector. A ball that starts clear of a surface answers a point
  /// on it. One that starts inside is met at once, and the point is only near
  /// the surface.
  SoftContact? sweep(
    Vector3 from,
    Vector3 direction,
    double distance,
    double radius,
  );
}
