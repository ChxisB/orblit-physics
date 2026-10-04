import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_scene/orblit_scene.dart';

/// Reads only a part's geometry. Body dynamics belong to its owning entity.
ShapePart compoundPart(BodyPart part, int Function(List<double>) hullOf) {
  final radius = part.radius;
  final straight = math.max(0.0, part.height / 2 - radius);
  final shape = switch (part.shape) {
    BodyShape.box => Shape.box(
      part.size.x / 2,
      part.size.y / 2,
      part.size.z / 2,
    ),
    BodyShape.sphere => Shape.sphere(radius),
    BodyShape.capsule => Shape.capsule(radius, straight),
    BodyShape.cylinder => Shape.cylinder(radius, part.height / 2),
    BodyShape.hull => Shape.hull(hullOf(part.hull)),
    BodyShape.mesh ||
    BodyShape.plane ||
    BodyShape.compound => throw ArgumentError('A part must be convex.'),
  };
  return ShapePart(
    shape: shape,
    at: part.centre.storage,
    rotation: part.rotation.storage,
    scale: part.scale.storage,
  );
}
