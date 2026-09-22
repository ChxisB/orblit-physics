import 'dart:math' as math;

import 'package:orblit_scene/orblit_scene.dart';
import 'package:vector_math/vector_math_64.dart';

/// Degrees about X, Y and Z as a rotation, composed Z, then Y, then X.
///
/// The order the stage draws a scene in, and so the only order that puts a
/// body where its entity is drawn. Euler angles do not commute: composed any
/// other way, a crate turned about two axes is simulated somewhere near where
/// it is drawn, which is worse than somewhere obviously wrong.
Matrix4 rotationOf(Vector3 degrees) => Matrix4.rotationZ(radians(degrees.z))
  ..multiply(Matrix4.rotationY(radians(degrees.y)))
  ..multiply(Matrix4.rotationX(radians(degrees.x)));

/// Where an entity sits relative to its parent. No transform is no offset: a
/// folder in the outliner does not move what is in it.
Matrix4 localOf(TransformComponent? transform) {
  if (transform == null) return Matrix4.identity();
  return Matrix4.identity()
    ..setTranslation(transform.position)
    ..multiply(rotationOf(transform.rotation))
    ..multiply(Matrix4.diagonal3(transform.scale));
}

/// Where an entity sits in the world: its own transform under every one of its
/// parents'. The identity for null or an id the document does not have.
///
/// [overriding] stands in for the transforms of entities that have moved but
/// are not in [document] yet, so a pass that moves a parent and then its child
/// measures the child against where the parent went.
Matrix4 worldOf(
  SceneDocument document,
  String? id, {
  Map<String, TransformComponent> overriding = const {},
}) {
  final chain = <SceneEntity>[];
  final seen = <String>{};
  var at = id;
  // A parent that is missing, or that is the entity itself, is the root — the
  // rule the stage follows, so a document it would draw is one this can read.
  // The set is only a guard: decode breaks every loop long before here.
  while (at != null && seen.add(at)) {
    final entity = document[at];
    if (entity == null) break;
    chain.add(entity);
    at = entity.parent;
  }

  final world = Matrix4.identity();
  for (final entity in chain.reversed) {
    final transform =
        overriding[entity.id] ?? entity[SceneComponents.transform];
    world.multiply(localOf(transform is TransformComponent ? transform : null));
  }
  return world;
}

/// A rotation as degrees about X, Y and Z: the inverse of [rotationOf].
///
/// Every rotation can be written two ways in these three angles, and any of
/// them can have a whole turn added. Given [near] — the angles the entity had
/// before — the answer is the writing closest to those, so a crate that tips
/// over turns smoothly in the inspector instead of jumping between two sets of
/// numbers that happen to mean the same thing.
///
/// A quarter turn about Y — a crate turned to face sideways — is where X and Z
/// turn about the same axis and only how far they turn between them is known.
/// There X keeps the angle [near] gives it, or none, and Z takes the rest.
Vector3 eulerOf(Matrix3 rotation, {Vector3? near}) {
  double r(int row, int column) => rotation.entry(row, column);

  final cosY = math.sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
  final y = math.atan2(-r(2, 0), cosY);
  // Near the quarter turn the angle X is read from is two numbers each about
  // as small as the rounding in them, and means nothing.
  final x = cosY > 1e-6 ? math.atan2(r(2, 1), r(2, 2)) : radians(near?.x ?? 0);
  // Z from what is left once X is taken back out, rather than from the matrix
  // as it came: whatever X turned out to be, Z makes up the difference, so the
  // angles always describe the rotation they were read from.
  final sinX = math.sin(x);
  final cosX = math.cos(x);
  final z = math.atan2(
    sinX * r(0, 2) - cosX * r(0, 1),
    cosX * r(1, 1) - sinX * r(1, 2),
  );

  final found = Vector3(degrees(x), degrees(y), degrees(z));
  if (near == null) return found;

  // The other writing of the same rotation: half a turn about X and Z, and Y
  // reflected about a quarter turn.
  final other = Vector3(found.x + 180, 180 - found.y, found.z + 180);
  final a = _unwrapped(found, near);
  final b = _unwrapped(other, near);
  return _apart(a, near) <= _apart(b, near) ? a : b;
}

/// [angles] with whole turns added to each until it is within half a turn of
/// the same angle in [near].
Vector3 _unwrapped(Vector3 angles, Vector3 near) => Vector3(
  _toward(angles.x, near.x),
  _toward(angles.y, near.y),
  _toward(angles.z, near.z),
);

double _toward(double angle, double near) =>
    angle + 360 * ((near - angle) / 360).roundToDouble();

double _apart(Vector3 a, Vector3 b) =>
    (a.x - b.x).abs() + (a.y - b.y).abs() + (a.z - b.z).abs();
