import 'dart:math' as math;
import 'dart:typed_data';

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

/// How close to a quarter turn about Y a rotation must be, as the cosine of
/// the Y angle, for X to be let go. Float rounding is the whole of what is
/// there to read when nothing is known of the angles it had.
const _roundingBand = 1e-6;

/// The same, when the angles it had are known. A body that lands flat keeps a
/// tilt of a few hundred-thousandths of a radian that nothing levels, and X
/// read from that is a large angle that means nothing, so the crate would be
/// shown turned about X and back about Z.
const _settledBand = 1e-3;

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
/// With [near] that holds within a thousandth of the quarter turn, and the
/// angles then differ from the rotation by about that.
Vector3 eulerOf(Matrix3 rotation, {Vector3? near}) {
  double r(int row, int column) => rotation.entry(row, column);

  final cosY = math.sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
  final y = math.atan2(-r(2, 0), cosY);
  final band = near == null ? _roundingBand : _settledBand;

  // Near the quarter turn the angle X is read from is two numbers each about
  // as small as the noise in them, and means nothing. It is then the angle
  // [near] has, in whichever of the two writings is the one nearby, which is
  // a half turn out in the other.
  final held = radians(near?.x ?? 0);
  final writings = cosY > band
      ? [_written(rotation, y, math.atan2(r(2, 1), r(2, 2)))]
      : [_written(rotation, y, held), _written(rotation, y, held - math.pi)];
  if (near == null) return writings.first;

  // The other writing of the same rotation: half a turn about X and Z, and Y
  // reflected about a quarter turn.
  final candidates = [
    for (final found in writings) ...[
      _unwrapped(found, near),
      _unwrapped(Vector3(found.x + 180, 180 - found.y, found.z + 180), near),
    ],
  ];
  return candidates.reduce(
    (a, b) => _apart(a, near) <= _apart(b, near) ? a : b,
  );
}

/// The angles for [rotation] given Y and X, in degrees. Z is read from what is
/// left once X is taken back out, rather than from the matrix as it came:
/// whatever X turned out to be, Z makes up the difference, so the angles always
/// describe the rotation they were read from.
Vector3 _written(Matrix3 rotation, double y, double x) {
  double r(int row, int column) => rotation.entry(row, column);

  final sinX = math.sin(x);
  final cosX = math.cos(x);
  final z = math.atan2(
    sinX * r(0, 2) - cosX * r(0, 1),
    cosX * r(1, 1) - sinX * r(1, 2),
  );
  return Vector3(degrees(x), degrees(y), degrees(z));
}

/// Writes into [out] the pose [alpha] of the way from [from] to [to], where
/// [alpha] is 0 to 1. A pose is seven floats: translation xyz then rotation
/// xyzw, as the world reports it.
///
/// The translation goes in a straight line. The rotation goes the short way
/// round along the arc, so a body turns at an even rate across the blend.
///
/// A rotation that has not changed is copied as it is. A body at rest is the
/// same pose in both, and rounding in the blend must not be read as movement.
void blendPose(
  Float32List from,
  Float32List to,
  double alpha,
  Float32List out,
) {
  for (var i = 0; i < 3; i++) {
    out[i] = from[i] + (to[i] - from[i]) * alpha;
  }
  if (_sameTurn(from, to)) {
    out.setRange(3, 7, to, 3);
    return;
  }

  // A turn and its negative are the same turn. Take the one that is near.
  final dot =
      from[3] * to[3] + from[4] * to[4] + from[5] * to[5] + from[6] * to[6];
  final cosine = dot.abs();
  var weightFrom = 1 - alpha;
  var weightTo = alpha;
  // Close together the arc is a line, and its sine is too small to divide by.
  if (cosine < 0.9995) {
    final angle = math.acos(cosine);
    final sine = math.sin(angle);
    weightFrom = math.sin((1 - alpha) * angle) / sine;
    weightTo = math.sin(alpha * angle) / sine;
  }
  if (dot < 0) weightTo = -weightTo;

  final turn = Quaternion(from[3], from[4], from[5], from[6])
    ..scale(weightFrom);
  turn
    ..add(Quaternion(to[3], to[4], to[5], to[6])..scale(weightTo))
    ..normalize();
  out[3] = turn.x;
  out[4] = turn.y;
  out[5] = turn.z;
  out[6] = turn.w;
}

bool _sameTurn(Float32List a, Float32List b) {
  for (var i = 3; i < 7; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
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
