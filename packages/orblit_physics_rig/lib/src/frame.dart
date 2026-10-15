import 'dart:math' as math;

import 'package:orblit_rig/orblit_rig.dart';
import 'package:vector_math/vector_math_64.dart';

/// Where something is and which way it faces, with no size.
///
/// A body's place in the world and a bone's place in the model are both
/// this, and the ragdoll's work is carrying one between the two.
final class Frame {
  Frame(Vector3 position, Quaternion rotation)
    : position = position.clone(),
      rotation = rotation.clone();

  /// The frame a matrix puts things in, with its scale taken out.
  factory Frame.fromMatrix(Matrix4 matrix) {
    final position = Vector3.zero();
    final rotation = Quaternion.identity();
    matrix.decompose(position, rotation, Vector3.zero());
    return Frame(position, rotation..normalize());
  }

  final Vector3 position;
  final Quaternion rotation;

  /// [vector] turned by this frame's rotation.
  ///
  /// By hand, because `Quaternion.rotated` turns by the inverse of the
  /// rotation it is given.
  Vector3 turn(Vector3 vector) {
    final axis = Vector3(rotation.x, rotation.y, rotation.z);
    final across = axis.cross(vector) * 2;
    return vector + across * rotation.w + axis.cross(across);
  }

  /// [local], which is given in this frame, as a frame of the outer space.
  Frame then(Frame local) => Frame(
    position + turn(local.position),
    (rotation * local.rotation)..normalize(),
  );

  /// The same frame with its position [factor] times as far from the origin.
  Frame scaled(double factor) => Frame(position * factor, rotation);

  /// This frame's place as seen from [outer].
  Frame within(Frame outer) {
    final back = Frame(Vector3.zero(), outer.rotation.conjugated());
    return Frame(
      back.turn(position - outer.position),
      (back.rotation * rotation)..normalize(),
    );
  }
}

/// The model's place in the world, which is what a rig's bones are measured
/// from.
final class ModelSpace {
  ModelSpace(this._frame)
    : rotation = Frame.fromMatrix(_frame.modelToWorld).rotation;

  final ModifierFrame _frame;

  /// The turn of the model's axes in the world's.
  final Quaternion rotation;

  /// How much longer a length is in the world than in the model.
  double get scale => _frame.scale;

  Frame toWorld(Frame model) => Frame(
    _frame.toWorld(model.position),
    (rotation * model.rotation)..normalize(),
  );

  Frame toModel(Frame world) => Frame(
    _frame.toModel(world.position),
    (rotation.conjugated() * world.rotation)..normalize(),
  );
}

/// The turn from [before] to [after] as a vector along its axis, whose length
/// is the angle in radians, the shorter way round.
Vector3 turnBetween(Quaternion before, Quaternion after) {
  final turn = after * before.conjugated();
  if (turn.w < 0) {
    turn.setValues(-turn.x, -turn.y, -turn.z, -turn.w);
  }
  final part = Vector3(turn.x, turn.y, turn.z);
  final sine = part.length;
  if (sine < 1e-9) return part * 2;
  return part * (2 * math.atan2(sine, turn.w) / sine);
}

/// [from] taken [amount] of the way to [to], by the shorter arc.
Quaternion slerpBetween(Quaternion from, Quaternion to, double amount) {
  var dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
  final sign = dot < 0 ? -1.0 : 1.0;
  dot *= sign;
  if (dot > 0.9995) {
    return Quaternion(
      from.x + (sign * to.x - from.x) * amount,
      from.y + (sign * to.y - from.y) * amount,
      from.z + (sign * to.z - from.z) * amount,
      from.w + (sign * to.w - from.w) * amount,
    )..normalize();
  }
  final angle = math.acos(dot.clamp(-1.0, 1.0));
  final sine = math.sin(angle);
  final a = math.sin((1 - amount) * angle) / sine;
  final b = math.sin(amount * angle) / sine * sign;
  return Quaternion(
    from.x * a + to.x * b,
    from.y * a + to.y * b,
    from.z * a + to.z * b,
    from.w * a + to.w * b,
  )..normalize();
}
