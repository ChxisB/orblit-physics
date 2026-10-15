import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_rig/orblit_rig.dart';
import 'package:vector_math/vector_math_64.dart';

import 'frame.dart';
import 'spec.dart';
import 'turn_angles.dart';

/// How a ragdoll body is held.
enum RagdollMode {
  /// A driven body that is set to where the pose puts it every step. It
  /// pushes what it meets and is pushed by nothing.
  animated,

  /// A free body whose joint's springs pull it to the pose's angles. It gives
  /// under a blow and comes back. A root has no joint to spring, so a powered
  /// root is animated.
  powered,

  /// A free body held by its joint's limits and nothing else.
  limp,
}

/// One body of a ragdoll with everything worked out from the rest pose, so
/// that nothing is measured from the armature again after it is built.
final class BodyPart {
  BodyPart._({
    required this.body,
    required this.index,
    required this.id,
    required this.parent,
    required this.shape,
    required this.offset,
    required this.restBone,
    required this.restBody,
    required this.axisTurn,
    required this.restInParent,
    required this.jointInParent,
  });

  /// Works a body out of [armature]'s rest pose. Lengths are in the world's
  /// units, scaled by [scale], and [parent] is the part this one hangs from.
  factory BodyPart.of(
    Armature armature,
    RagdollBody body, {
    required int index,
    required int id,
    required BodyPart? parent,
    required double scale,
  }) {
    final restBone = Frame.fromMatrix(armature.restOf(body.from));
    final last = armature[body.bone]!;
    final tail = armature
        .restOf(body.bone)
        .transform3(Vector3(0, last.length, 0));
    final span = tail - restBone.position;
    final length = span.length;

    // The capsule stands along the line from head to far tail, which is the
    // bone's own Y when the run of bones is straight and the nearest to it
    // when it is not.
    final back = Frame(Vector3.zero(), restBone.rotation.conjugated());
    final along = length < 1e-9 ? Vector3(0, 1, 0) : back.turn(span / length);
    final centre = back.turn(span * 0.5);
    final offset = Frame(centre, rotationBetween(Vector3(0, 1, 0), along));
    final restBody = restBone.then(offset);

    final twist = switch (body.joint) {
      final RagdollHinge hinge => hinge.axis,
      _ => Vector3(0, 1, 0),
    };
    final axisTurn = rotationBetween(Vector3(1, 0, 0), twist);
    final frame = Frame(restBone.position, restBone.rotation * axisTurn);

    return BodyPart._(
      body: body,
      index: index,
      id: id,
      parent: parent?.index,
      shape: Shape.capsule(
        body.radius * scale,
        math.max(0.0, length / 2 - body.radius) * scale,
      ),
      offset: offset,
      restBone: restBone,
      restBody: restBody,
      axisTurn: axisTurn,
      restInParent: parent == null
          ? restBody
          : restBody.within(parent.restBody).scaled(scale),
      jointInParent: parent == null
          ? frame
          : frame.within(parent.restBody).scaled(scale),
    );
  }

  final RagdollBody body;
  final int index;

  /// The body's name in the physics world, which is also its joint's.
  final int id;

  /// The part this one hangs from, or null for the root.
  final int? parent;

  final List<int> children = [];

  /// A capsule in the world's units.
  final Shape shape;

  /// The body's centre and turn as seen from the head of [RagdollBody.from].
  final Frame offset;

  /// The first bone's frame at rest, in the model's space.
  final Frame restBone;

  /// The body's frame at rest, in the model's space.
  final Frame restBody;

  /// The turn from the joint's frame to the first bone's own, which puts the
  /// joint's x axis along the bone for a ball and along the hinge for a
  /// hinge.
  final Quaternion axisTurn;

  /// The body's rest frame in its parent body's, lengths in world units.
  final Frame restInParent;

  /// The joint's frame in its parent body's, lengths in world units.
  final Frame jointInParent;

  RagdollMode mode = RagdollMode.animated;

  /// Whether the joint now standing has springs on it.
  bool sprung = false;

  /// The time on the ragdoll's clock this part goes limp, or null.
  double? limpAt;

  String get bone => body.bone;

  /// The turn of the joint's frame at rest in the model's space.
  Quaternion get restFrame => restBone.rotation * axisTurn;

  /// Where each way of the joint is pulled to for [turn], the second body's
  /// frame seen from the first's, kept inside what the joint allows.
  ///
  /// Six places in the order the joint's ways are named. A null leaves a way
  /// where it is, which is every way a pose has no say in.
  List<double?> restsFor(Quaternion turn) {
    final angles = turnAngles(turn);
    double within(double value, double low, double high) =>
        math.min(high, math.max(low, value));
    switch (body.joint) {
      case RagdollHinge(:final low, :final high):
        return [null, null, null, within(angles[0], low, high), null, null];
      case RagdollBall(:final swing, :final twistLow, :final twistHigh):
        return [
          null,
          null,
          null,
          within(angles[0], twistLow, twistHigh),
          within(angles[1], -swing, swing),
          within(angles[2], -swing, swing),
        ];
      case null:
        return List.filled(6, null);
    }
  }
}
