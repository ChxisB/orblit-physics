import 'package:orblit_rig/orblit_rig.dart';
import 'package:vector_math/vector_math_64.dart';

/// How one ragdoll body moves against the body it hangs from.
sealed class RagdollJoint {
  const RagdollJoint();
}

/// A joint that bends about one axis: a knee, an elbow, a finger.
///
/// [axis] is in the bone's own space, whose Y runs head to tail. [low] and
/// [high] are how far it may turn from the pose the rig rests in, in radians,
/// so a knee that only bends one way is `low: 0, high: 2.4`.
final class RagdollHinge extends RagdollJoint {
  RagdollHinge({Vector3? axis, this.low = 0, this.high = 2.4})
    : axis = (axis ?? Vector3(1, 0, 0)).normalized() {
    if (this.axis.length2 < 0.5) {
      throw ArgumentError.value(axis, 'axis', 'A hinge needs an axis.');
    }
    if (!low.isFinite || !high.isFinite || low > high) {
      throw ArgumentError('A hinge goes from low to high, with low first.');
    }
  }

  final Vector3 axis;
  final double low;
  final double high;
}

/// A joint that turns any way within a cone: a shoulder, a hip, a neck.
///
/// [swing] is how far the bone may tip from where it rests, in radians, in
/// any direction. [twistLow] and [twistHigh] are how far it may turn about its
/// own length.
final class RagdollBall extends RagdollJoint {
  RagdollBall({this.swing = 1.0, this.twistLow = -0.5, this.twistHigh = 0.5}) {
    if (!swing.isFinite || swing < 0 || swing > 3) {
      throw ArgumentError.value(swing, 'swing', 'Swing is 0 to 3 radians.');
    }
    if (!twistLow.isFinite || !twistHigh.isFinite || twistLow > twistHigh) {
      throw ArgumentError('Twist goes from low to high, with low first.');
    }
  }

  final double swing;
  final double twistLow;
  final double twistHigh;
}

/// One rigid body of a ragdoll, standing for a bone or a short run of them.
///
/// The body is a capsule from the head of [from] to the tail of [bone], so a
/// forearm is one bone and a spine of three bones is one body with `from` the
/// lowest and `bone` the highest. Bones between them, and every bone below
/// [bone] that has no body of its own, keep their animated pose against the
/// body: that is the map from a coarse physical skeleton to the full one.
///
/// The joint sits at the head of [from] and joins this body to [parent], which
/// is another body's [bone] name. A root has neither a parent nor a joint.
final class RagdollBody {
  RagdollBody({
    required this.bone,
    String? from,
    this.parent,
    this.joint,
    required this.radius,
    this.mass = 1.0,
    this.friction = 0.5,
  }) : from = from ?? bone {
    if (!radius.isFinite || radius <= 0) {
      throw ArgumentError.value(radius, 'radius', 'A radius is above 0.');
    }
    if (!mass.isFinite || mass <= 0) {
      throw ArgumentError.value(mass, 'mass', 'A mass is above 0.');
    }
    if ((parent == null) != (joint == null)) {
      throw ArgumentError(
        'Body "$bone" needs both a parent and a joint, or neither.',
      );
    }
  }

  /// The last bone the body covers, whose tail is the body's far end.
  final String bone;

  /// The first bone it covers, whose head is where its joint sits.
  final String from;

  /// The [bone] of the body this one hangs from, or null for the root.
  final String? parent;

  final RagdollJoint? joint;

  /// How thick the capsule is, in the model's units.
  final double radius;

  final double mass;
  final double friction;
}

/// The bodies of a ragdoll and how they hang together.
///
/// Parents are listed before the bodies that hang from them, and there is one
/// root. [check] says whether an armature has the bones, in the order the
/// spec expects.
final class RagdollSpec {
  RagdollSpec(List<RagdollBody> bodies) : bodies = List.unmodifiable(bodies) {
    if (bodies.isEmpty) {
      throw ArgumentError.value(bodies, 'bodies', 'Give at least one body.');
    }
    final seen = <String>{};
    for (final body in bodies) {
      final parent = body.parent;
      if (parent != null && !seen.contains(parent)) {
        throw ArgumentError(
          'Body "${body.bone}" hangs from "$parent", which is not listed '
          'before it.',
        );
      }
      if (!seen.add(body.bone)) {
        throw ArgumentError('Two bodies end at bone "${body.bone}".');
      }
    }
    if (bodies.where((b) => b.parent == null).length != 1) {
      throw ArgumentError('A ragdoll has one root body.');
    }
    if (bodies.first.parent != null) {
      throw ArgumentError('List the root body first.');
    }
  }

  final List<RagdollBody> bodies;

  /// Throws an [ArmatureError] if a bone is missing, a body's [from] is not
  /// above its bone, or a body hangs from one with no bone above its own.
  void check(Armature armature) {
    final covered = <String, String>{};
    for (final body in bodies) {
      final chain = chainOf(armature, body);
      for (final name in chain) {
        final other = covered[name];
        if (other != null) {
          throw ArmatureError(
            'Bone "$name" is covered by the bodies ending at "$other" and '
            '"${body.bone}".',
          );
        }
        covered[name] = body.bone;
      }
      _checkParent(armature, body);
    }
  }

  /// The bones a body covers, from its [RagdollBody.from] down to its bone.
  static List<String> chainOf(Armature armature, RagdollBody body) {
    for (final name in [body.from, body.bone]) {
      if (!armature.contains(name)) {
        throw ArmatureError(
          'No bone called "$name" in this armature, so the ragdoll has no '
          'body for it.',
        );
      }
    }
    final path = <String>[];
    String? name = body.bone;
    while (name != null) {
      path.add(name);
      if (name == body.from) return path.reversed.toList();
      name = armature[name]?.parent;
    }
    throw ArmatureError(
      'Bone "${body.from}" is not above "${body.bone}", so no body can run '
      'from one to the other.',
    );
  }

  void _checkParent(Armature armature, RagdollBody body) {
    final parent = body.parent;
    if (parent == null) return;
    // A body may hang from any bone the parent body covers, which is how a
    // hip hangs from the middle of a torso that runs from hips to spine.
    final held = chainOf(armature, bodies.firstWhere((b) => b.bone == parent));
    String? name = armature[body.from]?.parent;
    while (name != null) {
      if (held.contains(name)) return;
      name = armature[name]?.parent;
    }
    throw ArmatureError(
      'Body "${body.bone}" hangs from "$parent", which has no bone above '
      '"${body.from}" in this armature.',
    );
  }
}
