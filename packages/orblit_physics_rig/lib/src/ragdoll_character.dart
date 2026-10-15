import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'ragdoll.dart';
import 'ragdoll_pose.dart';

/// A walking character that can be knocked into its ragdoll and got up again.
///
/// While it stands the character is a capsule the physics world moves by
/// sweeping, and the ragdoll is animated, carried along with the pose.
/// [knock] takes the capsule out and lets the bodies go limp from the bone
/// that was hit. [standUp] puts the capsule down where the character lies
/// and makes the bodies animated again.
///
/// The ragdoll has to be built with the capsule's layers in `ignore`, so its
/// bodies take no notice of the capsule. The capsule is made here with the
/// ragdoll's layer taken out of what it cares about.
final class RagdollCharacter {
  RagdollCharacter({
    required this.ragdoll,
    required this.id,
    required Vector3 at,
    this.pose,
    this.shape = const Shape.capsule(0.3, 0.6),
    this.layers = Layers.everything,
    this.stepHeight = 0.3,
    this.steepest = math.pi / 4,
    this.skin = 0.01,
    this.strength = 500.0,
    this.friction = 0.5,
  }) {
    if (ragdoll.layers.cares & layers.is_ != 0) {
      throw ArgumentError(
        'The ragdoll still cares about the character\'s layer, so its bodies '
        'would push the capsule. Build it with ignore: ${layers.is_}.',
      );
    }
    _add(at);
  }

  final Ragdoll ragdoll;

  /// Where the output is mixed back from a fall, or null to cut straight to
  /// the animated pose.
  final RagdollPose? pose;

  /// The capsule's id in the physics world.
  final int id;

  final Shape shape;
  final Layers layers;
  final double stepHeight;
  final double steepest;
  final double skin;
  final double strength;
  final double friction;

  bool _standing = true;

  /// Whether the character has been knocked down and not yet got up.
  bool get isDown => !_standing;

  Physics get _physics => ragdoll.physics;

  /// Knocks the character down: the capsule goes, and [bone]'s body goes
  /// limp with [impulse] and the rest after it, [spread] seconds a joint.
  /// Hitting a character that is already down only hits the bodies again.
  void knock(
    String bone, {
    required Vector3 impulse,
    Vector3? at,
    double spread = 0.04,
  }) {
    if (_standing) {
      _physics.remove(id);
      _standing = false;
    }
    ragdoll.hit(bone, impulse: impulse, at: at, spread: spread);
  }

  /// Gets the character up under the ragdoll's root, and answers the point
  /// on the ground it stands at. The model is the game's to put there.
  ///
  /// A ray goes down `reach` metres from the root. If it meets no ground the
  /// character stays down and the answer is null. Every body is made
  /// animated, and the pose is mixed from the fall over [blend] seconds.
  ///
  /// Does nothing and answers null for a character that is standing.
  Vector3? standUp({double reach = 2, double blend = 0.4}) {
    if (_standing) return null;
    final root = ragdoll.rootPosition;
    final ground = _physics.cast(
      from: root.storage,
      direction: const [0.0, -1.0, 0.0],
      distance: reach,
      layers: ragdoll.layers,
    );
    if (ground == null) return null;

    final feet = Vector3(ground.at[0], ground.at[1], ground.at[2]);
    _add(feet);
    ragdoll.setAll(RagdollMode.animated);
    pose?.recover(blend);
    _standing = true;
    return feet;
  }

  /// Takes the capsule out of the world if it is there.
  void dispose() {
    if (_standing) _physics.remove(id);
    _standing = false;
  }

  /// Stands the capsule with its feet at [feet] and a skin above the ground.
  void _add(Vector3 feet) {
    final centre = feet.y + shape.y + shape.x + skin;
    _physics.addCharacter(
      id,
      shape: shape,
      at: [feet.x, centre, feet.z],
      stepHeight: stepHeight,
      steepest: steepest,
      skin: skin,
      strength: strength,
      friction: friction,
      layers: Layers(is_: layers.is_, cares: layers.cares & ~ragdoll.layer),
    );
  }
}
