import 'package:orblit_motion/orblit_motion.dart';
import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'character_walk.dart';

/// A character that goes where its clip's root goes.
///
/// A [ClipPlayer] plays the clip, and each step its root motion is turned by
/// the way the character faces and asked for as a velocity. The world still
/// decides how much of it the character gets, so a walk into a wall stops at
/// the wall and a walk up a step climbs it, whatever the clip says.
///
/// Which way the character faces is kept here as [heading], turned by the
/// clip's own turns, because a standing capsule is the same shape whichever
/// way it looks and the world keeps no facing for it. The game writes it to
/// the entity.
final class RootMotionWalk implements CharacterWalk {
  RootMotionWalk(
    this.player, {
    Quaternion? heading,
    this.gravity = const [0.0, -9.81, 0.0],
    this.scale = 1.0,
  }) : heading = heading ?? Quaternion.identity();

  /// Plays the clip. Started and stopped by the game, which is how a walk
  /// becomes an idle without the world hearing of it.
  final ClipPlayer player;

  /// Which way the character faces in the world.
  Quaternion heading;

  /// Metres per second squared, added to the fall while the character is not
  /// standing. The world's own is not asked, so a game that changes it tells
  /// the walk too.
  final List<double> gravity;

  /// How much the clip's distances are multiplied by, for a model scaled away
  /// from the size its clip was made at.
  final double scale;

  @override
  List<double> ask(double seconds, PhysicsFooting? footing) {
    final moved = player.advance(seconds).moved;
    // Through the matrix: `Quaternion.rotated` turns by the inverse.
    final along = heading.asRotationMatrix().transformed(moved.position)
      ..scale(scale / seconds);
    heading = (heading * moved.rotation)..normalize();

    final fall = footing?.velocity ?? const [0.0, 0.0, 0.0];
    return [along.x, along.y + fall[1] + gravity[1] * seconds, along.z];
  }
}
