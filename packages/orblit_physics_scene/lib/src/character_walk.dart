import 'package:orblit_physics/orblit_physics.dart';

/// What a character asks for, once a step, before the step is taken.
///
/// [ScenePhysics.advance] is the only caller. A frame that owes three steps
/// asks three times, so a walk that is driven by a clip's playhead follows
/// the world's own clock rather than the frame's.
abstract interface class CharacterWalk {
  /// The velocity the character would like for the next [seconds], in the
  /// world's axes and relative to whatever it stands on, with gravity in it.
  ///
  /// [footing] is what the last step made of the last request, or null before
  /// the character's first step. A character that is standing reads back no
  /// fall, so asking for gravity on top of that is how it keeps standing.
  List<double> ask(double seconds, PhysicsFooting? footing);
}
