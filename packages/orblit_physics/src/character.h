// A character: a body that walks rather than falls.
//
// Everything else in a world is either solved or driven. A solved body goes
// wherever the forces put it, and a driven body goes wherever its velocity
// says, through anything in the way. A character is neither. It is asked for
// a velocity, like a driven body, but it is moved by sweeping its shape
// through the world and sliding along what it meets, so where it ends up is
// what the world allowed of what it asked for.
//
// That is a different kind of motion from the solver's, and it is kept out of
// the solver on purpose. A character pushed about by impulses is a character
// that slides down every slope it stands on, bounces off every crate it walks
// into and jitters on every stair, and the fixes for each of those are the
// rules below, done badly.

#ifndef ORBLIT_PHYSICS_CHARACTER_H
#define ORBLIT_PHYSICS_CHARACTER_H

#include <cstdint>

#include "maths.h"
#include "orblit_physics.h"

namespace orblit {

/// A driven body the world moves by sweeping, and what it last stood on.
struct Character {
  OrblitPhysicsId id = 0;

  /// The tallest ledge it walks up without being asked to jump, in metres.
  float stepHeight = 0.0f;

  /// The cosine of the steepest slope it can stand on. Cosine because every
  /// question asked of it is a dot product with up.
  float slopeCos = 1.0f;

  /// The gap it keeps from everything, in metres.
  float skin = 0.01f;

  /// The most it pushes with, in newtons.
  float strength = 0.0f;

  /// The velocity it asks for, gravity included, relative to whatever it is
  /// standing on. After a step, what survived of it.
  Vec3 wanted;

  /// What was under it at the end of the last step. `ground` and `normal`
  /// are set whenever something was in reach, walkable or not, and
  /// `grounded` only when it could stand there.
  bool grounded = false;
  OrblitPhysicsId ground = 0;
  Vec3 normal;

  /// How far what it stood on moved it last step, per second, and how fast
  /// that turned it about up.
  Vec3 carried;
  float turning = 0.0f;

  /// Where it stood in its ground's own frame, and how that ground was
  /// turned then. The next step puts it back there, wherever the ground went.
  Vec3 local;
  Quat groundRotation;

  /// Lets go of whatever it was standing on and stops asking to move. For a
  /// teleport: a character placed somewhere new was not carried there.
  void forget() {
    wanted = {};
    grounded = false;
    ground = 0;
    normal = {};
    carried = {};
    turning = 0.0f;
  }
};

/// An impulse a character gave a free body, held until the step decides which
/// of its tries at moving it keeps, so a try it throws away shoves nothing.
struct Shove {
  uint32_t row = 0;
  Vec3 impulse;
  Vec3 at;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_CHARACTER_H
