// Moving a shape through the world to see what it meets first.
//
// The narrowphase next door answers "are these two touching now". This
// answers "when would they be", which is a different question and needs a
// different tool: a contact is found from two fixed placements, an impact is
// found by advancing one of them until it touches.
//
// The method is conservative advancement. Rather than stepping the shape
// forward in fixed hops — which walks straight through anything thinner than a
// hop — it asks how far the two shapes are apart, moves by no more than the
// distance that gap allows, and asks again. Every step is provably short of
// the impact, so nothing is ever missed, however thin it is.
//
// What that needs is a lower bound on the distance between two convex shapes,
// and the bound here is got from their support functions along a direction
// worth asking about. A direction that is merely good gives a bound that is
// merely safe, and the loop takes one more turn. A direction that is wrong
// cannot cause an impact to be stepped over, which is the property the whole
// thing rests on.
//
// Half-spaces are not done this way. A plane has no bounds and no support
// point to speak of, and the answer for one is exact in three lines.

#ifndef ORBLIT_PHYSICS_CAST_H
#define ORBLIT_PHYSICS_CAST_H

#include "maths.h"
#include "shape.h"

namespace orblit {

/// A shape somewhere, which is all either routine below needs of a body.
struct Placed {
  Shape shape;
  Vec3 at;
  Quat rotation;
};

/// Where a cast stopped.
struct Impact {
  Vec3 at;

  /// Out of the shape that was hit, towards the one that was cast — the same
  /// rule a manifold's normal follows, so a caller reading both does not have
  /// to hold two conventions in its head.
  Vec3 normal;

  /// How far along the cast direction, in metres.
  float distance = 0.0f;

  /// It was already overlapping before it moved anywhere. `distance` is zero
  /// and `at` is somewhere inside rather than where it came in, because there
  /// is no such place.
  bool started = false;
};

/// The furthest point of a placed shape along `direction`.
///
/// Undefined for a plane, which has no furthest anything. Every caller here
/// sends those down the other path.
Vec3 support(const Placed &of, const Vec3 &direction);

/// Moves `moving` along `direction` — which need not be a unit vector — for at
/// most `distance` metres, and reports the first touch against `fixed`.
///
/// False if it never touches, which includes moving away from something it is
/// already beside.
bool sweep(const Placed &moving, const Vec3 &direction, float distance,
           const Placed &fixed, Impact &out);

} // namespace orblit

#endif // ORBLIT_PHYSICS_CAST_H
