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
//
// Ground is done this way, a triangle at a time. It is not convex, but each
// of its triangles is, and the earliest touch among the triangles a shape
// could reach is the earliest touch with the ground — once touches at a seam
// between two triangles are thrown away, as the narrowphase throws them away.

#ifndef ORBLIT_PHYSICS_CAST_H
#define ORBLIT_PHYSICS_CAST_H

#include "body.h"
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

/// Which bodies a cast through the world may meet.
struct Sieve {
  /// The cast's own layers, read by the same rule bodies follow.
  uint32_t layerIs = 0;
  uint32_t layerCares = 0;

  /// Passed straight through. The first is nearly always whoever is casting;
  /// the second is for a character being carried, which must not meet what
  /// it is standing on while that thing is what moves it.
  OrblitPhysicsId ignore = 0;
  OrblitPhysicsId alsoIgnore = 0;

  /// Lets a cast that starts inside something leave it.
  ///
  /// An overlap is an impact at no distance, which is right for a query that
  /// asks what is here and wrong for a character that was pushed half into a
  /// wall and is trying to walk back out of it: every sweep it made would stop
  /// dead where it stands. With this set, an overlap the cast is moving away
  /// from is not an impact, and one it is moving further into still is.
  bool leaving = false;
};

/// The nearest body `moving` meets going `distance` along `direction`, which
/// must be a unit vector, and where. The row, or Bodies::kNone for nothing.
///
/// Every body is asked, whatever it is and whether or not it is asleep: this
/// is a question about where things are, and a crate that has settled is still
/// in the way.
uint32_t nearest(const Bodies &bodies, const Placed &moving, const Vec3 &direction,
                 float distance, const Sieve &sieve, Impact &out);

} // namespace orblit

#endif // ORBLIT_PHYSICS_CAST_H
