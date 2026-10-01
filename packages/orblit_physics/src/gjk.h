// How far apart two convex shapes are, or how far they overlap, and which way
// is out.
//
// Two convex shapes need no special case per pair. Take every point of one
// minus every point of the other: that set is convex too, and the shapes
// overlap exactly when it holds the origin. How far the origin is from it is
// the gap between the shapes, and when it is inside, how far it is from the
// nearest edge is how deep they overlap. Neither needs the set itself, only
// its furthest point along a direction, which every shape in `convex.h` gives
// in a handful of operations.
//
// All of it is in double precision. A float loses the answer in the middle,
// and what that looks like is a normal that points sideways for a box lying
// flat on another.

#ifndef ORBLIT_PHYSICS_GJK_H
#define ORBLIT_PHYSICS_GJK_H

#include "convex.h"
#include "maths.h"

namespace orblit {

/// Where two convex shapes touch, found from their surfaces.
struct Gap {
  /// Out of `b` towards `a`, a unit vector.
  Vec3 normal;

  /// How far the surfaces overlap along the normal. Never negative.
  float depth = 0.0f;

  /// Midway between the two surfaces, where they are nearest or deepest.
  Vec3 at;
};

/// Whether the surfaces of two convex shapes touch, and if so how.
///
/// Apart by less than their radii is touching: a round surface is held off its
/// core by its radius, and the cores may be well clear of each other.
bool separation(const Convex &a, const Convex &b, Gap &out);

/// The point of a shape's core nearest `to`, or `to` itself when it is inside.
/// The rounding of a sphere or a capsule is not part of the core.
Vec3 nearestOnCore(const Convex &shape, const Vec3 &to);

} // namespace orblit

#endif // ORBLIT_PHYSICS_GJK_H
