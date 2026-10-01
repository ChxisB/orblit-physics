// Contacts between convex shapes, found from their surfaces.
//
// `gjk.h` says how deep two convex shapes overlap and which way is out. That
// is one point, and one point cannot hold a box flat: it would rock about it.
// A solver needs the part of each shape that lies against the other, a face,
// an edge or a corner, and the points where they meet.
//
// So the shape whose face lies flat is the reference, and the other's feature
// is cut to the fence round that face. What is left and below the face is
// touching. A feature is cut to a fence in the same way whether the fence is
// the face of another shape or a triangle of the ground, which is why `press`
// is here for both.

#ifndef ORBLIT_PHYSICS_CONVEX_COLLIDE_H
#define ORBLIT_PHYSICS_CONVEX_COLLIDE_H

#include <cstdint>

#include "collide.h"
#include "convex.h"
#include "maths.h"
#include "shape.h"

namespace orblit {

/// The most points a feature has left once it is cut. Cutting a convex loop
/// with a plane adds one point at most, so a loop of `kMostFeature` points cut
/// to a fence of as many sides has no more than twice that.
constexpr uint32_t kMostClip = 2 * kMostFeature;

/// A flat surface, and the part of it that counts.
struct Surface {
  /// One plane of the fence. A point is inside when `dot(normal, p)` is no
  /// more than `offset`.
  struct Side {
    Vec3 normal;
    float offset = 0.0f;
  };

  /// Out of the solid.
  Vec3 normal;

  /// `dot(normal, p)` for any point `p` on the surface.
  float offset = 0.0f;

  Side sides[kMostFeature];
  uint32_t sideCount = 0;
};

/// The points of `incident` that are over the counted part of `surface` and
/// below it, each as a candidate pushing along the surface's normal. `radius`
/// is how far the incident shape's surface lies beyond the feature, for a
/// sphere or a capsule. Returns how many.
uint32_t press(const Surface &surface, const Feature &incident, float radius,
               Candidate out[kMostClip]);

/// Where two convex shapes touch, with the normal out of `b` towards `a`. Sets
/// `out.count` and returns whether there is a contact.
bool collideConvex(const Convex &a, const Convex &b, Manifold &out);

/// A convex shape against a plane body, the plane being `b`.
bool collidePlane(const Convex &shape, const Shape &plane, const Vec3 &at,
                  const Quat &rotation, Manifold &out);

} // namespace orblit

#endif // ORBLIT_PHYSICS_CONVEX_COLLIDE_H
