// A convex shape seen the two ways the narrowphase needs it.
//
// Finding how deep two convex shapes overlap needs one thing of each: its
// furthest point along a direction. Turning that depth into the four points a
// solver holds a stack with needs another: the face, edge or corner that lies
// against the other shape. Both are answered here for every shape that is
// convex, so the pairs do not each get a routine of their own.
//
// A shape is held as a core with a rounding. A sphere is a point grown by its
// radius and a capsule a segment grown by it, which keeps their curved ends
// out of the maths in `gjk.h`: the core is a polytope or a cylinder, and the
// radius is added back at the end.

#ifndef ORBLIT_PHYSICS_CONVEX_H
#define ORBLIT_PHYSICS_CONVEX_H

#include <cstdint>

#include "collide.h"
#include "exact.h"
#include "hull.h"
#include "maths.h"
#include "shape.h"

namespace orblit {

/// The most corners a face is carried at while it is clipped. A face with more
/// is carried by an even sample of its corners, which lies inside it: a clip
/// then keeps a little less than the whole contact and never more.
constexpr uint32_t kMostFeature = 32;

/// The part of a shape that lies against another: a corner, an edge between
/// two, or a face as a loop of them wound anticlockwise seen from outside.
struct Feature {
  Vec3 v[kMostFeature];
  uint32_t n = 0;

  /// Out of the shape. Set for a face only.
  Vec3 normal;

  bool isFace() const { return n >= 3; }
};

class Convex {
 public:
  /// The shape placed in the world. A plane and ground are not convex lumps
  /// and are not asked for.
  Convex(const Shape &shape, const Vec3 &at, const Quat &rotation);

  /// How far the surface lies outside the core: a sphere's or a capsule's
  /// radius, and nothing for the rest.
  float radius() const { return radius_; }

  /// A point inside the core.
  const D3 &middle() const { return middle_; }

  /// The core's furthest point along `direction`.
  D3 support(const D3 &direction) const;

  /// The core's face, edge or corner that lies furthest along `direction`, a
  /// unit vector. A face when `direction` is nearly its normal, an edge when it
  /// is nearly across one, and a corner otherwise.
  Feature feature(const Vec3 &direction) const;

 private:
  enum class Core { point, segment, box, cylinder, hull };

  D3 toLocal(const D3 &direction) const;
  D3 coreSupport(const D3 &direction) const;
  Vec3 worldNormal(const Vec3 &normal) const;
  D3 toWorld(const D3 &local) const;
  Vec3 worldPoint(double x, double y, double z) const;

  Feature boxFeature(const D3 &local) const;
  Feature cylinderFeature(const D3 &local) const;
  Feature hullFeature(const D3 &local) const;

  Core core_ = Core::point;
  D3 at_;
  D3 middle_;
  D3 axis_[3];
  Vec3 half_;
  float radius_ = 0.0f;
  const Hull *hull_ = nullptr;
  float rounding_ = 0.0f;
  Mat3 normals_;
  bool affine_ = false;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_CONVEX_H
