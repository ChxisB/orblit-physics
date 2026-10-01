// A convex hull: the smallest convex solid round a cloud of points.
//
// A body cannot be handed a cloud. Collision against a cloud means testing
// every point, and the points that matter are the few on the outside, so the
// cloud is cooked once into what is actually there: the corners that stick
// out, the flat faces between them, and the mass properties of the solid they
// enclose. Cooking is the slow part and happens once, when the hull is laid.
// What a step asks of a hull afterwards is its furthest corner along a
// direction, and which face or edge lies against a plane.
//
// A hull stays in the frame its points were given in. A body made from it is
// placed by the origin of that frame, and its weight is wherever the solid's
// centre falls, which is rarely the origin. The solver already turns bodies
// about a point that is not their origin, so the hull reports where that
// point is and gets on with being a shape.

#ifndef ORBLIT_PHYSICS_HULL_H
#define ORBLIT_PHYSICS_HULL_H

#include <cstdint>
#include <memory>
#include <vector>

#include "maths.h"

namespace orblit {

/// The most points a hull is cooked from, and the most corners it keeps.
///
/// Past a few hundred corners a hull stops being cheaper than the mesh it
/// stands for, and the answer is fewer points, not a bigger limit.
constexpr uint32_t kMostHullInput = 100000;
constexpr uint32_t kMostHullCorners = 255;

class Hull {
 public:
  /// A flat face of the hull, as a loop of corners wound anticlockwise seen
  /// from outside.
  struct Facet {
    Vec3 normal;
    uint32_t first = 0; ///< Into `loops()`.
    uint32_t count = 0;
  };

  /// The hull of `count` points, `xyz` being three floats each. Null when the
  /// points do not enclose any volume: fewer than four, all in one plane, or
  /// not numbers.
  ///
  /// When more than `kMostHullCorners` of them are corners, the hull is made of
  /// the corners that stick out furthest in each of a spread of directions. It
  /// is a smaller solid than was asked for, never a larger one, so nothing that
  /// was clear of the points is caught by it.
  static std::shared_ptr<const Hull> cook(const float *xyz, uint32_t count);

  const std::vector<Vec3> &corners() const { return corners_; }
  const std::vector<Facet> &facets() const { return facets_; }
  const std::vector<uint32_t> &loops() const { return loops_; }

  /// The corners that share an edge with corner `i`.
  const std::vector<uint32_t> &neighbours(uint32_t i) const {
    return neighbours_[i];
  }

  /// The corner furthest along `direction`, in the hull's own frame.
  Vec3 support(const Vec3 &direction) const {
    return corners_[supportIndex(direction)];
  }

  /// The same, as an index into `corners()`.
  uint32_t supportIndex(const Vec3 &direction) const;

  /// The facet that looks most squarely along `direction`.
  const Facet &facing(const Vec3 &direction) const;

  /// The centre of mass of the solid, in the hull's own frame.
  const Vec3 &centre() const { return centre_; }

  float volume() const { return volume_; }

  /// The inertia tensor of the solid about its centre, for a mass of one, in
  /// the hull's own frame. A body of mass `m` has `m` times this.
  const Mat3 &inertia() const { return inertia_; }

  /// The distance from the frame's origin to the furthest corner.
  float reach() const { return reach_; }

  const Bounds &bounds() const { return bounds_; }

 private:
  Hull() = default;

  std::vector<Vec3> corners_;
  std::vector<Facet> facets_;
  std::vector<uint32_t> loops_;
  std::vector<std::vector<uint32_t>> neighbours_;
  Vec3 centre_;
  Mat3 inertia_ = Mat3::zero();
  Bounds bounds_;
  float volume_ = 0.0f;
  float reach_ = 0.0f;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_HULL_H
