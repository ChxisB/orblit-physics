// How much of a body a flat fluid surface covers, and where the lift acts.
//
// Pure maths on a shape and a height, kept apart from the world for the reason
// mass.h is: the world decides when to ask and what to do with the answer.

#ifndef ORBLIT_PHYSICS_BUOYANCY_H
#define ORBLIT_PHYSICS_BUOYANCY_H

#include <algorithm>
#include <cmath>

#include "maths.h"
#include "shape.h"

namespace orblit {

/// The part of a body below a horizontal surface.
struct Wet {
  /// The share of the body's volume below the surface, from 0 to 1.
  float fraction = 0.0f;

  /// Where the middle of that part is, in the world. The fluid pushes up
  /// there, so a body that sits tilted is turned back by it.
  Vec3 centre;
};

namespace buoyancy {

/// Cells along each axis of a box. Six keeps a floating body steady at a cost
/// of 216 samples a step for the bodies that are in a fluid and no others.
constexpr int kCells = 6;

/// The most cells along the length of a capsule, which keeps a long one from
/// costing more than a few boxes.
constexpr int kMostAlong = 24;

/// Whether a point in the body's frame is inside it. The box fills its own
/// bounds. A capsule is every point within its radius of the straight part.
inline bool covers(const Shape &shape, const Vec3 &p) {
  if (shape.kind == ShapeKind::box) return true;
  const float run = std::fmax(-shape.size.y, std::fmin(shape.size.y, p.y));
  const float off = p.y - run;
  return p.x * p.x + off * off + p.z * p.z <= shape.size.x * shape.size.x;
}

/// A sphere's cap below the surface, which has a closed form. The cap is
/// straight under the centre, so the lift acts through it and never turns a
/// ball.
inline Wet sphere(const Shape &shape, const Vec3 &at, float surface) {
  const float r = shape.radius();
  const float under = surface - (at.y - r);
  const float depth = std::fmax(0.0f, std::fmin(2.0f * r, under));
  Wet out;
  out.fraction = depth * depth * (3.0f * r - depth) / (4.0f * r * r * r);
  out.centre = at;
  return out;
}

/// A box or a capsule cut into a grid of cells in its own frame.
struct Cells {
  /// Half the grid's size along each of the body's axes.
  Vec3 half;

  /// One cell's size along each axis.
  Vec3 size;
  int count[3] = {kCells, kCells, kCells};

  /// How far one cell reaches up and down in the world, which is what the ramp
  /// in `sampled` is across.
  float height = 0.0f;

  int total() const { return count[0] * count[1] * count[2]; }
};

inline Cells cellsOf(const Shape &shape, const Vec3 axes[3]) {
  Cells out;
  const bool capsule = shape.kind == ShapeKind::capsule;
  out.half = shape.size;
  if (capsule) {
    const float r = shape.size.x;
    out.half = {r, shape.size.y + r, r};
    const int along = static_cast<int>(std::ceil(kCells * out.half.y / r));
    out.count[1] = std::min(kMostAlong, along);
  }
  for (int i = 0; i < 3; ++i) {
    out.size[i] = 2.0f * out.half[i] / static_cast<float>(out.count[i]);
    out.height += std::fabs(axes[i].y) * out.size[i];
  }
  return out;
}

/// The cells counted by how much of each one's height is under the surface.
///
/// A cell wholly under counts one and wholly over counts nothing. In between
/// it counts the share, so the answer changes smoothly as the body sinks,
/// where counting whole cells would step and a floating body would buzz about
/// the step. A box turned square to the surface is exact. A capsule is as exact
/// as its cells are small, and at its middle exactly half.
inline Wet sampled(const Shape &shape, const Vec3 &at, const Quat &rotation,
                   float surface) {
  const Vec3 axes[3] = {rotate(rotation, {1.0f, 0.0f, 0.0f}),
                        rotate(rotation, {0.0f, 1.0f, 0.0f}),
                        rotate(rotation, {0.0f, 0.0f, 1.0f})};
  const Cells cells = cellsOf(shape, axes);

  float held = 0.0f;
  float wet = 0.0f;
  Vec3 lift;
  for (int n = 0; n < cells.total(); ++n) {
    const int index[3] = {n % cells.count[0],
                          (n / cells.count[0]) % cells.count[1],
                          n / (cells.count[0] * cells.count[1])};
    Vec3 local;
    Vec3 world = at;
    for (int i = 0; i < 3; ++i) {
      const float mid = (static_cast<float>(index[i]) + 0.5f) * cells.size[i];
      local[i] = mid - cells.half[i];
      world += axes[i] * local[i];
    }
    if (!covers(shape, local)) continue;
    const float below = (surface - world.y) / cells.height + 0.5f;
    const float share = std::fmax(0.0f, std::fmin(1.0f, below));
    held += 1.0f;
    wet += share;
    lift += world * share;
  }

  Wet out;
  if (wet > 0.0f) {
    out.fraction = wet / held;
    out.centre = lift * (1.0f / wet);
  }
  return out;
}

} // namespace buoyancy

/// The part of the body that `shape`, placed at `at` and turned by `rotation`,
/// has below a horizontal surface `surface` metres up. Spheres, boxes and
/// capsules only. Nothing else is covered by anything.
///
/// shortcut: other shapes and scaled ones float on nothing, add a shape's
/// cells here when a game needs a hull or a cylinder on the water.
inline Wet submerged(const Shape &shape, const Vec3 &at, const Quat &rotation,
                     float surface) {
  if (shape.affine) return {};
  switch (shape.kind) {
    case ShapeKind::sphere: return buoyancy::sphere(shape, at, surface);
    case ShapeKind::box:
    case ShapeKind::capsule:
      return buoyancy::sampled(shape, at, rotation, surface);
    default: return {};
  }
}

} // namespace orblit

#endif // ORBLIT_PHYSICS_BUOYANCY_H
