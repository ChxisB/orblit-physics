// What a body is, geometrically: its extent, its bounds and how hard it is to
// spin.
//
// A shape here is a few floats and a tag rather than a class with virtuals.
// There are three of them, every one fits in the same sixteen bytes, and a
// body store that can memcpy its shapes is a body store that can be handed
// about as a column. When a convex hull arrives it brings an index alongside
// the tag, not a base class.

#ifndef ORBLIT_PHYSICS_SHAPE_H
#define ORBLIT_PHYSICS_SHAPE_H

#include "maths.h"
#include "orblit_physics.h"

namespace orblit {

enum class ShapeKind : uint32_t {
  sphere = ORBLIT_PHYSICS_SPHERE,
  box = ORBLIT_PHYSICS_BOX,
  plane = ORBLIT_PHYSICS_PLANE,
};

struct Shape {
  ShapeKind kind = ShapeKind::sphere;

  /// Sphere: radius in x. Box: half extents. Plane: the outward normal.
  Vec3 size;

  /// Plane: how far along the normal the surface sits. Unused otherwise.
  float offset = 0.0f;

  static Shape sphere(float radius) {
    return {ShapeKind::sphere, {radius, radius, radius}, 0.0f};
  }

  static Shape box(const Vec3 &halfExtents) {
    return {ShapeKind::box, halfExtents, 0.0f};
  }

  static Shape plane(const Vec3 &normal, float offset) {
    return {ShapeKind::plane, normalised(normal), offset};
  }

  float radius() const { return size.x; }

  /// The distance from the centre to the furthest point of it.
  ///
  /// The broadphase pads by this to decide whether a pair is worth looking at
  /// properly, and sleeping uses it to find the fastest-moving point of a
  /// spinning body.
  float reach() const {
    switch (kind) {
      case ShapeKind::sphere: return size.x;
      case ShapeKind::box: return length(size);
      case ShapeKind::plane: return 0.0f;
    }
    return 0.0f;
  }

  /// Where it is in the world, as an axis-aligned box.
  ///
  /// A plane has no bounds worth writing down, so it gets an empty one and the
  /// broadphase never asks: a half-space against a box is not a question about
  /// overlap of bounds, and pretending otherwise would mean a bounds the size
  /// of the world that overlaps everything.
  Bounds boundsAt(const Vec3 &at, const Quat &rotation) const {
    switch (kind) {
      case ShapeKind::sphere: {
        const Vec3 r{size.x, size.x, size.x};
        return {at - r, at + r};
      }
      case ShapeKind::box: {
        // Each world axis reaches as far as the box's half extents projected
        // onto it, which is the absolute rotation matrix times the extents.
        const Vec3 cx = rotate(rotation, {1.0f, 0.0f, 0.0f});
        const Vec3 cy = rotate(rotation, {0.0f, 1.0f, 0.0f});
        const Vec3 cz = rotate(rotation, {0.0f, 0.0f, 1.0f});
        Vec3 r;
        for (int i = 0; i < 3; ++i) {
          r[i] = std::fabs(cx[i]) * size.x + std::fabs(cy[i]) * size.y +
                 std::fabs(cz[i]) * size.z;
        }
        return {at - r, at + r};
      }
      case ShapeKind::plane: return {at, at};
    }
    return {at, at};
  }

  /// One over the inertia about each of its own axes, for a body of `mass`.
  ///
  /// Inverted here rather than where it is used because the solver only ever
  /// divides by inertia, and a static body's infinite inertia is then simply a
  /// zero rather than a special case in every line that touches it.
  Vec3 inverseInertia(float mass) const {
    if (mass <= 0.0f) return {};
    switch (kind) {
      case ShapeKind::sphere: {
        const float i = 0.4f * mass * size.x * size.x;
        return i > 0.0f ? Vec3{1.0f / i, 1.0f / i, 1.0f / i} : Vec3{};
      }
      case ShapeKind::box: {
        // A solid box, about its centre. The extents are halves, so the usual
        // twelfth of the full width squared is a third of the half squared.
        const Vec3 h2{size.x * size.x, size.y * size.y, size.z * size.z};
        const Vec3 i{mass * (h2.y + h2.z) / 3.0f, mass * (h2.x + h2.z) / 3.0f,
                     mass * (h2.x + h2.y) / 3.0f};
        return {i.x > 0.0f ? 1.0f / i.x : 0.0f, i.y > 0.0f ? 1.0f / i.y : 0.0f,
                i.z > 0.0f ? 1.0f / i.z : 0.0f};
      }
      case ShapeKind::plane: return {};
    }
    return {};
  }

  /// Reads one out of a command's four floats, as the header describes them.
  static Shape fromCommand(uint32_t kind, const float size[4]) {
    switch (static_cast<ShapeKind>(kind)) {
      case ShapeKind::box: return box({size[0], size[1], size[2]});
      case ShapeKind::plane: return plane({size[0], size[1], size[2]}, size[3]);
      case ShapeKind::sphere: break;
    }
    return sphere(size[0]);
  }
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_SHAPE_H
