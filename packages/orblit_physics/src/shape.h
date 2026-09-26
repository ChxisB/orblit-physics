// What a body is, geometrically: its extent, its bounds and how hard it is to
// spin.
//
// A shape here is a few floats and a tag rather than a class with virtuals.
// There are five of them, every one is the same size, and a body store that
// can memcpy its shapes is a body store that can be handed about as a column.
// Ground, the one shape too big for a few floats, is a pointer to heights the
// world owns, not a base class — and a convex hull, when one arrives, will be
// the same.

#ifndef ORBLIT_PHYSICS_SHAPE_H
#define ORBLIT_PHYSICS_SHAPE_H

#include "heightfield.h"
#include "maths.h"
#include "orblit_physics.h"

namespace orblit {

enum class ShapeKind : uint32_t {
  sphere = ORBLIT_PHYSICS_SPHERE,
  box = ORBLIT_PHYSICS_BOX,
  plane = ORBLIT_PHYSICS_PLANE,
  capsule = ORBLIT_PHYSICS_CAPSULE,
  heightField = ORBLIT_PHYSICS_HEIGHT_FIELD,
};

struct Shape {
  ShapeKind kind = ShapeKind::sphere;

  /// Sphere: radius in x. Box: half extents. Plane: the outward normal.
  /// Capsule: radius in x, half the straight part in y.
  Vec3 size;

  /// Plane: how far along the normal the surface sits. Unused otherwise.
  float offset = 0.0f;

  /// Height field: the heights, owned by the world. Unused otherwise.
  const HeightField *field = nullptr;

  static Shape sphere(float radius) {
    return {ShapeKind::sphere, {radius, radius, radius}, 0.0f};
  }

  static Shape box(const Vec3 &halfExtents) {
    return {ShapeKind::box, halfExtents, 0.0f};
  }

  static Shape plane(const Vec3 &normal, float offset) {
    return {ShapeKind::plane, normalised(normal), offset};
  }

  /// A cylinder of `radius` and `2 * halfHeight`, capped with a hemisphere at
  /// each end, standing along its own y.
  ///
  /// Along y because a capsule is nearly always a character, a character
  /// stands up, and up is where gravity is not. A capsule that wanted to lie
  /// down was always going to be rotated anyway.
  static Shape capsule(float radius, float halfHeight) {
    return {ShapeKind::capsule, {radius, halfHeight, 0.0f}, 0.0f};
  }

  static Shape ground(const HeightField *field) {
    return {ShapeKind::heightField, {}, 0.0f, field};
  }

  float radius() const { return size.x; }

  /// Capsule: half the straight part, so the whole thing is
  /// `2 * (halfHeight() + radius())` tall.
  float halfHeight() const { return size.y; }

  /// The capsule's axis in the world, as a half-length vector from its centre.
  /// Zero for anything that is not a capsule, which makes the segment
  /// degenerate to a point and every routine below fall back to a sphere.
  Vec3 axisAt(const Quat &rotation) const {
    if (kind != ShapeKind::capsule) return {};
    return rotate(rotation, {0.0f, size.y, 0.0f});
  }

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
      case ShapeKind::capsule: return size.x + size.y;
      // Never solved, so never asked how fast its edge is going.
      case ShapeKind::heightField: return 0.0f;
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
      case ShapeKind::capsule: {
        // The segment's two ends, each grown by the radius. A capsule stood
        // upright is then no wider than it is, which a bounds built from
        // `reach()` in every direction would not be.
        const Vec3 half = absPerAxis(axisAt(rotation));
        const Vec3 r{size.x, size.x, size.x};
        return {at - half - r, at + half + r};
      }
      case ShapeKind::heightField: {
        if (field == nullptr) return {at, at};
        // The field's own box turned into the world: each world axis reaches
        // as far as the corners that go furthest along it.
        const Bounds own = field->bounds();
        Bounds out{{3.0e38f, 3.0e38f, 3.0e38f}, {-3.0e38f, -3.0e38f, -3.0e38f}};
        for (int i = 0; i < 8; ++i) {
          const Vec3 corner{(i & 1) ? own.high.x : own.low.x,
                            (i & 2) ? own.high.y : own.low.y,
                            (i & 4) ? own.high.z : own.low.z};
          const Vec3 p = at + rotate(rotation, corner);
          out.low = minPerAxis(out.low, p);
          out.high = maxPerAxis(out.high, p);
        }
        return out;
      }
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
      case ShapeKind::heightField: return {};
      case ShapeKind::capsule: {
        // A cylinder and two hemispheres, each taking the share of the mass
        // its volume is worth, and the caps moved out to the ends by the
        // parallel axis theorem. Doing it by weighted volume rather than
        // treating the whole thing as a cylinder matters most where capsules
        // are used most: a character capsule is nearly all cap.
        const float r = size.x;
        const float half = size.y;
        const float h = 2.0f * half;
        const float cylinder = kPi * r * r * h;
        const float caps = 4.0f / 3.0f * kPi * r * r * r;
        const float total = cylinder + caps;
        if (total <= kTiny) return {};

        const float mc = mass * cylinder / total;
        const float mh = mass * caps / total;
        const float along = 0.5f * mc * r * r + 0.4f * mh * r * r;
        const float across = mc * (h * h / 12.0f + r * r * 0.25f) +
                             mh * (0.4f * r * r + 0.25f * h * h + 0.375f * r * h);
        return {across > 0.0f ? 1.0f / across : 0.0f,
                along > 0.0f ? 1.0f / along : 0.0f,
                across > 0.0f ? 1.0f / across : 0.0f};
      }
    }
    return {};
  }

  /// Reads one out of a command's four floats, as the header describes them.
  ///
  /// Ground cannot be read out of four floats. A command naming it is read as
  /// a sphere, and whoever sent one refuses it before it gets here.
  static Shape fromCommand(uint32_t kind, const float size[4]) {
    switch (static_cast<ShapeKind>(kind)) {
      case ShapeKind::box: return box({size[0], size[1], size[2]});
      case ShapeKind::plane: return plane({size[0], size[1], size[2]}, size[3]);
      case ShapeKind::capsule: return capsule(size[0], size[1]);
      case ShapeKind::sphere:
      case ShapeKind::heightField: break;
    }
    return sphere(size[0]);
  }
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_SHAPE_H
