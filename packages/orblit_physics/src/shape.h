// Geometry and mass of a body. Cooked hulls, ground and compounds are
// immutable assets owned by the world and shared with snapshots.

#ifndef ORBLIT_PHYSICS_SHAPE_H
#define ORBLIT_PHYSICS_SHAPE_H

#include "heightfield.h"
#include "hull.h"
#include "maths.h"
#include "orblit_physics.h"

namespace orblit {

class Compound;
Vec3 compoundCentre(const Compound *of);
float compoundReach(const Compound *of);
Bounds compoundBounds(const Compound *of, const Vec3 &at, const Quat &rotation);
Bounds affineBounds(const struct Shape &of, const Vec3 &at, const Quat &rotation);


enum class ShapeKind : uint32_t {
  sphere = ORBLIT_PHYSICS_SPHERE,
  box = ORBLIT_PHYSICS_BOX,
  plane = ORBLIT_PHYSICS_PLANE,
  capsule = ORBLIT_PHYSICS_CAPSULE,
  heightField = ORBLIT_PHYSICS_HEIGHT_FIELD,
  cylinder = ORBLIT_PHYSICS_CYLINDER,
  hull = ORBLIT_PHYSICS_HULL,
  compound = ORBLIT_PHYSICS_COMPOUND,
};

struct Shape {
  ShapeKind kind = ShapeKind::sphere;

  /// Sphere: radius in x. Box: half extents. Plane: the outward normal.
  /// Capsule and cylinder: radius in x, half the length along y in y.
  Vec3 size;

  /// Plane: how far along the normal the surface sits. Unused otherwise.
  float offset = 0.0f;

  /// Height field: the heights, owned by the world. Unused otherwise.
  const HeightField *field = nullptr;

  /// Hull: the cooked corners and faces, owned by the world. Unused otherwise.
  const Hull *cooked = nullptr;
  const Compound *parts = nullptr;

  /// A convex part's linear map. Bounds and support use the same map.
  Mat3 linear = Mat3::diagonal({1.0f, 1.0f, 1.0f});
  bool affine = false;

  static Shape compound(const Compound *parts) {
    Shape out;
    out.kind = ShapeKind::compound;
    out.parts = parts;
    return out;
  }

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
    return {ShapeKind::heightField, {}, 0.0f, field, nullptr};
  }

  /// A solid cylinder of `radius` and `2 * halfHeight`, flat at both ends,
  /// standing along its own y for the reason a capsule does.
  static Shape cylinder(float radius, float halfHeight) {
    return {ShapeKind::cylinder, {radius, halfHeight, 0.0f}, 0.0f};
  }

  /// The convex solid `cooked` describes, in the frame its corners were given
  /// in. A body made from it is placed by that frame's origin.
  static Shape hull(const Hull *cooked) {
    return {ShapeKind::hull, {}, 0.0f, nullptr, cooked};
  }

  float radius() const { return size.x; }

  /// Capsule: half the straight part, so the whole thing is
  /// `2 * (halfHeight() + radius())` tall. Cylinder: half the whole.
  float halfHeight() const { return size.y; }

  /// Where the shape's own weight is, in the body's frame. The origin for
  /// every shape that is symmetric about it, which is all of them but a hull,
  /// whose corners are wherever they were given.
  Vec3 centre() const {
    if (kind == ShapeKind::compound) return compoundCentre(parts);
    const Vec3 own = kind == ShapeKind::hull && cooked != nullptr
                         ? cooked->centre() : Vec3{};
    return affine ? linear * own : own;
  }

  /// The capsule's axis in the world, as a half-length vector from its centre.
  /// Zero for anything that is not a capsule, which makes the segment
  /// degenerate to a point and every routine below fall back to a sphere. A
  /// cylinder is not a capsule with square ends, so it answers zero here too.
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
    if (affine) {
      const Bounds bounds = boundsAt({}, {});
      return length(maxPerAxis(absPerAxis(bounds.low), absPerAxis(bounds.high)));
    }
    switch (kind) {
      case ShapeKind::compound: return compoundReach(parts);
      case ShapeKind::sphere: return size.x;
      case ShapeKind::box: return length(size);
      case ShapeKind::plane: return 0.0f;
      case ShapeKind::capsule: return size.x + size.y;
      case ShapeKind::cylinder: return std::sqrt(size.x * size.x + size.y * size.y);
      case ShapeKind::hull: return cooked != nullptr ? cooked->reach() : 0.0f;
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
    if (affine) return affineBounds(*this, at, rotation);
    switch (kind) {
      case ShapeKind::compound: return compoundBounds(parts, at, rotation);
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
      case ShapeKind::cylinder: {
        // Along each world axis, the half length's share of it plus as much of
        // the end discs' radius as lies across that axis. Exact: the rim's
        // furthest point along an axis is the radius times the sine of the
        // angle the cylinder's own axis makes with it.
        const Vec3 up = rotate(rotation, {0.0f, 1.0f, 0.0f});
        Vec3 r;
        for (int i = 0; i < 3; ++i) {
          r[i] = size.y * std::fabs(up[i]) +
                 size.x * std::sqrt(std::fmax(0.0f, 1.0f - up[i] * up[i]));
        }
        return {at - r, at + r};
      }
      case ShapeKind::hull: {
        if (cooked == nullptr) return {at, at};
        // The corner that goes furthest along each world direction, asked in
        // the hull's own frame. Exact, and no dearer than turning the corners.
        Bounds out{at, at};
        for (int i = 0; i < 3; ++i) {
          Vec3 axis;
          axis[i] = 1.0f;
          const Vec3 own = unrotate(rotation, axis);
          out.high[i] = at[i] + dot(cooked->support(own), own);
          out.low[i] = at[i] + dot(cooked->support(-own), own);
        }
        return out;
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
      case ShapeKind::compound: return {};
      case ShapeKind::plane: return {};
      case ShapeKind::heightField: return {};
      // A hull's inertia is a whole matrix, not three numbers, and is asked
      // for through `inverseInertiaOf`.
      case ShapeKind::hull: return {};
      case ShapeKind::cylinder: {
        // A solid cylinder: half the mass times the radius squared about its
        // axis, and a twelfth of the mass times three radii squared plus the
        // full length squared across it.
        const float r2 = size.x * size.x;
        const float along = 0.5f * mass * r2;
        const float across = mass * (0.25f * r2 + size.y * size.y / 3.0f);
        return {across > 0.0f ? 1.0f / across : 0.0f,
                along > 0.0f ? 1.0f / along : 0.0f,
                across > 0.0f ? 1.0f / across : 0.0f};
      }
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
  /// Ground and a hull cannot be read out of four floats, since each is an
  /// object the world holds. A command naming one is read as a sphere, and
  /// whoever sent it makes the real shape or refuses it before it gets here.
  static Shape fromCommand(uint32_t kind, const float size[4]) {
    switch (static_cast<ShapeKind>(kind)) {
      case ShapeKind::box: return box({size[0], size[1], size[2]});
      case ShapeKind::plane: return plane({size[0], size[1], size[2]}, size[3]);
      case ShapeKind::capsule: return capsule(size[0], size[1]);
      case ShapeKind::cylinder: return cylinder(size[0], size[1]);
      case ShapeKind::sphere:
      case ShapeKind::heightField:
      case ShapeKind::hull:
      case ShapeKind::compound: break;
    }
    return sphere(size[0]);
  }
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_SHAPE_H
