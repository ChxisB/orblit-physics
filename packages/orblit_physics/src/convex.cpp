#include "convex.h"

#include <algorithm>
#include <cmath>

namespace orblit {
namespace {

/// How squarely a direction must look at a face for the face to be offered, as
/// a cosine. About six degrees. A face offered a little off square is harmless,
/// since the clip keeps only the points that are really below the other
/// shape's surface.
constexpr double kFace = 0.995;

/// How nearly across an edge a direction must lie for the edge to be offered,
/// as a cosine between the two. About three degrees.
constexpr double kEdge = 0.05;

/// The corners a cylinder's end disc is carried as. Sixteen puts a corner every
/// 22.5 degrees, which leaves the disc's flat sides 2% of a radius inside the
/// round.
constexpr uint32_t kDiscCorners = 16;

constexpr double kTwoPi = 6.283185307179586;

double signOf(double x) { return x >= 0.0 ? 1.0 : -1.0; }

} // namespace

Convex::Convex(const Shape &shape, const Vec3 &at, const Quat &rotation)
    : at_(at), middle_(at), half_(shape.size) {
  axis_[0] = D3(rotate(rotation, {1.0f, 0.0f, 0.0f}));
  axis_[1] = D3(rotate(rotation, {0.0f, 1.0f, 0.0f}));
  axis_[2] = D3(rotate(rotation, {0.0f, 0.0f, 1.0f}));

  switch (shape.kind) {
    case ShapeKind::sphere:
      radius_ = shape.radius();
      break;
    case ShapeKind::capsule:
      core_ = Core::segment;
      radius_ = shape.radius();
      break;
    case ShapeKind::box: core_ = Core::box; break;
    case ShapeKind::cylinder: core_ = Core::cylinder; break;
    case ShapeKind::hull:
      if (shape.cooked == nullptr) break;
      core_ = Core::hull;
      hull_ = shape.cooked;
      middle_ = toWorld(D3(hull_->centre()));
      break;
    case ShapeKind::plane:
    case ShapeKind::heightField: break;
  }
}

D3 Convex::toLocal(const D3 &d) const {
  return {dot(axis_[0], d), dot(axis_[1], d), dot(axis_[2], d)};
}

D3 Convex::toWorld(const D3 &l) const {
  return at_ + axis_[0] * l.x + axis_[1] * l.y + axis_[2] * l.z;
}

Vec3 Convex::worldPoint(double x, double y, double z) const {
  return toVec(toWorld({x, y, z}));
}

D3 Convex::support(const D3 &direction) const {
  const D3 d = toLocal(direction);
  switch (core_) {
    case Core::point: return at_;
    case Core::segment: return toWorld({0.0, signOf(d.y) * half_.y, 0.0});
    case Core::box:
      return toWorld({signOf(d.x) * half_.x, signOf(d.y) * half_.y,
                      signOf(d.z) * half_.z});
    case Core::cylinder: {
      // Straight along the axis, every point of the disc is as far as any
      // other, and the middle of it is as good as the rim.
      const double across = std::hypot(d.x, d.z);
      const double scale =
          across > 1.0e-12 * (across + std::fabs(d.y)) ? half_.x / across : 0.0;
      return toWorld({d.x * scale, signOf(d.y) * half_.y, d.z * scale});
    }
    case Core::hull: {
      // The hull answers in floats, so the direction is brought to a size they
      // hold. Only its direction matters.
      const double most =
          std::fmax(std::fabs(d.x), std::fmax(std::fabs(d.y), std::fabs(d.z)));
      if (most <= 0.0) return toWorld(D3(hull_->support({1.0f, 0.0f, 0.0f})));
      const Vec3 own{static_cast<float>(d.x / most), static_cast<float>(d.y / most),
                     static_cast<float>(d.z / most)};
      return toWorld(D3(hull_->support(own)));
    }
  }
  return at_;
}

Feature Convex::feature(const Vec3 &direction) const {
  const D3 d = toLocal(D3(direction));
  Feature f;
  switch (core_) {
    case Core::point: break;
    case Core::segment:
      if (std::fabs(d.y) > kEdge) break;
      f.n = 2;
      f.v[0] = worldPoint(0.0, -half_.y, 0.0);
      f.v[1] = worldPoint(0.0, half_.y, 0.0);
      return f;
    case Core::box: return boxFeature(d);
    case Core::cylinder: return cylinderFeature(d);
    case Core::hull: return hullFeature(d);
  }
  f.n = 1;
  f.v[0] = toVec(support(D3(direction)));
  return f;
}

Feature Convex::boxFeature(const D3 &d) const {
  const double a[3] = {std::fabs(d.x), std::fabs(d.y), std::fabs(d.z)};
  const double s[3] = {signOf(d.x), signOf(d.y), signOf(d.z)};
  const double h[3] = {half_.x, half_.y, half_.z};

  int big = 0;
  int small = 0;
  for (int i = 1; i < 3; ++i) {
    if (a[i] > a[big]) big = i;
    if (a[i] < a[small]) small = i;
  }

  // The box's corner with the given signs on each axis, as a world point.
  const auto corner = [&](const double sign[3]) {
    return worldPoint(sign[0] * h[0], sign[1] * h[1], sign[2] * h[2]);
  };

  Feature f;
  if (a[big] >= kFace) {
    // Wound the way `faceOf` winds the face of a box, which is anticlockwise
    // seen from outside for the positive side and clockwise for the negative,
    // so the negative side is read backwards.
    const int j = (big + 1) % 3;
    const int k = (big + 2) % 3;
    const double along[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
    f.n = 4;
    for (int i = 0; i < 4; ++i) {
      double sign[3];
      sign[big] = s[big];
      const int at = s[big] > 0.0 ? i : (4 - i) % 4;
      sign[j] = along[at][0];
      sign[k] = along[at][1];
      f.v[i] = corner(sign);
    }
    f.normal = toVec(axis_[big] * s[big]);
    return f;
  }

  double sign[3] = {s[0], s[1], s[2]};
  if (a[small] > kEdge) {
    f.n = 1;
    f.v[0] = corner(sign);
    return f;
  }

  f.n = 2;
  sign[small] = -1.0;
  f.v[0] = corner(sign);
  sign[small] = 1.0;
  f.v[1] = corner(sign);
  return f;
}

Feature Convex::cylinderFeature(const D3 &d) const {
  const double across = std::hypot(d.x, d.z);
  Feature f;

  if (std::fabs(d.y) >= kFace) {
    // Wound against the way the angle runs in the x-z plane, which is
    // clockwise seen from +y, so the top disc runs backwards.
    const double top = signOf(d.y);
    f.n = kDiscCorners;
    for (uint32_t i = 0; i < kDiscCorners; ++i) {
      const double angle = -top * kTwoPi * i / kDiscCorners;
      f.v[i] = worldPoint(half_.x * std::cos(angle), top * half_.y,
                          half_.x * std::sin(angle));
    }
    f.normal = toVec(axis_[1] * top);
    return f;
  }

  if (std::fabs(d.y) > kEdge) {
    f.n = 1;
    f.v[0] = toVec(toWorld({half_.x * d.x / across, signOf(d.y) * half_.y,
                            half_.x * d.z / across}));
    return f;
  }

  // Lying along its side: the line the surface touches the other along.
  const double x = half_.x * d.x / across;
  const double z = half_.x * d.z / across;
  f.n = 2;
  f.v[0] = worldPoint(x, -half_.y, z);
  f.v[1] = worldPoint(x, half_.y, z);
  return f;
}

Feature Convex::hullFeature(const D3 &d) const {
  const Vec3 own{static_cast<float>(d.x), static_cast<float>(d.y),
                 static_cast<float>(d.z)};
  const std::vector<Vec3> &corners = hull_->corners();
  Feature f;

  const Hull::Facet &facet = hull_->facing(own);
  if (dot(facet.normal, own) >= kFace) {
    const std::vector<uint32_t> &loops = hull_->loops();
    f.n = std::min(facet.count, kMostFeature);
    for (uint32_t i = 0; i < f.n; ++i) {
      const uint32_t at = loops[facet.first + i * facet.count / f.n];
      f.v[i] = toVec(toWorld(D3(corners[at])));
    }
    f.normal = toVec(toWorld(D3(facet.normal)) - at_);
    return f;
  }

  // The corner furthest along the direction, and of the edges leaving it the
  // one lying most nearly across it.
  const uint32_t top = hull_->supportIndex(own);
  f.n = 1;
  f.v[0] = toVec(toWorld(D3(corners[top])));

  uint32_t across = top;
  double least = kEdge;
  for (const uint32_t next : hull_->neighbours(top)) {
    const Vec3 edge = corners[next] - corners[top];
    const double reach = length(edge);
    if (reach <= 0.0f) continue;
    const double lean = std::fabs(dot(edge, own)) / reach;
    if (lean <= least) {
      least = lean;
      across = next;
    }
  }
  if (across != top) {
    f.n = 2;
    f.v[1] = toVec(toWorld(D3(corners[across])));
  }
  return f;
}

} // namespace orblit
