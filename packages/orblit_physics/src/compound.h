#pragma once

#include <memory>
#include <vector>

#include "shape.h"
#include "hull.h"

namespace orblit {

struct Placed;
struct Impact;
struct Manifold;

struct Part {
  Shape shape;
  Vec3 at;
};

/// Parts are immutable and share their cooked hulls with the world.
class Compound final {
 public:
  Compound(std::vector<Part> parts, std::vector<std::shared_ptr<const Hull>> hulls);

  const std::vector<Part> &parts() const { return parts_; }
  const Vec3 &centre() const { return centre_; }
  const Mat3 &inertia() const { return inertia_; }
  float reach() const { return reach_; }
  bool uses(const Hull *hull) const;

 private:
  std::vector<Part> parts_;
  std::vector<std::shared_ptr<const Hull>> hulls_;
  Vec3 centre_;
  Mat3 inertia_ = Mat3::zero();
  float reach_ = 0.0f;
};

/// Mass properties about the solid's centre, at unit mass.
Mat3 inertiaOf(const Shape &of);
float volumeOf(const Shape &of);

Vec3 supportParts(const Placed &of, const Vec3 &direction);

bool collideParts(const Placed &a, const Placed &b, Manifold &out);
bool sweepParts(const Placed &moving, const Vec3 &direction, float distance,
                const Placed &fixed, Impact &out);

} // namespace orblit
