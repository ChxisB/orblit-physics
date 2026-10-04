#pragma once

#include <memory>
#include <vector>

#include "bounds_tree.h"
#include "heightfield.h"

namespace orblit {

/// Two-sided triangle surfaces. Coincident vertices share seam information.
class TriangleMesh final {
 public:
  static std::shared_ptr<const TriangleMesh> cook(const float *xyz, uint32_t vertices,
                                                const uint32_t *indices, uint32_t count);
  TriangleMesh(std::vector<Facet> facets, const std::vector<Bounds> &bounds);
  const Bounds &bounds() const { return tree_.bounds(); }
  const Facet &facet(uint32_t i) const { return facets_[i]; }

  template <typename Visit>
  void visit(const Bounds &area, Visit visit) const { tree_.visit(area, visit); }

 private:
  std::vector<Facet> facets_;
  BoundsTree tree_;
};

struct Shape;
struct Manifold;
Bounds meshBounds(const TriangleMesh *mesh, const Vec3 &at, const Quat &rotation);
bool collideMesh(const Shape &shape, const Vec3 &at, const Quat &rotation,
                 const Shape &mesh, const Vec3 &meshAt, const Quat &meshRotation,
                 Manifold &out);

} // namespace orblit
