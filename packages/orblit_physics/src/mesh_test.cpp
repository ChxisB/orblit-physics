#include "bounds_tree.h"
#include "triangle_mesh.h"

#include <stdexcept>

namespace {

void treeChecks() {
  std::vector<orblit::Bounds> bounds;
  for (int i = 0; i < 10000; ++i) {
    const float x = static_cast<float>(i);
    bounds.push_back({{x, 0, 0}, {x + 0.5f, 1, 1}});
  }
  const orblit::BoundsTree tree(bounds);
  uint32_t visits = 0;
  bool met = false;
  tree.visit({{5000.1f, 0, 0}, {5000.2f, 1, 1}}, [&](uint32_t id) {
    ++visits;
    if (id == 5000) met = true;
  });
  if (!met || visits > 8) throw std::runtime_error("mesh tree did not prune distant leaves");
}

void seamChecks() {
  const float points[] = {-1, 0, -1, 1, 0, -1, 1, 0, 1,
                          -1, 0, -1, -1, 0, 1, 1, 0, 1};
  const uint32_t indices[] = {0, 2, 1, 3, 4, 5};
  const auto mesh = orblit::TriangleMesh::cook(points, 6, indices, 6);
  if (mesh == nullptr) throw std::runtime_error("mesh cooking failed");
  if (mesh->facet(0).across[0] != orblit::Across::seam ||
      mesh->facet(1).across[2] != orblit::Across::seam) {
    throw std::runtime_error("duplicated model vertices left a collision seam");
  }
}

} // namespace

void meshChecks() {
  treeChecks();
  seamChecks();
}
