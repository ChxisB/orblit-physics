#include "world.h"

namespace orblit {

bool World::mesh(OrblitPhysicsId id, const float *xyz, uint32_t vertices,
                 const uint32_t *indices, uint32_t count) {
  if (id == 0 || meshes_.count(id) != 0) return false;
  const auto cooked = TriangleMesh::cook(xyz, vertices, indices, count);
  if (cooked == nullptr) return false;
  meshes_[id] = cooked;
  return true;
}

bool World::dropMesh(OrblitPhysicsId id) {
  const auto found = meshes_.find(id);
  if (found == meshes_.end()) return false;
  for (uint32_t row = 0; row < bodies_.count(); ++row) {
    if (bodies_.shape(row).mesh == found->second.get()) return false;
  }
  meshes_.erase(found);
  return true;
}

} // namespace orblit
