#include "world.h"

#include <cmath>
#include <initializer_list>
#include <utility>

namespace orblit {
namespace {

bool allFinite(std::initializer_list<float> values) {
  for (float value : values) {
    if (!std::isfinite(value)) return false;
  }
  return true;
}

bool validPart(const OrblitPhysicsPart &of) {
  if (of.shape == ORBLIT_PHYSICS_HULL) return true;
  if (of.shape == ORBLIT_PHYSICS_BOX) {
    return allFinite({of.size[0], of.size[1], of.size[2]}) &&
           of.size[0] > 0 && of.size[1] > 0 && of.size[2] > 0;
  }
  if (!(of.size[0] > 0 && std::isfinite(of.size[0]))) return false;
  if (of.shape == ORBLIT_PHYSICS_SPHERE) return true;
  if (of.shape == ORBLIT_PHYSICS_CAPSULE) return std::isfinite(of.size[1]) && of.size[1] >= 0;
  return of.shape == ORBLIT_PHYSICS_CYLINDER && std::isfinite(of.size[1]) && of.size[1] > 0;
}

bool validMap(const Mat3 &map) {
  for (const Vec3 &row : map.row) {
    if (!allFinite({row.x, row.y, row.z})) return false;
  }
  const float determinant = dot(map.row[0], cross(map.row[1], map.row[2]));
  return determinant > 0 && std::isfinite(determinant) &&
         lengthSquared(inverse(map).row[0]) > 0;
}

bool validAsset(const Compound &of) {
  const Vec3 centre = of.centre();
  if (!allFinite({centre.x, centre.y, centre.z, of.reach()})) return false;
  for (const Vec3 &row : of.inertia().row) {
    if (!allFinite({row.x, row.y, row.z})) return false;
  }
  return true;
}

} // namespace

bool World::partFor(const OrblitPhysicsPart &from, Part &out) const {
  Shape shape;
  if (!validPart(from) || !shapeFor(from.shape, from.size, from.hull, shape)) return false;
  const Vec3 at{from.at[0], from.at[1], from.at[2]};
  if (!allFinite({at.x, at.y, at.z})) return false;
  shape.linear = {{from.linear[0], from.linear[1], from.linear[2]},
                  {from.linear[3], from.linear[4], from.linear[5]},
                  {from.linear[6], from.linear[7], from.linear[8]}};
  if (!validMap(shape.linear)) return false;
  shape.affine = true;
  const float volume = volumeOf(shape);
  if (!(volume > 0 && std::isfinite(volume))) return false;
  out = {shape, at};
  return true;
}

bool World::compound(OrblitPhysicsId id, const OrblitPhysicsPart *from, uint32_t count) {
  if (id == 0 || compounds_.count(id) != 0 || from == nullptr || count == 0 || count > 64) return false;
  std::vector<Part> parts;
  std::vector<std::shared_ptr<const Hull>> hulls;
  for (uint32_t i = 0; i < count; ++i) {
    Part part;
    if (!partFor(from[i], part)) return false;
    parts.push_back(part);
    if (part.shape.kind == ShapeKind::hull) hulls.push_back(hulls_.at(from[i].hull));
  }
  const auto asset = std::make_shared<const Compound>(std::move(parts), std::move(hulls));
  if (!validAsset(*asset)) return false;
  compounds_[id] = asset;
  return true;
}

bool World::dropCompound(OrblitPhysicsId id) {
  const auto found = compounds_.find(id);
  if (found == compounds_.end()) return false;
  for (uint32_t row = 0; row < bodies_.count(); ++row) {
    if (bodies_.shape(row).parts == found->second.get()) return false;
  }
  compounds_.erase(found);
  return true;
}

} // namespace orblit
