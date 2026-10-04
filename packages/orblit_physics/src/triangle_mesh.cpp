#include "triangle_mesh.h"

#include <array>
#include <cmath>
#include <map>
#include <utility>

#include "convex_collide.h"
#include "gjk.h"

namespace orblit {
namespace {

using Edge = std::pair<uint32_t, uint32_t>;
using Border = std::pair<uint32_t, int>;

bool readVertices(const float *xyz, uint32_t count, std::vector<Vec3> &out,
                  std::vector<uint32_t> &welded) {
  std::map<std::array<float, 3>, uint32_t> seen;
  for (uint32_t i = 0; i < count; ++i) {
    const Vec3 p{xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]};
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
    const auto found = seen.emplace(std::array<float, 3>{p.x, p.y, p.z}, seen.size());
    welded.push_back(found.first->second);
    out.push_back(p);
  }
  return true;
}

Facet facetOf(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
  Facet f{};
  f.v[0] = a;
  f.v[1] = b;
  f.v[2] = c;
  f.normal = normalised(cross(b - a, c - a));
  for (int k = 0; k < 3; ++k) {
    f.side[k] = normalised(cross(f.v[(k + 1) % 3] - f.v[k], f.normal));
    f.across[k] = Across::rim;
  }
  return f;
}

void join(std::vector<Facet> &facets, const std::map<Edge, std::vector<Border>> &edges) {
  for (const auto &entry : edges) {
    if (entry.second.size() != 2) continue;
    const Border a = entry.second[0], b = entry.second[1];
    Facet &fa = facets[a.first], &fb = facets[b.first];
    // Coplanar joins have no rim, even when an exporter duplicated vertices
    // for normals or UVs. Reversed winding is harmless on a two-sided sheet.
    if (std::fabs(dot(fa.normal, fb.normal)) > 0.99999f) {
      fa.across[a.second] = fb.across[b.second] = Across::seam;
    } else {
      fa.across[a.second] = fb.across[b.second] = Across::ground;
      fa.beyond[a.second] = fb.side[b.second];
      fb.beyond[b.second] = fa.side[a.second];
    }
  }
}

Bounds areaOf(const Facet &f) {
  return {minPerAxis(f.v[0], minPerAxis(f.v[1], f.v[2])),
          maxPerAxis(f.v[0], maxPerAxis(f.v[1], f.v[2]))};
}

void touch(const Facet &source, const Convex &shape, std::vector<Candidate> &into) {
  Facet f = source;
  if (dot(f.normal, toVec(shape.middle()) - f.v[0]) < 0) f.normal = -f.normal;
  Gap gap;
  if (!separation(shape, Convex(f), gap)) return;
  Surface face;
  face.normal = f.normal;
  face.offset = dot(f.normal, f.v[0]);
  face.sideCount = 3;
  for (int k = 0; k < 3; ++k) face.sides[k] = {f.side[k], dot(f.side[k], f.v[k])};
  Candidate clipped[kMostClip];
  const uint32_t count = press(face, shape.feature(-f.normal), shape.radius(), clipped);
  if (count != 0) {
    into.insert(into.end(), clipped, clipped + count);
  } else if (f.admits(gap.normal)) {
    into.push_back({gap.normal, gap.at, gap.depth});
  }
}

} // namespace

TriangleMesh::TriangleMesh(std::vector<Facet> facets, const std::vector<Bounds> &bounds)
    : facets_(std::move(facets)), tree_(bounds) {}

// One pass keeps triangle ids, bounds and welded edges aligned during cooking.
std::shared_ptr<const TriangleMesh> TriangleMesh::cook(const float *xyz, uint32_t vertices,
                                                     const uint32_t *indices, uint32_t count) {
  if (xyz == nullptr || indices == nullptr || vertices < 3 || count < 3 || count % 3 != 0) return nullptr;
  std::vector<Vec3> points;
  std::vector<uint32_t> welded;
  if (!readVertices(xyz, vertices, points, welded)) return nullptr;
  std::vector<Facet> facets;
  std::vector<Bounds> bounds;
  std::map<Edge, std::vector<Border>> edges;
  for (uint32_t i = 0; i < count; i += 3) {
    const uint32_t a = indices[i], b = indices[i + 1], c = indices[i + 2];
    if (a >= vertices || b >= vertices || c >= vertices) return nullptr;
    Facet f = facetOf(points[a], points[b], points[c]);
    if (lengthSquared(f.normal) < 0.5f) continue;
    if (!std::isfinite(f.normal.x) || !std::isfinite(f.normal.y) || !std::isfinite(f.normal.z)) return nullptr;
    const uint32_t id = facets.size();
    facets.push_back(f);
    bounds.push_back(areaOf(f));
    for (int k = 0; k < 3; ++k) {
      const uint32_t x = welded[indices[i + k]], y = welded[indices[i + (k + 1) % 3]];
      edges[{std::min(x, y), std::max(x, y)}].push_back({id, k});
    }
  }
  if (facets.empty()) return nullptr;
  join(facets, edges);
  return std::make_shared<const TriangleMesh>(std::move(facets), bounds);
}

Bounds meshBounds(const TriangleMesh *mesh, const Vec3 &at, const Quat &rotation) {
  if (mesh == nullptr) return {at, at};
  const Bounds &own = mesh->bounds();
  Bounds out{at + rotate(rotation, own.low), at + rotate(rotation, own.low)};
  for (int i = 1; i < 8; ++i) {
    const Vec3 p = at + rotate(rotation, {(i & 1) ? own.high.x : own.low.x,
                                        (i & 2) ? own.high.y : own.low.y,
                                        (i & 4) ? own.high.z : own.low.z});
    out.low = minPerAxis(out.low, p);
    out.high = maxPerAxis(out.high, p);
  }
  return out;
}

bool collideMesh(const Shape &shape, const Vec3 &at, const Quat &rotation,
                 const Shape &mesh, const Vec3 &meshAt, const Quat &meshRotation,
                 Manifold &out) {
  out.count = 0;
  if (mesh.mesh == nullptr || shape.kind == ShapeKind::mesh ||
      shape.kind == ShapeKind::plane || shape.kind == ShapeKind::heightField) return false;
  const Quat back = conjugate(meshRotation);
  const Vec3 localAt = rotate(back, at - meshAt);
  const Quat localRotation = back * rotation;
  const Convex convex(shape, localAt, localRotation);
  std::vector<Candidate> found;
  mesh.mesh->visit(shape.boundsAt(localAt, localRotation).grown(1.0e-4f), [&](uint32_t id) {
    touch(mesh.mesh->facet(id), convex, found);
  });
  reduce(found.data(), found.size(), out);
  out.normal = rotate(meshRotation, out.normal);
  for (uint32_t i = 0; i < out.count; ++i) {
    Contact &p = out.points[i];
    p.at = meshAt + rotate(meshRotation, p.at);
    p.normal = rotate(meshRotation, p.normal);
  }
  return out.count > 0;
}

} // namespace orblit
