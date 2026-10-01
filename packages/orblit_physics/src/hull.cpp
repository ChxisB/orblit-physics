#include "hull.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <utility>

#include "exact.h"

namespace orblit {
namespace {

// A hull is cooked in double. It is made once and read ten thousand times, so
// the extra precision costs nothing that matters, and it removes the faces that
// come out inside out because two floats rounded the wrong way.

/// How near a plane a point must be to count as on it, as a fraction of the
/// cloud's widest span. Points that close are rounding in the input, not shape.
constexpr double kFlat = 1.0e-6;

/// How closely two triangles must face the same way to be one flat face, as one
/// minus the cosine of the angle between them. About a hundredth of a degree.
///
/// Whatever this lets through is a rim that is not quite in one plane, by that
/// angle times the width of the face. It is tight so that the width of a big
/// face does not turn it into a visible error.
constexpr double kSameFace = 1.0e-8;

/// Above this many points the cloud is thinned before it is hulled, because the
/// hull is built by testing every point against every face.
constexpr uint32_t kThinAbove = 2048;
constexpr uint32_t kThinTo = 1024;

using Edge = std::pair<int, int>;

struct Triangle {
  int v[3];
  D3 normal;
  double offset = 0.0;
};

/// A flat face of the hull, as corner indices into the cloud.
struct Loop {
  std::vector<int> corners;
  D3 normal;
};

/// The moments of a solid about its own centre, as the integral of x_i * x_j.
struct Solid {
  double volume = 0.0;
  D3 centre;
  double moment[3][3] = {};
};

struct Cooked {
  std::vector<Vec3> corners;
  std::vector<Hull::Facet> facets;
  std::vector<uint32_t> loops;
  std::vector<std::vector<uint32_t>> neighbours;
};

bool readCloud(const float *xyz, uint32_t count, std::vector<D3> &out) {
  if (xyz == nullptr || count < 4 || count > kMostHullInput) return false;
  out.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    const D3 p(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return false;
    }
    out.push_back(p);
  }
  return true;
}

double widestSpan(const std::vector<D3> &p) {
  double widest = 0.0;
  for (int k = 0; k < 3; ++k) {
    double low = p[0][k];
    double high = p[0][k];
    for (const D3 &q : p) {
      low = std::fmin(low, q[k]);
      high = std::fmax(high, q[k]);
    }
    widest = std::fmax(widest, high - low);
  }
  return widest;
}

D3 centroidOf(const std::vector<D3> &p, const std::vector<int> &which) {
  D3 sum;
  for (int i : which) sum = sum + p[i];
  return sum * (1.0 / which.size());
}

Triangle triangleOf(const std::vector<D3> &p, int a, int b, int c) {
  Triangle t;
  t.v[0] = a;
  t.v[1] = b;
  t.v[2] = c;
  const D3 n = cross(p[b] - p[a], p[c] - p[a]);
  const double len = norm(n);
  if (len > 0.0) t.normal = n * (1.0 / len);
  t.offset = dot(t.normal, p[a]);
  return t;
}

/// The two points furthest apart along whichever axis the cloud spreads most
/// on. False when that is no further than `flat`.
bool widestPair(const std::vector<D3> &p, double flat, int &a, int &b) {
  int lo[3] = {0, 0, 0};
  int hi[3] = {0, 0, 0};
  for (int i = 1; i < static_cast<int>(p.size()); ++i) {
    for (int k = 0; k < 3; ++k) {
      if (p[i][k] < p[lo[k]][k]) lo[k] = i;
      if (p[i][k] > p[hi[k]][k]) hi[k] = i;
    }
  }

  double longest = flat;
  bool found = false;
  for (int k = 0; k < 3; ++k) {
    const double span = norm(p[hi[k]] - p[lo[k]]);
    if (span <= longest) continue;
    longest = span;
    a = lo[k];
    b = hi[k];
    found = true;
  }
  return found;
}

/// The point furthest from the line through `a` and `b`, or -1 when none is
/// further than `flat`.
int furthestFromLine(const std::vector<D3> &p, int a, int b, double flat) {
  const D3 line = p[b] - p[a];
  const double len = norm(line);
  int best = -1;
  double widest = flat;
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    const double off = norm(cross(line, p[i] - p[a])) / len;
    if (off > widest) {
      widest = off;
      best = i;
    }
  }
  return best;
}

/// The point furthest from the plane through `a`, `b` and `c`, or -1 when none
/// is further than `flat`.
int furthestFromPlane(const std::vector<D3> &p, int a, int b, int c, double flat) {
  const D3 n = cross(p[b] - p[a], p[c] - p[a]);
  const double len = norm(n);
  int best = -1;
  double highest = flat;
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    const double off = std::fabs(dot(n, p[i] - p[a])) / len;
    if (off > highest) {
      highest = off;
      best = i;
    }
  }
  return best;
}

/// Four points that enclose a volume, or false if there are none.
bool startingFour(const std::vector<D3> &p, double flat, int out[4]) {
  if (!widestPair(p, flat, out[0], out[1])) return false;
  out[2] = furthestFromLine(p, out[0], out[1], flat);
  if (out[2] < 0) return false;
  out[3] = furthestFromPlane(p, out[0], out[1], out[2], flat);
  return out[3] >= 0;
}

bool sees(const Triangle &t, const D3 &at, double flat) {
  return dot(t.normal, at) - t.offset > flat;
}

void addEdges(std::vector<Edge> &edges, const Triangle &t) {
  for (int k = 0; k < 3; ++k) edges.emplace_back(t.v[k], t.v[(k + 1) % 3]);
}

/// The edges whose partner, run the other way, is not in the list: the rim of
/// a patch of triangles that were all wound the same way.
std::vector<Edge> boundary(const std::vector<Edge> &edges) {
  std::vector<Edge> rim;
  for (const Edge &e : edges) {
    const Edge partner(e.second, e.first);
    if (std::find(edges.begin(), edges.end(), partner) == edges.end()) {
      rim.push_back(e);
    }
  }
  return rim;
}

/// The rim of the faces that can see `at`, which is where a new point is
/// stitched to the hull. Empty when none can, which is a point already inside.
std::vector<Edge> horizonOf(const std::vector<Triangle> &faces, const D3 &at,
                            double flat) {
  std::vector<Edge> edges;
  for (const Triangle &t : faces) {
    if (sees(t, at, flat)) addEdges(edges, t);
  }
  return boundary(edges);
}

void addPoint(std::vector<Triangle> &faces, const std::vector<D3> &p, int i,
              double flat) {
  const std::vector<Edge> horizon = horizonOf(faces, p[i], flat);
  if (horizon.empty()) return;

  faces.erase(std::remove_if(faces.begin(), faces.end(),
                             [&](const Triangle &t) { return sees(t, p[i], flat); }),
              faces.end());
  for (const Edge &e : horizon) faces.push_back(triangleOf(p, e.first, e.second, i));
}

/// The triangles of the hull of `p`, wound outward, or none when it is flat.
std::vector<Triangle> triangulate(const std::vector<D3> &p, double flat) {
  int four[4];
  if (!startingFour(p, flat, four)) return {};

  D3 middle;
  for (int i : four) middle = middle + p[i] * 0.25;

  std::vector<Triangle> faces;
  const int sides[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  for (const auto &s : sides) {
    Triangle t = triangleOf(p, four[s[0]], four[s[1]], four[s[2]]);
    if (sees(t, middle, 0.0)) t = triangleOf(p, four[s[0]], four[s[2]], four[s[1]]);
    faces.push_back(t);
  }

  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    if (i != four[0] && i != four[1] && i != four[2] && i != four[3]) {
      addPoint(faces, p, i, flat);
    }
  }
  return faces;
}

/// The members of `among` that stick out furthest in each of `count` evenly
/// spread directions, once each. Their hull is inside the hull of all of them,
/// and close to it.
///
/// Spread, not furthest from the middle. On a sphere every point is as far out
/// as any other, and ranking by distance keeps one cap of it.
std::vector<int> spread(const std::vector<D3> &p, const std::vector<int> &among,
                        uint32_t count) {
  const double golden = kPi * (3.0 - std::sqrt(5.0));
  std::vector<int> keep;
  for (uint32_t k = 0; k < count; ++k) {
    const double y = 1.0 - 2.0 * (k + 0.5) / count;
    const double ring = std::sqrt(1.0 - y * y);
    const D3 d(std::cos(golden * k) * ring, y, std::sin(golden * k) * ring);
    int best = among[0];
    for (int i : among) {
      if (dot(p[i], d) > dot(p[best], d)) best = i;
    }
    keep.push_back(best);
  }
  std::sort(keep.begin(), keep.end());
  keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
  return keep;
}

std::vector<D3> thinned(const std::vector<D3> &p) {
  std::vector<int> all(p.size());
  for (size_t i = 0; i < all.size(); ++i) all[i] = static_cast<int>(i);

  std::vector<D3> out;
  for (int i : spread(p, all, kThinTo)) out.push_back(p[i]);
  return out;
}

/// Adds the integral of x_i * x_j over the tetrahedron that has the origin and
/// the three points `v` as its corners.
///
/// The integral is volume / 20 times the sum of v_ki * v_kj over the corners
/// plus the product of the corners' sums. The origin's own corner is zero.
void addMoments(double moment[3][3], const D3 v[3], double volume) {
  for (int i = 0; i < 3; ++i) {
    const double sumI = v[0][i] + v[1][i] + v[2][i];
    for (int j = 0; j < 3; ++j) {
      const double sumJ = v[0][j] + v[1][j] + v[2][j];
      double own = 0.0;
      for (int k = 0; k < 3; ++k) own += v[k][i] * v[k][j];
      moment[i][j] += volume / 20.0 * (own + sumI * sumJ);
    }
  }
}

/// Volume, centre and moments of the solid under `faces`, as the sum of the
/// tetrahedra they make with `from`.
///
/// Measured from a point inside, so every tetrahedron has the same sign. From
/// a point outside, a thin hull loses its answer to the difference of two
/// large ones.
Solid measure(const std::vector<D3> &p, const std::vector<Triangle> &faces,
              const D3 &from) {
  Solid out;
  D3 weighted;
  for (const Triangle &t : faces) {
    const D3 v[3] = {p[t.v[0]] - from, p[t.v[1]] - from, p[t.v[2]] - from};
    const double volume = dot(v[0], cross(v[1], v[2])) / 6.0;
    out.volume += volume;
    weighted = weighted + (v[0] + v[1] + v[2]) * (volume * 0.25);
    addMoments(out.moment, v, volume);
  }
  if (!(out.volume > 0.0)) return out;

  const D3 away = weighted * (1.0 / out.volume);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) out.moment[i][j] -= out.volume * away[i] * away[j];
  }
  out.centre = away + from;
  return out;
}

/// The inertia about the centre for a mass of one. It is the trace of the
/// moments on the diagonal, less the moments.
Mat3 inertiaPerMass(const Solid &s) {
  const double trace = s.moment[0][0] + s.moment[1][1] + s.moment[2][2];
  Mat3 out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double about = (i == j ? trace : 0.0) - s.moment[i][j];
      out.row[i][j] = static_cast<float>(about / s.volume);
    }
  }
  return out;
}

bool sameFace(const Triangle &a, const Triangle &b) {
  return 1.0 - dot(a.normal, b.normal) <= kSameFace;
}

bool sharesEdge(const Triangle &a, const Triangle &b) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (a.v[i] == b.v[(j + 1) % 3] && a.v[(i + 1) % 3] == b.v[j]) return true;
    }
  }
  return false;
}

/// The triangles joined to `seed` through shared edges that all face the way
/// `seed` does.
///
/// Each is compared with `seed` and not with the triangle it was reached from,
/// so a gentle curve of slightly turned triangles is not walked into one face.
std::vector<size_t> growFrom(const std::vector<Triangle> &faces, size_t seed,
                             const std::vector<bool> &taken) {
  std::vector<bool> inGroup(faces.size(), false);
  std::vector<size_t> group;
  std::vector<size_t> pending{seed};
  inGroup[seed] = true;

  while (!pending.empty()) {
    const size_t f = pending.back();
    pending.pop_back();
    group.push_back(f);
    for (size_t g = 0; g < faces.size(); ++g) {
      if (taken[g] || inGroup[g]) continue;
      if (!sameFace(faces[seed], faces[g]) || !sharesEdge(faces[f], faces[g])) continue;
      inGroup[g] = true;
      pending.push_back(g);
    }
  }
  return group;
}

/// Triangles that share a plane, as groups of indices into `faces`.
std::vector<std::vector<size_t>> flatGroups(const std::vector<Triangle> &faces) {
  std::vector<std::vector<size_t>> groups;
  std::vector<bool> taken(faces.size(), false);
  for (size_t seed = 0; seed < faces.size(); ++seed) {
    if (taken[seed]) continue;
    groups.push_back(growFrom(faces, seed, taken));
    for (size_t f : groups.back()) taken[f] = true;
  }
  return groups;
}

/// The loop of corners round the rim of a group of triangles, wound as they
/// are, or empty if the rim is not one simple loop.
std::vector<int> outline(const std::vector<Triangle> &faces,
                         const std::vector<size_t> &group) {
  std::vector<Edge> edges;
  for (size_t f : group) addEdges(edges, faces[f]);

  std::map<int, int> next;
  for (const Edge &e : boundary(edges)) {
    if (!next.emplace(e.first, e.second).second) return {};
  }
  if (next.empty()) return {};

  std::vector<int> loop;
  const int start = next.begin()->first;
  int at = start;
  do {
    loop.push_back(at);
    const auto it = next.find(at);
    if (it == next.end()) return {};
    at = it->second;
  } while (at != start && loop.size() <= next.size());
  return loop.size() == next.size() ? loop : std::vector<int>();
}

/// The unit normal of a loop as a whole, so a face made of many triangles is as
/// true as its rim and not as the first triangle in it. Zero for a loop with
/// no area.
D3 normalOf(const std::vector<D3> &p, const std::vector<int> &loop) {
  D3 sum;
  for (size_t k = 0; k < loop.size(); ++k) {
    const D3 &a = p[loop[k]];
    const D3 &b = p[loop[(k + 1) % loop.size()]];
    sum.x += (a.y - b.y) * (a.z + b.z);
    sum.y += (a.z - b.z) * (a.x + b.x);
    sum.z += (a.x - b.x) * (a.y + b.y);
  }
  const double len = norm(sum);
  return len > 0.0 ? sum * (1.0 / len) : D3();
}

/// How far `at` is from the line through `a` and `b`. A pair that coincide
/// have no line, and everything is as near to them as it is to the point.
double distanceToLine(const D3 &a, const D3 &b, const D3 &at) {
  const double len = norm(b - a);
  return len > 0.0 ? norm(cross(b - a, at - a)) / len : norm(at - a);
}

/// `loop` without the corners that lie on the line between their neighbours.
///
/// A point part way along an edge of the solid is not a corner of it. Left in,
/// it spends one of the few corners a hull may have, and a face with nine
/// points down one side is a bigger polygon than any clip has room for.
void dropStraight(const std::vector<D3> &p, std::vector<int> &loop, double flat) {
  bool dropped = true;
  while (dropped && loop.size() > 3) {
    dropped = false;
    for (size_t k = 0; k < loop.size(); ++k) {
      const D3 &before = p[loop[(k + loop.size() - 1) % loop.size()]];
      const D3 &after = p[loop[(k + 1) % loop.size()]];
      if (distanceToLine(before, after, p[loop[k]]) > flat) continue;
      loop.erase(loop.begin() + static_cast<std::ptrdiff_t>(k));
      dropped = true;
      break;
    }
  }
}

/// The flat faces of the hull. Empty if any one of them has no clean loop,
/// because a hull with a face missing is open and contact against it would
/// leak.
std::vector<Loop> loopsOf(const std::vector<D3> &p,
                          const std::vector<Triangle> &faces, double flat) {
  std::vector<Loop> out;
  for (const std::vector<size_t> &group : flatGroups(faces)) {
    Loop loop;
    loop.corners = outline(faces, group);
    dropStraight(p, loop.corners, flat);
    if (loop.corners.size() < 3) return {};
    loop.normal = normalOf(p, loop.corners);
    if (!(norm(loop.normal) > 0.0)) return {};
    out.push_back(std::move(loop));
  }
  return out;
}

/// The corners the faces are made of. A point that ended up in the middle of a
/// face is not one.
std::vector<int> cornersOf(const std::vector<Loop> &loops) {
  std::vector<int> used;
  for (const Loop &loop : loops) {
    used.insert(used.end(), loop.corners.begin(), loop.corners.end());
  }
  std::sort(used.begin(), used.end());
  used.erase(std::unique(used.begin(), used.end()), used.end());
  return used;
}

/// A cloud of at most `kMostHullCorners` of the corners `used`. Its hull is a
/// smaller solid inside the one these came from.
std::vector<float> capped(const std::vector<D3> &p, const std::vector<int> &used) {
  std::vector<float> out;
  for (int i : spread(p, used, kMostHullCorners)) {
    for (int k = 0; k < 3; ++k) out.push_back(static_cast<float>(p[i][k]));
  }
  return out;
}

void addOnce(std::vector<uint32_t> &list, uint32_t value) {
  if (std::find(list.begin(), list.end(), value) == list.end()) {
    list.push_back(value);
  }
}

Cooked arrange(const std::vector<D3> &p, const std::vector<Loop> &loops,
               const std::vector<int> &used) {
  Cooked out;
  std::vector<uint32_t> index(p.size(), 0);
  for (size_t k = 0; k < used.size(); ++k) {
    index[used[k]] = static_cast<uint32_t>(k);
    out.corners.push_back(toVec(p[used[k]]));
  }
  out.neighbours.resize(used.size());

  for (const Loop &loop : loops) {
    Hull::Facet facet;
    facet.normal = toVec(loop.normal);
    facet.first = static_cast<uint32_t>(out.loops.size());
    facet.count = static_cast<uint32_t>(loop.corners.size());
    for (size_t k = 0; k < loop.corners.size(); ++k) {
      const uint32_t a = index[loop.corners[k]];
      const uint32_t b = index[loop.corners[(k + 1) % loop.corners.size()]];
      out.loops.push_back(a);
      addOnce(out.neighbours[a], b);
      addOnce(out.neighbours[b], a);
    }
    out.facets.push_back(facet);
  }
  return out;
}

Bounds boundsOf(const std::vector<Vec3> &corners) {
  Bounds out{corners[0], corners[0]};
  for (const Vec3 &c : corners) {
    out.low = minPerAxis(out.low, c);
    out.high = maxPerAxis(out.high, c);
  }
  return out;
}

float reachOf(const std::vector<Vec3> &corners) {
  float reach = 0.0f;
  for (const Vec3 &c : corners) reach = std::fmax(reach, length(c));
  return reach;
}

} // namespace

std::shared_ptr<const Hull> Hull::cook(const float *xyz, uint32_t count) {
  std::vector<D3> points;
  if (!readCloud(xyz, count, points)) return nullptr;
  const double flat = kFlat * widestSpan(points);
  if (!(flat > 0.0)) return nullptr;
  if (points.size() > kThinAbove) points = thinned(points);

  const std::vector<Triangle> faces = triangulate(points, flat);
  const std::vector<Loop> loops = loopsOf(points, faces, flat);
  if (loops.size() < 4) return nullptr;

  const std::vector<int> used = cornersOf(loops);
  if (used.size() > kMostHullCorners) {
    const std::vector<float> kept = capped(points, used);
    return cook(kept.data(), static_cast<uint32_t>(kept.size() / 3));
  }

  const Solid solid = measure(points, faces, centroidOf(points, used));
  if (!(solid.volume > 0.0)) return nullptr;

  Cooked parts = arrange(points, loops, used);
  std::shared_ptr<Hull> hull(new Hull());
  hull->corners_ = std::move(parts.corners);
  hull->facets_ = std::move(parts.facets);
  hull->loops_ = std::move(parts.loops);
  hull->neighbours_ = std::move(parts.neighbours);
  hull->bounds_ = boundsOf(hull->corners_);
  hull->reach_ = reachOf(hull->corners_);
  hull->volume_ = static_cast<float>(solid.volume);
  hull->centre_ = toVec(solid.centre);
  hull->inertia_ = inertiaPerMass(solid);
  return hull;
}

uint32_t Hull::supportIndex(const Vec3 &direction) const {
  uint32_t best = 0;
  float most = dot(corners_[0], direction);
  for (uint32_t i = 1; i < corners_.size(); ++i) {
    const float along = dot(corners_[i], direction);
    if (along > most) {
      most = along;
      best = i;
    }
  }
  return best;
}

const Hull::Facet &Hull::facing(const Vec3 &direction) const {
  size_t best = 0;
  float most = dot(facets_[0].normal, direction);
  for (size_t i = 1; i < facets_.size(); ++i) {
    const float along = dot(facets_[i].normal, direction);
    if (along > most) {
      most = along;
      best = i;
    }
  }
  return facets_[best];
}

} // namespace orblit
