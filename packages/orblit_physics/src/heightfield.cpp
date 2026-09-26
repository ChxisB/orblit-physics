#include "heightfield.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "collide.h"

namespace orblit {
namespace {

const Vec3 kUp{0.0f, 1.0f, 0.0f};

/// How far past an edge a point may be and still count as inside it, in
/// metres. Enough that a point exactly on the line between two triangles is
/// inside both rather than neither.
constexpr float kOnEdge = 1.0e-4f;

/// How far a push may lean over an edge, as a cosine, before it counts as
/// leaving over it. Flat ground's own rounding leans a push by about this.
constexpr float kLean = 1.0e-3f;

/// Two candidates nearer than this are one point, in metres.
constexpr float kSamePoint = 1.0e-3f;

/// How far below everything a box is measured against a triangle's column
/// ends, in metres. A column is bottomless, but one with a bottom a long way
/// down gives the same answers, and a way out through the bottom is then
/// always the longest way out and never taken.
constexpr float kBottom = 1000.0f;

float nan() { return std::numeric_limits<float>::quiet_NaN(); }

/// The three corners of triangle `which` of cell (c, r), as sample offsets
/// from the cell's own corner, wound so the normal comes out of the ground.
///
/// Triangle 0 is sample (0, 0), (1, 1), (1, 0): the side of the diagonal
/// nearer +x. Triangle 1 is (0, 0), (0, 1), (1, 1).
constexpr int kCorner[2][3][2] = {
    {{0, 0}, {1, 1}, {1, 0}},
    {{0, 0}, {0, 1}, {1, 1}},
};

/// Which triangle is across each edge: an offset to its cell, and which of
/// that cell's two it is. Edge `k` runs from corner `k` to corner `k + 1`.
constexpr int kAcross[2][3][3] = {
    // 0: the diagonal, then x = c + 1, then z = r.
    {{0, 0, 1}, {1, 0, 1}, {0, -1, 1}},
    // 1: x = c, then z = r + 1, then the diagonal.
    {{-1, 0, 0}, {0, 1, 0}, {0, 0, 0}},
};

Vec3 normalOf(const Vec3 v[3]) {
  return normalised(cross(v[1] - v[0], v[2] - v[0]));
}

/// Whether `p` is inside all three of a triangle's edges, measured along
/// `out[k]` from corner `k`.
bool within(const Facet &f, const Vec3 out[3], const Vec3 &p) {
  for (int k = 0; k < 3; ++k) {
    if (dot(p - f.v[k], out[k]) > kOnEdge) return false;
  }
  return true;
}

// --- the pool ------------------------------------------------------------- //
//
// Every triangle a shape is over is asked on its own, and each answers with
// however many points it has. The four that go to the solver are chosen from
// all of them at the end, rather than kept as they arrive, because which four
// is a question about all of them: the deepest first, then the three that
// spread furthest from it, so a box across four cells is held at its corners
// and not at four points bunched in one cell.

struct Candidate {
  Vec3 normal;
  Vec3 at;
  float depth;
};

std::vector<Candidate> &pool() {
  // Kept between calls so a step does not allocate. One per thread, because
  // nothing about a narrowphase should stop two worlds stepping at once.
  static thread_local std::vector<Candidate> points;
  return points;
}

void add(std::vector<Candidate> &into, const Vec3 &normal, const Vec3 &at,
         float depth) {
  into.push_back({normal, at, depth});
}

void place(Manifold &out, const Candidate &c) {
  Contact &point = out.points[out.count++];
  point.at = c.at;
  point.normal = c.normal;
  point.depth = c.depth;
  point.normalImpulse = 0.0f;
  point.frictionImpulse[0] = 0.0f;
  point.frictionImpulse[1] = 0.0f;
}

/// The deepest candidate and the three that spread furthest from it. A point
/// that adds no spread is not added, which is also how the same point found
/// by two triangles is kept once.
void reduce(const std::vector<Candidate> &from, Manifold &out) {
  out.count = 0;
  if (from.empty()) return;

  size_t first = 0;
  for (size_t i = 1; i < from.size(); ++i) {
    if (from[i].depth > from[first].depth) first = i;
  }
  place(out, from[first]);
  out.normal = from[first].normal;
  const Vec3 a = from[first].at;

  size_t second = first;
  float furthest = kSamePoint * kSamePoint;
  for (size_t i = 0; i < from.size(); ++i) {
    const float away = lengthSquared(from[i].at - a);
    if (away > furthest) {
      furthest = away;
      second = i;
    }
  }
  if (second == first) return;
  place(out, from[second]);
  const Vec3 b = from[second].at;

  size_t third = first;
  float widest = kSamePoint * kSamePoint * kSamePoint * kSamePoint;
  for (size_t i = 0; i < from.size(); ++i) {
    const float area = lengthSquared(cross(b - a, from[i].at - a));
    if (area > widest) {
      widest = area;
      third = i;
    }
  }
  if (third == first) return;
  place(out, from[third]);
  const Vec3 c = from[third].at;

  // The fourth is whichever lies furthest outside the triangle of the other
  // three, measured by the area it would add on its most outward side.
  const Vec3 facing = cross(b - a, c - a);
  size_t fourth = first;
  float most = -kSamePoint * kSamePoint * length(facing);
  for (size_t i = 0; i < from.size(); ++i) {
    const Vec3 &p = from[i].at;
    const float outside =
        std::fmin(dot(facing, cross(a - p, b - p)),
                  std::fmin(dot(facing, cross(b - p, c - p)),
                            dot(facing, cross(c - p, a - p))));
    if (outside < most) {
      most = outside;
      fourth = i;
    }
  }
  if (fourth != first) place(out, from[fourth]);
}

// --- round shapes --------------------------------------------------------- //
//
// A sphere against one triangle is asked about its centre, and a capsule
// about a handful of points along its segment: its ends, and wherever the
// segment crosses into or out of the triangle, and wherever it passes nearest
// each edge. Those are the only places the answer can change its mind, so
// asking at them and nowhere else misses nothing.

/// A ball at `p` against one triangle.
///
/// Above the surface, it is over the triangle if the line straight down the
/// normal lands on it, which is what makes the distance to the face the true
/// distance. Below the surface, it is in the triangle's ground if it is
/// inside the triangle's column, because every point under the field is in
/// exactly one column and so every ball that has sunk in is pushed out by
/// someone.
void ball(const Facet &f, const Vec3 &p, float radius,
          std::vector<Candidate> &into) {
  const float above = dot(f.normal, p - f.v[0]);
  if (above > radius) return;

  if (within(f, above >= 0.0f ? f.side : f.wall, p)) {
    const float depth = radius - above;
    add(into, f.normal, p - f.normal * ((above + radius) * 0.5f), depth);
    return;
  }

  const Vec3 q = f.nearest(p);
  const Vec3 d = p - q;
  const float squared = lengthSquared(d);
  if (squared >= radius * radius || squared < kTiny) return;
  const float dist = std::sqrt(squared);
  const Vec3 n = d * (1.0f / dist);
  if (!f.admits(n)) return;
  add(into, n, (p - n * radius + q) * 0.5f, radius - dist);
}

void round(const Facet &f, const Vec3 &centre, const Vec3 &half, float radius,
           std::vector<Candidate> &into) {
  if (lengthSquared(half) < kTiny) {
    ball(f, centre, radius, into);
    return;
  }

  // Two ends, six crossings and three nearest passes: eleven at most.
  const float lsq = lengthSquared(half);
  float at[11];
  int n = 0;
  at[n++] = -1.0f;
  at[n++] = 1.0f;
  const auto crossing = [&](const Vec3 &from, const Vec3 &across) {
    const float rate = dot(half, across);
    if (std::fabs(rate) < kTiny) return;
    const float t = dot(from - centre, across) / rate;
    if (t > -1.0f && t < 1.0f) at[n++] = t;
  };
  for (int k = 0; k < 3; ++k) {
    // Where the segment crosses each edge's line, both on the surface and
    // down the column's wall.
    crossing(f.v[k], f.side[k]);
    crossing(f.v[k], f.wall[k]);

    const Vec3 &from = f.v[k];
    Vec3 onSegment, onEdge;
    closestOnSegments(centre - half, half * 2.0f, from, f.v[(k + 1) % 3] - from,
                      onSegment, onEdge);
    at[n++] = clamped(dot(onSegment - centre, half) / lsq, -1.0f, 1.0f);
  }

  for (int i = 0; i < n; ++i) ball(f, centre + half * at[i], radius, into);
}

// --- boxes ---------------------------------------------------------------- //
//
// A box against one triangle's column, by separating axes: the triangle's
// face, the column's three walls, the box's three faces, and the edges of
// each crossed with the edges of the other. The axis that overlaps least is
// the way out — unless the triangle says that way is not open, and then the
// way out is up through its face, which always is.

enum class Axis { face, wall, boxFace, edges, upright };

/// The corners of the column: the triangle, and the same three corners
/// dropped to `floor`.
struct Column {
  Vec3 corner[6];

  void span(const Vec3 &m, float &low, float &high) const {
    low = high = dot(m, corner[0]);
    for (int i = 1; i < 6; ++i) {
      const float d = dot(m, corner[i]);
      low = std::fmin(low, d);
      high = std::fmax(high, d);
    }
  }
};

struct Box {
  Vec3 centre;
  Vec3 axis[3];
  Vec3 half;

  void span(const Vec3 &m, float &low, float &high) const {
    const float middle = dot(m, centre);
    float reach = 0.0f;
    for (int i = 0; i < 3; ++i) reach += half[i] * std::fabs(dot(m, axis[i]));
    low = middle - reach;
    high = middle + reach;
  }

  /// The corner furthest along `m`.
  Vec3 corner(const Vec3 &m) const {
    Vec3 p = centre;
    for (int i = 0; i < 3; ++i) {
      p += axis[i] * (dot(axis[i], m) >= 0.0f ? half[i] : -half[i]);
    }
    return p;
  }
};

struct Way {
  Vec3 out; // Out of the ground, towards the box.
  float depth = 3.0e38f;
  float score = 3.0e38f;
  Axis axis = Axis::face;
  int index = 0; // The box's axis, for a box face or a pair of edges.
  int edge = 0;  // The triangle's edge, for a pair of edges.
};

/// Measures one axis, and keeps it if it is the least overlap so far. False
/// when the box and the column are apart along it, which settles everything.
bool measure(const Box &box, const Column &column, const Vec3 &along, float bias,
             Axis axis, int index, int edge, Way &best) {
  const float squared = lengthSquared(along);
  // A cross product of two parallel edges is nothing, and whatever it would
  // have found one of the face axes finds.
  if (squared < 1.0e-6f) return true;
  const Vec3 m = along * (1.0f / std::sqrt(squared));

  float boxLow, boxHigh, columnLow, columnHigh;
  box.span(m, boxLow, boxHigh);
  column.span(m, columnLow, columnHigh);
  if (boxHigh < columnLow || columnHigh < boxLow) return false;

  const float up = columnHigh - boxLow;
  const float down = boxHigh - columnLow;
  const float depth = std::fmin(up, down);
  if (depth * bias < best.score) {
    best.out = up <= down ? m : -m;
    best.depth = depth;
    best.score = depth * bias;
    best.axis = axis;
    best.index = index;
    best.edge = edge;
  }
  return true;
}

void boxOnFace(const Facet &f, const Box &box, std::vector<Candidate> &into,
               size_t &found) {
  const Vec3 &n = f.normal;
  const float surface = dot(n, f.v[0]);

  // The face of the box that looks most squarely down at the triangle, cut
  // to the triangle's column. What is left and below the surface is touching.
  int face = 0;
  for (int i = 1; i < 3; ++i) {
    if (std::fabs(dot(box.axis[i], n)) > std::fabs(dot(box.axis[face], n))) {
      face = i;
    }
  }
  const float sign = dot(box.axis[face], n) > 0.0f ? -1.0f : 1.0f;
  Poly poly = faceOf(box.centre, box.axis, box.half, face, sign);
  for (int k = 0; k < 3; ++k) {
    poly = clipTo(poly, f.wall[k], dot(f.wall[k], f.v[k]));
  }
  for (uint32_t i = 0; i < poly.n; ++i) {
    const float depth = surface - dot(n, poly.v[i]);
    if (depth < 0.0f) continue;
    add(into, n, poly.v[i] + n * (depth * 0.5f), depth);
    ++found;
  }
}

void boxOnCorners(const Facet &f, const Box &box, const Way &way,
                  std::vector<Candidate> &into, size_t &found) {
  // The box's own face is the one looking back along the way out, and the
  // triangle is cut to that face's edges. Whatever of it is inside the box
  // is touching.
  const int face = way.index;
  const float floor = dot(way.out, box.centre) - box.half[face];

  Poly poly;
  poly.v[0] = f.v[0];
  poly.v[1] = f.v[1];
  poly.v[2] = f.v[2];
  poly.n = 3;
  for (int side = 1; side <= 2; ++side) {
    const int k = (face + side) % 3;
    for (int s = -1; s <= 1; s += 2) {
      const Vec3 m = box.axis[k] * static_cast<float>(s);
      poly = clipTo(poly, m, dot(box.centre, m) + box.half[k]);
    }
  }
  for (uint32_t i = 0; i < poly.n; ++i) {
    const float depth = dot(way.out, poly.v[i]) - floor;
    if (depth < 0.0f) continue;
    add(into, way.out, poly.v[i] - way.out * (depth * 0.5f), depth);
    ++found;
  }
}

void boxOnEdge(const Facet &f, const Box &box, const Way &way,
               std::vector<Candidate> &into, size_t &found) {
  // The box's edge along its own axis that is nearest the ground, and the
  // triangle's edge, where they pass closest.
  const int i = way.index;
  Vec3 middle = box.centre;
  for (int j = 0; j < 3; ++j) {
    if (j == i) continue;
    middle += box.axis[j] *
              (dot(box.axis[j], way.out) > 0.0f ? -box.half[j] : box.half[j]);
  }
  const Vec3 &from = f.v[way.edge];
  const Vec3 edge = f.v[(way.edge + 1) % 3] - from;
  Vec3 onBox, onTriangle;
  closestOnSegments(middle - box.axis[i] * box.half[i],
                    box.axis[i] * (2.0f * box.half[i]), from, edge, onBox,
                    onTriangle);
  add(into, way.out, (onBox + onTriangle) * 0.5f, way.depth);
  ++found;
}

void boxAgainst(const Facet &f, const Box &box, float floor,
                std::vector<Candidate> &into) {
  Column column;
  for (int k = 0; k < 3; ++k) {
    column.corner[k] = f.v[k];
    column.corner[3 + k] = {f.v[k].x, floor, f.v[k].z};
  }

  // Edge axes have to win by a margin, so a box settling flat keeps the face
  // contact that holds it at four corners rather than flicking to an edge
  // that holds it at one whenever rounding makes that edge look a hair
  // shallower. The same margin, and for the same reason, on the column's
  // walls.
  Way best;
  if (!measure(box, column, f.normal, 1.0f, Axis::face, 0, 0, best)) return;
  for (int i = 0; i < 3; ++i) {
    if (!measure(box, column, box.axis[i], 1.0f, Axis::boxFace, i, 0, best)) {
      return;
    }
  }
  for (int k = 0; k < 3; ++k) {
    if (!measure(box, column, f.wall[k], 1.05f, Axis::wall, 0, k, best)) return;
  }
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) {
      const Vec3 edge = f.v[(k + 1) % 3] - f.v[k];
      if (!measure(box, column, cross(box.axis[i], edge), 1.05f, Axis::edges, i,
                   k, best)) {
        return;
      }
    }
    if (!measure(box, column, cross(box.axis[i], kUp), 1.05f, Axis::upright, i,
                 0, best)) {
      return;
    }
  }

  // Whichever way won, the triangle has the last word on whether it is open.
  // Up through the face always is.
  if (best.axis != Axis::face && !f.admits(best.out)) {
    float low, high;
    box.span(f.normal, low, high);
    best.out = f.normal;
    best.depth = dot(f.normal, f.v[0]) - low;
    best.axis = Axis::face;
  }

  size_t found = 0;
  switch (best.axis) {
    case Axis::face: boxOnFace(f, box, into, found); break;
    case Axis::boxFace: boxOnCorners(f, box, best, into, found); break;
    case Axis::edges: boxOnEdge(f, box, best, into, found); break;
    case Axis::wall:
    case Axis::upright: break;
  }

  // A wall, or an overlap the clipping could not put a point on: one point,
  // at the corner of the box deepest along the way out.
  if (found == 0) {
    const Vec3 corner = box.corner(-best.out);
    add(into, best.out, corner + best.out * (best.depth * 0.5f), best.depth);
  }
}

} // namespace

// --- the field ------------------------------------------------------------ //

void HeightField::lay(uint32_t columns, uint32_t rows, float spacing,
                      const float *heights, bool margin) {
  columns_ = columns;
  rows_ = rows;
  spacing_ = spacing > 0.0f ? spacing : 1.0f;
  margin_ = margin ? 1 : 0;
  heights_.assign(static_cast<size_t>(columns) * rows, nan());
  lowest_ = 0.0f;
  highest_ = -1.0f;
  if (heights == nullptr) return;

  bool any = false;
  for (size_t i = 0; i < heights_.size(); ++i) {
    const float h = heights[i];
    // Anything that is not a number is a hole, infinities included: ground
    // at infinity is not ground anyone can stand on.
    if (!std::isfinite(h)) continue;
    heights_[i] = h;
    if (!any) {
      lowest_ = highest_ = h;
      any = true;
    } else {
      lowest_ = std::fmin(lowest_, h);
      highest_ = std::fmax(highest_, h);
    }
  }
}

Bounds HeightField::bounds() const {
  const auto last = [&](uint32_t samples) {
    const int64_t i = static_cast<int64_t>(samples) - 1 - margin_;
    return static_cast<float>(std::max(i, margin_)) * spacing_;
  };
  const float first = static_cast<float>(margin_) * spacing_;
  // Down as far as a float goes, less enough that growing it cannot overflow.
  return {{first, -1.0e30f, first},
          {last(columns_), empty() ? -1.0e30f : highest_, last(rows_)}};
}

float HeightField::height(int64_t c, int64_t r) const {
  if (c < 0 || r < 0 || c >= columns_ || r >= rows_) return nan();
  return heights_[static_cast<size_t>(r) * columns_ + static_cast<size_t>(c)];
}

bool HeightField::cellsUnder(const Bounds &area, int64_t &c0, int64_t &r0,
                             int64_t &c1, int64_t &r1) const {
  const int64_t first = margin_;
  const int64_t lastColumn = static_cast<int64_t>(columns_) - 2 - margin_;
  const int64_t lastRow = static_cast<int64_t>(rows_) - 2 - margin_;
  if (lastColumn < first || lastRow < first) return false;

  // Clamped as floats first, so a body a long way off cannot overflow the
  // conversion to a whole number.
  const auto cell = [&](float at, int64_t last) {
    const float i = std::floor(at / spacing_);
    return static_cast<int64_t>(clamped(i, static_cast<float>(first - 1),
                                        static_cast<float>(last + 1)));
  };
  c0 = cell(area.low.x, lastColumn);
  c1 = cell(area.high.x, lastColumn);
  r0 = cell(area.low.z, lastRow);
  r1 = cell(area.high.z, lastRow);
  if (c1 < first || r1 < first || c0 > lastColumn || r0 > lastRow) return false;
  c0 = std::max(c0, first);
  r0 = std::max(r0, first);
  c1 = std::min(c1, lastColumn);
  r1 = std::min(r1, lastRow);
  return true;
}

bool HeightField::cellHeights(int64_t c, int64_t r, float &low,
                              float &high) const {
  bool any = false;
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < 2; ++i) {
      const float h = height(c + i, r + j);
      if (std::isnan(h)) continue;
      low = any ? std::fmin(low, h) : h;
      high = any ? std::fmax(high, h) : h;
      any = true;
    }
  }
  return any;
}

bool HeightField::corners(int64_t c, int64_t r, int which, Vec3 v[3]) const {
  if (c < 0 || r < 0 || c + 1 >= columns_ || r + 1 >= rows_) return false;
  for (int k = 0; k < 3; ++k) {
    const int64_t i = c + kCorner[which][k][0];
    const int64_t j = r + kCorner[which][k][1];
    const float h = height(i, j);
    if (std::isnan(h)) return false;
    v[k] = {static_cast<float>(i) * spacing_, h, static_cast<float>(j) * spacing_};
  }
  return true;
}

bool HeightField::facet(int64_t c, int64_t r, int which, Facet &out) const {
  if (!corners(c, r, which, out.v)) return false;
  out.normal = normalOf(out.v);

  for (int k = 0; k < 3; ++k) {
    const Vec3 edge = out.v[(k + 1) % 3] - out.v[k];
    out.side[k] = normalised(cross(edge, out.normal));
    out.wall[k] = normalised(cross(edge, kUp));

    const int nc = static_cast<int>(c) + kAcross[which][k][0];
    const int nr = static_cast<int>(r) + kAcross[which][k][1];
    const int nw = kAcross[which][k][2];
    if (nc < 0 || nr < 0 || nc + 1 >= static_cast<int64_t>(columns_) ||
        nr + 1 >= static_cast<int64_t>(rows_)) {
      out.across[k] = Across::seam;
      continue;
    }
    Vec3 next[3];
    if (!corners(nc, nr, nw, next)) {
      out.across[k] = Across::rim;
      continue;
    }
    // The neighbour's own side on this edge, which runs the other way for it.
    out.across[k] = Across::ground;
    out.beyond[k] = normalised(cross(-edge, normalOf(next)));
  }
  return true;
}

// --- a triangle ----------------------------------------------------------- //

bool Facet::admits(const Vec3 &push) const {
  for (int k = 0; k < 3; ++k) {
    if (dot(push, side[k]) <= kLean) continue;
    switch (across[k]) {
      case Across::seam: return false;
      case Across::rim: break;
      case Across::ground:
        if (dot(push, beyond[k]) < -kLean) return false;
        break;
    }
  }
  return true;
}

Vec3 Facet::straightened(const Vec3 &push) const {
  Vec3 n = push;
  for (int k = 0; k < 3; ++k) {
    if (dot(n, side[k]) <= 0.0f) continue;
    switch (across[k]) {
      // Carrying on as it was, as far as anything here knows.
      case Across::seam: n -= side[k] * dot(n, side[k]); break;
      case Across::rim: break;
      case Across::ground: {
        const float past = dot(n, beyond[k]);
        if (past < 0.0f) n -= beyond[k] * past;
        break;
      }
    }
  }
  return normalised(n);
}

bool Facet::over(const Vec3 &p, float slack) const {
  for (int k = 0; k < 3; ++k) {
    if (dot(p - v[k], side[k]) > slack) return false;
  }
  return true;
}

Vec3 Facet::nearest(const Vec3 &p) const {
  // Which of the triangle's seven regions `p` is over — three corners, three
  // edges, the face — worked out from as few dot products as it takes.
  const Vec3 &a = v[0];
  const Vec3 &b = v[1];
  const Vec3 &c = v[2];
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  const Vec3 ap = p - a;
  const float d1 = dot(ab, ap);
  const float d2 = dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f) return a;

  const Vec3 bp = p - b;
  const float d3 = dot(ab, bp);
  const float d4 = dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) return b;

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    return a + ab * (d1 / (d1 - d3));
  }

  const Vec3 cp = p - c;
  const float d5 = dot(ab, cp);
  const float d6 = dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6) return c;

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    return a + ac * (d2 / (d2 - d6));
  }

  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
  }

  const float denom = 1.0f / (va + vb + vc);
  return a + ab * (vb * denom) + ac * (vc * denom);
}

Vec3 Facet::furthest(const Vec3 &direction) const {
  int best = 0;
  for (int k = 1; k < 3; ++k) {
    if (dot(v[k], direction) > dot(v[best], direction)) best = k;
  }
  return v[best];
}

// --- a shape against ground ----------------------------------------------- //

bool collideGround(const Shape &a, const Vec3 &atA, const Quat &rotA,
                   const Shape &ground, const Vec3 &atB, const Quat &rotB,
                   Manifold &out) {
  out.count = 0;
  const HeightField *field = ground.field;
  if (field == nullptr || field->empty()) return false;
  if (a.kind == ShapeKind::plane || a.kind == ShapeKind::heightField) {
    return false;
  }

  // Everything below happens in the field's own frame, where sample (c, r)
  // is at (c * spacing, height, r * spacing).
  const Quat back = conjugate(rotB);
  const Vec3 at = rotate(back, atA - atB);
  const Quat rotation = back * rotA;
  const Bounds area = a.boundsAt(at, rotation).grown(kOnEdge);

  int64_t c0, r0, c1, r1;
  if (!field->cellsUnder(area, c0, r0, c1, r1)) return false;

  std::vector<Candidate> &found = pool();
  found.clear();

  Box box;
  if (a.kind == ShapeKind::box) {
    box.centre = at;
    box.axis[0] = rotate(rotation, {1.0f, 0.0f, 0.0f});
    box.axis[1] = rotate(rotation, {0.0f, 1.0f, 0.0f});
    box.axis[2] = rotate(rotation, {0.0f, 0.0f, 1.0f});
    box.half = a.size;
  }
  const Vec3 half = a.axisAt(rotation);

  Facet f;
  for (int64_t r = r0; r <= r1; ++r) {
    for (int64_t c = c0; c <= c1; ++c) {
      float low, high;
      if (!field->cellHeights(c, r, low, high)) continue;
      // Entirely above this cell's highest corner, it cannot be touching
      // either of its triangles. Below is never skipped: below is inside.
      if (area.low.y > high) continue;
      for (int which = 0; which < 2; ++which) {
        if (!field->facet(c, r, which, f)) continue;
        if (a.kind == ShapeKind::box) {
          boxAgainst(f, box, std::fmin(area.low.y, low) - kBottom, found);
        } else {
          round(f, at, half, a.radius(), found);
        }
      }
    }
  }

  reduce(found, out);
  if (out.count == 0) return false;

  // Back into the world.
  for (uint32_t i = 0; i < out.count; ++i) {
    out.points[i].normal = rotate(rotB, out.points[i].normal);
    out.points[i].at = atB + rotate(rotB, out.points[i].at);
  }
  out.normal = out.points[0].normal;
  return true;
}

} // namespace orblit
