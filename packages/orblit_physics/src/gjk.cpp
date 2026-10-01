#include "gjk.h"

#include <algorithm>
#include <cmath>

namespace orblit {
namespace {

/// Most rounds either search runs. Both settle in a handful for shapes this
/// size, and a cap is what keeps a stubborn pair from stalling a step.
constexpr int kMostSteps = 64;
constexpr int kMostExpansions = 48;

/// How much nearer a new point may bring the origin, as a share of the squared
/// distance, before the search calls the answer found.
constexpr double kProgress = 1.0e-10;

/// A squared distance under which the origin counts as on the set. That is a
/// nanometre.
constexpr double kOnSet = 1.0e-18;

/// How much further out the set may go past the nearest face, in metres, before
/// the face is called the edge of it.
constexpr double kConverged = 1.0e-9;

/// How far above a face a point must be to see it, in metres.
constexpr double kVisible = 1.0e-12;

/// How far a point must stand off a line or a plane to be a new corner of the
/// polytope rather than another point on it.
constexpr double kIndependent = 1.0e-9;

constexpr int kMostVertices = 64;
constexpr int kMostFaces = 256;

/// One point of the set of differences, with the point of each shape it is the
/// difference of. The witnesses are what turn the answer back into contacts.
struct Vertex {
  D3 w;
  D3 a;
  D3 b;
};

Vertex supportOf(const Convex &a, const Convex &b, const D3 &direction) {
  Vertex v;
  v.a = a.support(direction);
  v.b = b.support(-direction);
  v.w = v.a - v.b;
  return v;
}

// --- the nearest point of a simplex --------------------------------------- //

/// The part of a simplex nearest the origin: which of its corners are kept,
/// with what weight each, and how far off it is.
struct Nearest {
  int index[3] = {0, 0, 0};
  double weight[3] = {0.0, 0.0, 0.0};
  int count = 0;
  double squared = 1.0e300;
};

Nearest onSegment(const D3 &a, const D3 &b, int ia, int ib) {
  const D3 ab = b - a;
  const double along = dot(ab, ab);
  const double t =
      along > 0.0 ? std::fmin(1.0, std::fmax(0.0, -dot(a, ab) / along)) : 0.0;

  Nearest r;
  if (t <= 0.0 || t >= 1.0) {
    r.count = 1;
    r.index[0] = t <= 0.0 ? ia : ib;
    r.weight[0] = 1.0;
  } else {
    r.count = 2;
    r.index[0] = ia;
    r.index[1] = ib;
    r.weight[0] = 1.0 - t;
    r.weight[1] = t;
  }
  const D3 p = a + ab * t;
  r.squared = dot(p, p);
  return r;
}

Nearest onTriangle(const D3 &a, const D3 &b, const D3 &c, const int index[3]) {
  const D3 ab = b - a;
  const D3 ac = c - a;
  const D3 n = cross(ab, ac);
  const double n2 = dot(n, n);

  // The origin's share of each corner, from the areas of the three triangles
  // it makes with the edges. All three are positive when it is over the face.
  if (n2 > 1.0e-20 * dot(ab, ab) * dot(ac, ac)) {
    const double wa = dot(cross(b, c), n) / n2;
    const double wb = dot(cross(c, a), n) / n2;
    const double wc = 1.0 - wa - wb;
    if (wa >= 0.0 && wb >= 0.0 && wc >= 0.0) {
      Nearest r;
      r.count = 3;
      for (int i = 0; i < 3; ++i) r.index[i] = index[i];
      r.weight[0] = wa;
      r.weight[1] = wb;
      r.weight[2] = wc;
      const D3 p = a * wa + b * wb + c * wc;
      r.squared = dot(p, p);
      return r;
    }
  }

  Nearest best = onSegment(a, b, index[0], index[1]);
  const Nearest bc = onSegment(b, c, index[1], index[2]);
  const Nearest ca = onSegment(c, a, index[2], index[0]);
  if (bc.squared < best.squared) best = bc;
  if (ca.squared < best.squared) best = ca;
  return best;
}

/// Up to four points whose hull is closing in on the origin.
struct Simplex {
  Vertex v[4];
  double weight[4] = {0.0, 0.0, 0.0, 0.0};
  int n = 0;

  D3 point() const {
    D3 p;
    for (int i = 0; i < n; ++i) p = p + v[i].w * weight[i];
    return p;
  }

  D3 onA() const {
    D3 p;
    for (int i = 0; i < n; ++i) p = p + v[i].a * weight[i];
    return p;
  }

  D3 onB() const {
    D3 p;
    for (int i = 0; i < n; ++i) p = p + v[i].b * weight[i];
    return p;
  }

  bool holds(const Vertex &w) const {
    for (int i = 0; i < n; ++i) {
      const D3 d = v[i].w - w.w;
      if (dot(d, d) <= 1.0e-24) return true;
    }
    return false;
  }
};

/// Whether the origin is inside a tetrahedron, and if not, the nearest part of
/// it. A flat tetrahedron has no inside, so every one of its faces is asked.
bool insideTetrahedron(const Simplex &s, Nearest &nearest) {
  bool inside = true;
  for (int skip = 0; skip < 4; ++skip) {
    int idx[3];
    int k = 0;
    for (int i = 0; i < 4; ++i) {
      if (i != skip) idx[k++] = i;
    }
    const D3 &p = s.v[idx[0]].w;
    const D3 n = cross(s.v[idx[1]].w - p, s.v[idx[2]].w - p);
    const double towards = dot(s.v[skip].w - p, n);
    const double origin = dot(-p, n);
    const bool outside =
        towards > 0.0 ? origin < 0.0 : (towards < 0.0 ? origin > 0.0 : true);
    if (!outside) continue;

    inside = false;
    const Nearest face = onTriangle(p, s.v[idx[1]].w, s.v[idx[2]].w, idx);
    if (face.squared < nearest.squared) nearest = face;
  }
  return inside;
}

/// Cuts `s` down to the part of it nearest the origin, and weighs it. True
/// when the origin is inside instead, with nothing nearer to find.
bool closestPart(Simplex &s) {
  Nearest part;
  const int all[3] = {0, 1, 2};
  switch (s.n) {
    case 1:
      s.weight[0] = 1.0;
      return false;
    case 2: part = onSegment(s.v[0].w, s.v[1].w, 0, 1); break;
    case 3: part = onTriangle(s.v[0].w, s.v[1].w, s.v[2].w, all); break;
    default:
      if (insideTetrahedron(s, part)) return true;
      break;
  }

  Vertex kept[3];
  for (int i = 0; i < part.count; ++i) kept[i] = s.v[part.index[i]];
  s.n = part.count;
  for (int i = 0; i < s.n; ++i) {
    s.v[i] = kept[i];
    s.weight[i] = part.weight[i];
  }
  return false;
}

// --- the gap -------------------------------------------------------------- //

enum class Verdict { apart, overlapping };

/// Closes in on the origin until it is found apart from the set, with `s` the
/// part of the set nearest it, or inside it.
Verdict search(const Convex &a, const Convex &b, Simplex &s) {
  D3 first = b.middle() - a.middle();
  if (dot(first, first) <= 0.0) first = D3(1.0, 0.0, 0.0);
  s.v[0] = supportOf(a, b, first);
  s.n = 1;

  for (int step = 0; step < kMostSteps; ++step) {
    if (closestPart(s)) return Verdict::overlapping;
    const D3 v = s.point();
    const double v2 = dot(v, v);
    if (v2 <= kOnSet) return Verdict::overlapping;

    // The set's furthest point towards the origin. If it is no nearer than
    // where the search already is, nothing is.
    const Vertex w = supportOf(a, b, -v);
    if (v2 - dot(v, w.w) <= kProgress * v2 || s.holds(w)) return Verdict::apart;
    s.v[s.n++] = w;
  }
  return Verdict::apart;
}

// --- the overlap ---------------------------------------------------------- //

struct Penetration {
  /// Out of `b` towards `a`.
  D3 normal;
  double depth = 0.0;
  D3 onA;
  D3 onB;
};

/// A polytope grown outwards, face by face, until its nearest face to the
/// origin is on the edge of the set of differences.
///
/// The origin is inside the set, so the nearest edge of the set is the way
/// out that costs least: its distance is how deep the shapes overlap and its
/// direction is the way to separate them.
class Polytope {
 public:
  Polytope(const Convex &a, const Convex &b) : a_(a), b_(b) {}

  /// Starts from `start`, which holds the origin or has it on its edge.
  /// False when the set of differences is flat, and has no inside to leave.
  bool penetration(const Simplex &start, Penetration &out) {
    if (!seed(start)) return false;
    for (int step = 0; step < kMostExpansions; ++step) {
      const Face &face = faces_[nearest()];
      const Vertex w = supportOf(a_, b_, face.normal);
      if (dot(face.normal, w.w) - face.distance <= kConverged) break;
      if (vertexCount_ == kMostVertices) break;
      vertices_[vertexCount_] = w;
      if (!expand(vertexCount_)) break;
      ++vertexCount_;
    }
    out = through(faces_[nearest()]);
    return true;
  }

 private:
  struct Face {
    int v[3] = {0, 0, 0};
    /// Out of the polytope.
    D3 normal;
    double distance = 1.0e300;
  };

  struct Edge {
    int from;
    int to;
  };

  const D3 &at(int i) const { return vertices_[i].w; }

  bool seed(const Simplex &start) {
    vertexCount_ = 0;
    for (int i = 0; i < start.n; ++i) vertices_[vertexCount_++] = start.v[i];
    while (vertexCount_ < 4) {
      if (!grow()) return false;
    }

    faceCount_ = 0;
    for (int skip = 0; skip < 4; ++skip) {
      int i[3];
      int k = 0;
      for (int j = 0; j < 4; ++j) {
        if (j != skip) i[k++] = j;
      }
      // Wound away from the corner it does not hold, whichever way round the
      // simplex came.
      const D3 n = cross(at(i[1]) - at(i[0]), at(i[2]) - at(i[0]));
      if (dot(at(skip) - at(i[0]), n) > 0.0) std::swap(i[1], i[2]);
      push(i[0], i[1], i[2]);
    }
    return true;
  }

  /// Adds a corner that lifts the polytope into one more dimension.
  bool grow() {
    D3 direction[6];
    const int count = candidates(direction);
    for (int i = 0; i < count; ++i) {
      if (norm(direction[i]) <= 0.0) continue;
      const Vertex w = supportOf(a_, b_, direction[i]);
      if (!independent(w)) continue;
      vertices_[vertexCount_++] = w;
      return true;
    }
    return false;
  }

  /// Directions that cannot all lie in what there is: the axes for a point,
  /// across the line for a line, either way out of the plane for a plane.
  int candidates(D3 out[6]) const {
    const D3 axes[3] = {D3(1.0, 0.0, 0.0), D3(0.0, 1.0, 0.0), D3(0.0, 0.0, 1.0)};
    if (vertexCount_ == 1) {
      for (int i = 0; i < 3; ++i) {
        out[2 * i] = axes[i];
        out[2 * i + 1] = -axes[i];
      }
      return 6;
    }

    if (vertexCount_ == 2) {
      const D3 edge = at(1) - at(0);
      for (int i = 0; i < 3; ++i) {
        out[2 * i] = cross(edge, axes[i]);
        out[2 * i + 1] = -out[2 * i];
      }
      return 6;
    }

    const D3 n = cross(at(1) - at(0), at(2) - at(0));
    out[0] = n;
    out[1] = -n;
    return 2;
  }

  bool independent(const Vertex &w) const {
    const D3 d = w.w - at(0);
    if (vertexCount_ == 1) return norm(d) > kIndependent;

    const D3 e = at(1) - at(0);
    if (vertexCount_ == 2) return norm(cross(e, d)) > kIndependent * norm(e);

    const D3 n = cross(e, at(2) - at(0));
    return std::fabs(dot(n, d)) > kIndependent * norm(n);
  }

  void push(int i, int j, int k) {
    Face &f = faces_[faceCount_++];
    f.v[0] = i;
    f.v[1] = j;
    f.v[2] = k;

    const D3 e1 = at(j) - at(i);
    const D3 e2 = at(k) - at(i);
    const D3 n = cross(e1, e2);
    const double size = norm(n);
    if (size <= 1.0e-14 * norm(e1) * norm(e2) || size <= 0.0) {
      // A sliver has no direction worth trusting. It never wins and is never
      // seen, and stays in only to keep the surface closed.
      f.normal = D3();
      f.distance = 1.0e300;
      return;
    }
    f.normal = n * (1.0 / size);
    f.distance = std::fmax(0.0, dot(f.normal, at(i)));
  }

  int nearest() const {
    int best = 0;
    for (int i = 1; i < faceCount_; ++i) {
      if (faces_[i].distance < faces_[best].distance) best = i;
    }
    return best;
  }

  /// Whether a face in `visible` runs from `from` to `to`.
  bool runs(const bool visible[], int from, int to) const {
    for (int f = 0; f < faceCount_; ++f) {
      if (!visible[f]) continue;
      for (int k = 0; k < 3; ++k) {
        if (faces_[f].v[k] == from && faces_[f].v[(k + 1) % 3] == to) return true;
      }
    }
    return false;
  }

  /// Replaces the faces that can see corner `added` with a cone of faces from
  /// it to the edge of what it could see.
  bool expand(int added) {
    bool visible[kMostFaces];
    int seen = 0;
    for (int f = 0; f < faceCount_; ++f) {
      const Face &face = faces_[f];
      visible[f] = dot(face.normal, at(added) - at(face.v[0])) > kVisible;
      seen += visible[f] ? 1 : 0;
    }
    if (seen == 0) return false;

    Edge horizon[kMostFaces];
    int edges = 0;
    for (int f = 0; f < faceCount_; ++f) {
      if (!visible[f]) continue;
      for (int k = 0; k < 3; ++k) {
        const int from = faces_[f].v[k];
        const int to = faces_[f].v[(k + 1) % 3];
        if (runs(visible, to, from)) continue;
        if (edges == kMostFaces) return false;
        horizon[edges++] = {from, to};
      }
    }
    if (edges == 0 || faceCount_ - seen + edges > kMostFaces) return false;

    int kept = 0;
    for (int f = 0; f < faceCount_; ++f) {
      if (!visible[f]) faces_[kept++] = faces_[f];
    }
    faceCount_ = kept;
    for (int e = 0; e < edges; ++e) push(horizon[e].from, horizon[e].to, added);
    return true;
  }

  /// The way out through `face`, and the point of each shape it is between.
  Penetration through(const Face &face) const {
    const Vertex &p = vertices_[face.v[0]];
    const Vertex &q = vertices_[face.v[1]];
    const Vertex &r = vertices_[face.v[2]];

    // The nearest point of the face to the origin, as a share of each corner.
    // The face is not always the true edge of the set, so the origin's
    // projection onto it may be a little outside it.
    const D3 n = cross(q.w - p.w, r.w - p.w);
    const double n2 = dot(n, n);
    double wp = 1.0;
    double wq = 0.0;
    double wr = 0.0;
    if (n2 > 0.0) {
      wp = std::fmax(0.0, dot(cross(q.w, r.w), n) / n2);
      wq = std::fmax(0.0, dot(cross(r.w, p.w), n) / n2);
      wr = std::fmax(0.0, dot(cross(p.w, q.w), n) / n2);
      const double sum = wp + wq + wr;
      if (sum > 0.0) {
        wp /= sum;
        wq /= sum;
        wr /= sum;
      }
    }

    Penetration out;
    out.normal = -face.normal;
    out.depth = face.distance;
    out.onA = p.a * wp + q.a * wq + r.a * wr;
    out.onB = p.b * wp + q.b * wq + r.b * wr;
    return out;
  }

  const Convex &a_;
  const Convex &b_;
  Vertex vertices_[kMostVertices];
  int vertexCount_ = 0;
  Face faces_[kMostFaces];
  int faceCount_ = 0;
};

/// A gap along `normal` between the point `onA` of one core and `onB` of the
/// other, with each surface standing its radius off its core.
Gap gapOf(const D3 &normal, double depth, const D3 &onA, const D3 &onB,
          double radiusA, double radiusB) {
  Gap out;
  out.normal = toVec(normal);
  out.depth = static_cast<float>(depth);
  out.at = toVec(((onA - normal * radiusA) + (onB + normal * radiusB)) * 0.5);
  return out;
}

/// What two shapes whose cores overlap in nothing a polytope can be grown in
/// have to go on: where they are. A capsule through a sphere's centre is one.
Gap coincident(const Convex &a, const Convex &b) {
  D3 apart = a.middle() - b.middle();
  const double distance = norm(apart);
  apart = distance > 1.0e-9 ? apart * (1.0 / distance) : D3(0.0, 1.0, 0.0);
  const double reach = static_cast<double>(a.radius()) + b.radius();
  return gapOf(apart, reach, a.middle(), b.middle(), a.radius(), b.radius());
}

} // namespace

bool separation(const Convex &a, const Convex &b, Gap &out) {
  const double radiusA = a.radius();
  const double radiusB = b.radius();

  Simplex s;
  if (search(a, b, s) == Verdict::apart) {
    const D3 v = s.point();
    const double distance = norm(v);
    if (distance > radiusA + radiusB) return false;
    out = gapOf(v * (1.0 / distance), radiusA + radiusB - distance, s.onA(),
                s.onB(), radiusA, radiusB);
    return true;
  }

  Polytope polytope(a, b);
  Penetration p;
  if (!polytope.penetration(s, p)) {
    out = coincident(a, b);
    return true;
  }
  out = gapOf(p.normal, p.depth + radiusA + radiusB, p.onA, p.onB, radiusA,
              radiusB);
  return true;
}

Vec3 nearestOnCore(const Convex &shape, const Vec3 &to) {
  // The point as a shape of its own, so the same search that measures a gap
  // between two shapes measures this one.
  const Convex point(Shape::sphere(0.0f), to, Quat());
  Simplex s;
  if (search(shape, point, s) == Verdict::overlapping) return to;
  return toVec(s.onA());
}

} // namespace orblit
