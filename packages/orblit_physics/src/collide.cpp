#include "collide.h"

namespace orblit {
namespace {

/// The three world axes of a rotated body.
void axesOf(const Quat &q, Vec3 axis[3]) {
  axis[0] = rotate(q, {1.0f, 0.0f, 0.0f});
  axis[1] = rotate(q, {0.0f, 1.0f, 0.0f});
  axis[2] = rotate(q, {0.0f, 0.0f, 1.0f});
}

/// A plane body as a world normal and the value dot(normal, x) takes on its
/// surface. Everything below the plane has a smaller value than that.
void planeInWorld(const Shape &s, const Vec3 &at, const Quat &rot, Vec3 &normal,
                  float &surface) {
  normal = rotate(rot, s.size);
  surface = s.offset + dot(normal, at);
}

void clearImpulses(Contact &c) {
  c.normalImpulse = 0.0f;
  c.frictionImpulse[0] = 0.0f;
  c.frictionImpulse[1] = 0.0f;
}

void one(Manifold &out, const Vec3 &normal, const Vec3 &at, float depth) {
  out.normal = normal;
  out.points[0].at = at;
  out.points[0].depth = depth;
  clearImpulses(out.points[0]);
  out.count = 1;
}

/// Adds a point, or takes the place of the shallowest one already there.
///
/// Four is the budget, and which four matters: the deepest are the ones the
/// solver has the most work to do on, and dropping one of them for a point
/// that is barely touching is how a corner sinks through a floor.
void keepDeepest(Manifold &out, const Vec3 &at, float depth) {
  uint32_t slot = out.count;
  if (out.count == kMaxContacts) {
    slot = 0;
    for (uint32_t i = 1; i < kMaxContacts; ++i) {
      if (out.points[i].depth < out.points[slot].depth) slot = i;
    }
    if (depth <= out.points[slot].depth) return;
  } else {
    ++out.count;
  }
  out.points[slot].at = at;
  out.points[slot].depth = depth;
  clearImpulses(out.points[slot]);
}

bool flipped(bool hit, Manifold &out) {
  if (hit) out.normal = -out.normal;
  return hit;
}

// --- pairs ---------------------------------------------------------------- //

bool sphereSphere(const Shape &a, const Vec3 &pa, const Shape &b,
                  const Vec3 &pb, Manifold &out) {
  const Vec3 d = pa - pb;
  const float reach = a.radius() + b.radius();
  const float squared = lengthSquared(d);
  if (squared > reach * reach) return false;

  // Two spheres at the same point have no direction to push along. Up is as
  // arbitrary as any other answer and better than the nan.
  const float dist = std::sqrt(squared);
  const Vec3 n = dist > kTiny ? d * (1.0f / dist) : Vec3{0.0f, 1.0f, 0.0f};
  const Vec3 at = (pa - n * a.radius() + pb + n * b.radius()) * 0.5f;
  one(out, n, at, reach - dist);
  return true;
}

bool spherePlane(const Shape &a, const Vec3 &pa, const Shape &b, const Vec3 &pb,
                 const Quat &qb, Manifold &out) {
  Vec3 n;
  float surface;
  planeInWorld(b, pb, qb, n, surface);

  const float above = dot(n, pa) - surface;
  const float depth = a.radius() - above;
  if (depth < 0.0f) return false;

  one(out, n, pa - n * (a.radius() - depth * 0.5f), depth);
  return true;
}

bool sphereBox(const Shape &a, const Vec3 &pa, const Shape &b, const Vec3 &pb,
               const Quat &qb, Manifold &out) {
  const Vec3 local = unrotate(qb, pa - pb);
  Vec3 closest;
  for (int i = 0; i < 3; ++i) closest[i] = clamped(local[i], -b.size[i], b.size[i]);

  const Vec3 d = local - closest;
  const float squared = lengthSquared(d);
  const float radius = a.radius();
  if (squared > radius * radius) return false;

  if (squared > kTiny) {
    const float dist = std::sqrt(squared);
    one(out, rotate(qb, d * (1.0f / dist)), pb + rotate(qb, closest),
        radius - dist);
    return true;
  }

  // The centre is inside the box, so there is no nearest surface point to
  // push away from. Leave through the nearest face: any other direction would
  // drag the sphere through more box than it is already in.
  int axis = 0;
  float least = b.size[0] - std::fabs(local[0]);
  for (int i = 1; i < 3; ++i) {
    const float gap = b.size[i] - std::fabs(local[i]);
    if (gap < least) {
      least = gap;
      axis = i;
    }
  }

  const float sign = local[axis] < 0.0f ? -1.0f : 1.0f;
  Vec3 outward;
  outward[axis] = sign;
  Vec3 surface = local;
  surface[axis] = sign * b.size[axis];
  one(out, rotate(qb, outward), pb + rotate(qb, surface), radius + least);
  return true;
}

bool boxPlane(const Shape &a, const Vec3 &pa, const Quat &qa, const Shape &b,
              const Vec3 &pb, const Quat &qb, Manifold &out) {
  Vec3 n;
  float surface;
  planeInWorld(b, pb, qb, n, surface);

  Vec3 axis[3];
  axesOf(qa, axis);

  out.normal = n;
  out.count = 0;
  for (int i = 0; i < 8; ++i) {
    const Vec3 corner = pa + axis[0] * ((i & 1) ? a.size.x : -a.size.x) +
                        axis[1] * ((i & 2) ? a.size.y : -a.size.y) +
                        axis[2] * ((i & 4) ? a.size.z : -a.size.z);
    const float depth = surface - dot(n, corner);
    if (depth < 0.0f) continue;
    keepDeepest(out, corner + n * (depth * 0.5f), depth);
  }
  return out.count > 0;
}

// --- box against box ------------------------------------------------------ //

/// A face being clipped. Eight is the most a quad can have after four cuts.
struct Poly {
  Vec3 v[8];
  uint32_t n = 0;
};

/// One face of a box, wound so consecutive corners share an edge.
Poly faceOf(const Vec3 &centre, const Vec3 axis[3], const Vec3 &half, int i,
            float sign) {
  const int j = (i + 1) % 3;
  const int k = (i + 2) % 3;
  const Vec3 middle = centre + axis[i] * (sign * half[i]);
  const Vec3 u = axis[j] * half[j];
  const Vec3 w = axis[k] * half[k];

  Poly p;
  p.v[0] = middle + u + w;
  p.v[1] = middle - u + w;
  p.v[2] = middle - u - w;
  p.v[3] = middle + u - w;
  p.n = 4;
  return p;
}

/// The part of `in` on the near side of a plane, cut where it crosses.
Poly clipTo(const Poly &in, const Vec3 &n, float surface) {
  Poly out;
  for (uint32_t i = 0; i < in.n && out.n < 8; ++i) {
    const Vec3 &p = in.v[i];
    const Vec3 &q = in.v[(i + 1) % in.n];
    const float dp = dot(n, p) - surface;
    const float dq = dot(n, q) - surface;
    if (dp <= 0.0f) out.v[out.n++] = p;
    if (dp * dq < 0.0f && out.n < 8) {
      out.v[out.n++] = p + (q - p) * (dp / (dp - dq));
    }
  }
  return out;
}

/// The nearest pair of points on two segments, each given as a start and a
/// full direction.
void closestOnSegments(const Vec3 &p1, const Vec3 &d1, const Vec3 &p2,
                       const Vec3 &d2, Vec3 &c1, Vec3 &c2) {
  const Vec3 r = p1 - p2;
  const float a = dot(d1, d1);
  const float e = dot(d2, d2);
  const float b = dot(d1, d2);
  const float c = dot(d1, r);
  const float f = dot(d2, r);

  const float denom = a * e - b * b;
  float s = denom > kTiny ? clamped((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
  const float t = e > kTiny ? clamped((b * s + f) / e, 0.0f, 1.0f) : 0.0f;
  s = a > kTiny ? clamped((b * t - c) / a, 0.0f, 1.0f) : 0.0f;

  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
}

bool boxBox(const Shape &sa, const Vec3 &pa, const Quat &qa, const Shape &sb,
            const Vec3 &pb, const Quat &qb, Manifold &out) {
  Vec3 ax[3], bx[3];
  axesOf(qa, ax);
  axesOf(qb, bx);
  const Vec3 between = pb - pa;

  float bestScore = 3.0e38f;
  float bestDepth = 0.0f;
  int bestAxis = -1;
  Vec3 bestNormal;  // Always from a towards b.

  // `bias` above one makes an axis have to win by that much. Edge axes get it
  // so that a box settling flat keeps its four-point face contact instead of
  // flicking to a one-point edge contact whenever rounding makes an edge look
  // a hair shallower.
  const auto test = [&](const Vec3 &along, int which, float bias) {
    const float squared = lengthSquared(along);
    // A cross product of two parallel edges is nothing; the case it would have
    // caught is already covered by the face axes.
    if (squared < 1.0e-6f) return true;

    const Vec3 n = along * (1.0f / std::sqrt(squared));
    float reachA = 0.0f, reachB = 0.0f;
    for (int i = 0; i < 3; ++i) {
      reachA += sa.size[i] * std::fabs(dot(n, ax[i]));
      reachB += sb.size[i] * std::fabs(dot(n, bx[i]));
    }

    const float apart = dot(between, n);
    const float overlap = reachA + reachB - std::fabs(apart);
    if (overlap < 0.0f) return false;

    const float score = overlap * bias;
    if (score < bestScore) {
      bestScore = score;
      bestDepth = overlap;
      bestAxis = which;
      bestNormal = apart < 0.0f ? -n : n;
    }
    return true;
  };

  for (int i = 0; i < 3; ++i) {
    if (!test(ax[i], i, 1.0f)) return false;
    if (!test(bx[i], 3 + i, 1.0f)) return false;
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (!test(cross(ax[i], bx[j]), 6 + i * 3 + j, 1.05f)) return false;
    }
  }
  if (bestAxis < 0) return false;

  if (bestAxis >= 6) {
    // Two edges crossing. One point, where they pass closest.
    const int i = (bestAxis - 6) / 3;
    const int j = (bestAxis - 6) % 3;
    Vec3 ca = pa, cb = pb;
    for (int k = 0; k < 3; ++k) {
      if (k == i) continue;
      ca += ax[k] * (dot(ax[k], bestNormal) > 0.0f ? sa.size[k] : -sa.size[k]);
    }
    for (int k = 0; k < 3; ++k) {
      if (k == j) continue;
      cb += bx[k] * (dot(bx[k], bestNormal) < 0.0f ? sb.size[k] : -sb.size[k]);
    }

    Vec3 p1, p2;
    closestOnSegments(ca - ax[i] * sa.size[i], ax[i] * (2.0f * sa.size[i]),
                      cb - bx[j] * sb.size[j], bx[j] * (2.0f * sb.size[j]), p1,
                      p2);
    one(out, -bestNormal, (p1 + p2) * 0.5f, bestDepth);
    return true;
  }

  // A face against a face. Clip the other box's nearest face to this one's
  // edges and keep whatever is left below the surface.
  const bool referenceIsA = bestAxis < 3;
  const Vec3 outward = referenceIsA ? bestNormal : -bestNormal;
  const Vec3 *rAxis = referenceIsA ? ax : bx;
  const Vec3 &rCentre = referenceIsA ? pa : pb;
  const Vec3 &rHalf = referenceIsA ? sa.size : sb.size;
  const Vec3 *iAxis = referenceIsA ? bx : ax;
  const Vec3 &iCentre = referenceIsA ? pb : pa;
  const Vec3 &iHalf = referenceIsA ? sb.size : sa.size;
  const int rIndex = referenceIsA ? bestAxis : bestAxis - 3;

  // The incident face is whichever of the other box's faces looks most
  // directly back at us.
  int iIndex = 0;
  float squarest = std::fabs(dot(iAxis[0], outward));
  for (int i = 1; i < 3; ++i) {
    const float aligned = std::fabs(dot(iAxis[i], outward));
    if (aligned > squarest) {
      squarest = aligned;
      iIndex = i;
    }
  }
  const float iSign = dot(iAxis[iIndex], outward) > 0.0f ? -1.0f : 1.0f;

  Poly poly = faceOf(iCentre, iAxis, iHalf, iIndex, iSign);
  for (int side = 1; side <= 2; ++side) {
    const int k = (rIndex + side) % 3;
    for (int sign = -1; sign <= 1; sign += 2) {
      const Vec3 n = rAxis[k] * static_cast<float>(sign);
      poly = clipTo(poly, n, dot(rCentre, n) + rHalf[k]);
    }
  }

  const float surface = dot(rCentre, outward) + rHalf[rIndex];
  out.normal = referenceIsA ? -outward : outward;
  out.count = 0;
  for (uint32_t i = 0; i < poly.n; ++i) {
    const float depth = surface - dot(poly.v[i], outward);
    if (depth < 0.0f) continue;
    keepDeepest(out, poly.v[i] + outward * (depth * 0.5f), depth);
  }
  return out.count > 0;
}

} // namespace

bool collide(const Shape &a, const Vec3 &atA, const Quat &rotA, const Shape &b,
             const Vec3 &atB, const Quat &rotB, Manifold &out) {
  out.count = 0;
  switch (a.kind) {
    case ShapeKind::sphere:
      switch (b.kind) {
        case ShapeKind::sphere: return sphereSphere(a, atA, b, atB, out);
        case ShapeKind::box: return sphereBox(a, atA, b, atB, rotB, out);
        case ShapeKind::plane: return spherePlane(a, atA, b, atB, rotB, out);
      }
      return false;
    case ShapeKind::box:
      switch (b.kind) {
        case ShapeKind::sphere:
          return flipped(sphereBox(b, atB, a, atA, rotA, out), out);
        case ShapeKind::box: return boxBox(a, atA, rotA, b, atB, rotB, out);
        case ShapeKind::plane:
          return boxPlane(a, atA, rotA, b, atB, rotB, out);
      }
      return false;
    case ShapeKind::plane:
      switch (b.kind) {
        case ShapeKind::sphere:
          return flipped(spherePlane(b, atB, a, atA, rotA, out), out);
        case ShapeKind::box:
          return flipped(boxPlane(b, atB, rotB, a, atA, rotA, out), out);
        // Two half-spaces either miss entirely or overlap in a region with no
        // finite contact to write down. Neither is worth a manifold.
        case ShapeKind::plane: return false;
      }
      return false;
  }
  return false;
}

} // namespace orblit
