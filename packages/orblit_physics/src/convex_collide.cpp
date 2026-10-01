#include "convex_collide.h"

#include <cmath>
#include <utility>

#include "gjk.h"

namespace orblit {
namespace {

/// How far past a fence a point may be and still count as inside it, in
/// metres. The same as the ground's, and for the same reason: a point exactly
/// on the line between two faces is inside both rather than neither.
constexpr float kSlack = 1.0e-4f;

/// How nearly parallel two edges must be, as a cosine, to be pressed along
/// their length rather than met at one point.
constexpr float kParallel = 0.999f;

/// Two points nearer than this are one, in metres.
constexpr float kSamePoint = 1.0e-3f;

/// A feature being cut. A loop is closed, a line or a point is not.
struct Clip {
  Vec3 v[kMostClip];
  uint32_t n = 0;

  void add(const Vec3 &p) {
    if (n < kMostClip) v[n++] = p;
  }
};

float over(const Surface::Side &side, const Vec3 &p) {
  return dot(side.normal, p) - side.offset;
}

/// `in` with what lies outside one side of the fence cut off. A loop is cut
/// where its edges cross the side, and a line the same, with its last point
/// kept for it has no edge back to the first.
void cutTo(const Surface::Side &side, const Clip &in, bool closed, Clip &out) {
  out.n = 0;
  if (in.n == 0) return;

  const uint32_t edges = closed ? in.n : in.n - 1;
  for (uint32_t i = 0; i < edges; ++i) {
    const Vec3 &from = in.v[i];
    const Vec3 &to = in.v[(i + 1) % in.n];
    const float a = over(side, from);
    const float b = over(side, to);
    if (a <= kSlack) out.add(from);
    if ((a <= kSlack) != (b <= kSlack)) {
      out.add(from + (to - from) * clamped(a / (a - b), 0.0f, 1.0f));
    }
  }
  if (!closed && over(side, in.v[in.n - 1]) <= kSlack) out.add(in.v[in.n - 1]);
}

Clip cutAll(const Surface &surface, const Feature &feature) {
  Clip first;
  Clip second;
  for (uint32_t i = 0; i < feature.n; ++i) first.v[i] = feature.v[i];
  first.n = feature.n;

  const bool closed = feature.isFace();
  Clip *from = &first;
  Clip *to = &second;
  for (uint32_t k = 0; k < surface.sideCount; ++k) {
    cutTo(surface.sides[k], *from, closed, *to);
    std::swap(from, to);
  }
  return *from;
}

/// A face as a surface: its plane, fenced by one plane through each edge.
Surface surfaceOf(const Feature &face) {
  Surface s;
  s.normal = face.normal;
  s.offset = dot(face.normal, face.v[0]);
  for (uint32_t i = 0; i < face.n; ++i) {
    const Vec3 along = face.v[(i + 1) % face.n] - face.v[i];
    const Vec3 out = cross(along, face.normal);
    if (lengthSquared(out) < kTiny) continue;
    const Vec3 side = normalised(out);
    s.sides[s.sideCount++] = {side, dot(side, face.v[i])};
  }
  return s;
}

void flip(Candidate out[], uint32_t count) {
  for (uint32_t i = 0; i < count; ++i) out[i].normal = -out[i].normal;
}

/// The contact when a face of one shape lies against the other. Whichever
/// face looks more squarely at the other shape is the one pressed against.
/// None when neither shape shows a face.
uint32_t pressFace(const Feature &fa, const Feature &fb, float radiusA,
                   float radiusB, const Vec3 &normal, Candidate out[kMostClip]) {
  if (!fa.isFace() && !fb.isFace()) return 0;

  const float alignA = fa.isFace() ? dot(fa.normal, -normal) : -2.0f;
  const float alignB = fb.isFace() ? dot(fb.normal, normal) : -2.0f;
  if (alignB >= alignA) return press(surfaceOf(fb), fa, radiusA, out);

  const uint32_t n = press(surfaceOf(fa), fb, radiusB, out);
  flip(out, n);
  return n;
}

/// The contact when two edges lie side by side: the stretch where they run
/// together, held at both ends so one does not pivot on the other. None when
/// they are not both edges, or are not parallel, or do not overlap.
uint32_t pressEdge(const Feature &ea, const Feature &eb, const Gap &gap,
                   Candidate out[kMostClip]) {
  if (ea.n != 2 || eb.n != 2) return 0;

  const Vec3 d = normalised(eb.v[1] - eb.v[0]);
  if (std::fabs(dot(d, normalised(ea.v[1] - ea.v[0]))) < kParallel) return 0;

  const float a0 = dot(d, ea.v[0]);
  const float a1 = dot(d, ea.v[1]);
  const float b0 = dot(d, eb.v[0]);
  const float b1 = dot(d, eb.v[1]);
  const float low = std::fmax(std::fmin(a0, a1), std::fmin(b0, b1));
  const float high = std::fmin(std::fmax(a0, a1), std::fmax(b0, b1));
  if (high < low) return 0;

  // Along the line, from where the nearest points were found.
  const float base = dot(d, gap.at);
  out[0] = {gap.normal, gap.at + d * (low - base), gap.depth};
  if (high - low < kSamePoint) return 1;
  out[1] = {gap.normal, gap.at + d * (high - base), gap.depth};
  return 2;
}

} // namespace

uint32_t press(const Surface &surface, const Feature &incident, float radius,
               Candidate out[kMostClip]) {
  const Clip clip = cutAll(surface, incident);

  uint32_t found = 0;
  for (uint32_t i = 0; i < clip.n; ++i) {
    const float depth =
        surface.offset - dot(surface.normal, clip.v[i]) + radius;
    if (depth < 0.0f) continue;
    out[found++] = {surface.normal,
                    clip.v[i] + surface.normal * (depth * 0.5f - radius), depth};
  }
  return found;
}

bool collideConvex(const Convex &a, const Convex &b, Manifold &out) {
  out.count = 0;
  Gap gap;
  if (!separation(a, b, gap)) return false;

  const Feature fa = a.feature(-gap.normal);
  const Feature fb = b.feature(gap.normal);

  Candidate found[kMostClip];
  uint32_t n = pressFace(fa, fb, a.radius(), b.radius(), gap.normal, found);
  if (n == 0) n = pressEdge(fa, fb, gap, found);
  if (n == 0) {
    // A corner against anything, or two edges crossing: one point is all
    // there is, and it is where the two surfaces are nearest.
    found[0] = {gap.normal, gap.at, gap.depth};
    n = 1;
  }
  reduce(found, n, out);
  return true;
}

bool collidePlane(const Convex &shape, const Shape &plane, const Vec3 &at,
                  const Quat &rotation, Manifold &out) {
  Surface surface;
  planeInWorld(plane, at, rotation, surface.normal, surface.offset);

  // The plane is `b`, so its normal already points out of it towards the shape.
  Candidate found[kMostClip];
  const uint32_t n =
      press(surface, shape.feature(-surface.normal), shape.radius(), found);
  reduce(found, n, out);
  return out.count > 0;
}

} // namespace orblit
