#include "cast.h"

#include "collide.h"

namespace orblit {
namespace {

/// Close enough to touching to call it a touch, in metres.
///
/// Conservative advancement approaches the impact without ever reaching it, so
/// something has to say when to stop. Well under the solver's slop, so a cast
/// never reports a hit the solver would then have to push apart.
constexpr float kTouching = 1.0e-4f;

/// A cast that needs more turns than this is one against a shape it is sliding
/// along rather than approaching. Stopping is the right answer there, and the
/// last bound is still short of the impact, so stopping is also safe.
constexpr uint32_t kMostSteps = 32;

Placed movedTo(const Placed &of, const Vec3 &at) { return {of.shape, at, of.rotation}; }

/// A plane body as a world normal and the value dot(normal, x) takes on its
/// surface.
void planeInWorld(const Placed &of, Vec3 &normal, float &surface) {
  normal = rotate(of.rotation, of.shape.size);
  surface = of.shape.offset + dot(normal, of.at);
}

/// The nearest point of a shape's core to `to` — the core being what is left
/// of it once the rounding is taken off: a point for a sphere, a segment for a
/// capsule, and a box for a box.
///
/// Used only to find a direction worth measuring along, never to answer a
/// distance, so the iteration below can stop whenever it likes.
Vec3 nearestInCore(const Placed &of, const Vec3 &to) {
  switch (of.shape.kind) {
    case ShapeKind::sphere: return of.at;
    case ShapeKind::capsule: {
      const Vec3 half = of.shape.axisAt(of.rotation);
      const float lsq = lengthSquared(half);
      if (lsq < kTiny) return of.at;
      return of.at + half * clamped(dot(to - of.at, half) / lsq, -1.0f, 1.0f);
    }
    case ShapeKind::box: {
      const Vec3 local = unrotate(of.rotation, to - of.at);
      const Vec3 &e = of.shape.size;
      const Vec3 inside{clamped(local.x, -e.x, e.x), clamped(local.y, -e.y, e.y),
                        clamped(local.z, -e.z, e.z)};
      return of.at + rotate(of.rotation, inside);
    }
    case ShapeKind::plane: return to;
  }
  return of.at;
}

/// A direction from `a` towards `b` worth measuring the gap along.
///
/// Found by stepping between the two cores: nearest point on one to the
/// other, then back again. Each step can only shorten what is between them,
/// so it settles rather than wanders. Six is well past where it stops moving
/// for shapes of the size a game uses, and a seventh would not make the
/// answer below any more correct — only the bound slightly tighter.
Vec3 between(const Placed &a, const Placed &b) {
  // Started from a's centre, because the loop asks b first and so never reads
  // an onB it has not written.
  Vec3 onA = a.at;
  Vec3 onB;
  for (int i = 0; i < 6; ++i) {
    onB = nearestInCore(b, onA);
    onA = nearestInCore(a, onB);
  }

  const Vec3 apart = onB - onA;
  if (lengthSquared(apart) > kTiny * kTiny) return normalised(apart);

  // The cores meet, which happens whenever one shape is inside the other and
  // also whenever two rounded shapes touch core to core. Centre to centre is
  // the next best question to ask.
  return normalised(b.at - a.at);
}

/// How far apart two shapes are along `v`, at least.
///
/// The true distance is never smaller than this, whatever `v` is, which is
/// what makes it safe to advance by. Negative means they overlap along `v`
/// and says nothing about whether they overlap at all.
float gapAlong(const Placed &a, const Placed &b, const Vec3 &v) {
  return dot(v, support(b, -v)) - dot(v, support(a, v));
}

bool sweepPlane(const Placed &moving, const Vec3 &direction, float distance,
                const Placed &fixed, Impact &out) {
  Vec3 n;
  float surface;
  planeInWorld(fixed, n, surface);

  // The lowest point of the moving shape, and how fast that height falls. A
  // shape translating without turning keeps the same lowest point throughout,
  // so this is exact rather than advanced towards.
  const float lowest = dot(n, support(moving, -n)) - surface;
  const float closing = dot(n, direction);

  if (lowest <= 0.0f) {
    out.started = true;
    out.distance = 0.0f;
    out.normal = n;
    out.at = support(moving, -n);
    return true;
  }
  if (closing >= 0.0f) return false;

  const float when = lowest / -closing;
  if (when > distance) return false;

  out.started = false;
  out.distance = when;
  out.normal = n;
  out.at = support(movedTo(moving, moving.at + direction * when), -n);
  return true;
}

} // namespace

Vec3 support(const Placed &of, const Vec3 &direction) {
  switch (of.shape.kind) {
    case ShapeKind::sphere:
      return of.at + normalised(direction) * of.shape.radius();
    case ShapeKind::capsule: {
      const Vec3 half = of.shape.axisAt(of.rotation);
      const Vec3 end = dot(direction, half) >= 0.0f ? half : -half;
      return of.at + end + normalised(direction) * of.shape.radius();
    }
    case ShapeKind::box: {
      Vec3 furthest = of.at;
      for (int i = 0; i < 3; ++i) {
        Vec3 axis;
        axis[i] = 1.0f;
        const Vec3 world = rotate(of.rotation, axis);
        furthest += world * (dot(direction, world) >= 0.0f ? of.shape.size[i]
                                                           : -of.shape.size[i]);
      }
      return furthest;
    }
    case ShapeKind::plane: return of.at;
  }
  return of.at;
}

bool sweep(const Placed &moving, const Vec3 &direction, float distance,
           const Placed &fixed, Impact &out) {
  // Casting a half-space is asking where an infinite flat thing first touches
  // something, which has no answer worth giving.
  if (moving.shape.kind == ShapeKind::plane) return false;
  if (fixed.shape.kind == ShapeKind::plane) {
    return sweepPlane(moving, direction, distance, fixed, out);
  }
  if (!(distance > 0.0f)) distance = 0.0f;

  // Already overlapping is asked of the narrowphase rather than inferred from
  // a bound. It is the one placement where an exact answer is already to
  // hand, the bound below is a lower bound and so cannot prove an overlap
  // anyway, and "how far until it touches" has no answer for something that
  // is touching.
  Manifold inside;
  if (collide(moving.shape, moving.at, moving.rotation, fixed.shape, fixed.at,
              fixed.rotation, inside) &&
      inside.count > 0) {
    uint32_t deepest = 0;
    for (uint32_t i = 1; i < inside.count; ++i) {
      if (inside.points[i].depth > inside.points[deepest].depth) deepest = i;
    }
    out.started = true;
    out.distance = 0.0f;
    out.normal = inside.normal;
    out.at = inside.points[deepest].at;
    return true;
  }

  float when = 0.0f;
  for (uint32_t step = 0; step < kMostSteps; ++step) {
    const Placed now = movedTo(moving, moving.at + direction * when);
    const Vec3 v = between(now, fixed);
    if (lengthSquared(v) < kTiny) break; // Concentric, and therefore inside.

    const float gap = gapAlong(now, fixed, v);
    if (gap <= kTouching) {
      out.started = false;
      out.distance = when;
      // Out of the fixed shape towards the moving one, which is the opposite
      // of the direction the gap was measured in.
      out.normal = -v;
      out.at = support(now, v);
      return true;
    }

    // How fast the gap is closing along the direction it was measured in.
    // Nought or less and it never will: this is what makes a shape beside
    // something it is moving away from a miss rather than a hit at infinity.
    const float closing = dot(v, direction);
    if (closing <= kTiny) return false;

    when += gap / closing;
    if (when > distance) return false;
  }

  // Out of turns. Whatever `when` reached is still short of the impact, so
  // reporting a touch there is the conservative answer rather than a wrong
  // one: it stops the caller slightly early, never slightly late.
  const Placed now = movedTo(moving, moving.at + direction * when);
  const Vec3 v = between(now, fixed);
  out.started = false;
  out.distance = when;
  out.normal = lengthSquared(v) < kTiny ? -normalised(direction) : -v;
  out.at = support(now, v);
  return true;
}

} // namespace orblit
