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
    case ShapeKind::plane:
    case ShapeKind::heightField: return to;
  }
  return of.at;
}

/// A direction from `a` towards the thing `nearestOfB` describes worth
/// measuring the gap along, or `otherwise` when the cores are too close
/// together to give one. `nearestOfB` gives the nearest point of that thing's
/// core to a point.
///
/// Found by stepping between the two cores: nearest point on one to the
/// other, then back again. Each step can only shorten what is between them,
/// so it settles rather than wanders. Six is well past where it stops moving
/// for shapes of the size a game uses, and a seventh would not make the
/// answer below any more correct — only the bound slightly tighter.
template <typename Near>
Vec3 between(const Placed &a, const Near &nearestOfB, const Vec3 &otherwise) {
  // Started from a's centre, because the loop asks b first and so never reads
  // an onB it has not written.
  Vec3 onA = a.at;
  Vec3 onB;
  for (int i = 0; i < 6; ++i) {
    onB = nearestOfB(onA);
    onA = nearestInCore(a, onB);
  }

  // Closer than touching, the two points are the same point give or take the
  // last bits of a float, and the line between them points anywhere. That is
  // every ray at the moment it lands, since a ray is all core: a direction
  // read off it there would tilt the normal of a flat face by however the
  // rounding fell. The caller's last good direction is the better answer.
  const Vec3 apart = onB - onA;
  if (lengthSquared(apart) > kTouching * kTouching) return normalised(apart);
  return otherwise;
}

/// Centre to centre: the direction to ask along when nothing better is known
/// yet. Rarely right, never unsafe, and only ever the first guess.
Vec3 centreToCentre(const Placed &a, const Placed &b) {
  return normalised(b.at - a.at);
}

/// How far apart a shape and the thing `furthestOfB` describes are along
/// `v`, at least. `furthestOfB` gives that thing's furthest point along a
/// direction.
///
/// The true distance is never smaller than this, whatever `v` is, which is
/// what makes it safe to advance by. Negative means they overlap along `v`
/// and says nothing about whether they overlap at all.
template <typename Far>
float gapAlong(const Placed &a, const Far &furthestOfB, const Vec3 &v) {
  return dot(v, furthestOfB(-v)) - dot(v, support(a, v));
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

/// Conservative advancement of `moving` towards one convex thing, known only
/// by the nearest point of its core to a point and its furthest point along
/// a direction. `v` is the first direction to measure along.
template <typename Near, typename Far>
bool advance(const Placed &moving, const Vec3 &direction, float distance, Vec3 v,
             const Near &nearestOfB, const Far &furthestOfB, Impact &out) {
  float when = 0.0f;
  for (uint32_t step = 0; step < kMostSteps; ++step) {
    const Placed now = movedTo(moving, moving.at + direction * when);
    v = between(now, nearestOfB, v);
    if (lengthSquared(v) < kTiny) break; // Concentric, and therefore inside.

    // How fast the gap is closing along the direction it was measured in.
    // Nought or less and it never will: this is what makes a shape beside
    // something it is moving away from a miss rather than a hit at infinity.
    // Asked before whether they touch, because touching and not closing is a
    // miss too — a capsule standing against a wall and dropping onto the
    // floor slides down the wall's face without ever getting any closer to
    // it. The gap between two convex shapes as one of them moves in a line
    // only ever bends upwards, so if it is not shrinking now it never will.
    const float closing = dot(v, direction);
    if (closing <= kTiny) return false;

    const float gap = gapAlong(now, furthestOfB, v);
    if (gap <= kTouching) {
      out.started = false;
      out.distance = when;
      // Out of the fixed shape towards the moving one, which is the opposite
      // of the direction the gap was measured in.
      out.normal = -v;
      out.at = support(now, v);
      return true;
    }

    when += gap / closing;
    if (when > distance) return false;
  }

  // Out of turns. Whatever `when` reached is still short of the impact, so
  // reporting a touch there is the conservative answer rather than a wrong
  // one: it stops the caller slightly early, never slightly late.
  const Placed now = movedTo(moving, moving.at + direction * when);
  v = between(now, nearestOfB, v);
  out.started = false;
  out.distance = when;
  out.normal = lengthSquared(v) < kTiny ? -normalised(direction) : -v;
  out.at = support(now, v);
  return true;
}

/// A shape moving towards ground, which is not one convex thing but a pool of
/// triangles: each it could reach is advanced towards on its own, and the
/// earliest touch is kept among those the triangle would push along.
///
/// A touch a triangle would not push along is a seam — a ball sliding over
/// flat ground reaching the edge of the triangle ahead — and is no touch at
/// all, for the same reason the narrowphase gives it no contact.
bool sweepGround(const Placed &moving, const Vec3 &direction, float distance,
                 const Placed &fixed, Impact &out) {
  const HeightField *field = fixed.shape.field;
  if (field == nullptr || field->empty()) return false;

  // In the field's own frame, where its triangles are.
  const Quat back = conjugate(fixed.rotation);
  const Placed local{moving.shape, rotate(back, moving.at - fixed.at),
                     back * moving.rotation};
  const Vec3 way = rotate(back, direction);

  const Bounds begins = local.shape.boundsAt(local.at, local.rotation);
  const Bounds ends =
      local.shape.boundsAt(local.at + way * distance, local.rotation);
  const Bounds swept = Bounds{minPerAxis(begins.low, ends.low),
                              maxPerAxis(begins.high, ends.high)}
                           .grown(kTouching);

  int64_t c0, r0, c1, r1;
  if (!field->cellsUnder(swept, c0, r0, c1, r1)) return false;

  bool hit = false;
  Impact first;
  Facet f;
  for (int64_t r = r0; r <= r1; ++r) {
    for (int64_t c = c0; c <= c1; ++c) {
      float low, high;
      if (!field->cellHeights(c, r, low, high)) continue;
      if (swept.low.y > high) continue;
      for (int which = 0; which < 2; ++which) {
        if (!field->facet(c, r, which, f)) continue;
        // Only as far as the nearest touch so far: anything past it is no
        // answer, and a triangle it cannot reach in time is given up early.
        Impact impact;
        if (!advance(
                local, way, hit ? first.distance : distance, -f.normal,
                [&](const Vec3 &to) { return f.nearest(to); },
                [&](const Vec3 &along) { return f.furthest(along); }, impact)) {
          continue;
        }
        // A ray, or a box, that lands within a touch of a face has no
        // direction left to measure along, and the advance reports the last
        // one it had — which, near an edge, leans out over it, and the
        // triangle refuses it as its neighbour's. But a core that has come
        // down over the face has met the face.
        const Placed there = movedTo(local, local.at + way * impact.distance);
        Vec3 onCore = there.at;
        for (int i = 0; i < 6; ++i) {
          onCore = nearestInCore(there, f.nearest(onCore));
        }
        if (f.over(onCore, kTouching)) impact.normal = f.normal;
        if (!f.admits(impact.normal)) continue;
        // A ball skimming flat ground within a touch of it meets the edge of
        // the triangle ahead leaning back by a hair. The ground there is the
        // face it is skimming, and moving along a face is not meeting it.
        impact.normal = f.straightened(impact.normal);
        if (dot(way, impact.normal) > -kTiny) continue;
        if (hit && impact.distance >= first.distance) continue;
        first = impact;
        hit = true;
      }
    }
  }
  if (!hit) return false;

  out.started = false;
  out.distance = first.distance;
  out.normal = rotate(fixed.rotation, first.normal);
  out.at = fixed.at + rotate(fixed.rotation, first.at);
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
    case ShapeKind::plane:
    case ShapeKind::heightField: return of.at;
  }
  return of.at;
}

bool sweep(const Placed &moving, const Vec3 &direction, float distance,
           const Placed &fixed, Impact &out) {
  // Casting a half-space is asking where an infinite flat thing first touches
  // something, which has no answer worth giving, and ground is no better.
  if (moving.shape.kind == ShapeKind::plane) return false;
  if (moving.shape.kind == ShapeKind::heightField) return false;
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

  if (fixed.shape.kind == ShapeKind::heightField) {
    return sweepGround(moving, direction, distance, fixed, out);
  }
  return advance(
      moving, direction, distance, centreToCentre(moving, fixed),
      [&](const Vec3 &to) { return nearestInCore(fixed, to); },
      [&](const Vec3 &along) { return support(fixed, along); }, out);
}

uint32_t nearest(const Bodies &bodies, const Placed &moving, const Vec3 &direction,
                 float distance, const Sieve &sieve, Impact &out) {
  if (moving.shape.kind == ShapeKind::plane) return Bodies::kNone;
  if (moving.shape.kind == ShapeKind::heightField) return Bodies::kNone;
  if (!(distance > 0.0f)) distance = 0.0f;

  // Where the cast could possibly reach, as one box: the shape at each end of
  // its travel and everything between. Grown by a hair so a body it meets
  // exactly edge on is not filtered out before it is looked at properly.
  const Bounds begins = moving.shape.boundsAt(moving.at, moving.rotation);
  const Bounds ends =
      moving.shape.boundsAt(moving.at + direction * distance, moving.rotation);
  const Bounds swept = Bounds{minPerAxis(begins.low, ends.low),
                              maxPerAxis(begins.high, ends.high)}
                           .grown(1.0e-3f);

  uint32_t hit = Bodies::kNone;
  const uint32_t count = bodies.count();
  for (uint32_t row = 0; row < count; ++row) {
    const OrblitPhysicsId id = bodies.id(row);
    if (id == sieve.ignore || (sieve.alsoIgnore != 0 && id == sieve.alsoIgnore)) {
      continue;
    }
    if (!interact(sieve.layerIs, sieve.layerCares, bodies.layerIs(row),
                  bodies.layerCares(row))) {
      continue;
    }
    // A half-space has no bounds to test against — it is half the world — so
    // it is always asked properly, the way the broadphase treats it.
    const bool half = bodies.shape(row).kind == ShapeKind::plane;
    if (!half && !swept.overlaps(bodies.bounds(row))) continue;

    const Placed fixed{bodies.shape(row), bodies.at(row), bodies.rotation(row)};
    Impact impact;
    if (!sweep(moving, direction, distance, fixed, impact)) continue;
    if (sieve.leaving && impact.started && dot(direction, impact.normal) >= 0.0f) {
      continue;
    }
    if (hit != Bodies::kNone && impact.distance >= out.distance) continue;
    out = impact;
    hit = row;
  }
  return hit;
}

} // namespace orblit
