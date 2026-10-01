// Where two shapes touch, and how deeply.
//
// The narrowphase answers one question per pair and answers it completely: a
// direction to push along, and up to four points along it. Four is enough to
// hold a box flat on the ground — a face resting on a face needs its corners
// held, and three would let it rock about the line between them — and more
// than four buys nothing a solver can use, because the fifth point is always
// inside the hull of the others.
//
// Every normal here points out of `b` towards `a`. Not "from the first shape"
// or "from the lighter one": one rule, stated once, so no caller has to work
// out which way round it got its arguments.
//
// Each point carries its own normal. Between two convex shapes they are all
// the same, and the manifold's copy is enough. Against ground they are not:
// a box resting across a crease touches one slope with two corners and the
// other slope with the other two, and pushing all four out along either
// slope's normal would shove it sideways off the crease it is sitting in.
//
// Impulses live on the contact rather than beside it, because the solver warm
// starts: the push that held this pair apart last tick is very nearly the push
// that holds it apart this tick, and starting from it turns a stack that sinks
// and recovers into a stack that just stands there.

#ifndef ORBLIT_PHYSICS_COLLIDE_H
#define ORBLIT_PHYSICS_COLLIDE_H

#include <cstddef>
#include <cstdint>

#include "maths.h"
#include "rule.h"
#include "shape.h"

namespace orblit {

constexpr uint32_t kMaxContacts = 4;

struct Contact {
  /// Midway between the two surfaces, in world space.
  Vec3 at;

  /// Out of `b` towards `a`, along which this point is pushed apart.
  Vec3 normal;

  /// How far they overlap along the normal. Always positive here.
  float depth = 0.0f;

  /// Carried over from the last step, if the same point was there.
  float normalImpulse = 0.0f;
  float frictionImpulse[2] = {0.0f, 0.0f};
};

struct Manifold {
  uint32_t a = 0;
  uint32_t b = 0;

  /// The deepest point's normal: the one direction to report, or to step
  /// out along, when a caller wants the pair rather than its points.
  Vec3 normal;

  Contact points[kMaxContacts];
  uint32_t count = 0;

  /// What the game has said about this pair, if anything, with the two move
  /// scales in the order of `a` and `b`. The world fills it in between the
  /// narrowphase and the solver, which read it and change nothing.
  Rule rule;

  /// Whether `a` holds the larger id of the pair, so the normal runs against
  /// the order the pair is keyed in. The world sets it where it keys the pair.
  /// It is kept because removing a body moves another into its row, and a
  /// list of contacts taken then still has to know which way each one faces.
  bool reversed = false;
};

/// The point of a touching manifold that overlaps most. `count` must not be
/// zero.
inline const Contact &deepest(const Manifold &manifold) {
  const Contact *best = &manifold.points[0];
  for (uint32_t i = 1; i < manifold.count; ++i) {
    if (manifold.points[i].depth > best->depth) best = &manifold.points[i];
  }
  return *best;
}

/// Fills `out.normal` and `out.points` for two placed shapes, and returns
/// whether they touch at all. `out.a` and `out.b` are left alone.
///
/// Two height fields, or a height field and a plane, never touch: ground
/// against ground is nothing a solver has anything to move.
bool collide(const Shape &a, const Vec3 &atA, const Quat &rotA, const Shape &b,
             const Vec3 &atB, const Quat &rotB, Manifold &out);

/// A shape against ground, the ground being `b`. Defined beside the height
/// field, because nearly all of it is about which of the field's triangles
/// may push, and which way.
bool collideGround(const Shape &a, const Vec3 &atA, const Quat &rotA,
                   const Shape &ground, const Vec3 &atB, const Quat &rotB,
                   Manifold &out);

/// A face being clipped. Eight is the most a quad can have after four cuts.
struct Poly {
  Vec3 v[8];
  uint32_t n = 0;
};

/// One face of a box, wound so consecutive corners share an edge.
Poly faceOf(const Vec3 &centre, const Vec3 axis[3], const Vec3 &half, int i,
            float sign);

/// The part of `in` on the near side of a plane, cut where it crosses.
Poly clipTo(const Poly &in, const Vec3 &n, float surface);

/// The point of a segment nearest `to`, the segment given as a centre and a
/// half-length vector — which is how every capsule in here carries its axis.
Vec3 closestOnSegment(const Vec3 &centre, const Vec3 &half, const Vec3 &to);

/// The nearest pair of points on two segments, each given as a start and a
/// full direction.
void closestOnSegments(const Vec3 &p1, const Vec3 &d1, const Vec3 &p2,
                       const Vec3 &d2, Vec3 &c1, Vec3 &c2);

/// Adds a point, or takes the place of the shallowest one already there.
///
/// Four is the budget, and which four matters: the deepest are the ones the
/// solver has the most work to do on, and dropping one of them for a point
/// that is barely touching is how a corner sinks through a floor.
void keepDeepest(Manifold &out, const Vec3 &normal, const Vec3 &at, float depth);

/// A contact found on the way to a manifold. A shape over ground, or lying on
/// a face, finds more of them than a manifold holds.
struct Candidate {
  Vec3 normal;
  Vec3 at;
  float depth;
};

/// Fills `out` with the deepest candidate and the three that spread furthest
/// from it. A point that adds no spread is not added, which is also how the
/// same point found twice is kept once.
///
/// Which four is a question about all of them, not about the order they
/// arrive in: the deepest first, then the ones that hold the shape at its
/// corners rather than at four points bunched in one place.
void reduce(const Candidate *from, size_t count, Manifold &out);

/// A plane body as a world normal and the value `dot(normal, x)` takes on its
/// surface. Everything below the plane has a smaller value than that.
void planeInWorld(const Shape &s, const Vec3 &at, const Quat &rot, Vec3 &normal,
                  float &surface);

} // namespace orblit

#endif // ORBLIT_PHYSICS_COLLIDE_H
