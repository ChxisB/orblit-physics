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

#include <cstdint>

#include "maths.h"
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
};

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

} // namespace orblit

#endif // ORBLIT_PHYSICS_COLLIDE_H
