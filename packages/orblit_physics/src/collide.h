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

  /// How far they overlap along the normal. Always positive here.
  float depth = 0.0f;

  /// Carried over from the last step, if the same point was there.
  float normalImpulse = 0.0f;
  float frictionImpulse[2] = {0.0f, 0.0f};
};

struct Manifold {
  uint32_t a = 0;
  uint32_t b = 0;
  Vec3 normal;
  Contact points[kMaxContacts];
  uint32_t count = 0;
};

/// Fills `out.normal` and `out.points` for two placed shapes, and returns
/// whether they touch at all. `out.a` and `out.b` are left alone.
bool collide(const Shape &a, const Vec3 &atA, const Quat &rotA, const Shape &b,
             const Vec3 &atB, const Quat &rotB, Manifold &out);

} // namespace orblit

#endif // ORBLIT_PHYSICS_COLLIDE_H
