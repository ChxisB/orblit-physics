// What a body's controls make of its mass: where its weight is, how hard it is
// to push along each axis and how hard it is to spin.
//
// Kept apart from the body store because it is pure maths on a shape, a mass
// and a handful of numbers. The store decides when to ask and where to keep the
// answer.

#ifndef ORBLIT_PHYSICS_MASS_H
#define ORBLIT_PHYSICS_MASS_H

#include <cstdint>

#include "maths.h"
#include "orblit_physics.h"
#include "shape.h"
#include "compound.h"

namespace orblit {

/// How a body moves, apart from what shape it is. `OrblitPhysicsControls`
/// without the id, and the value of each field that leaves a body as made.
struct Controls {
  uint32_t locks = 0;
  float gravityScale = 1.0f;
  float maxSpeed = 0.0f;
  float maxSpin = 0.0f;

  /// Where the weight is wanted, as a distance from the shape's own middle, in
  /// the body's frame. Only a free body has it.
  Vec3 centre;

  /// The inertia wanted about that point. Used only when every part is above
  /// zero.
  Vec3 inertia;

  bool locksMove(int axis) const {
    return (locks & (ORBLIT_PHYSICS_LOCK_MOVE_X << axis)) != 0;
  }

  bool locksTurn(int axis) const {
    return (locks & (ORBLIT_PHYSICS_LOCK_TURN_X << axis)) != 0;
  }

  bool locksAnyTurn() const {
    return (locks & (ORBLIT_PHYSICS_LOCK_TURN_X | ORBLIT_PHYSICS_LOCK_TURN_Y |
                     ORBLIT_PHYSICS_LOCK_TURN_Z)) != 0;
  }
};

/// Everything a motion, a mass and a set of controls decide about how a body
/// answers a push.
struct MassProperties {
  float inverseMass = 0.0f;

  /// Inverse mass on each world axis: zero where a move is locked. A push
  /// along a locked axis has nothing to move, and the solver reads that as an
  /// immovable wall in that direction.
  Vec3 gain;

  /// From the origin the body is placed by to the point it turns about. Zero
  /// for anything that is not free, so a fixed or driven body turns about the
  /// origin it is placed by.
  Vec3 offset;

  /// In the body's own frame, about `offset`. Turn locks are in the world's
  /// frame, so they wait until the body's rotation is known.
  Mat3 inverseInertia = Mat3::zero();
};

/// `inertia`, which is about a body's middle, turned into the inertia about a
/// point `centre` away from it.
///
/// That is the parallel axis theorem's share: turning about a point off to one
/// side means swinging the whole mass round it.
inline Mat3 aboutCentre(Mat3 inertia, float mass, const Vec3 &centre) {
  const float away = lengthSquared(centre);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const float across = centre[i] * centre[j];
      inertia.row[i][j] += mass * ((i == j ? away : 0.0f) - across);
    }
  }
  return inertia;
}

/// A hull's inertia is a whole matrix about its middle, not three numbers
/// about axes, so it takes its own way in.
inline Mat3 hullInertia(const Shape &shape, float mass) {
  return shape.cooked != nullptr ? shape.cooked->inertia() * mass
                                 : Mat3::zero();
}

/// The inertia, inverted, that the controls call for.
///
/// A given inertia is used as it is, since it is about the centre of mass
/// already. The shape's own is about the shape's middle, so a centre away from
/// it costs the parallel axis theorem's share.
inline Mat3 inverseInertiaOf(const Shape &shape, float mass,
                             const Controls &controls) {
  const Vec3 &given = controls.inertia;
  if (given.x > 0.0f && given.y > 0.0f && given.z > 0.0f) {
    return Mat3::diagonal({1.0f / given.x, 1.0f / given.y, 1.0f / given.z});
  }

  if (shape.kind == ShapeKind::compound || shape.affine) {
    return inverse(aboutCentre(inertiaOf(shape) * mass, mass, controls.centre));
  }
  if (shape.kind == ShapeKind::hull) {
    return inverse(aboutCentre(hullInertia(shape, mass), mass, controls.centre));
  }

  const Vec3 own = shape.inverseInertia(mass);
  if (lengthSquared(controls.centre) <= 0.0f) return Mat3::diagonal(own);

  // A zero inverse is an inertia the shape does not have, so it stays zero.
  const Mat3 about = Mat3::diagonal({own.x > 0.0f ? 1.0f / own.x : 0.0f,
                                     own.y > 0.0f ? 1.0f / own.y : 0.0f,
                                     own.z > 0.0f ? 1.0f / own.z : 0.0f});
  return inverse(aboutCentre(about, mass, controls.centre));
}

inline MassProperties massOf(const Shape &shape, bool free, float mass,
                             const Controls &controls) {
  MassProperties out;
  if (!free) return out;

  out.inverseMass = 1.0f / mass;
  for (int i = 0; i < 3; ++i) {
    out.gain[i] = controls.locksMove(i) ? 0.0f : out.inverseMass;
  }
  out.offset = shape.centre() + controls.centre;
  out.inverseInertia = inverseInertiaOf(shape, mass, controls);
  return out;
}

/// `inverseInertia`, in the world's frame, once the turns that are locked are
/// held still.
///
/// Holding an axis is a constraint, and a constraint takes the torque it needs
/// wherever it needs it. What is left is the inverse of the free axes' block of
/// the inertia itself, not the free axes' block of its inverse: for a body that
/// is not lined up with the axes, the two differ, and only the first one
/// leaves the locked axis exactly still.
inline Mat3 holdTurns(const Mat3 &inverseInertia, const Controls &controls) {
  if (!controls.locksAnyTurn()) return inverseInertia;

  int free[3];
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    if (!controls.locksTurn(i)) free[count++] = i;
  }

  const Mat3 inertia = inverse(inverseInertia);
  Mat3 out = Mat3::zero();
  if (count == 1) {
    const float about = inertia.row[free[0]][free[0]];
    if (about > 0.0f) out.row[free[0]][free[0]] = 1.0f / about;
  } else if (count == 2) {
    const int a = free[0];
    const int b = free[1];
    const float p = inertia.row[a][a];
    const float q = inertia.row[a][b];
    const float r = inertia.row[b][b];
    const float det = p * r - q * q;
    if (det > 1.0e-9f * p * r) {
      out.row[a][a] = r / det;
      out.row[a][b] = -q / det;
      out.row[b][a] = -q / det;
      out.row[b][b] = p / det;
    }
  }
  return out;
}

} // namespace orblit

#endif // ORBLIT_PHYSICS_MASS_H
