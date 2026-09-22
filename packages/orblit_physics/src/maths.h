// The vector maths the solver runs on.
//
// Written here rather than taken from a maths library on purpose. This is the
// only part of the engine that every other part includes, it has to compile
// for every platform the core does, and the whole of it is two hundred lines —
// a dependency for that is a dependency that outweighs what it carries.
//
// Single precision throughout. A world large enough for float to be the
// problem has a bigger problem than its solver, and the answer to it is a
// shifting origin rather than a wider float.

#ifndef ORBLIT_PHYSICS_MATHS_H
#define ORBLIT_PHYSICS_MATHS_H

#include <cmath>

namespace orblit {

constexpr float kPi = 3.14159265358979323846f;

/// Below this a length is treated as zero rather than divided by.
///
/// Squared lengths are compared against its square, so a direction that is
/// really rounding error never becomes a unit vector pointing somewhere
/// arbitrary — which is how a contact normal ends up sideways.
constexpr float kTiny = 1.0e-9f;

struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;

  constexpr Vec3() = default;
  constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

  constexpr Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator-() const { return {-x, -y, -z}; }
  constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

  Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vec3 &operator-=(const Vec3 &o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  Vec3 &operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

  /// Component by index, so a loop over the three axes does not become three
  /// copies of the same line.
  float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  float &operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
};

constexpr Vec3 operator*(float s, const Vec3 &v) { return v * s; }

constexpr float dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

constexpr float lengthSquared(const Vec3 &v) { return dot(v, v); }

constexpr float clamped(float v, float low, float high) {
  return v < low ? low : (v > high ? high : v);
}

inline float length(const Vec3 &v) { return std::sqrt(dot(v, v)); }

/// Zero rather than a nan when there is no direction to speak of.
inline Vec3 normalised(const Vec3 &v) {
  const float lsq = lengthSquared(v);
  if (lsq < kTiny * kTiny) return {};
  return v * (1.0f / std::sqrt(lsq));
}

inline Vec3 minPerAxis(const Vec3 &a, const Vec3 &b) {
  return {std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z)};
}

inline Vec3 maxPerAxis(const Vec3 &a, const Vec3 &b) {
  return {std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z)};
}

inline Vec3 absPerAxis(const Vec3 &v) {
  return {std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)};
}

/// Two unit vectors at right angles to `n` and to each other.
///
/// Friction acts across the contact normal and needs some pair of directions
/// spanning that plane; which pair does not matter, only that it is stable and
/// never degenerate. Building it from whichever axis `n` leans on least is
/// what keeps it out of the degenerate case.
inline void perpendiculars(const Vec3 &n, Vec3 &u, Vec3 &v) {
  const Vec3 a = absPerAxis(n);
  const Vec3 axis = (a.x <= a.y && a.x <= a.z)   ? Vec3{1.0f, 0.0f, 0.0f}
                    : (a.y <= a.z)               ? Vec3{0.0f, 1.0f, 0.0f}
                                                 : Vec3{0.0f, 0.0f, 1.0f};
  u = normalised(cross(n, axis));
  v = cross(n, u);
}

// ------------------------------------------------------------------------- //

/// A rotation, xyzw, kept unit length.
struct Quat {
  float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

  constexpr Quat() = default;
  constexpr Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

  static Quat axisAngle(const Vec3 &axis, float radians) {
    const Vec3 n = normalised(axis);
    const float h = radians * 0.5f;
    const float s = std::sin(h);
    return {n.x * s, n.y * s, n.z * s, std::cos(h)};
  }
};

/// `a` then `b`, read left to right the way the transforms are applied.
constexpr Quat operator*(const Quat &a, const Quat &b) {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

/// Identity rather than a nan when it has degenerated to nothing.
inline Quat normalised(const Quat &q) {
  const float lsq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (lsq < kTiny) return {};
  const float s = 1.0f / std::sqrt(lsq);
  return {q.x * s, q.y * s, q.z * s, q.w * s};
}

constexpr Quat conjugate(const Quat &q) { return {-q.x, -q.y, -q.z, q.w}; }

/// `v` turned by `q`.
inline Vec3 rotate(const Quat &q, const Vec3 &v) {
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t = cross(u, v) * 2.0f;
  return v + t * q.w + cross(u, t);
}

/// `v` turned by the inverse of `q` — world into the body's own frame.
inline Vec3 unrotate(const Quat &q, const Vec3 &v) {
  return rotate(conjugate(q), v);
}

/// `q` advanced by spinning at `spin` for `delta` seconds.
///
/// The first-order step, renormalised. Exact integration of a rotation whose
/// axis is itself moving has no closed form worth the cost here, and the error
/// this leaves is smaller than the error in the velocity it was handed.
inline Quat integrate(const Quat &q, const Vec3 &spin, float delta) {
  const Quat d{spin.x * delta * 0.5f, spin.y * delta * 0.5f,
               spin.z * delta * 0.5f, 0.0f};
  const Quat stepped = d * q;
  return normalised(
      Quat{q.x + stepped.x, q.y + stepped.y, q.z + stepped.z, q.w + stepped.w});
}

// ------------------------------------------------------------------------- //

/// A 3x3 matrix, rows first. Used for one thing: an inertia tensor in world
/// space, which is a rotation away from the diagonal one the shape has.
struct Mat3 {
  Vec3 row[3];

  constexpr Mat3() = default;
  constexpr Mat3(const Vec3 &a, const Vec3 &b, const Vec3 &c) : row{a, b, c} {}

  static constexpr Mat3 diagonal(const Vec3 &d) {
    return {{d.x, 0.0f, 0.0f}, {0.0f, d.y, 0.0f}, {0.0f, 0.0f, d.z}};
  }

  static constexpr Mat3 zero() { return {{}, {}, {}}; }

  constexpr Vec3 operator*(const Vec3 &v) const {
    return {dot(row[0], v), dot(row[1], v), dot(row[2], v)};
  }
};

/// The inverse inertia of a body turned by `q`, from the diagonal `inverse`
/// its shape has in its own frame.
///
/// R * I * transpose(R), written out. A body's resistance to being spun
/// depends on which way round it is, and a long box tipped on its end is the
/// case where assuming otherwise is visible.
inline Mat3 rotatedInverseInertia(const Quat &q, const Vec3 &inverse) {
  const Vec3 c0 = rotate(q, {1.0f, 0.0f, 0.0f});
  const Vec3 c1 = rotate(q, {0.0f, 1.0f, 0.0f});
  const Vec3 c2 = rotate(q, {0.0f, 0.0f, 1.0f});
  // Columns of R scaled by the diagonal, then times transpose(R).
  const Vec3 s0 = c0 * inverse.x;
  const Vec3 s1 = c1 * inverse.y;
  const Vec3 s2 = c2 * inverse.z;
  Mat3 out;
  for (int i = 0; i < 3; ++i) {
    out.row[i] = {s0[i] * c0.x + s1[i] * c1.x + s2[i] * c2.x,
                  s0[i] * c0.y + s1[i] * c1.y + s2[i] * c2.y,
                  s0[i] * c0.z + s1[i] * c1.z + s2[i] * c2.z};
  }
  return out;
}

// ------------------------------------------------------------------------- //

/// An axis-aligned box, for the broadphase and for nothing else.
struct Bounds {
  Vec3 low;
  Vec3 high;

  bool overlaps(const Bounds &o) const {
    return low.x <= o.high.x && high.x >= o.low.x && low.y <= o.high.y &&
           high.y >= o.low.y && low.z <= o.high.z && high.z >= o.low.z;
  }

  /// Grown by `m` on every side. Used to keep a pair in the broadphase for a
  /// frame after it stops overlapping, so a contact that is about to come back
  /// is not rediscovered from nothing.
  Bounds grown(float m) const {
    return {low - Vec3{m, m, m}, high + Vec3{m, m, m}};
  }
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_MATHS_H
