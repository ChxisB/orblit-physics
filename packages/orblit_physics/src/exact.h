// A double precision vector, for the few places where a float is not enough.
//
// Cooking a hull and finding how deep two shapes overlap are both a chain of
// small differences of large numbers. A float loses the answer in the middle of
// the chain, and the failure is a face that comes out inside out or a normal
// that points sideways. Neither runs often enough for the wider type to cost
// anything that matters, and both hand back floats at the end.

#ifndef ORBLIT_PHYSICS_EXACT_H
#define ORBLIT_PHYSICS_EXACT_H

#include <cmath>

#include "maths.h"

namespace orblit {

struct D3 {
  double x = 0.0, y = 0.0, z = 0.0;

  D3() = default;
  D3(double x, double y, double z) : x(x), y(y), z(z) {}
  explicit D3(const Vec3 &v) : x(v.x), y(v.y), z(v.z) {}

  double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  D3 operator-() const { return {-x, -y, -z}; }
};

inline D3 operator+(const D3 &a, const D3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline D3 operator-(const D3 &a, const D3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline D3 operator*(const D3 &a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(const D3 &a, const D3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline D3 cross(const D3 &a, const D3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const D3 &a) { return std::sqrt(dot(a, a)); }

inline Vec3 toVec(const D3 &a) {
  return {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)};
}

} // namespace orblit

#endif // ORBLIT_PHYSICS_EXACT_H
