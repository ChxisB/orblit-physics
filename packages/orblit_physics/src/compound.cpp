#include "compound.h"

#include <utility>

#include "cast.h"
#include "convex.h"
#include "mass.h"

namespace orblit {
namespace {

float trace(const Mat3 &of) { return of.row[0].x + of.row[1].y + of.row[2].z; }

Mat3 scaledInertia(const Mat3 &inertia, const Mat3 &map) {
  // Scaling acts on the second moment, whose diagonal is half the inertia's
  // trace minus each inertia diagonal. This also retains off-axis terms.
  Mat3 moment = inertia * -1.0f;
  for (int i = 0; i < 3; ++i) moment.row[i][i] += trace(inertia) * 0.5f;
  moment = map * moment * map.transposed();
  Mat3 out = moment * -1.0f;
  for (int i = 0; i < 3; ++i) out.row[i][i] += trace(moment);
  return out;
}

Placed placedPart(const Placed &body, const Part &part) {
  return {part.shape, body.at + rotate(body.rotation, part.at), body.rotation};
}

template <typename Visit>
void eachPart(const Placed &of, Visit visit) {
  if (of.shape.kind != ShapeKind::compound) {
    visit(of);
    return;
  }
  for (const Part &part : of.shape.parts->parts()) visit(placedPart(of, part));
}

} // namespace

Mat3 inertiaOf(const Shape &of) {
  if (of.kind == ShapeKind::compound) return of.parts->inertia();
  Mat3 own;
  if (of.kind == ShapeKind::hull) {
    own = of.cooked->inertia();
  } else {
    const Vec3 inverse = of.inverseInertia(1.0f);
    own = Mat3::diagonal({inverse.x > 0.0f ? 1.0f / inverse.x : 0.0f,
                         inverse.y > 0.0f ? 1.0f / inverse.y : 0.0f,
                         inverse.z > 0.0f ? 1.0f / inverse.z : 0.0f});
  }
  return of.affine ? scaledInertia(own, of.linear) : own;
}

float volumeOf(const Shape &of) {
  float volume = 0.0f;
  const float r = of.radius();
  switch (of.kind) {
    case ShapeKind::sphere: volume = 4.0f / 3.0f * kPi * r * r * r; break;
    case ShapeKind::box: volume = 8.0f * of.size.x * of.size.y * of.size.z; break;
    case ShapeKind::capsule:
      volume = kPi * r * r * (2.0f * of.halfHeight() + 4.0f / 3.0f * r);
      break;
    case ShapeKind::cylinder: volume = 2.0f * kPi * r * r * of.halfHeight(); break;
    case ShapeKind::hull: volume = of.cooked->volume(); break;
    case ShapeKind::plane:
    case ShapeKind::heightField:
    case ShapeKind::compound: break;
  }
  if (of.affine) volume *= dot(of.linear.row[0], cross(of.linear.row[1], of.linear.row[2]));
  return volume;
}

Compound::Compound(std::vector<Part> parts,
                   std::vector<std::shared_ptr<const Hull>> hulls)
    : parts_(std::move(parts)), hulls_(std::move(hulls)) {
  double volume = 0.0;
  for (const Part &part : parts_) volume += volumeOf(part.shape);
  for (const Part &part : parts_) {
    const float share = volumeOf(part.shape) / volume;
    centre_ += (part.at + part.shape.centre()) * share;
    reach_ = std::fmax(reach_, length(part.at) + part.shape.reach());
  }
  for (const Part &part : parts_) {
    const float share = volumeOf(part.shape) / volume;
    const Vec3 away = part.at + part.shape.centre() - centre_;
    const Mat3 contribution = aboutCentre(inertiaOf(part.shape) * share, share, away);
    for (int i = 0; i < 3; ++i) inertia_.row[i] += contribution.row[i];
  }
}

bool Compound::uses(const Hull *hull) const {
  for (const auto &held : hulls_) {
    if (held.get() == hull) return true;
  }
  return false;
}

Vec3 compoundCentre(const Compound *of) { return of != nullptr ? of->centre() : Vec3{}; }
float compoundReach(const Compound *of) { return of != nullptr ? of->reach() : 0.0f; }

Bounds compoundBounds(const Compound *of, const Vec3 &at, const Quat &rotation) {
  if (of == nullptr) return {at, at};
  Bounds out{{3.0e38f, 3.0e38f, 3.0e38f}, {-3.0e38f, -3.0e38f, -3.0e38f}};
  for (const Part &part : of->parts()) {
    const Bounds own = part.shape.boundsAt(at + rotate(rotation, part.at), rotation);
    out.low = minPerAxis(out.low, own.low);
    out.high = maxPerAxis(out.high, own.high);
  }
  return out;
}

Bounds affineBounds(const Shape &of, const Vec3 &at, const Quat &rotation) {
  const Convex convex(of, at, rotation);
  Bounds out;
  for (int i = 0; i < 3; ++i) {
    Vec3 axis;
    axis[i] = 1.0f;
    out.high[i] = toVec(convex.support(D3(axis)))[i] + convex.radius();
    out.low[i] = toVec(convex.support(D3(-axis)))[i] - convex.radius();
  }
  return out;
}

Vec3 supportParts(const Placed &of, const Vec3 &direction) {
  Vec3 best = of.at;
  float furthest = -3.0e38f;
  eachPart(of, [&](const Placed &part) {
    const Vec3 point = support(part, direction);
    const float reach = dot(point, direction);
    if (reach > furthest) {
      furthest = reach;
      best = point;
    }
  });
  return best;
}

bool collideParts(const Placed &a, const Placed &b, Manifold &out) {
  std::vector<Candidate> candidates;
  eachPart(a, [&](const Placed &pa) {
    eachPart(b, [&](const Placed &pb) {
      Manifold touch;
      if (!collide(pa.shape, pa.at, pa.rotation, pb.shape, pb.at, pb.rotation, touch)) return;
      for (uint32_t i = 0; i < touch.count; ++i) {
        const Contact &p = touch.points[i];
        candidates.push_back({p.normal, p.at, p.depth});
      }
    });
  });
  reduce(candidates.data(), candidates.size(), out);
  return out.count > 0;
}

bool sweepParts(const Placed &moving, const Vec3 &direction, float distance,
                const Placed &fixed, Impact &out) {
  bool found = false;
  eachPart(moving, [&](const Placed &a) {
    eachPart(fixed, [&](const Placed &b) {
      Impact hit;
      if (sweep(a, direction, distance, b, hit) && (!found || hit.distance < out.distance)) {
        out = hit;
        found = true;
      }
    });
  });
  return found;
}

} // namespace orblit
