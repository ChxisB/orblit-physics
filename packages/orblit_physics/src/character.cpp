// How a character moves: carried, pushed out, walked, then set down.
//
// Every step does the same four things to each character, in this order:
//
//   1. Carry it with what it stood on. The ground has already moved this
//      step, so the character is put back where it stood on it, turned with
//      it, by a sweep that ignores the ground and nothing else. A platform
//      that passes under a low beam scrapes its passenger off.
//
//   2. Push it out of anything driven or fixed that it now overlaps. A door
//      that closed on it, a lift that rose into it. This is the only way a
//      character is ever pushed, and why a crate never is able to: the crate
//      is the solver's, and the solver treats the character as a wall.
//
//   3. Walk: sweep it along what it asked for, and slide along whatever it
//      meets, up to four times. When a step or a ledge stops it, try again
//      from a step higher and keep whichever try got further.
//
//   4. Look down. If there is a floor within reach, it is standing: set it
//      down on it at the skin, and take away any part of its velocity that
//      was heading into it.
//
// What counts as a floor is the question everything turns on, and a rounded
// shape makes it harder than it looks. A capsule meeting the corner of a step
// touches it on the curve of its base, and the normal there leans by however
// far round the curve the corner is, which says nothing about whether the
// thing it is touching is a floor. So a contact that leans too far to stand
// on is looked past: a short ray dropped just beyond it finds the top of
// whatever it touched, and if that top is walkable the corner is an edge of a
// floor rather than a slope. That is what lets it climb a stair without
// every riser being a wall, and stand on the lip of one without sliding off.

#include <algorithm>
#include <cmath>
#include <vector>

#include "cast.h"
#include "collide.h"
#include "world.h"

namespace orblit {
namespace {

/// Shorter than this is not a move worth sweeping, in metres.
constexpr float kNoMove = 1.0e-6f;

/// How many surfaces one move may slide along. Three walls meeting in a
/// corner is the most a move ever needs; the fourth is for the floor.
constexpr int kMostSlides = 4;

/// How many times it is pushed out of overlaps before it gives up. A character
/// still overlapping after this is being crushed, and pushing harder would
/// only move which wall it is inside.
constexpr int kMostPushOuts = 4;

/// How far past a leaning contact, and from how high above it, the ray that
/// looks for the top of a step starts, in metres. Beyond by enough to clear
/// the corner's own rounding, above by enough to start over the top of any
/// step it could be touching the edge of.
constexpr float kBeyond = 0.02f;
constexpr float kAbove = 0.05f;

/// A normal that rises less than this is a wall or a ceiling, not a slope. The
/// edge check is not worth making for a surface that faces sideways.
constexpr float kFlat = 1.0e-3f;

/// Slack on the steepest slope, so a floor that is exactly as steep as allowed
/// is a floor whichever way the last bit of the float went.
constexpr float kLevel = 1.0e-4f;

/// How much further a step up must get than the walk it replaces to be kept,
/// in metres. More than rounding, so a character against a wall is not lifted
/// and set down again every frame for the sake of nothing.
constexpr float kProgress = 1.0e-4f;

enum class Surface { floor, slope, wall };

/// What a contact is, to something that walks.
struct Underfoot {
  Surface surface = Surface::wall;

  /// The contact's own normal, which is what a move is slid along.
  Vec3 normal;

  /// The normal of the surface it would stand on, which is what is reported.
  /// Different from `normal` only for the edge of a step, where the contact
  /// is on the corner and the floor is the top.
  Vec3 top;

  /// A point on the surface it would stand on.
  Vec3 where;

  /// Touching a floor by its edge rather than lying on it.
  bool edge = false;
};

/// Where a slide ended, what was left of the velocity, and whether anything
/// other than a floor stopped it.
struct Slid {
  Vec3 at;
  Vec3 velocity;
  bool blocked = false;
};

/// `v` with the part along `up` taken out.
Vec3 flattened(const Vec3 &v, const Vec3 &up) { return v - up * dot(v, up); }

/// How far `q` turns about `up`, in radians, between minus and plus pi.
///
/// A turn is split into a twist about an axis and a swing of that axis, and
/// the twist is all this keeps: a turntable that also tilts turns a character
/// standing on it only by the part that is about up. The sign of w is
/// normalised first because q and -q are the same turn, and only one of them
/// gives the short way round.
float turnAbout(const Quat &q, const Vec3 &up) {
  const float along = q.x * up.x + q.y * up.y + q.z * up.z;
  return q.w < 0.0f ? 2.0f * std::atan2(-along, -q.w)
                    : 2.0f * std::atan2(along, q.w);
}

/// One character's view of the world for one step: what it can meet and how
/// it slides along it. Reads the bodies and never writes them, so whatever it
/// works out is thrown away unless the step decides to keep it.
class Walker {
 public:
  Walker(const Bodies &bodies, const Character &character, uint32_t row,
         const Vec3 &up, float delta)
      : bodies_(bodies),
        character_(character),
        up_(up),
        delta_(delta),
        shape_(bodies.shape(row)),
        rotation_(bodies.rotation(row)) {
    sieve_.layerIs = bodies.layerIs(row);
    sieve_.layerCares = bodies.layerCares(row);
    sieve_.ignore = character.id;
    sieve_.leaving = true;
  }

  float skin() const { return character_.skin; }

  /// The nearest thing it meets going `distance` along `direction` from
  /// `from`, as a row or Bodies::kNone.
  uint32_t meet(const Vec3 &from, const Vec3 &direction, float distance,
                OrblitPhysicsId alsoIgnore, Impact &hit) const {
    Sieve sieve = sieve_;
    sieve.alsoIgnore = alsoIgnore;
    return nearest(bodies_, Placed{shape_, from, rotation_}, direction, distance,
                   sieve, hit);
  }

  bool walkable(float rises) const {
    return rises + kLevel >= character_.slopeCos;
  }

  /// What `hit` is to a character centred at `at`.
  Underfoot underfoot(const Vec3 &at, const Impact &hit) const {
    Underfoot foot;
    foot.normal = hit.normal;
    foot.top = hit.normal;
    foot.where = hit.at;

    const float rises = dot(hit.normal, up_);
    if (walkable(rises)) {
      foot.surface = Surface::floor;
      return foot;
    }
    if (rises <= kFlat) {
      foot.surface = Surface::wall;
      return foot;
    }
    foot.surface = Surface::slope;

    // Leaning too far to stand on, but maybe only because it is a corner.
    // Look just past it, from just above it, for the top of what it touched.
    const Vec3 out = flattened(hit.at - at, up_);
    if (lengthSquared(out) < kTiny) return foot;
    const Vec3 from = hit.at + normalised(out) * kBeyond + up_ * kAbove;

    Sieve sieve = sieve_;
    sieve.leaving = false;
    Impact top;
    const Placed ray{Shape::sphere(0.0f), from, Quat{}};
    if (nearest(bodies_, ray, -up_, kAbove + kBeyond, sieve, top) ==
        Bodies::kNone) {
      return foot;
    }
    // Started inside something is a ray that began within a wall, which
    // means what it touched goes up further than a step's edge would.
    if (top.started || !walkable(dot(top.normal, up_))) return foot;

    foot.surface = Surface::floor;
    foot.top = top.normal;
    foot.where = top.at;
    foot.edge = true;
    return foot;
  }

  /// `r` with whatever part of it goes into `foot` taken away, by the rule
  /// for that kind of surface.
  Vec3 clip(const Vec3 &r, const Underfoot &foot) const {
    const Vec3 &n = foot.normal;
    const float into = dot(r, n);
    if (into >= 0.0f) return r;

    switch (foot.surface) {
      case Surface::floor: {
        if (foot.edge) return acrossEdge(r, n);
        // Straight up out of the floor, rather than along its normal, so the
        // part of the move across the ground is kept exactly. Walking up a
        // ramp at a speed is walking at that speed, not slower by the cosine
        // of its angle, and walking into one does not push you sideways.
        return r - up_ * (into / std::fmax(dot(n, up_), kFlat));
      }
      case Surface::slope: {
        // Too steep to stand on. Sliding down it is allowed; being carried
        // up it by walking into it is exactly what a slope limit is for, so
        // a slide that would climb is stopped as if at a wall.
        const Vec3 along = r - n * into;
        if (dot(along, up_) <= 0.0f) return along;
        return acrossWall(r, n);
      }
      case Surface::wall:
        return r - n * into;
    }
    return r;
  }

  /// Slides from `from` by `move`, meeting at most kMostSlides surfaces, and
  /// clips `velocity` against each one it meets. Pushes a free body it walks
  /// into, when `shoves` is somewhere to put the push.
  Slid slide(const Vec3 &from, const Vec3 &move, const Vec3 &velocity,
             OrblitPhysicsId alsoIgnore, std::vector<Shove> *shoves) const {
    Slid out{from, velocity, false};
    Vec3 left = move;
    Vec3 met[kMostSlides];
    int surfaces = 0;

    for (int turn = 0; turn < kMostSlides; ++turn) {
      const float distance = length(left);
      if (distance < kNoMove) break;
      const Vec3 direction = left * (1.0f / distance);

      Impact hit;
      const uint32_t row =
          meet(out.at, direction, distance + skin(), alsoIgnore, hit);
      if (row == Bodies::kNone) {
        out.at += left;
        return out;
      }

      // Stop short by the skin, measured along the normal rather than along
      // the move: coming at a wall at a shallow angle, the skin along the move
      // is a long way back.
      const float into = -dot(direction, hit.normal);
      const float travel =
          hit.started ? 0.0f
                      : clamped(hit.distance - skin() / std::fmax(into, 1.0e-3f),
                                0.0f, distance);
      out.at += direction * travel;
      left -= direction * travel;

      const Underfoot foot = underfoot(out.at, hit);
      if (foot.surface != Surface::floor || foot.edge) out.blocked = true;
      if (shoves != nullptr && foot.surface != Surface::floor) {
        shove(row, hit.normal, out.velocity, *shoves);
      }

      left = clip(left, foot);
      out.velocity = clip(out.velocity, foot);

      // Two surfaces at once, like a wall and the floor or a corner between
      // two walls: sliding along the second must not push back into the
      // first, so it goes along the line where they meet. If that line runs
      // into a third, it is wedged, and goes nowhere.
      for (int k = 0; k < surfaces; ++k) {
        if (dot(left, met[k]) >= -kTiny) continue;
        const Vec3 line = cross(met[k], foot.normal);
        if (lengthSquared(line) < kTiny) continue;
        const Vec3 crease = normalised(line);
        left = crease * dot(left, crease);
        out.velocity = crease * dot(out.velocity, crease);
        for (int m = 0; m < surfaces; ++m) {
          if (m != k && dot(left, met[m]) < -kTiny) {
            left = {};
            out.velocity = {};
          }
        }
        break;
      }
      met[surfaces++] = foot.normal;
    }
    return out;
  }

  /// Pushes it out of whatever fixed or driven body it overlaps, as far as
  /// the skin beyond touching.
  ///
  /// Not out of free bodies, which the solver pushes out of it, and not out
  /// of other characters, which would push back: two characters that have
  /// been placed inside each other walk apart instead.
  Vec3 pushedOut(Vec3 at, uint32_t self) const {
    Manifold manifold;
    for (int pass = 0; pass < kMostPushOuts; ++pass) {
      bool moved = false;
      const Bounds mine = shape_.boundsAt(at, rotation_).grown(skin());
      const uint32_t count = bodies_.count();
      for (uint32_t row = 0; row < count; ++row) {
        if (row == self) continue;
        const Motion motion = bodies_.motion(row);
        if (motion != Motion::fixed && motion != Motion::driven) continue;
        if (!interact(sieve_.layerIs, sieve_.layerCares, bodies_.layerIs(row),
                      bodies_.layerCares(row))) {
          continue;
        }
        const bool half = bodies_.shape(row).kind == ShapeKind::plane;
        if (!half && !mine.overlaps(bodies_.bounds(row))) continue;
        if (!collide(shape_, at, rotation_, bodies_.shape(row), bodies_.at(row),
                     bodies_.rotation(row), manifold) ||
            manifold.count == 0) {
          continue;
        }
        float deepest = 0.0f;
        for (uint32_t c = 0; c < manifold.count; ++c) {
          deepest = std::fmax(deepest, manifold.points[c].depth);
        }
        at += manifold.normal * (deepest + skin());
        moved = true;
      }
      if (!moved) break;
    }
    return at;
  }

  /// The walk again, from a step higher: up, across, and back down onto
  /// something. False if there is no room to go up, nothing to come down on,
  /// or what it comes down on is not a floor within a step of where it
  /// started.
  bool stepUp(const Vec3 &from, const Vec3 &move, const Vec3 &velocity,
              std::vector<Shove> *shoves, Slid &out) const {
    const Vec3 across = flattened(move, up_);
    if (length(across) < kNoMove) return false;

    float rise = character_.stepHeight;
    Impact above;
    if (meet(from, up_, rise + skin(), 0, above) != Bodies::kNone) {
      rise = above.started ? 0.0f : std::fmax(above.distance - skin(), 0.0f);
    }
    if (rise < kNoMove) return false;

    const Slid moved = slide(from + up_ * rise, across, velocity, 0, shoves);

    Impact below;
    if (meet(moved.at, -up_, rise + 2.0f * skin(), 0, below) == Bodies::kNone) {
      return false;
    }
    if (below.started) return false;
    const Underfoot foot = underfoot(moved.at, below);
    if (foot.surface != Surface::floor) return false;

    // The top it lands on must be within a step of its feet. Without this a
    // rounded base lifted a step high can come down on the corner of a wall
    // nearly a step and a radius tall, and call the corner a floor.
    const float sole = dot(support(Placed{shape_, from, rotation_}, -up_), up_);
    if (dot(foot.where, up_) - sole > character_.stepHeight + skin()) {
      return false;
    }

    out.at = moved.at - up_ * std::fmax(below.distance - skin(), 0.0f);
    out.velocity = clip(moved.velocity, foot);
    out.blocked = moved.blocked;
    return true;
  }

 private:
  /// Across the edge of a floor it is touching by the corner: nothing down,
  /// and nothing into the corner. The step up is what gets it over.
  Vec3 acrossEdge(const Vec3 &r, const Vec3 &n) const {
    Vec3 out = r;
    const float down = dot(out, up_);
    if (down < 0.0f) out -= up_ * down;
    return acrossWall(out, n);
  }

  /// `r` stopped against `n` as if `n` stood straight up.
  Vec3 acrossWall(const Vec3 &r, const Vec3 &n) const {
    const Vec3 flat = flattened(n, up_);
    if (lengthSquared(flat) < kTiny) return r;
    const Vec3 across = normalised(flat);
    const float into = dot(r, across);
    return into < 0.0f ? r - across * into : r;
  }

  /// Pushes a free body it walked into, along the ground, as hard as it
  /// takes to make it keep pace and no harder than the character may push.
  ///
  /// Through the body's centre, so a crate slides rather than tips: where a
  /// capsule touches a box is a long way from anything a push would come
  /// from, and a crate that cartwheels every time it is walked into is not
  /// one anybody asked for.
  void shove(uint32_t row, const Vec3 &normal, const Vec3 &velocity,
             std::vector<Shove> &shoves) const {
    if (bodies_.motion(row) != Motion::free) return;
    if (character_.strength <= 0.0f || bodies_.inverseMass(row) <= 0.0f) return;

    const Vec3 flat = flattened(-normal, up_);
    if (lengthSquared(flat) < kTiny) return;
    const Vec3 along = normalised(flat);

    const float closing = dot(velocity - bodies_.velocity(row), along);
    if (closing <= 0.0f) return;
    const float impulse = std::fmin(closing / bodies_.inverseMass(row),
                                    character_.strength * delta_);
    shoves.push_back({row, along * impulse, bodies_.at(row)});
  }

  const Bodies &bodies_;
  const Character &character_;
  Vec3 up_;
  float delta_;
  Shape shape_;
  Quat rotation_;
  Sieve sieve_;
};

} // namespace

void World::moveCharacters(float delta) {
  if (characters_.empty()) return;

  // Up is against gravity, whatever gravity is. A world with none has no
  // floors to speak of, so it keeps the usual up rather than none.
  const float fall = length(gravity_);
  const Vec3 up = fall > kTiny ? gravity_ * (-1.0f / fall) : Vec3{0.0f, 1.0f, 0.0f};

  for (Character &character : characters_) {
    const uint32_t row = bodies_.rowOf(character.id);
    if (row == Bodies::kNone) continue;
    moveCharacter(character, row, up, delta);
  }
}

void World::moveCharacter(Character &character, uint32_t row, const Vec3 &up,
                          float delta) {
  const Walker walker(bodies_, character, row, up, delta);
  const Vec3 start = bodies_.at(row);
  Vec3 at = start;

  // 1. Carried by what it stood on, if that moves.
  character.carried = {};
  character.turning = 0.0f;
  if (character.grounded && character.ground != 0) {
    const uint32_t ground = bodies_.rowOf(character.ground);
    if (ground != Bodies::kNone && bodies_.motion(ground) != Motion::fixed) {
      const Quat &turned = bodies_.rotation(ground);
      const Vec3 target = bodies_.at(ground) + rotate(turned, character.local);
      const Vec3 ride = target - at;
      if (length(ride) >= kNoMove) {
        const Slid carried = walker.slide(at, ride, Vec3{}, character.ground, nullptr);
        character.carried = (carried.at - at) * (1.0f / delta);
        at = carried.at;
      }
      character.turning =
          turnAbout(turned * conjugate(character.groundRotation), up) / delta;
    }
  }

  // 2. Pushed out of anything that moved into it.
  const Vec3 before = at;
  at = walker.pushedOut(at, row);
  const Vec3 shoved = at - before;

  // 3. The walk, and the walk from a step higher if something stopped it.
  const Vec3 asked = character.wanted;
  const Vec3 move = asked * delta;
  shoves_.clear();
  Slid walked = walker.slide(at, move, asked, 0, &shoves_);
  if (walked.blocked && character.grounded && character.stepHeight > 0.0f) {
    trial_.clear();
    Slid stepped;
    if (walker.stepUp(at, move, asked, &trial_, stepped)) {
      const Vec3 ahead = normalised(flattened(move, up));
      if (dot(stepped.at - at, ahead) > dot(walked.at - at, ahead) + kProgress) {
        walked = stepped;
        shoves_.swap(trial_);
      }
    }
  }
  at = walked.at;
  Vec3 velocity = walked.velocity;

  // 4. What is underfoot. Further down if it was standing and is not jumping,
  //    so walking down a stair keeps it on the stair rather than launching it
  //    off each tread.
  const bool rising = dot(asked, up) > kTiny;
  const float reach =
      character.grounded && !rising ? character.stepHeight + 2.0f * walker.skin()
                                    : 2.0f * walker.skin();
  character.grounded = false;
  character.ground = 0;
  character.normal = {};

  Impact below;
  const uint32_t ground = walker.meet(at, -up, reach, 0, below);
  if (ground != Bodies::kNone) {
    const Underfoot foot = walker.underfoot(at, below);
    character.ground = bodies_.id(ground);
    character.normal = foot.top;
    if (foot.surface == Surface::floor && !rising) {
      character.grounded = true;
      // Set down at the skin: dropped onto a tread below, or lifted back to
      // the skin if it had settled closer than that.
      if (!below.started) at -= up * (below.distance - walker.skin());
      // Nothing up or down in what it asks for next, once it is standing.
      // Walking up a ramp is a slide the next step makes again, not a speed
      // upwards to keep: kept, the game would add gravity to it, find it
      // still rising, and launch the character off the top of every step.
      // Walking down one would build up a fall it never takes.
      velocity = flattened(walker.clip(velocity, foot), up);
      character.local = unrotate(bodies_.rotation(ground), at - bodies_.at(ground));
      character.groundRotation = bodies_.rotation(ground);
    }
  }

  bodies_.at(row) = at;
  // What it actually did, less being pushed out of things, which is a
  // correction rather than a motion. This is what the solver sees when a
  // crate is up against it, so it is the truth rather than the request.
  bodies_.velocity(row) = (at - start - shoved) * (1.0f / delta);
  bodies_.spin(row) = {};
  bodies_.refresh(row);
  character.wanted = velocity;

  for (const Shove &shove : shoves_) push(shove.row, shove.impulse, shove.at);
}

} // namespace orblit
