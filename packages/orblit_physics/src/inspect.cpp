// What a world can say about itself: a copy of its state to go back to, the
// contacts of its last step, and counts of what it holds.
//
// None of it is on the path of a step. A step neither builds nor keeps anything
// here, so a game that never asks pays for none of it.

#include <algorithm>
#include <utility>
#include <vector>

#include "world.h"

namespace orblit {
namespace {

using Touch = std::pair<const PairKey, Manifold>;

OrblitPhysicsContact contactOf(const PairKey &key, const Contact &point,
                               float turn) {
  OrblitPhysicsContact out{};
  out.a = key.a;
  out.b = key.b;
  out.at[0] = point.at.x;
  out.at[1] = point.at.y;
  out.at[2] = point.at.z;
  out.normal[0] = point.normal.x * turn;
  out.normal[1] = point.normal.y * turn;
  out.normal[2] = point.normal.z * turn;
  out.depth = point.depth;
  out.impulse = point.normalImpulse;
  return out;
}

} // namespace

Snapshot World::snapshot() const {
  Snapshot copy;
  copy.settings = settings_;
  copy.gravity = gravity_;
  copy.bodies = bodies_;
  copy.joints = joints_;
  copy.fields = fields_;
  copy.hulls = hulls_;
  // A step ends by swapping this step's contacts into the "was" sets, so what
  // the next step reads, and what the last one found, are the same thing.
  copy.touching = wasTouching_;
  copy.sensing = wasSensing_;
  copy.zones = zones_;
  copy.rules = rules_;
  copy.characters = characters_;
  copy.events = events_;
  copy.pairs = pairs_;
  return copy;
}

void World::restore(const Snapshot &from) {
  settings_ = from.settings;
  solving_ = solvingFor(settings_);
  gravity_ = from.gravity;
  bodies_ = from.bodies;
  joints_ = from.joints;
  fields_ = from.fields;
  hulls_ = from.hulls;
  wasTouching_ = from.touching;
  wasSensing_ = from.sensing;
  zones_ = from.zones;
  rules_ = from.rules;
  characters_ = from.characters;
  events_ = from.events;
  pairs_ = from.pairs;
}

uint32_t World::contacts(OrblitPhysicsContact *out, uint32_t capacity) const {
  std::vector<const Touch *> touches;
  touches.reserve(wasTouching_.size());
  for (const Touch &touch : wasTouching_) touches.push_back(&touch);
  // A hash table lists its pairs in whatever order its buckets hold, which is
  // not the same for two worlds that hold the same pairs.
  if (capacity > 0) {
    std::sort(touches.begin(), touches.end(),
              [](const Touch *l, const Touch *r) { return l->first < r->first; });
  }

  uint32_t total = 0;
  for (const Touch *touch : touches) {
    const PairKey &key = touch->first;
    const Manifold &manifold = touch->second;

    // The rows a manifold names are not read here: removing a body moves
    // another into its row, and that other body's pairs are still good. The
    // pair is listed while both its ids are in the world.
    if (bodies_.rowOf(key.a) == Bodies::kNone ||
        bodies_.rowOf(key.b) == Bodies::kNone) {
      continue;
    }

    // A point's normal is out of the manifold's second body, which is not
    // always the key's.
    const float turn = manifold.reversed ? -1.0f : 1.0f;
    for (uint32_t c = 0; c < manifold.count; ++c, ++total) {
      if (total < capacity) out[total] = contactOf(key, manifold.points[c], turn);
    }
  }
  return total;
}

void World::stats(OrblitPhysicsStats &out) const {
  out = OrblitPhysicsStats{};
  out.bodies = bodies_.count();
  for (uint32_t row = 0; row < out.bodies; ++row) {
    switch (bodies_.motion(row)) {
      case Motion::fixed: ++out.staticBodies; break;
      // A character is a driven body the character pass moves, so it counts
      // with the driven ones and the three counts still add up to `bodies`.
      case Motion::driven:
      case Motion::character: ++out.kinematicBodies; break;
      case Motion::free: ++out.dynamicBodies; break;
    }
    if (bodies_.asleep(row)) ++out.asleep;
    if (bodies_.sensor(row)) ++out.triggers;
  }

  out.characters = static_cast<uint32_t>(characters_.size());
  out.joints = static_cast<uint32_t>(joints_.all().size());
  out.zones = static_cast<uint32_t>(zones_.size());
  out.rules = static_cast<uint32_t>(rules_.size());

  out.pairs = pairs_;
  out.touching = static_cast<uint32_t>(wasTouching_.size());
  for (const Touch &touch : wasTouching_) out.points += touch.second.count;
}

} // namespace orblit
