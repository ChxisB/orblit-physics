// Times the solver on scenes that cost something, through the C ABI.
//
// Built and run by tool/benchmark.sh. Every scene is seeded, so two runs on one
// machine build the same bodies and differ only in the clock. The step time is
// the one the world measures around its own step call, read back from the
// stats, so nothing here is timed twice.
//
// Each scene runs three times and the run with the lowest median is the one
// printed. The counts do not change between runs. The clock does, with whatever
// else the machine is doing, and the quickest run is the one least disturbed.
//
// Output is a markdown table, so a run can be pasted into the README.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <vector>

#include "orblit_physics.h"

namespace {

constexpr float kStep = 1.0f / 60.0f;
constexpr int kRepeats = 3;

OrblitPhysicsCommand made(OrblitPhysicsId id, uint32_t shape, float x, float y,
                          float z) {
  OrblitPhysicsCommand command{};
  command.kind = ORBLIT_PHYSICS_CREATE;
  command.shape = shape;
  command.id = id;
  command.at[0] = x;
  command.at[1] = y;
  command.at[2] = z;
  command.motion = ORBLIT_PHYSICS_DYNAMIC;
  command.mass = 1.0f;
  command.friction = 0.5f;
  command.damping[0] = 0.05f;
  command.damping[1] = 0.05f;
  command.layerIs = 1;
  command.layerCares = 0xFFFFFFFFu;
  return command;
}

OrblitPhysicsCommand sphere(OrblitPhysicsId id, float radius, float x, float y,
                            float z) {
  OrblitPhysicsCommand command = made(id, ORBLIT_PHYSICS_SPHERE, x, y, z);
  command.size[0] = radius;
  return command;
}

OrblitPhysicsCommand box(OrblitPhysicsId id, float half, float x, float y,
                         float z) {
  OrblitPhysicsCommand command = made(id, ORBLIT_PHYSICS_BOX, x, y, z);
  command.size[0] = half;
  command.size[1] = half;
  command.size[2] = half;
  return command;
}

OrblitPhysicsCommand ground(OrblitPhysicsId id) {
  OrblitPhysicsCommand command = made(id, ORBLIT_PHYSICS_PLANE, 0.0f, 0.0f, 0.0f);
  command.size[1] = 1.0f;
  command.motion = ORBLIT_PHYSICS_STATIC;
  return command;
}

void submit(OrblitPhysics *world, const OrblitPhysicsCommand &command) {
  orblit_physics_submit(world, &command, 1);
}

/// What a run of steps cost and what the world looked like while it did.
struct Run {
  std::vector<float> micros;
  uint32_t bodies = 0;
  uint32_t peakPairs = 0;
  uint32_t peakTouching = 0;
  uint32_t asleep = 0;
  uint32_t dynamic = 0;

  /// The first step after which every free body was asleep, or -1.
  int allAsleepAt = -1;
};

Run stepFor(OrblitPhysics *world, int steps) {
  Run run;
  run.micros.reserve(static_cast<size_t>(steps));
  OrblitPhysicsStats stats{};
  for (int i = 0; i < steps; ++i) {
    orblit_physics_step(world, kStep);
    orblit_physics_stats(world, &stats);
    run.micros.push_back(stats.stepMicroseconds);
    run.peakPairs = std::max(run.peakPairs, stats.pairs);
    run.peakTouching = std::max(run.peakTouching, stats.touching);
    const bool settled = stats.dynamicBodies > 0 && stats.asleep == stats.dynamicBodies;
    if (settled && run.allAsleepAt < 0) run.allAsleepAt = i + 1;
  }
  run.bodies = stats.bodies;
  run.asleep = stats.asleep;
  run.dynamic = stats.dynamicBodies;
  return run;
}

float percentile(std::vector<float> values, float fraction) {
  if (values.empty()) return 0.0f;
  std::sort(values.begin(), values.end());
  const size_t last = values.size() - 1;
  return values[static_cast<size_t>(std::lround(fraction * static_cast<float>(last)))];
}

float mean(const std::vector<float> &values) {
  if (values.empty()) return 0.0f;
  double total = 0.0;
  for (const float value : values) total += value;
  return static_cast<float>(total / static_cast<double>(values.size()));
}

/// The run of `scene` with the lowest median step.
Run quickest(const std::function<Run()> &scene) {
  Run best = scene();
  for (int i = 1; i < kRepeats; ++i) {
    Run next = scene();
    if (percentile(next.micros, 0.5f) < percentile(best.micros, 0.5f)) {
      best = std::move(next);
    }
  }
  return best;
}

void header() {
  std::printf("| Scene | Bodies | Mean | Median | 95th | Worst | Pairs | Touching | Asleep |\n");
  std::printf("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |\n");
}

void row(const char *scene, const Run &run) {
  char asleep[48];
  if (run.allAsleepAt >= 0) {
    std::snprintf(asleep, sizeof asleep, "all, by step %d", run.allAsleepAt);
  } else {
    std::snprintf(asleep, sizeof asleep, "%u of %u", run.asleep, run.dynamic);
  }
  std::printf("| %s | %u | %.0f | %.0f | %.0f | %.0f | %u | %u | %s |\n", scene,
              run.bodies, mean(run.micros), percentile(run.micros, 0.5f),
              percentile(run.micros, 0.95f), percentile(run.micros, 1.0f),
              run.peakPairs, run.peakTouching, asleep);
}

/// Marks body `id` to be stopped at the first fixed body it would pass.
void makeContinuous(OrblitPhysics *world, OrblitPhysicsId id) {
  OrblitPhysicsControls controls{};
  controls.body = id;
  controls.gravityScale = 1.0f;
  controls.continuous = true;
  controls.quiet = true;
  orblit_physics_controls(world, &controls);
}

/// Spheres spread thin and dropped, so there are many bodies and few contacts.
/// What this costs is finding that nothing touches. With `continuous` every
/// sphere is also cast along its move each step.
Run scatter(bool continuous) {
  OrblitPhysics *world = orblit_physics_create(nullptr);
  submit(world, ground(1));
  std::mt19937 random(7);
  std::uniform_real_distribution<float> across(-40.0f, 40.0f);
  std::uniform_real_distribution<float> up(1.0f, 20.0f);
  for (OrblitPhysicsId id = 2; id < 2002; ++id) {
    submit(world, sphere(id, 0.25f, across(random), up(random), across(random)));
    if (continuous) makeContinuous(world, id);
  }
  const Run run = stepFor(world, 300);
  orblit_physics_destroy(world);
  return run;
}

/// Boxes in a block, dropped on each other. What this costs is contacts.
Run heap() {
  OrblitPhysics *world = orblit_physics_create(nullptr);
  submit(world, ground(1));
  std::mt19937 random(11);
  std::uniform_real_distribution<float> jitter(-0.05f, 0.05f);
  OrblitPhysicsId id = 2;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      for (int z = 0; z < 10; ++z) {
        submit(world, box(id++, 0.25f, 0.6f * static_cast<float>(x) + jitter(random),
                          0.5f + 0.6f * static_cast<float>(y),
                          0.6f * static_cast<float>(z) + jitter(random)));
      }
    }
  }
  const Run run = stepFor(world, 600);
  orblit_physics_destroy(world);
  return run;
}

/// A wall of boxes set down exactly. It is one island by any reckoning, so it
/// is the scene that shows whether a stack sleeps as one. `settle` steps run
/// before the measured `steps`, which is how the cost of a level at rest is
/// told apart from the cost of getting there.
Run wall(int across, int high, int settle, int steps) {
  OrblitPhysics *world = orblit_physics_create(nullptr);
  submit(world, ground(1));
  OrblitPhysicsId id = 2;
  for (int level = 0; level < high; ++level) {
    for (int column = 0; column < across; ++column) {
      submit(world, box(id++, 0.5f, static_cast<float>(column),
                        0.5f + static_cast<float>(level), 0.0f));
    }
  }
  for (int i = 0; i < settle; ++i) orblit_physics_step(world, kStep);
  const Run run = stepFor(world, steps);
  orblit_physics_destroy(world);
  return run;
}

struct Casting {
  float micros = 0.0f;
  int hits = 0;
};

/// A level: 5,000 fixed boxes spread at the density `casting` uses, over ground,
/// with 200 spheres dropped into a 40 m patch of it. Everything fixed is there
/// to be looked past, so this is the scene a broadphase that re-sorts fixed
/// bodies every step pays for.
Run level(bool continuous) {
  OrblitPhysics *world = orblit_physics_create(nullptr);
  submit(world, ground(1));

  std::mt19937 random(17);
  constexpr int kFixed = 5000;
  const float half = 40.0f * std::sqrt(static_cast<float>(kFixed) / 100.0f);
  std::uniform_real_distribution<float> level(-half, half);
  OrblitPhysicsId id = 2;
  for (int i = 0; i < kFixed; ++i, ++id) {
    OrblitPhysicsCommand command = box(id, 0.5f, level(random), 0.5f, level(random));
    command.motion = ORBLIT_PHYSICS_STATIC;
    submit(world, command);
  }

  std::uniform_real_distribution<float> patch(-20.0f, 20.0f);
  std::uniform_real_distribution<float> height(2.0f, 10.0f);
  for (int i = 0; i < 200; ++i, ++id) {
    submit(world, sphere(id, 0.25f, patch(random), height(random), patch(random)));
    if (continuous) makeContinuous(world, id);
  }

  const Run run = stepFor(world, 300);
  orblit_physics_destroy(world);
  return run;
}

/// How long a ray takes, with the bodies where they are. The boxes are spread
/// at one density, 100 to a 80 m square, so a bigger count is a bigger level
/// rather than a denser one. Every ray is 40 m long, so the same few boxes lie
/// along it whatever the count. What grows is what a cast has to look past.
Casting casting(int count) {
  OrblitPhysics *world = orblit_physics_create(nullptr);
  std::mt19937 random(13);
  const float half = 40.0f * std::sqrt(static_cast<float>(count) / 100.0f);
  std::uniform_real_distribution<float> across(-half, half);
  std::uniform_real_distribution<float> turn(0.0f, 6.2831853f);
  for (OrblitPhysicsId id = 1; id <= static_cast<OrblitPhysicsId>(count); ++id) {
    OrblitPhysicsCommand command = box(id, 0.5f, across(random), 0.5f, across(random));
    command.motion = ORBLIT_PHYSICS_STATIC;
    submit(world, command);
  }

  constexpr int kRays = 5000;
  Casting result;
  const auto started = std::chrono::steady_clock::now();
  for (int i = 0; i < kRays; ++i) {
    OrblitPhysicsCast query{};
    query.layerIs = 1;
    query.layerCares = 0xFFFFFFFFu;
    query.from[0] = across(random);
    query.from[1] = 0.5f;
    query.from[2] = across(random);
    const float angle = turn(random);
    query.direction[0] = std::cos(angle);
    query.direction[2] = std::sin(angle);
    query.distance = 40.0f;
    OrblitPhysicsHit hit{};
    if (orblit_physics_cast(world, &query, &hit)) ++result.hits;
  }
  const std::chrono::duration<double, std::micro> took =
      std::chrono::steady_clock::now() - started;
  result.micros = static_cast<float>(took.count() / kRays);
  orblit_physics_destroy(world);
  return result;
}

void castRow(int count) {
  Casting best = casting(count);
  for (int i = 1; i < kRepeats; ++i) {
    const Casting next = casting(count);
    if (next.micros < best.micros) best = next;
  }
  std::printf("| %d static boxes | %.2f | %d of 5000 |\n", count, best.micros,
              best.hits);
}

/// A small sphere fired flat at a thin wall, with no gravity. The wall is 0.1 m
/// thick and the sphere 0.2 m across. `start` is how far back from 0 it begins,
/// so that a run can vary where in a step the sphere reaches the wall.
bool crossesWall(float speed, float start) {
  OrblitPhysicsSettings settings;
  orblit_physics_defaults(&settings);
  settings.gravity[1] = 0.0f;
  OrblitPhysics *world = orblit_physics_create(&settings);

  OrblitPhysicsCommand slab = box(1, 0.05f, 10.0f, 0.0f, 0.0f);
  slab.size[1] = 2.0f;
  slab.size[2] = 2.0f;
  slab.motion = ORBLIT_PHYSICS_STATIC;
  submit(world, slab);

  submit(world, sphere(2, 0.1f, -start, 0.0f, 0.0f));
  OrblitPhysicsCommand fire{};
  fire.kind = ORBLIT_PHYSICS_VELOCITY;
  fire.id = 2;
  fire.vector[0] = speed;
  submit(world, fire);

  const int steps = static_cast<int>(std::ceil(14.0f / (speed * kStep))) + 2;
  for (int i = 0; i < steps; ++i) orblit_physics_step(world, kStep);

  float at[7] = {};
  orblit_physics_transform(world, 2, at);
  orblit_physics_destroy(world);
  return at[0] > 10.0f;
}

/// How many of sixteen starts, spread over one step's travel, got through.
int crossings(float speed) {
  constexpr int kStarts = 16;
  int crossed = 0;
  for (int i = 0; i < kStarts; ++i) {
    const float start = speed * kStep * static_cast<float>(i) / kStarts;
    if (crossesWall(speed, start)) ++crossed;
  }
  return crossed;
}

void tunnelling() {
  std::printf("| Speed, m/s | 5 | 10 | 20 | 40 | 80 | 160 | 320 |\n");
  std::printf("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n");
  std::printf("| Starts of 16 that crossed |");
  for (const float speed : {5.0f, 10.0f, 20.0f, 40.0f, 80.0f, 160.0f, 320.0f}) {
    std::printf(" %d |", crossings(speed));
  }
  std::printf("\n");
}

/// Whether the first argument, if there is one, names this part of the run.
bool wanted(const char *filter, const char *name) {
  return filter == nullptr || std::strstr(name, filter) != nullptr;
}

} // namespace

int main(int argc, char **argv) {
  const char *filter = argc > 1 ? argv[1] : nullptr;
  std::printf("Step time in microseconds, at 60 steps a second.\n\n");
  header();
  const auto scene = [&](const char *name, const std::function<Run()> &make) {
    if (wanted(filter, name)) row(name, quickest(make));
  };
  scene("Scatter, 2,000 spheres", [] { return scatter(false); });
  scene("Scatter, 2,000 continuous spheres", [] { return scatter(true); });
  scene("Heap, 1,000 boxes", heap);
  scene("Wall, 500 boxes", [] { return wall(25, 20, 0, 600); });
  scene("Wall, 500 boxes, 120 steps after 600",
        [] { return wall(25, 20, 600, 120); });
  scene("Wall, 2,000 boxes", [] { return wall(50, 40, 0, 600); });
  scene("Wall, 2,000 boxes, 120 steps after 600",
        [] { return wall(50, 40, 600, 120); });
  scene("Wall, 2,000 boxes, 100 wide and 20 high",
        [] { return wall(100, 20, 0, 600); });
  scene("Wall, 2,000 boxes, 100 wide and 20 high, 120 steps after 600",
        [] { return wall(100, 20, 600, 120); });
  scene("Level, 5,000 fixed boxes and 200 spheres", [] { return level(false); });
  scene("Level, 5,000 fixed boxes and 200 continuous spheres",
        [] { return level(true); });

  if (wanted(filter, "casts")) {
    std::printf("\nMicroseconds for one ray cast.\n\n");
    std::printf("| World | Per cast | Hits |\n| --- | ---: | --- |\n");
    castRow(100);
    castRow(1000);
    castRow(5000);
  }

  if (wanted(filter, "tunnelling")) {
    std::printf("\nWhether a fast sphere gets through.\n\n");
    tunnelling();
  }
  return 0;
}
