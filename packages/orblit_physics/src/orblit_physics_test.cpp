// A standalone check of the engine through its own ABI, so a solver bug is
// found here rather than through two language bindings. Built by
// tool/check_native.sh, not by the package — Dart's tests cover the same
// ground from the other side of the hook.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "orblit_physics.h"

namespace {

int failures = 0;

void check(bool condition, const char *what) {
  if (!condition) {
    std::printf("FAIL  %s\n", what);
    failures++;
  } else {
    std::printf("ok    %s\n", what);
  }
}

OrblitPhysicsCommand sphereAt(OrblitPhysicsId id, float radius, float x, float y,
                              float z) {
  OrblitPhysicsCommand made{};
  made.kind = ORBLIT_PHYSICS_CREATE;
  made.shape = ORBLIT_PHYSICS_SPHERE;
  made.id = id;
  made.size[0] = radius;
  made.at[0] = x;
  made.at[1] = y;
  made.at[2] = z;
  made.motion = ORBLIT_PHYSICS_DYNAMIC;
  made.mass = 1.0f;
  made.friction = 0.5f;
  made.damping[0] = 0.05f;
  made.damping[1] = 0.05f;
  made.layerIs = 1;
  made.layerCares = 0xFFFFFFFFu;
  return made;
}

OrblitPhysicsCommand boxAt(OrblitPhysicsId id, float half, float x, float y,
                           float z) {
  OrblitPhysicsCommand made = sphereAt(id, half, x, y, z);
  made.shape = ORBLIT_PHYSICS_BOX;
  made.size[0] = half;
  made.size[1] = half;
  made.size[2] = half;
  return made;
}

OrblitPhysicsCommand groundPlane(OrblitPhysicsId id) {
  OrblitPhysicsCommand made{};
  made.kind = ORBLIT_PHYSICS_CREATE;
  made.shape = ORBLIT_PHYSICS_PLANE;
  made.id = id;
  made.size[1] = 1.0f; // Outward normal, straight up.
  made.motion = ORBLIT_PHYSICS_STATIC;
  made.friction = 0.5f;
  made.layerIs = 1;
  made.layerCares = 0xFFFFFFFFu;
  return made;
}

void submit(OrblitPhysics *physics, const OrblitPhysicsCommand &command) {
  orblit_physics_submit(physics, &command, 1);
}

/// Steps at a fixed sixtieth for `seconds`, the way a caller with a fixed
/// update would.
void run(OrblitPhysics *physics, float seconds) {
  const float delta = 1.0f / 60.0f;
  for (int i = 0; i < static_cast<int>(seconds / delta); ++i) {
    orblit_physics_step(physics, delta);
  }
}

float heightOf(OrblitPhysics *physics, OrblitPhysicsId id) {
  float transform[7] = {0};
  orblit_physics_transform(physics, id, transform);
  return transform[1];
}

float speedOf(OrblitPhysics *physics, OrblitPhysicsId id) {
  float motion[6] = {0};
  orblit_physics_velocity(physics, id, motion);
  return std::sqrt(motion[0] * motion[0] + motion[1] * motion[1] +
                   motion[2] * motion[2]);
}

bool near(float a, float b, float tolerance) {
  return std::fabs(a - b) < tolerance;
}

// --- the checks ----------------------------------------------------------- //

void settling() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 4.0f, 0.0f));

  run(physics, 3.0f);

  const float height = heightOf(physics, 2);
  check(height > 0.45f && height < 0.52f, "a sphere dropped on a plane rests on it");
  check(speedOf(physics, 2) < 0.05f, "and comes to a stop");
  check(orblit_physics_asleep(physics, 2), "and then goes to sleep");
  check(!orblit_physics_asleep(physics, 1), "a static body never sleeps");

  orblit_physics_destroy(physics);
}

void touching() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 1.0f, 0.0f));

  bool began = false;
  float force = 0.0f;
  float normalY = 0.0f;
  for (int i = 0; i < 120 && !began; ++i) {
    orblit_physics_step(physics, 1.0f / 60.0f);
    uint32_t count = 0;
    const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
    for (uint32_t e = 0; e < count; ++e) {
      if (events[e].kind != ORBLIT_PHYSICS_TOUCH_BEGAN) continue;
      began = true;
      force = events[e].force;
      normalY = events[e].normal[1];
      check(events[e].a == 1 && events[e].b == 2, "a touch names the pair, smaller id first");
    }
  }
  check(began, "landing reports a touch beginning");
  check(force > 0.0f, "with how hard it hit");
  // The plane is `a` here, so out of `b` towards `a` points down.
  check(normalY < -0.9f, "and a normal out of b towards a");

  // Taking the ground away ends the touch.
  OrblitPhysicsCommand gone{};
  gone.kind = ORBLIT_PHYSICS_DESTROY;
  gone.id = 1;
  submit(physics, gone);
  orblit_physics_step(physics, 1.0f / 60.0f);

  uint32_t count = 0;
  const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
  bool ended = false;
  for (uint32_t e = 0; e < count; ++e) {
    if (events[e].kind == ORBLIT_PHYSICS_TOUCH_ENDED) ended = true;
  }
  check(ended, "destroying one of a pair ends the touch");
  check(!orblit_physics_alive(physics, 1), "and the body is gone");
  check(orblit_physics_count(physics) == 1, "leaving the other one");

  orblit_physics_destroy(physics);
}

void bouncing() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));

  OrblitPhysicsCommand ball = sphereAt(2, 0.5f, 0.0f, 2.0f, 0.0f);
  ball.restitution = 0.8f;
  submit(physics, ball);

  run(physics, 0.7f);
  float highest = 0.0f;
  for (int i = 0; i < 72; ++i) {
    orblit_physics_step(physics, 1.0f / 60.0f);
    highest = std::fmax(highest, heightOf(physics, 2));
  }
  check(highest > 1.0f, "a bouncy ball comes back up");

  OrblitPhysics *dead = orblit_physics_create(nullptr);
  submit(dead, groundPlane(1));
  submit(dead, sphereAt(2, 0.5f, 0.0f, 2.0f, 0.0f));
  run(dead, 0.7f);
  float deadHighest = 0.0f;
  for (int i = 0; i < 72; ++i) {
    orblit_physics_step(dead, 1.0f / 60.0f);
    deadHighest = std::fmax(deadHighest, heightOf(dead, 2));
  }
  check(deadHighest < 0.6f, "one with no restitution lands dead");

  orblit_physics_destroy(dead);
  orblit_physics_destroy(physics);
}

void rubbing() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.49f, 0.0f));

  OrblitPhysicsCommand shove{};
  shove.kind = ORBLIT_PHYSICS_VELOCITY;
  shove.id = 2;
  shove.vector[0] = 3.0f;
  submit(physics, shove);

  run(physics, 3.0f);
  check(speedOf(physics, 2) < 0.1f, "friction brings a sliding box to a halt");

  OrblitPhysics *ice = orblit_physics_create(nullptr);
  OrblitPhysicsCommand slippery = groundPlane(1);
  slippery.friction = 0.0f;
  submit(ice, slippery);
  OrblitPhysicsCommand slider = boxAt(2, 0.5f, 0.0f, 0.49f, 0.0f);
  slider.friction = 0.0f;
  slider.damping[0] = 0.0f;
  submit(ice, slider);
  shove.vector[0] = 3.0f;
  submit(ice, shove);
  run(ice, 1.0f);
  check(speedOf(ice, 2) > 2.5f, "and with none it keeps going");

  orblit_physics_destroy(ice);
  orblit_physics_destroy(physics);
}

void stacking() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  submit(physics, boxAt(3, 0.5f, 0.0f, 1.5f, 0.0f));
  submit(physics, boxAt(4, 0.5f, 0.0f, 2.5f, 0.0f));

  run(physics, 4.0f);

  const float bottom = heightOf(physics, 2);
  const float middle = heightOf(physics, 3);
  const float top = heightOf(physics, 4);
  check(near(bottom, 0.5f, 0.06f), "the bottom of a stack holds its height");
  check(near(middle - bottom, 1.0f, 0.06f), "and the one above sits on it");
  check(near(top - middle, 1.0f, 0.06f), "and so does the one above that");
  check(speedOf(physics, 4) < 0.05f, "and the stack is still");

  orblit_physics_destroy(physics);
}

void layering() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand ground = groundPlane(1);
  ground.layerIs = 1;
  ground.layerCares = 0;
  submit(physics, ground);

  OrblitPhysicsCommand ghost = sphereAt(2, 0.5f, 0.0f, 1.0f, 0.0f);
  ghost.layerIs = 2;
  ghost.layerCares = 0;
  submit(physics, ghost);

  run(physics, 1.5f);
  check(heightOf(physics, 2) < -0.5f, "layers that do not care pass through");

  OrblitPhysics *caring = orblit_physics_create(nullptr);
  submit(caring, ground);
  ghost.layerCares = 1;
  submit(caring, ghost);
  run(caring, 1.5f);
  check(heightOf(caring, 2) > 0.4f, "and one that cares is stopped by one that does not");

  orblit_physics_destroy(caring);
  orblit_physics_destroy(physics);
}

void shoving() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 4.0f, 0.0f));
  run(physics, 3.0f);
  check(orblit_physics_asleep(physics, 2), "a settled body is asleep");

  OrblitPhysicsCommand kick{};
  kick.kind = ORBLIT_PHYSICS_IMPULSE;
  kick.id = 2;
  kick.vector[0] = 4.0f;
  kick.spin[0] = 0.0f;
  kick.spin[1] = heightOf(physics, 2);
  kick.spin[2] = 0.0f;
  submit(physics, kick);
  check(!orblit_physics_asleep(physics, 2), "an impulse wakes it");

  uint32_t count = 0;
  const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
  bool woke = false;
  for (uint32_t e = 0; e < count; ++e) {
    if (events[e].kind == ORBLIT_PHYSICS_WOKE && events[e].a == 2) woke = true;
  }
  check(woke, "and says so");

  orblit_physics_step(physics, 1.0f / 60.0f);
  check(speedOf(physics, 2) > 1.0f, "and it moves off");

  // Off centre, the same impulse should spin it as well as move it.
  OrblitPhysics *spun = orblit_physics_create(nullptr);
  submit(spun, sphereAt(2, 0.5f, 0.0f, 0.0f, 0.0f));
  OrblitPhysicsCommand glance = kick;
  glance.spin[1] = 0.5f;
  submit(spun, glance);
  float motion[6] = {0};
  orblit_physics_velocity(spun, 2, motion);
  check(std::fabs(motion[5]) > 0.1f, "an impulse away from the centre spins it");

  orblit_physics_destroy(spun);
  orblit_physics_destroy(physics);
}

void driving() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand lift = boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f);
  lift.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, lift);
  submit(physics, sphereAt(2, 0.5f, 0.0f, 1.2f, 0.0f));

  OrblitPhysicsCommand rise{};
  rise.kind = ORBLIT_PHYSICS_VELOCITY;
  rise.id = 1;
  rise.vector[1] = 0.5f;
  submit(physics, rise);

  run(physics, 2.0f);
  check(heightOf(physics, 1) > 0.9f, "a kinematic body goes where it is driven");
  check(heightOf(physics, 2) > 1.4f, "and carries what is resting on it");
  check(speedOf(physics, 1) > 0.4f, "and nothing slows it down");

  orblit_physics_destroy(physics);
}

void reading() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.5f, 1.0f, 2.0f, 3.0f));
  submit(physics, sphereAt(3, 0.5f, 4.0f, 5.0f, 6.0f));

  // A column with room for a scale after the rotation, the way a transform in
  // the entity store is laid out.
  const uint32_t stride = 10;
  std::vector<float> column(stride * 3, -1.0f);
  const OrblitPhysicsId ids[3] = {2, 99, 3};
  const uint32_t written = orblit_physics_read(physics, ids, 3, column.data(), stride, 1);

  check(written == 2, "reading skips an id the world does not know");
  check(near(column[1], 1.0f, 1e-5f) && near(column[3], 3.0f, 1e-5f),
        "and writes at the offset it was given");
  check(near(column[stride * 2 + 1], 4.0f, 1e-5f), "the stride apart it was given");
  check(near(column[7], 1.0f, 1e-5f), "with the rotation after the translation");
  check(column[0] == -1.0f && column[8] == -1.0f, "and touches nothing else");
  check(column[stride + 1] == -1.0f, "least of all the place the unknown id would have had");
  check(orblit_physics_read(physics, ids, 3, column.data(), 6, 0) == 0,
        "a stride too small to hold a transform writes nothing");

  orblit_physics_destroy(physics);
}

void refusing() {
  OrblitPhysicsSettings settings;
  std::memset(&settings, 0, sizeof(settings));
  orblit_physics_defaults(&settings);
  check(settings.gravity[1] < -9.0f, "the defaults have gravity");
  check(settings.velocitySteps >= 2 && settings.positionSteps >= 1,
        "and enough passes for friction to work");
  check(settings.sleeping, "and sleeping on");

  OrblitPhysics *physics = orblit_physics_create(&settings);
  submit(physics, sphereAt(0, 0.5f, 0.0f, 0.0f, 0.0f));
  check(orblit_physics_count(physics) == 0, "zero is never a body");

  submit(physics, sphereAt(2, 0.5f, 0.0f, 0.0f, 0.0f));
  submit(physics, sphereAt(2, 0.5f, 9.0f, 9.0f, 9.0f));
  check(orblit_physics_count(physics) == 1, "creating the same id twice is refused");
  check(near(heightOf(physics, 2), 0.0f, 1e-5f), "and does not move the first one");

  OrblitPhysicsCommand stale{};
  stale.kind = ORBLIT_PHYSICS_IMPULSE;
  stale.id = 404;
  stale.vector[0] = 100.0f;
  submit(physics, stale);
  check(orblit_physics_count(physics) == 1, "a command for a body that is gone is ignored");

  check(!orblit_physics_transform(physics, 404, nullptr), "reading nowhere fails");
  check(orblit_physics_count(nullptr) == 0, "and so does reading no world");
  orblit_physics_step(nullptr, 1.0f);
  orblit_physics_destroy(nullptr);
  check(true, "and calling into no world at all is survivable");

  orblit_physics_destroy(physics);
}

} // namespace

int main() {
  settling();
  touching();
  bouncing();
  rubbing();
  stacking();
  layering();
  shoving();
  driving();
  reading();
  refusing();

  std::printf(failures == 0 ? "\nALL PASSED\n" : "\n%d FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
