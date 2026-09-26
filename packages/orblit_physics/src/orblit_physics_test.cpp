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

OrblitPhysicsCommand capsuleAt(OrblitPhysicsId id, float radius,
                               float halfHeight, float x, float y, float z) {
  OrblitPhysicsCommand made = sphereAt(id, radius, x, y, z);
  made.shape = ORBLIT_PHYSICS_CAPSULE;
  made.size[0] = radius;
  made.size[1] = halfHeight;
  return made;
}

/// Turns a body a quarter circle about z, which lays a capsule along x.
void layDown(OrblitPhysicsCommand &command) {
  command.rotation[2] = 0.70710678f;
  command.rotation[3] = 0.70710678f;
}

OrblitPhysicsCast rayFrom(float x, float y, float z, float dx, float dy,
                          float dz, float distance) {
  OrblitPhysicsCast made{};
  made.shape = 0;
  made.layerIs = 1;
  made.layerCares = 0xFFFFFFFFu;
  made.from[0] = x;
  made.from[1] = y;
  made.from[2] = z;
  made.direction[0] = dx;
  made.direction[1] = dy;
  made.direction[2] = dz;
  made.distance = distance;
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

void capsules() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));

  // Radius 0.3 with half a metre of straight either side: 1.6 tall, stood up.
  submit(physics, capsuleAt(2, 0.3f, 0.5f, 0.0f, 4.0f, 0.0f));

  OrblitPhysicsCommand lying = capsuleAt(3, 0.3f, 0.5f, 3.0f, 4.0f, 0.0f);
  layDown(lying);
  submit(physics, lying);

  run(physics, 3.0f);

  check(near(heightOf(physics, 2), 0.8f, 0.05f),
        "a capsule dropped upright stands on its cap");
  check(near(heightOf(physics, 3), 0.3f, 0.05f),
        "and one laid on its side rests on the cylinder");
  check(orblit_physics_asleep(physics, 3),
        "which settles rather than rocking, being held at both ends");

  orblit_physics_destroy(physics);

  // A capsule laid across a box: the other two-point case, and the one that
  // goes through the face clipping rather than the plane.
  OrblitPhysics *onABox = orblit_physics_create(nullptr);
  OrblitPhysicsCommand plinth = boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f);
  plinth.motion = ORBLIT_PHYSICS_STATIC;
  submit(onABox, plinth);

  OrblitPhysicsCommand across = capsuleAt(2, 0.2f, 0.4f, 0.0f, 2.0f, 0.0f);
  layDown(across);
  submit(onABox, across);

  run(onABox, 3.0f);
  check(near(heightOf(onABox, 2), 0.7f, 0.05f),
        "a capsule laid across a box rests on its top face");
  check(orblit_physics_asleep(onABox, 2), "and settles there");
  orblit_physics_destroy(onABox);

  // Nothing falling, nothing sleeping: just the shapes pushing each other.
  OrblitPhysicsSettings still;
  orblit_physics_defaults(&still);
  still.gravity[1] = 0.0f;
  still.sleeping = false;

  OrblitPhysics *pair = orblit_physics_create(&still);
  submit(pair, capsuleAt(1, 0.3f, 0.5f, -0.1f, 0.0f, 0.0f));
  submit(pair, capsuleAt(2, 0.3f, 0.5f, 0.1f, 0.0f, 0.0f));
  run(pair, 2.0f);

  float left[7] = {0};
  float right[7] = {0};
  orblit_physics_transform(pair, 1, left);
  orblit_physics_transform(pair, 2, right);
  check(right[0] - left[0] > 0.55f,
        "two capsules pushed into each other end up side by side");
  check(near(left[1], 0.0f, 0.05f) && near(right[1], 0.0f, 0.05f),
        "and are not squeezed past one another along their length");
  orblit_physics_destroy(pair);

  // The same torque about each of two axes. A capsule is much easier to spin
  // about its own length than end over end, and a sphere's inertia tensor
  // wrongly used here would make the two the same.
  OrblitPhysics *spun = orblit_physics_create(&still);
  submit(spun, capsuleAt(1, 0.2f, 0.8f, 0.0f, 0.0f, 0.0f));
  submit(spun, capsuleAt(2, 0.2f, 0.8f, 5.0f, 0.0f, 0.0f));

  OrblitPhysicsCommand aboutItself{};
  aboutItself.kind = ORBLIT_PHYSICS_IMPULSE;
  aboutItself.id = 1;
  aboutItself.vector[2] = 1.0f;
  aboutItself.spin[0] = 0.2f;
  submit(spun, aboutItself);

  OrblitPhysicsCommand endOverEnd{};
  endOverEnd.kind = ORBLIT_PHYSICS_IMPULSE;
  endOverEnd.id = 2;
  endOverEnd.vector[2] = 1.0f;
  endOverEnd.spin[0] = 5.0f;
  endOverEnd.spin[1] = 0.2f;
  submit(spun, endOverEnd);

  orblit_physics_step(spun, 1.0f / 60.0f);
  float alone[6] = {0};
  float over[6] = {0};
  orblit_physics_velocity(spun, 1, alone);
  orblit_physics_velocity(spun, 2, over);
  check(std::fabs(alone[4]) > 5.0f * std::fabs(over[3]),
        "a capsule spins far more freely about its own length than across it");
  orblit_physics_destroy(spun);
}

void casting() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  OrblitPhysicsCommand crate = boxAt(2, 0.5f, 0.0f, 5.0f, 0.0f);
  crate.motion = ORBLIT_PHYSICS_STATIC;
  submit(physics, crate);

  OrblitPhysicsHit hit{};
  const OrblitPhysicsCast down = rayFrom(0.0f, 10.0f, 0.0f, 0.0f, -1.0f, 0.0f, 100.0f);
  check(orblit_physics_cast(physics, &down, &hit), "a ray fired down hits something");
  check(hit.body == 2, "and it is the crate, not the ground beyond it");
  check(near(hit.distance, 4.5f, 0.01f), "at the distance to its top face");
  check(near(hit.normal[1], 1.0f, 0.01f), "with a normal out of the face it met");
  check(near(hit.at[1], 5.5f, 0.01f), "and a point on that face");
  check(!hit.started, "and it did not begin inside anything");

  const OrblitPhysicsCast aside = rayFrom(0.3f, 10.0f, 0.2f, 0.0f, -1.0f, 0.0f, 100.0f);
  check(orblit_physics_cast(physics, &aside, &hit) &&
            near(hit.normal[1], 1.0f, 1.0e-4f),
        "and away from the middle of the face the normal is still straight up");

  OrblitPhysicsCast unmeasured = down;
  unmeasured.direction[1] = -10.0f;
  check(orblit_physics_cast(physics, &unmeasured, &hit) &&
            near(hit.distance, 4.5f, 0.01f),
        "a direction that is not a unit vector is still answered in metres");

  const OrblitPhysicsCast up = rayFrom(0.0f, 10.0f, 0.0f, 0.0f, 1.0f, 0.0f, 100.0f);
  check(!orblit_physics_cast(physics, &up, &hit), "a ray fired away from everything misses");

  OrblitPhysicsCast stops = down;
  stops.distance = 1.0f;
  check(!orblit_physics_cast(physics, &stops, &hit), "and so does one that stops short");

  OrblitPhysicsCast ball = down;
  ball.shape = ORBLIT_PHYSICS_SPHERE;
  ball.size[0] = 0.25f;
  check(orblit_physics_cast(physics, &ball, &hit) &&
            near(hit.distance, 4.25f, 0.02f),
        "a sphere cast stops its own radius earlier than a ray");

  OrblitPhysicsCast walking{};
  walking.shape = ORBLIT_PHYSICS_CAPSULE;
  walking.size[0] = 0.3f;
  walking.size[1] = 0.6f;
  walking.from[0] = -5.0f;
  walking.from[1] = 5.0f;
  walking.direction[0] = 1.0f;
  walking.distance = 10.0f;
  walking.layerIs = 1;
  walking.layerCares = 0xFFFFFFFFu;
  check(orblit_physics_cast(physics, &walking, &hit) &&
            near(hit.distance, 4.2f, 0.02f),
        "a capsule cast sideways stops a radius short of the face");
  check(near(hit.normal[0], -1.0f, 0.02f), "against the face it walked into");

  // Closer to the face than a cast calls touching, but not in it: exactly
  // flush is an overlap of nothing, which the narrowphase counts as begun.
  OrblitPhysicsCast grazing = walking;
  grazing.from[0] = -0.80005f;
  grazing.direction[0] = 0.0f;
  grazing.direction[1] = -1.0f;
  grazing.distance = 1.0f;
  check(!orblit_physics_cast(physics, &grazing, &hit),
        "and one against that face and dropping past it misses");

  const OrblitPhysicsCast begun = rayFrom(0.0f, 5.0f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f);
  check(orblit_physics_cast(physics, &begun, &hit), "a cast that begins inside a body reports it");
  check(hit.started && hit.distance == 0.0f, "as begun, at no distance at all");

  OrblitPhysicsCast skipping = begun;
  skipping.ignore = 2;
  check(!orblit_physics_cast(physics, &skipping, &hit),
        "and passes clean through the body it was told to ignore");

  const OrblitPhysicsCast beside = rayFrom(3.0f, 2.0f, 0.0f, 0.0f, -1.0f, 0.0f, 10.0f);
  check(orblit_physics_cast(physics, &beside, &hit) && hit.body == 1 &&
            near(hit.distance, 2.0f, 0.01f),
        "a ray beside the crate reaches the ground behind it");

  OrblitPhysicsCast flat = down;
  flat.shape = ORBLIT_PHYSICS_PLANE;
  flat.size[1] = 1.0f;
  check(!orblit_physics_cast(physics, &flat, &hit), "casting a half-space is refused");
  check(!orblit_physics_cast(nullptr, &down, &hit), "and casting into no world is survivable");

  orblit_physics_destroy(physics);

  // Layers, in a world with one body in it, because a body that cares about
  // everything would be hit whatever the cast asked for.
  OrblitPhysics *filtered = orblit_physics_create(nullptr);
  OrblitPhysicsCommand quiet = boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f);
  quiet.motion = ORBLIT_PHYSICS_STATIC;
  quiet.layerIs = 2;
  quiet.layerCares = 0;
  submit(filtered, quiet);

  OrblitPhysicsCast curious = rayFrom(0.0f, 5.0f, 0.0f, 0.0f, -1.0f, 0.0f, 10.0f);
  curious.layerIs = 4;
  curious.layerCares = 2;
  check(orblit_physics_cast(filtered, &curious, &hit), "a cast meets a body it cares about");
  curious.layerCares = 8;
  check(!orblit_physics_cast(filtered, &curious, &hit), "and passes through one it does not");
  orblit_physics_destroy(filtered);
}

// --- the character course -------------------------------------------------- //
//
// Stairs, a ledge, two ramps, a lift, a turntable, crates and a moving wall:
// everything a character is for, walked the way a game would walk it.

constexpr float kStep = 1.0f / 60.0f;

OrblitPhysicsCommand slabAt(OrblitPhysicsId id, float hx, float hy, float hz,
                            float x, float y, float z) {
  OrblitPhysicsCommand made = boxAt(id, hx, x, y, z);
  made.size[1] = hy;
  made.size[2] = hz;
  made.motion = ORBLIT_PHYSICS_STATIC;
  return made;
}

/// A fixed half-space rising along +x at `angle` from the ground, from x =
/// `foot` onwards. Below the ground before that, so it and a ground plane
/// together are a flat floor running into a ramp.
OrblitPhysicsCommand rampFrom(OrblitPhysicsId id, float foot, float angle) {
  OrblitPhysicsCommand made = groundPlane(id);
  made.size[0] = -std::sin(angle);
  made.size[1] = std::cos(angle);
  made.size[3] = -foot * std::sin(angle);
  return made;
}

/// A person: a capsule 1.8 m tall and 0.6 across, feet a skin above `feet`,
/// that climbs 0.3 m, walks up 45 degrees and pushes with 500 N.
void addCharacter(OrblitPhysics *physics, OrblitPhysicsId id, float x, float feet,
                  float z) {
  OrblitPhysicsCommand made = capsuleAt(id, 0.3f, 0.6f, x, feet + 0.91f, z);
  made.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, made);

  OrblitPhysicsCommand character{};
  character.kind = ORBLIT_PHYSICS_CHARACTER;
  character.id = id;
  character.size[0] = 0.3f;
  character.size[1] = 0.78539816f;
  character.size[2] = 0.01f;
  character.size[3] = 500.0f;
  submit(physics, character);
}

void drive(OrblitPhysics *physics, OrblitPhysicsId id, float vx, float vy,
           float vz, float spinY) {
  OrblitPhysicsCommand made{};
  made.kind = ORBLIT_PHYSICS_VELOCITY;
  made.id = id;
  made.vector[0] = vx;
  made.vector[1] = vy;
  made.vector[2] = vz;
  made.spin[1] = spinY;
  submit(physics, made);
}

OrblitPhysicsFooting footingOf(OrblitPhysics *physics, OrblitPhysicsId id) {
  OrblitPhysicsFooting out{};
  orblit_physics_footing(physics, &id, 1, &out);
  return out;
}

float coordinateOf(OrblitPhysics *physics, OrblitPhysicsId id, int axis) {
  float transform[7] = {0};
  orblit_physics_transform(physics, id, transform);
  return transform[axis];
}

/// Walks a character the way a game does, a fixed step at a time: ask to go
/// at (vx, vz) across the ground, keep whatever it was falling at, and add a
/// step of gravity to that. Returns how many steps it was not standing.
int walk(OrblitPhysics *physics, OrblitPhysicsId id, float vx, float vz,
         float seconds) {
  int airborne = 0;
  const int steps = static_cast<int>(seconds / kStep + 0.5f);
  for (int i = 0; i < steps; ++i) {
    const OrblitPhysicsFooting footing = footingOf(physics, id);
    OrblitPhysicsCommand ask{};
    ask.kind = ORBLIT_PHYSICS_VELOCITY;
    ask.id = id;
    ask.vector[0] = vx;
    ask.vector[1] = footing.velocity[1] - 9.81f * kStep;
    ask.vector[2] = vz;
    submit(physics, ask);
    orblit_physics_step(physics, kStep);
    if (!footingOf(physics, id).grounded) airborne++;
  }
  return airborne;
}

/// Five 0.2 m risers with 0.3 m treads from x = 1, up to a landing at 1.0 m
/// that runs on to x = 8. Ids 10 to 14, the landing last.
void buildStairs(OrblitPhysics *physics) {
  for (int i = 0; i < 5; ++i) {
    const float top = 0.2f * static_cast<float>(i + 1);
    const float from = 1.0f + 0.3f * static_cast<float>(i);
    const float to = i == 4 ? 8.0f : from + 0.3f;
    submit(physics, slabAt(10 + i, (to - from) / 2.0f, top / 2.0f, 1.0f,
                           (from + to) / 2.0f, top / 2.0f, 0.0f));
  }
}

void walking() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  addCharacter(physics, 2, 0.0f, 0.0f, 0.0f);

  const int airborne = walk(physics, 2, 2.0f, 0.0f, 1.0f);
  check(near(coordinateOf(physics, 2, 0), 2.0f, 0.05f),
        "a character walks across flat ground at the speed it asks for");
  check(airborne == 0, "and never leaves it");
  check(near(heightOf(physics, 2), 0.91f, 0.002f), "and stands a skin above it");

  const OrblitPhysicsFooting footing = footingOf(physics, 2);
  check(footing.ground == 1 && near(footing.normal[1], 1.0f, 1.0e-4f),
        "and says what it is standing on");
  check(near(footing.velocity[0], 2.0f, 1.0e-4f) &&
            near(footing.velocity[1], 0.0f, 1.0e-4f),
        "with the fall taken out of what it asked for and the walk left in");
  check(near(speedOf(physics, 2), 2.0f, 0.05f),
        "and its velocity is what it did, for the solver to see");

  orblit_physics_destroy(physics);
}

void climbing() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  buildStairs(physics);
  addCharacter(physics, 2, 0.0f, 0.0f, 0.0f);

  walk(physics, 2, 1.5f, 0.0f, 3.0f);
  check(coordinateOf(physics, 2, 0) > 3.0f,
        "a character walks up 0.2 m stairs with a 0.3 m step");
  check(near(heightOf(physics, 2), 1.91f, 0.005f),
        "and stands on the landing at the top");
  check(footingOf(physics, 2).ground == 14, "which is what it says it is on");

  const int airborne = walk(physics, 2, -1.5f, 0.0f, 3.0f);
  check(coordinateOf(physics, 2, 0) < 0.5f &&
            near(heightOf(physics, 2), 0.91f, 0.005f),
        "and back down them to the ground");
  check(airborne == 0, "without once leaving them on the way down");

  orblit_physics_destroy(physics);
}

void blocking() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, slabAt(3, 1.0f, 0.25f, 1.0f, 2.0f, 0.25f, 0.0f));
  addCharacter(physics, 2, 0.0f, 0.0f, 0.0f);

  walk(physics, 2, 1.5f, 0.0f, 2.0f);
  const float x = coordinateOf(physics, 2, 0);
  check(x > 0.65f && x < 0.7f, "a 0.5 m ledge stops a character that climbs 0.3 m");
  check(near(heightOf(physics, 2), 0.91f, 0.002f),
        "and it stays on the ground in front of it");

  orblit_physics_destroy(physics);
}

void slopes() {
  OrblitPhysics *steep = orblit_physics_create(nullptr);
  submit(steep, groundPlane(1));
  submit(steep, rampFrom(3, 1.0f, 1.0471976f));
  addCharacter(steep, 2, 0.0f, 0.0f, 0.0f);
  walk(steep, 2, 1.5f, 0.0f, 3.0f);
  check(coordinateOf(steep, 2, 0) < 1.3f && heightOf(steep, 2) < 1.1f,
        "a 60 degree slope stops a character that climbs 45");
  orblit_physics_destroy(steep);

  OrblitPhysics *gentle = orblit_physics_create(nullptr);
  submit(gentle, groundPlane(1));
  submit(gentle, rampFrom(3, 1.0f, 0.43633231f));
  addCharacter(gentle, 2, 0.0f, 0.0f, 0.0f);
  walk(gentle, 2, 1.5f, 0.0f, 3.0f);
  check(coordinateOf(gentle, 2, 0) > 4.3f && heightOf(gentle, 2) > 2.4f,
        "and a 25 degree one it walks up at full speed");
  check(footingOf(gentle, 2).ground == 3 &&
            near(footingOf(gentle, 2).normal[0], -0.42261826f, 1.0e-3f),
        "and says it is on the ramp, and which way the ramp faces");

  const float x = coordinateOf(gentle, 2, 0);
  const float y = heightOf(gentle, 2);
  walk(gentle, 2, 0.0f, 0.0f, 1.0f);
  check(near(coordinateOf(gentle, 2, 0), x, 1.0e-3f) &&
            near(heightOf(gentle, 2), y, 1.0e-3f),
        "and stands still on it without sliding back down");
  orblit_physics_destroy(gentle);
}

void riding() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand lift = slabAt(1, 1.0f, 0.1f, 1.0f, 0.0f, 0.0f, 0.0f);
  lift.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, lift);
  addCharacter(physics, 2, 0.0f, 0.1f, 0.0f);

  drive(physics, 1, 0.0f, 1.0f, 0.0f, 0.0f);
  int airborne = walk(physics, 2, 0.0f, 0.0f, 1.0f);
  check(heightOf(physics, 1) > 0.95f &&
            near(heightOf(physics, 2) - heightOf(physics, 1), 1.01f, 0.005f),
        "a character rides a lift up");
  check(footingOf(physics, 2).ground == 1 &&
            near(footingOf(physics, 2).carried[1], 1.0f, 0.01f),
        "and says what carried it and how fast");

  drive(physics, 1, 0.0f, -1.0f, 0.0f, 0.0f);
  airborne += walk(physics, 2, 0.0f, 0.0f, 1.0f);
  check(near(heightOf(physics, 2) - heightOf(physics, 1), 1.01f, 0.005f),
        "and back down again");
  check(airborne == 0, "standing on it the whole way");

  orblit_physics_destroy(physics);
}

void turning() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand table = slabAt(1, 3.0f, 0.1f, 3.0f, 0.0f, 0.0f, 0.0f);
  table.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, table);
  addCharacter(physics, 2, 1.0f, 0.1f, 0.0f);
  drive(physics, 1, 0.0f, 0.0f, 0.0f, 1.0f);

  // A quarter turn, which takes (1, 0) round to (0, -1) about +y.
  walk(physics, 2, 0.0f, 0.0f, 1.5707963f);
  check(near(coordinateOf(physics, 2, 0), 0.0f, 0.05f) &&
            near(coordinateOf(physics, 2, 2), -1.0f, 0.05f),
        "a character on a turntable goes round with it");
  check(near(footingOf(physics, 2).turning, 1.0f, 0.01f),
        "and says how fast it is being turned");

  orblit_physics_destroy(physics);
}

void pushing() {
  OrblitPhysics *open = orblit_physics_create(nullptr);
  submit(open, groundPlane(1));
  OrblitPhysicsCommand crate = boxAt(3, 0.4f, 1.5f, 0.4f, 0.0f);
  crate.mass = 10.0f;
  submit(open, crate);
  addCharacter(open, 2, 0.0f, 0.0f, 0.0f);

  walk(open, 2, 1.5f, 0.0f, 2.0f);
  check(coordinateOf(open, 3, 0) > 2.3f, "a character pushes a crate it walks into");
  check(coordinateOf(open, 2, 0) < coordinateOf(open, 3, 0) - 0.69f,
        "without walking into it");
  orblit_physics_destroy(open);

  OrblitPhysics *walled = orblit_physics_create(nullptr);
  submit(walled, groundPlane(1));
  submit(walled, slabAt(4, 0.25f, 1.0f, 2.0f, 3.25f, 1.0f, 0.0f));
  crate.at[0] = 2.6f;
  submit(walled, crate);
  addCharacter(walled, 2, 0.0f, 0.0f, 0.0f);

  walk(walled, 2, 1.5f, 0.0f, 3.0f);
  check(coordinateOf(walled, 3, 0) < 2.62f, "and cannot push one through a wall");
  check(coordinateOf(walled, 2, 0) < 1.91f, "or walk through it either");
  orblit_physics_destroy(walled);
}

void crowding() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  OrblitPhysicsCommand wall = slabAt(3, 0.25f, 1.0f, 2.0f, 2.0f, 1.0f, 0.0f);
  wall.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, wall);
  drive(physics, 3, -1.0f, 0.0f, 0.0f, 0.0f);
  addCharacter(physics, 2, 0.0f, 0.0f, 0.0f);

  const int airborne = walk(physics, 2, 0.0f, 0.0f, 2.0f);
  check(coordinateOf(physics, 2, 0) < coordinateOf(physics, 3, 0) - 0.55f,
        "a driven wall pushes a character out of its way");
  check(airborne == 0 && near(heightOf(physics, 2), 0.91f, 0.002f),
        "along the ground, rather than up or through it");

  orblit_physics_destroy(physics);
}

void footings() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(3, 0.5f, 3.0f, 0.5f, 0.0f));
  addCharacter(physics, 2, 0.0f, 0.0f, 0.0f);
  orblit_physics_step(physics, kStep);

  const OrblitPhysicsId ids[3] = {99, 2, 3};
  OrblitPhysicsFooting out[3] = {};
  out[0].ground = 12345;
  out[2].ground = 12345;
  check(orblit_physics_footing(physics, ids, 3, out) == 1,
        "reading footing counts only the characters");
  check(out[1].grounded && out[1].ground == 1, "and fills in the one it found");
  check(out[0].ground == 12345 && out[2].ground == 12345,
        "and leaves every other slot alone");

  OrblitPhysicsCommand up{};
  up.kind = ORBLIT_PHYSICS_PLACE;
  up.id = 2;
  up.at[1] = 5.0f;
  up.rotation[3] = 1.0f;
  submit(physics, up);
  check(!footingOf(physics, 2).grounded && footingOf(physics, 2).ground == 0,
        "placing a character forgets what it stood on");

  run(physics, 1.0f);
  check(near(heightOf(physics, 2), 5.0f, 1.0e-4f),
        "and it does not fall by itself");

  const int airborne = walk(physics, 2, 0.0f, 0.0f, 2.0f);
  check(airborne > 0 && footingOf(physics, 2).grounded &&
            near(heightOf(physics, 2), 0.91f, 0.002f),
        "but does when the game adds gravity, and lands");

  OrblitPhysicsCommand gone{};
  gone.kind = ORBLIT_PHYSICS_DESTROY;
  gone.id = 2;
  submit(physics, gone);
  OrblitPhysicsFooting none{};
  const OrblitPhysicsId two = 2;
  check(orblit_physics_footing(physics, &two, 1, &none) == 0,
        "destroying a character forgets it");

  OrblitPhysicsCommand refused{};
  refused.kind = ORBLIT_PHYSICS_CHARACTER;
  refused.id = 3;
  submit(physics, refused);
  const OrblitPhysicsId three = 3;
  check(orblit_physics_footing(physics, &three, 1, &none) == 0,
        "and only a kinematic body can become one");

  orblit_physics_destroy(physics);
}

// --- ground ---------------------------------------------------------------- //

float flat(float, float) { return 0.0f; }

/// Two slopes of one in two, meeting in a crease along x = 0.
float valley(float x, float) { return 0.5f * std::fabs(x); }

/// A ridge along x = 0, falling away three in ten either side.
float ridge(float x, float) { return 1.0f - 0.3f * std::fabs(x); }

/// About 17 degrees, rising towards +x.
float incline(float x, float) { return 0.3f * x; }

/// Lumpy, and the same along no line.
float bumps(float x, float z) {
  return std::sin(x * 1.3f) + 0.6f * std::cos(z * 1.7f + 0.4f) + 0.05f * x;
}

/// `columns` by `rows` heights read off `surface`, `spacing` apart, with
/// sample (0, 0) at (x, z).
std::vector<float> sampled(uint32_t columns, uint32_t rows, float spacing,
                           float x, float z, float (*surface)(float, float)) {
  std::vector<float> heights(static_cast<size_t>(columns) * rows);
  for (uint32_t r = 0; r < rows; ++r) {
    for (uint32_t c = 0; c < columns; ++c) {
      heights[r * columns + c] = surface(x + c * spacing, z + r * spacing);
    }
  }
  return heights;
}

OrblitPhysicsGround groundOf(OrblitPhysicsId id, const std::vector<float> &heights,
                             uint32_t columns, uint32_t rows, float x, float z) {
  OrblitPhysicsGround made{};
  made.id = id;
  made.heights = heights.data();
  made.columns = columns;
  made.rows = rows;
  made.spacing = 1.0f;
  made.at[0] = x;
  made.at[2] = z;
  made.friction = 0.5f;
  made.layerIs = 1;
  made.layerCares = 0xFFFFFFFFu;
  return made;
}

/// Eight metres square of `surface` in 1 m cells, centred on the origin.
bool lay(OrblitPhysics *physics, OrblitPhysicsId id,
         float (*surface)(float, float)) {
  const std::vector<float> heights = sampled(9, 9, 1.0f, -4.0f, -4.0f, surface);
  const OrblitPhysicsGround ground = groundOf(id, heights, 9, 9, -4.0f, -4.0f);
  return orblit_physics_ground(physics, &ground);
}

/// How far a body's own up has tipped away from the world's, as the cosine
/// between them.
float levelOf(OrblitPhysics *physics, OrblitPhysicsId id) {
  float t[7] = {0};
  orblit_physics_transform(physics, id, t);
  return 1.0f - 2.0f * (t[3] * t[3] + t[5] * t[5]);
}

void resting() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  check(lay(physics, 1, flat), "ground is laid");
  check(orblit_physics_alive(physics, 1) && orblit_physics_count(physics) == 1,
        "and is a body like any other");

  submit(physics, sphereAt(2, 0.5f, 0.3f, 3.0f, 0.2f));
  // Its bottom face spans four cells and the corner sample between them.
  submit(physics, boxAt(3, 0.5f, 2.0f, 3.0f, 2.0f));
  OrblitPhysicsCommand lying = capsuleAt(4, 0.3f, 0.6f, -2.0f, 3.0f, 1.5f);
  layDown(lying);
  submit(physics, lying);
  run(physics, 3.0f);

  check(near(heightOf(physics, 2), 0.5f, 0.03f),
        "a sphere dropped on flat ground rests on it");
  check(near(heightOf(physics, 3), 0.5f, 0.03f) && levelOf(physics, 3) > 0.999f,
        "and a box across four of its cells rests on it level");
  check(near(heightOf(physics, 4), 0.3f, 0.03f),
        "and a capsule lying across several");
  check(orblit_physics_asleep(physics, 2) && orblit_physics_asleep(physics, 3) &&
            orblit_physics_asleep(physics, 4),
        "and all three settle enough to sleep");

  orblit_physics_destroy(physics);
}

void rolling() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  // Two fields side by side, meeting along x = 0.
  const std::vector<float> west = sampled(5, 9, 1.0f, -4.0f, -4.0f, flat);
  const std::vector<float> east = sampled(5, 9, 1.0f, 0.0f, -4.0f, flat);
  const OrblitPhysicsGround a = groundOf(1, west, 5, 9, -4.0f, -4.0f);
  const OrblitPhysicsGround b = groundOf(2, east, 5, 9, 0.0f, -4.0f);
  orblit_physics_ground(physics, &a);
  orblit_physics_ground(physics, &b);

  submit(physics, sphereAt(3, 0.5f, -3.0f, 0.5f, 0.37f));
  submit(physics, boxAt(4, 0.5f, -2.0f, 0.5f, -2.3f));
  run(physics, 0.5f);
  drive(physics, 3, 4.0f, 0.0f, 0.5f, 0.0f);
  drive(physics, 4, 5.0f, 0.0f, 0.0f, 0.0f);

  float ball = 0.0f;
  float box = 0.0f;
  float tipped = 1.0f;
  for (int i = 0; i < 90; ++i) {
    orblit_physics_step(physics, kStep);
    ball = std::fmax(ball, heightOf(physics, 3));
    box = std::fmax(box, heightOf(physics, 4));
    tipped = std::fmin(tipped, levelOf(physics, 4));
  }

  check(ball < 0.51f, "a ball rolling over ground never hops at a seam");
  check(coordinateOf(physics, 3, 0) > 1.0f,
        "not even the one between two fields, which it crossed");
  check(box < 0.51f && tipped > 0.999f,
        "and a box sliding over it neither hops nor trips");
  check(coordinateOf(physics, 4, 0) > 0.0f, "into the next field as well");

  orblit_physics_destroy(physics);
}

void creases() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  lay(physics, 1, valley);
  submit(physics, boxAt(2, 0.5f, 0.0f, 3.0f, -1.7f));
  submit(physics, sphereAt(3, 0.5f, 0.0f, 3.0f, 1.8f));
  run(physics, 4.0f);

  // Its bottom corners on the slopes either side, a quarter of a metre up.
  check(near(coordinateOf(physics, 2, 0), 0.0f, 0.03f) &&
            near(heightOf(physics, 2), 0.75f, 0.03f) &&
            levelOf(physics, 2) > 0.999f,
        "a box in a valley sits level across the crease");
  check(speedOf(physics, 2) < 0.05f,
        "and stays there, not shoved along either slope");
  // Touching both slopes, its centre is its radius over the cosine up.
  check(near(coordinateOf(physics, 3, 0), 0.0f, 0.03f) &&
            near(heightOf(physics, 3), 0.559f, 0.03f),
        "and a ball settles into the bottom of it");

  orblit_physics_destroy(physics);
}

void holes() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  std::vector<float> heights = sampled(9, 9, 1.0f, -4.0f, -4.0f, flat);
  // Nine samples gone, and with them every triangle touching one: a hole
  // from x = -2 to 2 and z = -2 to 2.
  for (int r = 3; r <= 5; ++r) {
    for (int c = 3; c <= 5; ++c) heights[r * 9 + c] = std::nanf("");
  }
  const OrblitPhysicsGround ground = groundOf(1, heights, 9, 9, -4.0f, -4.0f);
  orblit_physics_ground(physics, &ground);

  submit(physics, sphereAt(2, 0.3f, 0.2f, 3.0f, 0.1f));
  submit(physics, sphereAt(3, 0.3f, 3.2f, 3.0f, 3.1f));
  submit(physics, sphereAt(4, 0.3f, -3.4f, 0.3f, 0.0f));
  run(physics, 0.25f);
  drive(physics, 4, 3.0f, 0.0f, 0.0f, 0.0f);
  run(physics, 2.0f);

  check(heightOf(physics, 2) < -2.0f, "a ball dropped over a hole falls through");
  check(near(heightOf(physics, 3), 0.3f, 0.03f), "and one beside it does not");
  check(heightOf(physics, 4) < -1.0f, "and one rolled towards it goes over the rim");

  OrblitPhysicsHit hit{};
  const OrblitPhysicsCast down = rayFrom(0.5f, 5.0f, 0.5f, 0.0f, -1.0f, 0.0f, 20.0f);
  check(!orblit_physics_cast(physics, &down, &hit), "a ray down the hole meets nothing");

  orblit_physics_destroy(physics);
}

void castingGround() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  lay(physics, 1, incline);
  const float tilt = 1.0f / std::sqrt(1.09f);

  OrblitPhysicsHit hit{};
  const OrblitPhysicsCast ray = rayFrom(0.3f, 10.0f, 0.7f, 0.0f, -1.0f, 0.0f, 20.0f);
  check(orblit_physics_cast(physics, &ray, &hit) && hit.body == 1 &&
            near(hit.at[1], 0.09f, 1.0e-3f) && near(hit.distance, 9.91f, 1.0e-3f),
        "a ray down onto ground hits it where it is");
  check(near(hit.normal[0], -0.3f * tilt, 1.0e-3f) &&
            near(hit.normal[1], tilt, 1.0e-3f) && !hit.started,
        "and says which way the slope faces");

  OrblitPhysicsCast ball = ray;
  ball.shape = ORBLIT_PHYSICS_SPHERE;
  ball.size[0] = 0.5f;
  // Its centre ends up a radius off the slope, measured along the slope's
  // normal: more than a radius straight up.
  check(orblit_physics_cast(physics, &ball, &hit) &&
            near(hit.distance, 10.0f - 0.09f - 0.5f / tilt, 5.0e-3f),
        "and a ball cast down lands a radius off the slope");

  OrblitPhysicsCast across = rayFrom(-3.5f, 0.3f, 0.0f, 1.0f, 0.0f, 0.0f, 10.0f);
  across.shape = ORBLIT_PHYSICS_SPHERE;
  across.size[0] = 0.25f;
  check(orblit_physics_cast(physics, &across, &hit) &&
            near(hit.distance, 3.5f + (0.3f - 0.25f / tilt) / 0.3f, 5.0e-3f) &&
            near(hit.normal[1], tilt, 1.0e-3f),
        "a ball cast along the ground meets the hill in front of it");

  const OrblitPhysicsCast under = rayFrom(0.0f, -2.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
  check(orblit_physics_cast(physics, &under, &hit) && hit.started,
        "a cast from underground starts inside it");

  OrblitPhysicsCast refused = ray;
  refused.shape = ORBLIT_PHYSICS_HEIGHT_FIELD;
  check(!orblit_physics_cast(physics, &refused, &hit), "and ground cannot be cast");

  orblit_physics_destroy(physics);

  physics = orblit_physics_create(nullptr);
  lay(physics, 1, flat);
  // Skimming the ground, one a hair above it and one a centimetre above.
  bool met = false;
  const float gaps[2] = {5.0e-5f, 0.01f};
  for (const float gap : gaps) {
    for (int i = 0; i < 8; ++i) {
      OrblitPhysicsCast skim =
          rayFrom(-3.3f, 0.5f + gap, -2.9f + 0.7f * i, 1.0f, 0.0f, 0.37f * (i % 3), 6.0f);
      skim.shape = ORBLIT_PHYSICS_SPHERE;
      skim.size[0] = 0.5f;
      met = met || orblit_physics_cast(physics, &skim, &hit);
    }
  }
  check(!met, "a ball skimming flat ground is not stopped by the seams in it");

  orblit_physics_destroy(physics);

  // Straight down a hair either side of every edge and diagonal, where the
  // triangle nearest is only just the one underneath.
  physics = orblit_physics_create(nullptr);
  lay(physics, 1, bumps);
  int missed = 0;
  const float hairs[4] = {-1.0e-3f, -1.0e-5f, 1.0e-5f, 1.0e-3f};
  for (int r = -3; r < 3; ++r) {
    for (int c = -3; c < 3; ++c) {
      for (const float d : hairs) {
        const float points[3][2] = {{c + d, r + 0.37f},
                                    {c + 0.37f, r + d},
                                    {c + 0.61f + d, r + 0.61f}};
        for (const auto &p : points) {
          const float x = p[0], z = p[1];
          const float cx = std::floor(x), cz = std::floor(z);
          const float u = x - cx, v = z - cz;
          const float h00 = bumps(cx, cz), h10 = bumps(cx + 1, cz);
          const float h01 = bumps(cx, cz + 1), h11 = bumps(cx + 1, cz + 1);
          const float drawn = u >= v ? h00 + u * (h10 - h00) + v * (h11 - h10)
                                     : h00 + v * (h01 - h00) + u * (h11 - h01);
          const OrblitPhysicsCast down =
              rayFrom(x, 5.0f, z, 0.0f, -1.0f, 0.0f, 10.0f);
          if (!orblit_physics_cast(physics, &down, &hit) ||
              !near(hit.at[1], drawn, 1.0e-3f)) {
            missed++;
          }
        }
      }
    }
  }
  check(missed == 0, "a ray down beside an edge meets the ground under it");

  orblit_physics_destroy(physics);
}

void reshaping() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  lay(physics, 1, flat);
  submit(physics, boxAt(2, 0.5f, 0.5f, 0.5f, 0.5f));
  run(physics, 3.0f);
  check(orblit_physics_asleep(physics, 2), "a crate on ground falls asleep on it");

  const std::vector<float> raised = sampled(9, 9, 1.0f, -4.0f, -4.0f, flat);
  std::vector<float> higher(raised.size(), 0.3f);
  OrblitPhysicsGround ground = groundOf(1, higher, 9, 9, -4.0f, -4.0f);
  check(orblit_physics_ground(physics, &ground) && orblit_physics_count(physics) == 2,
        "laying ground again replaces it");
  check(!orblit_physics_asleep(physics, 2), "and wakes what was asleep on it");
  run(physics, 2.0f);
  check(near(heightOf(physics, 2), 0.8f, 0.03f),
        "which ends up on the raised ground, not in it");

  std::vector<float> lower(raised.size(), -1.0f);
  ground.heights = lower.data();
  orblit_physics_ground(physics, &ground);
  run(physics, 2.0f);
  check(near(heightOf(physics, 2), -0.5f, 0.03f),
        "and falls with it when it is lowered");

  OrblitPhysicsCommand gone{};
  gone.kind = ORBLIT_PHYSICS_DESTROY;
  gone.id = 1;
  submit(physics, gone);
  check(!orblit_physics_alive(physics, 1) && orblit_physics_count(physics) == 1,
        "and ground is destroyed like any other body");

  orblit_physics_destroy(physics);
}

void hiking() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const std::vector<float> west = sampled(5, 9, 1.0f, -4.0f, -4.0f, flat);
  const std::vector<float> east = sampled(5, 9, 1.0f, 0.0f, -4.0f, flat);
  const OrblitPhysicsGround a = groundOf(1, west, 5, 9, -4.0f, -4.0f);
  const OrblitPhysicsGround b = groundOf(2, east, 5, 9, 0.0f, -4.0f);
  orblit_physics_ground(physics, &a);
  orblit_physics_ground(physics, &b);
  addCharacter(physics, 3, -3.0f, 0.0f, 0.4f);

  const int airborne = walk(physics, 3, 2.0f, 0.0f, 3.0f);
  check(near(coordinateOf(physics, 3, 0), 3.0f, 0.1f) && airborne == 0,
        "a character walks over ground and from one field onto the next");
  check(near(heightOf(physics, 3), 0.91f, 0.005f) &&
            footingOf(physics, 3).ground == 2,
        "a skin above it, and says which field it is on");
  orblit_physics_destroy(physics);

  physics = orblit_physics_create(nullptr);
  lay(physics, 1, incline);
  addCharacter(physics, 2, -3.0f, -0.9f + 0.2f, 0.3f);
  walk(physics, 2, 2.0f, 0.0f, 2.5f);
  const OrblitPhysicsFooting footing = footingOf(physics, 2);
  check(coordinateOf(physics, 2, 0) > 1.0f && footing.grounded &&
            footing.ground == 1,
        "and up a 17 degree hill");
  orblit_physics_destroy(physics);
}

void ridges() {
  // A ridge along the line between two fields, each laid with a ring of the
  // other's samples round it.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const std::vector<float> west = sampled(7, 11, 1.0f, -5.0f, -5.0f, ridge);
  const std::vector<float> east = sampled(7, 11, 1.0f, -1.0f, -5.0f, ridge);
  OrblitPhysicsGround a = groundOf(1, west, 7, 11, -5.0f, -5.0f);
  OrblitPhysicsGround b = groundOf(2, east, 7, 11, -1.0f, -5.0f);
  a.margin = true;
  b.margin = true;
  check(orblit_physics_ground(physics, &a) && orblit_physics_ground(physics, &b),
        "ground is laid with a margin");

  submit(physics, sphereAt(3, 0.5f, 0.0f, 2.0f, 0.3f));
  float deepest = 0.0f;
  for (int i = 0; i < 150; ++i) {
    orblit_physics_step(physics, kStep);
    const float x = coordinateOf(physics, 3, 0);
    deepest = std::fmax(deepest, 0.5f - (heightOf(physics, 3) - ridge(x, 0.0f)));
  }
  check(deepest < 0.05f,
        "a ball on a ridge between two fields laid with margins is held up by it");

  // The margin is never stood on: past the west field's own edge at x = 0
  // there is only the east field.
  const OrblitPhysicsCast down = rayFrom(0.5f, 5.0f, 0.0f, 0.0f, -1.0f, 0.0f, 10.0f);
  OrblitPhysicsHit hit{};
  check(orblit_physics_cast(physics, &down, &hit) && hit.body == 2,
        "and the margin is never stood on");

  orblit_physics_destroy(physics);
}

void refusingGround() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const std::vector<float> heights = sampled(9, 9, 1.0f, 0.0f, 0.0f, flat);
  const OrblitPhysicsGround good = groundOf(1, heights, 9, 9, 0.0f, 0.0f);

  OrblitPhysicsGround bad = good;
  bad.id = 0;
  bool laid = orblit_physics_ground(physics, &bad);
  bad = good;
  bad.heights = nullptr;
  laid = laid || orblit_physics_ground(physics, &bad);
  bad = good;
  bad.columns = 1;
  laid = laid || orblit_physics_ground(physics, &bad);
  bad = good;
  bad.spacing = 0.0f;
  laid = laid || orblit_physics_ground(physics, &bad);
  bad = good;
  bad.spacing = std::nanf("");
  laid = laid || orblit_physics_ground(physics, &bad);
  bad = good;
  bad.columns = 3;
  bad.margin = true;
  laid = laid || orblit_physics_ground(physics, &bad);
  laid = laid || orblit_physics_ground(physics, nullptr);
  laid = laid || orblit_physics_ground(nullptr, &good);
  check(!laid && orblit_physics_count(physics) == 0,
        "ground with nothing to stand on is refused");

  OrblitPhysicsCommand made = groundPlane(5);
  made.shape = ORBLIT_PHYSICS_HEIGHT_FIELD;
  submit(physics, made);
  check(!orblit_physics_alive(physics, 5), "and cannot be made by a command");

  orblit_physics_destroy(physics);
}

// --- joints --------------------------------------------------------------- //

/// A joint of `kind` from `a` to `b` at (x, y, z), its frame the world's.
OrblitPhysicsJoint jointOf(OrblitPhysicsId id, uint32_t kind, OrblitPhysicsId a,
                           OrblitPhysicsId b, float x, float y, float z) {
  OrblitPhysicsJoint made{};
  made.id = id;
  made.kind = kind;
  made.a = a;
  made.b = b;
  made.at[0] = x;
  made.at[1] = y;
  made.at[2] = z;
  return made;
}

/// Turns a joint's frame so its x axis points along world y: a quarter turn
/// about z.
void pointUp(OrblitPhysicsJoint &joint) {
  joint.rotation[2] = 0.70710678f;
  joint.rotation[3] = 0.70710678f;
}

/// Turns a joint's frame so its x axis points straight down.
void pointDown(OrblitPhysicsJoint &joint) {
  joint.rotation[2] = -0.70710678f;
  joint.rotation[3] = 0.70710678f;
}

/// Limits axis `which` of a joint to between `low` and `high`.
void limitTo(OrblitPhysicsJoint &joint, int which, float low, float high) {
  joint.limited |= 1u << which;
  joint.low[which] = low;
  joint.high[which] = high;
}

bool join(OrblitPhysics *physics, const OrblitPhysicsJoint &joint) {
  return orblit_physics_join(physics, &joint);
}

OrblitPhysicsJointState stateOf(OrblitPhysics *physics, OrblitPhysicsId joint) {
  OrblitPhysicsJointState out{};
  orblit_physics_joint(physics, joint, &out);
  return out;
}

void setMotion(OrblitPhysics *physics, OrblitPhysicsId id, float vx, float vy,
               float vz, float sx, float sy, float sz) {
  OrblitPhysicsCommand made{};
  made.kind = ORBLIT_PHYSICS_VELOCITY;
  made.id = id;
  made.vector[0] = vx;
  made.vector[1] = vy;
  made.vector[2] = vz;
  made.spin[0] = sx;
  made.spin[1] = sy;
  made.spin[2] = sz;
  submit(physics, made);
}

float spinOf(OrblitPhysics *physics, OrblitPhysicsId id, int axis) {
  float motion[6] = {0};
  orblit_physics_velocity(physics, id, motion);
  return motion[3 + axis];
}

float distanceBetween(OrblitPhysics *physics, OrblitPhysicsId a, OrblitPhysicsId b) {
  float x = 0.0f;
  for (int axis = 0; axis < 3; ++axis) {
    const float d = coordinateOf(physics, a, axis) - coordinateOf(physics, b, axis);
    x += d * d;
  }
  return std::sqrt(x);
}

float distanceFrom(OrblitPhysics *physics, OrblitPhysicsId id, float x, float y,
                   float z) {
  const float dx = coordinateOf(physics, id, 0) - x;
  const float dy = coordinateOf(physics, id, 1) - y;
  const float dz = coordinateOf(physics, id, 2) - z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// How far a body has turned from how it was made, in radians, if it was
/// made unturned.
float turnOf(OrblitPhysics *physics, OrblitPhysicsId id) {
  float transform[7] = {0};
  orblit_physics_transform(physics, id, transform);
  return 2.0f * std::acos(std::fmin(std::fabs(transform[6]), 1.0f));
}

/// How many events of `kind` the last step reported.
int eventsOf(OrblitPhysics *physics, uint32_t kind) {
  uint32_t count = 0;
  const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
  int found = 0;
  for (uint32_t i = 0; i < count; ++i) {
    if (events[i].kind == kind) found++;
  }
  return found;
}

void step(OrblitPhysics *physics) { orblit_physics_step(physics, kStep); }

void welding() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 1.0f, 2.0f, 0.0f));
  check(join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_FIXED, 2, 0, 0.0f, 2.0f, 0.0f)),
        "a box is welded to the world a metre to its side");
  run(physics, 2.0f);
  check(near(coordinateOf(physics, 2, 0), 1.0f, 0.01f) &&
            near(heightOf(physics, 2), 2.0f, 0.01f) && turnOf(physics, 2) < 0.01f,
        "and stays where it was welded, held out against its own weight");
  orblit_physics_destroy(physics);

  // Two boxes welded side by side and dropped: they land as one.
  physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.25f, 0.0f, 2.0f, 0.0f));
  submit(physics, boxAt(3, 0.25f, 0.5f, 2.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_FIXED, 2, 3, 0.25f, 2.0f, 0.0f));
  setMotion(physics, 2, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 2.0f);
  run(physics, 3.0f);
  const OrblitPhysicsJointState state = stateOf(physics, 1);
  check(near(distanceBetween(physics, 2, 3), 0.5f, 0.01f) &&
            std::fabs(state.angles[0]) < 0.01f && std::fabs(state.angles[1]) < 0.01f &&
            std::fabs(state.angles[2]) < 0.01f,
        "two welded boxes thrown spinning at the ground land as one");
  orblit_physics_destroy(physics);
}

void swinging() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 1.0f, 3.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 0, 0.0f, 3.0f, 0.0f));

  // Held out level and let go: it swings down and up the other side, never
  // further from the pivot than it started, and never higher.
  float stretch = 0.0f;
  float highest = -1.0f;
  float lowest = 10.0f;
  float furthest = 0.0f;
  for (int i = 0; i < 300; ++i) {
    step(physics);
    stretch = std::fmax(stretch, std::fabs(distanceFrom(physics, 2, 0.0f, 3.0f, 0.0f) - 1.0f));
    highest = std::fmax(highest, heightOf(physics, 2));
    lowest = std::fmin(lowest, heightOf(physics, 2));
    furthest = std::fmin(furthest, coordinateOf(physics, 2, 0));
  }
  check(stretch < 0.01f, "a ball on a point joint swings a metre from its pivot");
  check(near(lowest, 2.0f, 0.02f) && furthest < -0.9f,
        "down through the bottom and up the other side");
  check(highest < 3.01f, "and never climbs above where it was let go");
  orblit_physics_destroy(physics);
}

void hinging() {
  // A door, hung by its edge on an upright hinge, driven round by a motor.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand door = boxAt(2, 0.5f, 0.5f, 1.0f, 0.0f);
  door.size[1] = 1.0f;
  door.size[2] = 0.05f;
  submit(physics, door);
  OrblitPhysicsJoint hinge = jointOf(1, ORBLIT_PHYSICS_JOINT_HINGE, 2, 0, 0.0f, 1.0f, 0.0f);
  pointUp(hinge);
  hinge.speed = 1.0f;
  hinge.strength = 100.0f;
  check(join(physics, hinge), "a door is hung on a hinge");
  run(physics, 1.0f);
  OrblitPhysicsJointState state = stateOf(physics, 1);
  // The door is `a` and the world `b`, so it is the world that turns, as the
  // door sees it, and the door that turns the other way.
  check(near(state.angles[0], 1.0f, 0.05f),
        "a motor turns it at the speed asked for");
  check(near(distanceFrom(physics, 2, 0.0f, 1.0f, 0.0f), 0.5f, 0.01f) &&
            near(heightOf(physics, 2), 1.0f, 0.01f),
        "about its hinge, which it neither leaves nor sags from");
  check(std::fabs(state.offset[0]) < 0.01f && std::fabs(state.offset[1]) < 0.01f &&
            std::fabs(state.offset[2]) < 0.01f && std::fabs(state.angles[1]) < 0.01f &&
            std::fabs(state.angles[2]) < 0.01f,
        "and its state says it is on the hinge and turning only about it");
  check(state.torque > 0.0f && state.torque < 100.0f + 1.0f,
        "and how hard it held, which is no harder than the motor's strength");
  check(coordinateOf(physics, 2, 2) > 0.3f,
        "and the door turns the other way round from the angle it reads");
  orblit_physics_destroy(physics);

  // With the world as `a` it reads the door's own angle, and the same motor
  // turns it right-handed about up, which takes its far edge towards -z.
  physics = orblit_physics_create(nullptr);
  submit(physics, door);
  hinge.a = 0;
  hinge.b = 2;
  check(join(physics, hinge), "the world may be either end of a joint");
  run(physics, 1.0f);
  check(near(stateOf(physics, 1).angles[0], 1.0f, 0.05f) &&
            coordinateOf(physics, 2, 2) < -0.3f,
        "and with the world as `a`, the angle is the door's own");
  orblit_physics_destroy(physics);
  hinge.a = 2;
  hinge.b = 0;

  // The same door with a stop at half a radian each way, flung open.
  physics = orblit_physics_create(nullptr);
  submit(physics, door);
  hinge.speed = 0.0f;
  hinge.strength = 0.0f;
  limitTo(hinge, 3, -0.5f, 0.5f);
  join(physics, hinge);
  setMotion(physics, 2, 0.0f, 0.0f, 0.0f, 0.0f, -6.0f, 0.0f);
  float widest = 0.0f;
  for (int i = 0; i < 120; ++i) {
    step(physics);
    widest = std::fmax(widest, stateOf(physics, 1).angles[0]);
  }
  check(widest > 0.45f && widest < 0.53f, "a hinge with a limit stops at it");
  check(near(distanceFrom(physics, 2, 0.0f, 1.0f, 0.0f), 0.5f, 0.01f),
        "and hitting the stop does not tear it off its hinge");
  orblit_physics_destroy(physics);
}

void sliding() {
  // A box on an upright rail that lets it drop a metre.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 0.0f, 3.0f, 0.0f));
  OrblitPhysicsJoint rail = jointOf(1, ORBLIT_PHYSICS_JOINT_SLIDER, 2, 0, 0.0f, 3.0f, 0.0f);
  pointUp(rail);
  // The box is `a`: as it falls the world's point rises above it, so the
  // offset along the rail grows.
  limitTo(rail, 0, 0.0f, 1.0f);
  join(physics, rail);
  setMotion(physics, 2, 2.0f, 0.0f, -1.0f, 1.0f, 2.0f, 3.0f);
  run(physics, 2.0f);
  check(near(heightOf(physics, 2), 2.0f, 0.02f),
        "a box on a slider falls the length of its limit and stops");
  check(near(coordinateOf(physics, 2, 0), 0.0f, 0.01f) &&
            near(coordinateOf(physics, 2, 2), 0.0f, 0.01f) && turnOf(physics, 2) < 0.01f,
        "never leaving the rail or turning, however it was thrown");
  check(near(stateOf(physics, 1).offset[0], 1.0f, 0.02f), "and says how far along it is");
  orblit_physics_destroy(physics);

  // The same rail with a motor winding it back up.
  physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 0.0f, 2.0f, 0.0f));
  rail.at[1] = 2.0f;
  rail.limited = 0;
  rail.speed = -0.5f;
  rail.strength = 100.0f;
  join(physics, rail);
  run(physics, 1.0f);
  check(near(heightOf(physics, 2), 2.5f, 0.03f),
        "a slider's motor lifts it at the speed asked for");
  orblit_physics_destroy(physics);

  // Too weak to hold it up, it lets it down slowly instead.
  physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 0.0f, 2.0f, 0.0f));
  rail.speed = 0.0f;
  rail.strength = 5.0f;
  join(physics, rail);
  run(physics, 1.0f);
  const float fell = 2.0f - heightOf(physics, 2);
  check(fell > 1.0f && fell < 4.9f / 2.0f,
        "and one weaker than the weight on it gives way, but not all the way");
  orblit_physics_destroy(physics);
}

void distances() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 0.0f, 3.0f, 0.0f));
  submit(physics, sphereAt(3, 0.1f, 2.0f, 3.0f, 0.0f));
  OrblitPhysicsJoint rod = jointOf(1, ORBLIT_PHYSICS_JOINT_DISTANCE, 2, 3, 0.0f, 3.0f, 0.0f);
  rod.to[0] = 2.0f;
  rod.to[1] = 3.0f;
  join(physics, rod);
  setMotion(physics, 3, 0.0f, 4.0f, 3.0f, 0.0f, 0.0f, 0.0f);
  float stretch = 0.0f;
  for (int i = 0; i < 120; ++i) {
    step(physics);
    stretch = std::fmax(stretch, std::fabs(distanceBetween(physics, 2, 3) - 2.0f));
  }
  check(stretch < 0.01f, "two balls on a rod stay its length apart however they are thrown");
  check(near(stateOf(physics, 1).offset[0], 2.0f, 0.01f), "and a rod's state is its length");
  orblit_physics_destroy(physics);

  // A rope two metres long, tied a metre above a ball: slack, then taut.
  physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 0.0f, 3.0f, 0.0f));
  OrblitPhysicsJoint rope = jointOf(1, ORBLIT_PHYSICS_JOINT_DISTANCE, 2, 0, 0.0f, 3.0f, 0.0f);
  rope.to[1] = 4.0f;
  limitTo(rope, 0, 0.0f, 2.0f);
  join(physics, rope);
  run(physics, 0.3f);
  check(heightOf(physics, 2) < 2.6f, "a ball on a slack rope falls freely");
  run(physics, 3.0f);
  check(near(heightOf(physics, 2), 2.0f, 0.02f), "until the rope is taut, and hangs from it");
  setMotion(physics, 2, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  run(physics, 0.2f);
  check(heightOf(physics, 2) > 2.3f, "and a rope does not push: thrown up, it rises");
  orblit_physics_destroy(physics);
}

void cones() {
  // An arm: a capsule hanging from a shoulder, swung hard sideways.
  float widest[2] = {0.0f, 0.0f};
  for (int limited = 0; limited < 2; ++limited) {
    OrblitPhysics *physics = orblit_physics_create(nullptr);
    submit(physics, capsuleAt(2, 0.1f, 0.4f, 0.0f, 2.0f, 0.0f));
    OrblitPhysicsJoint shoulder = jointOf(
        1, limited ? ORBLIT_PHYSICS_JOINT_CONE : ORBLIT_PHYSICS_JOINT_POINT, 2, 0,
        0.0f, 2.5f, 0.0f);
    pointDown(shoulder);
    shoulder.swing = 0.5f;
    join(physics, shoulder);
    setMotion(physics, 2, 4.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 180; ++i) {
      step(physics);
      const OrblitPhysicsJointState state = stateOf(physics, 1);
      const float swung =
          std::sqrt(state.angles[1] * state.angles[1] + state.angles[2] * state.angles[2]);
      widest[limited] = std::fmax(widest[limited], swung);
    }
    if (limited) {
      check(distanceFrom(physics, 2, 0.0f, 2.5f, 0.0f) < 0.51f,
            "and the arm stays on its shoulder");
    }
    orblit_physics_destroy(physics);
  }
  check(widest[0] > 1.0f, "an arm on a point joint swings as far as it is thrown");
  check(widest[1] > 0.45f && widest[1] < 0.55f,
        "and on a cone, no further than the cone, whichever way it is thrown");

  // A cone that also stops it twisting.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, capsuleAt(2, 0.1f, 0.4f, 0.0f, 2.0f, 0.0f));
  OrblitPhysicsJoint shoulder = jointOf(1, ORBLIT_PHYSICS_JOINT_CONE, 2, 0, 0.0f, 2.5f, 0.0f);
  pointDown(shoulder);
  shoulder.swing = 0.5f;
  limitTo(shoulder, 3, -0.2f, 0.2f);
  join(physics, shoulder);
  setMotion(physics, 2, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f, 0.0f);
  float twisted = 0.0f;
  for (int i = 0; i < 60; ++i) {
    step(physics);
    twisted = std::fmax(twisted, std::fabs(stateOf(physics, 1).angles[0]));
  }
  check(twisted < 0.23f, "a cone with a twist limit stops the arm twisting past it");
  orblit_physics_destroy(physics);
}

void sixAxes() {
  // Locked along x and z, a half-metre of travel along y, locked about x and
  // z, and free to turn about y: nothing the other kinds do. Turning about y
  // leaves `a`'s y axis where it is, so the travel stays upright.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 0.0f, 3.0f, 0.0f));
  OrblitPhysicsJoint joint = jointOf(1, ORBLIT_PHYSICS_JOINT_SIX_AXIS, 2, 0, 0.0f, 3.0f, 0.0f);
  limitTo(joint, 0, 0.0f, 0.0f);
  limitTo(joint, 1, 0.0f, 0.5f);
  limitTo(joint, 2, 0.0f, 0.0f);
  limitTo(joint, 3, 0.0f, 0.0f);
  limitTo(joint, 5, 0.0f, 0.0f);
  join(physics, joint);
  setMotion(physics, 2, 1.0f, 0.0f, 1.0f, 0.5f, 1.0f, 0.5f);
  run(physics, 1.0f);
  check(near(heightOf(physics, 2), 2.5f, 0.02f) && near(coordinateOf(physics, 2, 0), 0.0f, 0.01f) &&
            near(coordinateOf(physics, 2, 2), 0.0f, 0.01f),
        "a six-axis joint limits each way on its own");
  check(spinOf(physics, 2, 1) > 0.9f && std::fabs(spinOf(physics, 2, 0)) < 0.01f &&
            std::fabs(spinOf(physics, 2, 2)) < 0.01f,
        "and leaves free the one axis it does not limit");
  orblit_physics_destroy(physics);
}

void colliding() {
  // Two overlapping boxes joined where they overlap, as a limb's two halves
  // are at the elbow.
  bool touched[2] = {false, false};
  for (int collide = 0; collide < 2; ++collide) {
    OrblitPhysics *physics = orblit_physics_create(nullptr);
    submit(physics, boxAt(2, 0.25f, 0.0f, 3.0f, 0.0f));
    submit(physics, boxAt(3, 0.25f, 0.3f, 3.0f, 0.0f));
    OrblitPhysicsJoint joint = jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 3, 0.15f, 3.0f, 0.0f);
    joint.collide = collide != 0;
    join(physics, joint);
    for (int i = 0; i < 30; ++i) {
      step(physics);
      if (eventsOf(physics, ORBLIT_PHYSICS_TOUCH_BEGAN) > 0) touched[collide] = true;
    }
    orblit_physics_destroy(physics);
  }
  check(!touched[0], "two joined bodies do not collide with each other");
  check(touched[1], "unless the joint asks them to");
}

void sleepingJoined() {
  // A chain of three hanging from the world, damped so it settles soon.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  for (OrblitPhysicsId link = 2; link <= 4; ++link) {
    OrblitPhysicsCommand made =
        sphereAt(link, 0.1f, 0.0f, 3.0f - 0.5f * static_cast<float>(link - 1), 0.0f);
    made.damping[0] = 2.0f;
    made.damping[1] = 2.0f;
    submit(physics, made);
  }
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 0, 0.0f, 3.0f, 0.0f));
  join(physics, jointOf(2, ORBLIT_PHYSICS_JOINT_POINT, 3, 2, 0.0f, 2.5f, 0.0f));
  join(physics, jointOf(3, ORBLIT_PHYSICS_JOINT_POINT, 4, 3, 0.0f, 2.0f, 0.0f));
  setMotion(physics, 4, 0.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  int together = 0;
  int apart = 0;
  for (int i = 0; i < 1200 && !orblit_physics_asleep(physics, 2); ++i) {
    step(physics);
    const int slept = eventsOf(physics, ORBLIT_PHYSICS_SLEPT);
    if (slept == 3) together++;
    if (slept > 0 && slept < 3) apart++;
  }
  check(together == 1 && apart == 0, "a hanging chain falls asleep all at once, not a link at a time");

  OrblitPhysicsCommand wake{};
  wake.kind = ORBLIT_PHYSICS_WAKE;
  wake.id = 4;
  submit(physics, wake);
  check(!orblit_physics_asleep(physics, 2) && !orblit_physics_asleep(physics, 3) &&
            !orblit_physics_asleep(physics, 4),
        "and waking its end wakes all of it");
  orblit_physics_destroy(physics);

  // A ball dropped on the end of a sleeping chain wakes the chain.
  physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 0.0f, 2.5f, 0.0f));
  submit(physics, sphereAt(3, 0.1f, 0.0f, 2.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 0, 0.0f, 3.0f, 0.0f));
  join(physics, jointOf(2, ORBLIT_PHYSICS_JOINT_POINT, 3, 2, 0.0f, 2.5f, 0.0f));
  run(physics, 2.0f);
  const bool slept = orblit_physics_asleep(physics, 2) && orblit_physics_asleep(physics, 3);
  submit(physics, sphereAt(5, 0.1f, 0.0f, 1.8f, 0.0f));
  step(physics);
  step(physics);
  check(slept && !orblit_physics_asleep(physics, 2), "and so does something touching it");
  orblit_physics_destroy(physics);

  // A ball hanging from a lift that rises too slowly for it to seem to move.
  physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand lift = boxAt(2, 0.25f, 0.0f, 3.0f, 0.0f);
  lift.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, lift);
  submit(physics, sphereAt(3, 0.1f, 0.0f, 2.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 3, 2, 0.0f, 2.75f, 0.0f));
  run(physics, 2.0f);
  const bool restedFirst = orblit_physics_asleep(physics, 3);
  drive(physics, 2, 0.0f, 0.02f, 0.0f, 0.0f);
  check(restedFirst && !orblit_physics_asleep(physics, 3),
        "setting a body going wakes what hangs from it");
  run(physics, 5.0f);
  check(!orblit_physics_asleep(physics, 3) && near(heightOf(physics, 3), 2.1f, 0.01f),
        "and it does not sleep while what it hangs from is moving");
  orblit_physics_destroy(physics);
}

void letGo() {
  // A ball hung from a beam, which is then taken away.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, slabAt(2, 1.0f, 0.1f, 0.1f, 0.0f, 3.0f, 0.0f));
  submit(physics, sphereAt(3, 0.1f, 0.0f, 2.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 3, 2, 0.0f, 3.0f, 0.0f));
  run(physics, 2.0f);
  const bool hung = orblit_physics_asleep(physics, 3) && near(heightOf(physics, 3), 2.0f, 0.01f);
  OrblitPhysicsCommand gone{};
  gone.kind = ORBLIT_PHYSICS_DESTROY;
  gone.id = 2;
  submit(physics, gone);
  OrblitPhysicsJointState state{};
  check(hung && !orblit_physics_joint(physics, 1, &state) && !orblit_physics_asleep(physics, 3),
        "destroying a body removes its joints and wakes what was joined to it");
  step(physics);
  check(eventsOf(physics, ORBLIT_PHYSICS_BROKE) == 0, "without saying anything broke");
  run(physics, 0.5f);
  check(heightOf(physics, 3) < 1.0f, "and what hung from it falls");
  check(!orblit_physics_unjoin(physics, 1), "and the joint cannot be removed again");
  orblit_physics_destroy(physics);

  // The same, let go by removing the joint instead.
  physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(3, 0.1f, 0.0f, 2.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 3, 0, 0.0f, 3.0f, 0.0f));
  run(physics, 2.0f);
  check(orblit_physics_unjoin(physics, 1) && !orblit_physics_asleep(physics, 3),
        "removing a joint wakes its bodies");
  run(physics, 0.5f);
  check(heightOf(physics, 3) < 1.0f, "so what it held up falls");
  orblit_physics_destroy(physics);

  // Ground laid again under the same id keeps what is tied to it.
  physics = orblit_physics_create(nullptr);
  lay(physics, 1, flat);
  submit(physics, sphereAt(3, 0.1f, 0.0f, 1.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_DISTANCE, 3, 1, 0.0f, 1.0f, 0.0f));
  lay(physics, 1, flat);
  run(physics, 1.0f);
  check(orblit_physics_joint(physics, 1, &state) && near(heightOf(physics, 3), 1.0f, 0.02f),
        "laying ground again keeps the joints tied to it");
  orblit_physics_destroy(physics);
}

void breaking() {
  // Tied with a thread that takes five newtons, holding a kilogram.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 0.0f, 2.0f, 0.0f));
  OrblitPhysicsJoint thread = jointOf(7, ORBLIT_PHYSICS_JOINT_POINT, 2, 0, 0.0f, 2.1f, 0.0f);
  thread.breakingForce = 5.0f;
  join(physics, thread);
  step(physics);
  uint32_t count = 0;
  const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
  const bool said = count == 1 && events[0].kind == ORBLIT_PHYSICS_BROKE &&
                    events[0].a == 7 && events[0].b == 0 && events[0].force > 5.0f &&
                    near(events[0].at[1], 2.1f, 0.01f);
  OrblitPhysicsJointState state{};
  check(said && !orblit_physics_joint(physics, 7, &state),
        "a joint pulled harder than it can take breaks, and says which and how hard");
  run(physics, 0.5f);
  check(heightOf(physics, 2) < 1.0f, "and lets go of what it held");
  orblit_physics_destroy(physics);

  // A stronger one holds the weight, then breaks when the ball is yanked.
  physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 0.0f, 2.0f, 0.0f));
  thread.breakingForce = 20.0f;
  join(physics, thread);
  run(physics, 1.0f);
  state = stateOf(physics, 7);
  check(orblit_physics_joint(physics, 7, &state) && near(state.force, 9.81f, 0.2f),
        "one that can take the weight holds it, and says what it is holding");
  OrblitPhysicsCommand yank{};
  yank.kind = ORBLIT_PHYSICS_IMPULSE;
  yank.id = 2;
  yank.vector[1] = -2.0f;
  yank.spin[1] = 2.0f;
  submit(physics, yank);
  step(physics);
  check(eventsOf(physics, ORBLIT_PHYSICS_BROKE) == 1, "until it is yanked");
  orblit_physics_destroy(physics);

  // A shelf welded to a wall, which turns rather than pulls.
  physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 1.0f, 2.0f, 0.0f));
  OrblitPhysicsJoint weld = jointOf(3, ORBLIT_PHYSICS_JOINT_FIXED, 2, 0, 0.0f, 2.0f, 0.0f);
  weld.breakingTorque = 5.0f;
  join(physics, weld);
  bool broke = false;
  for (int i = 0; i < 30 && !broke; ++i) {
    step(physics);
    events = orblit_physics_events(physics, &count);
    broke = count == 1 && events[0].kind == ORBLIT_PHYSICS_BROKE && events[0].force > 5.0f;
  }
  check(broke, "a weld breaks on torque too, and says what torque");
  orblit_physics_destroy(physics);
}

void refusingJoints() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(2, 0.1f, 0.0f, 2.0f, 0.0f));
  submit(physics, sphereAt(3, 0.1f, 1.0f, 2.0f, 0.0f));
  const OrblitPhysicsJoint good = jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 3, 0.5f, 2.0f, 0.0f);

  OrblitPhysicsJoint bad = good;
  bad.id = 0;
  bool made = join(physics, bad);
  bad = good;
  bad.a = 0;
  bad.b = 0;
  made = made || join(physics, bad);
  bad = good;
  bad.a = 9;
  made = made || join(physics, bad);
  bad = good;
  bad.b = 9;
  made = made || join(physics, bad);
  bad = good;
  bad.b = 2;
  made = made || join(physics, bad);
  bad = good;
  bad.kind = 0;
  made = made || join(physics, bad);
  bad = good;
  bad.kind = 8;
  made = made || join(physics, bad);
  bad = good;
  bad.at[0] = std::nanf("");
  made = made || join(physics, bad);
  bad = good;
  bad.high[3] = std::nanf("");
  made = made || join(physics, bad);
  bad = good;
  bad.rotation[1] = INFINITY;
  made = made || join(physics, bad);
  made = made || orblit_physics_join(physics, nullptr);
  made = made || orblit_physics_join(nullptr, &good);
  OrblitPhysicsJointState state{};
  check(!made && !orblit_physics_joint(physics, 1, &state), "a joint that cannot be made is refused");

  check(join(physics, good) && !join(physics, good), "and so is one made twice");
  bad = good;
  bad.id = 2;
  check(join(physics, bad), "but a joint may share a number with a body");
  check(!orblit_physics_unjoin(physics, 9) && !orblit_physics_joint(physics, 9, &state) &&
            !orblit_physics_joint(physics, 1, nullptr) && !orblit_physics_unjoin(nullptr, 1),
        "and a joint that is not there cannot be read or removed");
  orblit_physics_destroy(physics);
}

void ragdoll() {
  // A figure of capsules and a ball, jointed the way a body is, thrown
  // tumbling at the ground.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));

  const float lift = 2.0f;
  OrblitPhysicsCommand torso = capsuleAt(2, 0.15f, 0.25f, 0.0f, lift + 1.2f, 0.0f);
  torso.mass = 20.0f;
  submit(physics, torso);
  OrblitPhysicsCommand head = sphereAt(3, 0.12f, 0.0f, lift + 1.72f, 0.0f);
  head.mass = 4.0f;
  submit(physics, head);

  struct Limb {
    OrblitPhysicsId upper;
    OrblitPhysicsId lower;
    float x;
    float top;
    float length;
  };
  // Arms from the shoulders, legs from the hips, each two capsules.
  const Limb limbs[] = {
      {4, 5, -0.3f, lift + 1.5f, 0.3f},
      {6, 7, 0.3f, lift + 1.5f, 0.3f},
      {8, 9, -0.12f, lift + 0.8f, 0.4f},
      {10, 11, 0.12f, lift + 0.8f, 0.4f},
  };
  OrblitPhysicsId joint = 1;
  bool made = join(physics, [&] {
    OrblitPhysicsJoint neck = jointOf(joint++, ORBLIT_PHYSICS_JOINT_CONE, 3, 2, 0.0f, lift + 1.6f, 0.0f);
    pointDown(neck);
    neck.swing = 0.6f;
    limitTo(neck, 3, -0.8f, 0.8f);
    return neck;
  }());
  for (const Limb &limb : limbs) {
    const float half = limb.length / 2.0f;
    OrblitPhysicsCommand upper = capsuleAt(limb.upper, 0.06f, half - 0.06f, limb.x, limb.top - half, 0.0f);
    upper.mass = 3.0f;
    submit(physics, upper);
    OrblitPhysicsCommand lower = capsuleAt(limb.lower, 0.05f, half - 0.05f, limb.x,
                                           limb.top - limb.length - half, 0.0f);
    lower.mass = 2.0f;
    submit(physics, lower);

    OrblitPhysicsJoint socket = jointOf(joint++, ORBLIT_PHYSICS_JOINT_CONE, limb.upper, 2, limb.x, limb.top, 0.0f);
    pointDown(socket);
    socket.swing = 1.2f;
    limitTo(socket, 3, -0.5f, 0.5f);
    made = join(physics, socket) && made;

    OrblitPhysicsJoint bend = jointOf(joint++, ORBLIT_PHYSICS_JOINT_HINGE, limb.lower, limb.upper,
                                      limb.x, limb.top - limb.length, 0.0f);
    // About x, which for a figure facing +z is the way an elbow or a knee
    // bends, and only one way.
    limitTo(bend, 3, 0.0f, 2.2f);
    made = join(physics, bend) && made;
  }
  check(made, "a ragdoll is jointed together");

  setMotion(physics, 2, 1.0f, 2.0f, 0.5f, 3.0f, 1.0f, 4.0f);

  const OrblitPhysicsId last = 11;
  const OrblitPhysicsId joints = joint - 1;
  bool finite = true;
  float highest = 0.0f;
  float torn = 0.0f;
  for (int i = 0; i < 600; ++i) {
    step(physics);
    for (OrblitPhysicsId body = 2; body <= last; ++body) {
      float transform[7] = {0};
      orblit_physics_transform(physics, body, transform);
      for (const float v : transform) finite = finite && std::isfinite(v);
      highest = std::fmax(highest, transform[1]);
    }
    for (OrblitPhysicsId id = 1; id <= joints; ++id) {
      const OrblitPhysicsJointState state = stateOf(physics, id);
      torn = std::fmax(torn, std::sqrt(state.offset[0] * state.offset[0] +
                                       state.offset[1] * state.offset[1] +
                                       state.offset[2] * state.offset[2]));
    }
  }
  check(finite, "and thrown at the ground, stays finite");
  check(highest < lift + 3.0f, "does not explode");
  check(torn < 0.05f, "and does not come apart at any joint");

  bool resting = true;
  bool above = true;
  for (OrblitPhysicsId body = 2; body <= last; ++body) {
    resting = resting && (orblit_physics_asleep(physics, body) || speedOf(physics, body) < 0.05f);
    above = above && heightOf(physics, body) > 0.0f;
  }
  check(above, "comes to rest on the ground, not in it");
  check(resting, "and settles there rather than twitching");
  orblit_physics_destroy(physics);
}

void chains() {
  // Twelve links held out level from a wall and dropped, damped enough that
  // they come to hang within the test.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const int links = 12;
  for (int i = 0; i < links; ++i) {
    OrblitPhysicsCommand made = capsuleAt(10 + i, 0.05f, 0.1f, 0.15f + 0.3f * i, 5.0f, 0.0f);
    made.damping[0] = 0.5f;
    made.damping[1] = 0.5f;
    submit(physics, made);
  }
  // The links lie along x, and a capsule stands along y, so turn each over.
  for (int i = 0; i < links; ++i) {
    OrblitPhysicsCommand turn{};
    turn.kind = ORBLIT_PHYSICS_PLACE;
    turn.id = 10 + i;
    turn.at[0] = 0.15f + 0.3f * i;
    turn.at[1] = 5.0f;
    layDown(turn);
    submit(physics, turn);
  }
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 10, 0, 0.0f, 5.0f, 0.0f));
  for (int i = 1; i < links; ++i) {
    join(physics, jointOf(1 + i, ORBLIT_PHYSICS_JOINT_POINT, 10 + i, 9 + i, 0.3f * i, 5.0f, 0.0f));
  }
  float stretch = 0.0f;
  for (int i = 0; i < 900; ++i) {
    step(physics);
    for (int j = 1; j <= links; ++j) {
      const OrblitPhysicsJointState state = stateOf(physics, j);
      stretch = std::fmax(stretch, std::sqrt(state.offset[0] * state.offset[0] +
                                             state.offset[1] * state.offset[1] +
                                             state.offset[2] * state.offset[2]));
    }
  }
  check(stretch < 0.03f, "a chain dropped from level stays together at every link");
  // The last link's middle is three and a half links from the wall.
  check(near(heightOf(physics, 10 + links - 1), 5.0f - 3.45f, 0.03f),
        "and comes to hang its full length below where it was tied");
  orblit_physics_destroy(physics);
}

} // namespace

int main() {
  settling();
  capsules();
  casting();
  touching();
  bouncing();
  rubbing();
  stacking();
  layering();
  shoving();
  driving();
  reading();
  refusing();
  walking();
  climbing();
  blocking();
  slopes();
  riding();
  turning();
  pushing();
  crowding();
  footings();
  resting();
  rolling();
  creases();
  holes();
  castingGround();
  reshaping();
  hiking();
  ridges();
  refusingGround();
  welding();
  swinging();
  hinging();
  sliding();
  distances();
  cones();
  sixAxes();
  colliding();
  sleepingJoined();
  letGo();
  breaking();
  refusingJoints();
  ragdoll();
  chains();

  std::printf(failures == 0 ? "\nALL PASSED\n" : "\n%d FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
