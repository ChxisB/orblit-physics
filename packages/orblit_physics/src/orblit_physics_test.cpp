// A standalone check of the engine through its own ABI, so a solver bug is
// found here rather than through two language bindings. Built by
// tool/check_native.sh, not by the package — Dart's tests cover the same
// ground from the other side of the hook.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <random>
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

OrblitPhysicsCommand cylinderAt(OrblitPhysicsId id, float radius,
                                float halfHeight, float x, float y, float z) {
  OrblitPhysicsCommand made = sphereAt(id, radius, x, y, z);
  made.shape = ORBLIT_PHYSICS_CYLINDER;
  made.size[0] = radius;
  made.size[1] = halfHeight;
  return made;
}

/// A body made from a hull that `layHull` has laid. The size is the shape's
/// own business, so none is given.
OrblitPhysicsCommand hullAt(OrblitPhysicsId id, OrblitPhysicsId hull, float x,
                            float y, float z) {
  OrblitPhysicsCommand made = sphereAt(id, 0.0f, x, y, z);
  made.shape = ORBLIT_PHYSICS_HULL;
  made.hull = hull;
  return made;
}

/// The eight corners of the box from `low` to `high`, as the triples a hull is
/// cooked from.
std::vector<float> cornersOf(const float low[3], const float high[3]) {
  std::vector<float> points;
  for (int corner = 0; corner < 8; ++corner) {
    points.push_back(corner & 1 ? high[0] : low[0]);
    points.push_back(corner & 2 ? high[1] : low[1]);
    points.push_back(corner & 4 ? high[2] : low[2]);
  }
  return points;
}

/// The corners of a cube of side `2 * half` about the origin.
std::vector<float> cubeOf(float half) {
  const float low[3] = {-half, -half, -half};
  const float high[3] = {half, half, half};
  return cornersOf(low, high);
}

bool layHull(OrblitPhysics *physics, OrblitPhysicsId id,
             const std::vector<float> &points) {
  return orblit_physics_hull(physics, id, points.data(),
                             static_cast<uint32_t>(points.size() / 3));
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

// --- asking the world ------------------------------------------------------ //
//
// Triggers, zones, belts and per-pair rules, and the queries that go with
// them: what a game asks of a world beyond where everything is now.

OrblitPhysicsCommand triggerAt(OrblitPhysicsId id, float half, float x, float y,
                               float z) {
  OrblitPhysicsCommand made = boxAt(id, half, x, y, z);
  made.motion = ORBLIT_PHYSICS_STATIC;
  made.sensor = true;
  return made;
}

/// A query for whatever contains one point: a ray's shape is a sphere of no
/// size, cast nowhere.
OrblitPhysicsCast pointAt(float x, float y, float z) {
  return rayFrom(x, y, z, 0.0f, 0.0f, 0.0f, 0.0f);
}

/// Steps a world and keeps every event it reported, since a step drops the
/// ones before it.
struct Watch {
  explicit Watch(OrblitPhysics *world) : physics(world) {}

  void collect() {
    uint32_t count = 0;
    const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
    seen.insert(seen.end(), events, events + count);
  }

  void run(float seconds) {
    const int steps = static_cast<int>(seconds / kStep + 0.5f);
    for (int i = 0; i < steps; ++i) {
      orblit_physics_step(physics, kStep);
      collect();
    }
  }

  /// How many events of `kind` named exactly `a` then `b`.
  int count(uint32_t kind, OrblitPhysicsId a, OrblitPhysicsId b) const {
    int found = 0;
    for (const OrblitPhysicsEvent &event : seen) {
      if (event.kind == kind && event.a == a && event.b == b) found++;
    }
    return found;
  }

  OrblitPhysics *physics;
  std::vector<OrblitPhysicsEvent> seen;
};

std::vector<OrblitPhysicsId> overlapping(OrblitPhysics *physics,
                                         const OrblitPhysicsCast &query,
                                         uint32_t capacity) {
  std::vector<OrblitPhysicsId> ids(capacity);
  ids.resize(orblit_physics_overlap(physics, &query, ids.data(), capacity));
  return ids;
}

std::vector<OrblitPhysicsHit> hitsAlong(OrblitPhysics *physics,
                                        const OrblitPhysicsCast &query,
                                        uint32_t capacity) {
  std::vector<OrblitPhysicsHit> hits(capacity);
  hits.resize(orblit_physics_cast_all(physics, &query, hits.data(), capacity));
  return hits;
}

bool holds(const std::vector<OrblitPhysicsId> &ids,
           std::initializer_list<OrblitPhysicsId> wanted) {
  return ids == std::vector<OrblitPhysicsId>(wanted);
}

void surfaceOf(OrblitPhysics *physics, OrblitPhysicsId id, float along, float up) {
  OrblitPhysicsCommand set{};
  set.kind = ORBLIT_PHYSICS_SURFACE;
  set.id = id;
  set.vector[0] = along;
  set.vector[1] = up;
  submit(physics, set);
}

void destroyBody(OrblitPhysics *physics, OrblitPhysicsId id) {
  OrblitPhysicsCommand gone{};
  gone.kind = ORBLIT_PHYSICS_DESTROY;
  gone.id = id;
  submit(physics, gone);
}

void triggering() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand floor = groundPlane(9);
  floor.size[3] = -6.0f; // Surface at y = -6.
  submit(physics, floor);
  submit(physics, triggerAt(1, 1.0f, 0.0f, 0.0f, 0.0f));
  submit(physics, sphereAt(2, 0.25f, 0.0f, 3.0f, 0.0f));

  Watch watch(physics);
  watch.run(2.0f);
  check(watch.count(ORBLIT_PHYSICS_ENTERED, 1, 2) == 1,
        "a body falling into a trigger reports entering it once, trigger first");
  check(watch.count(ORBLIT_PHYSICS_EXITED, 1, 2) == 1, "and leaving it once");
  check(watch.count(ORBLIT_PHYSICS_TOUCH_BEGAN, 1, 2) == 0,
        "a trigger is overlapped and never touched");
  check(near(heightOf(physics, 2), -5.75f, 0.1f),
        "and it pushes nothing: the body falls straight through to the floor");

  // A trigger is a fact about where things are, so it holds for a body that
  // is asleep, and does not wake it.
  OrblitPhysicsCommand sleeper = sphereAt(3, 0.25f, 0.0f, 0.0f, 0.0f);
  sleeper.asleep = true;
  submit(physics, sleeper);
  submit(physics, slabAt(4, 0.25f, 0.25f, 0.25f, 0.5f, 0.0f, 0.0f));
  watch.run(0.2f);
  check(watch.count(ORBLIT_PHYSICS_ENTERED, 1, 3) == 1,
        "a body created asleep inside a trigger is reported in it");
  check(orblit_physics_asleep(physics, 3), "and stays asleep");
  check(watch.count(ORBLIT_PHYSICS_ENTERED, 1, 4) == 0,
        "a static body in a trigger is not reported, since nothing about it changes");

  destroyBody(physics, 1);
  watch.run(0.1f);
  check(watch.count(ORBLIT_PHYSICS_EXITED, 1, 3) == 1,
        "destroying a trigger reports everything in it leaving");
  orblit_physics_destroy(physics);
}

void triggeringCharacters() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, triggerAt(2, 0.5f, 1.5f, 1.0f, 0.0f));
  addCharacter(physics, 3, 0.0f, 0.0f, 0.0f);

  // Walked by hand rather than with `walk`, which steps past the events.
  Watch watch(physics);
  for (int i = 0; i < 90; ++i) {
    OrblitPhysicsCommand ask{};
    ask.kind = ORBLIT_PHYSICS_VELOCITY;
    ask.id = 3;
    ask.vector[0] = 2.0f;
    ask.vector[1] = footingOf(physics, 3).velocity[1] - 9.81f * kStep;
    submit(physics, ask);
    orblit_physics_step(physics, kStep);
    watch.collect();
  }
  check(coordinateOf(physics, 3, 0) > 2.5f,
        "a character walks straight through a trigger in its way");
  check(watch.count(ORBLIT_PHYSICS_ENTERED, 2, 3) == 1 &&
            watch.count(ORBLIT_PHYSICS_EXITED, 2, 3) == 1,
        "and sets it off, which is what a trigger is for");
  orblit_physics_destroy(physics);
}

void staying() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  OrblitPhysicsCommand resting = boxAt(2, 0.5f, 0.0f, 0.49f, 0.0f);
  resting.stay = true;
  submit(physics, resting);
  submit(physics, boxAt(3, 0.5f, 5.0f, 0.49f, 0.0f));

  Watch watch(physics);
  watch.run(0.3f);
  check(watch.count(ORBLIT_PHYSICS_TOUCH_BEGAN, 1, 2) == 1, "a touch begins once");
  check(watch.count(ORBLIT_PHYSICS_TOUCH_STAY, 1, 2) >= 15,
        "and a body that asks is told every step it goes on");
  check(watch.count(ORBLIT_PHYSICS_TOUCH_ENDED, 1, 2) == 0, "without it ending");
  check(watch.count(ORBLIT_PHYSICS_TOUCH_STAY, 1, 3) == 0,
        "one that has not asked is only told when it begins and ends");
  orblit_physics_destroy(physics);
}

void stayingInside() {
  // `stay` on a trigger asks for everything in it. On a body it asks for every
  // trigger that body is in.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand asking = triggerAt(1, 1.0f, 0.0f, 0.0f, 0.0f);
  asking.stay = true;
  submit(physics, asking);
  submit(physics, triggerAt(2, 1.0f, 0.0f, 0.0f, 0.0f));

  OrblitPhysicsCommand keen = sphereAt(3, 0.25f, 0.0f, 0.0f, 0.0f);
  keen.motion = ORBLIT_PHYSICS_KINEMATIC;
  keen.stay = true;
  submit(physics, keen);
  OrblitPhysicsCommand quiet = sphereAt(4, 0.25f, 0.5f, 0.0f, 0.0f);
  quiet.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, quiet);

  Watch watch(physics);
  watch.run(0.2f);
  check(watch.count(ORBLIT_PHYSICS_INSIDE, 1, 3) >= 10 &&
            watch.count(ORBLIT_PHYSICS_INSIDE, 1, 4) >= 10,
        "a trigger that asks is told every step of everything inside it");
  check(watch.count(ORBLIT_PHYSICS_INSIDE, 2, 3) >= 10,
        "and so is one holding a body that asks");
  check(watch.count(ORBLIT_PHYSICS_ENTERED, 2, 4) == 1 &&
            watch.count(ORBLIT_PHYSICS_INSIDE, 2, 4) == 0,
        "while neither asking is told when things enter and leave only");
  orblit_physics_destroy(physics);
}

OrblitPhysicsZone zoneOn(OrblitPhysicsId body, uint32_t overrides, int32_t priority,
                         float gravity) {
  OrblitPhysicsZone made{};
  made.body = body;
  made.overrides = overrides;
  made.priority = priority;
  made.gravity[1] = gravity;
  return made;
}

struct Outcome {
  float height;
  float speed;
};

/// A sphere hung at rest ten metres up and thrown along x at `throwing`, inside
/// each of `zones`. The zones are all cubes forty metres across, so the sphere
/// never leaves one in the second it is watched for.
Outcome inZones(const std::vector<OrblitPhysicsZone> &zones, float throwing) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  for (const OrblitPhysicsZone &zone : zones) {
    submit(physics, triggerAt(zone.body, 20.0f, 0.0f, 10.0f, 0.0f));
    orblit_physics_zone(physics, &zone);
  }
  submit(physics, sphereAt(100, 0.25f, 0.0f, 10.0f, 0.0f));
  drive(physics, 100, throwing, 0.0f, 0.0f, 0.0f);
  run(physics, 1.0f);
  float motion[6] = {0};
  orblit_physics_velocity(physics, 100, motion);
  const Outcome outcome{heightOf(physics, 100), motion[0]};
  orblit_physics_destroy(physics);
  return outcome;
}

void zoning() {
  const uint32_t gravity = ORBLIT_PHYSICS_ZONE_GRAVITY;

  check(inZones({}, 0.0f).height < 6.0f, "with no zone a sphere falls");
  check(near(inZones({zoneOn(1, gravity, 0, 0.0f)}, 0.0f).height, 10.0f, 0.05f),
        "a zone with no gravity holds it where it is");
  check(inZones({zoneOn(1, gravity, 0, 9.81f)}, 0.0f).height > 12.0f,
        "and one with gravity upwards lifts it");

  check(inZones({zoneOn(1, gravity, 0, 0.0f), zoneOn(2, gravity, 5, 9.81f)}, 0.0f)
                .height > 12.0f,
        "where zones overlap, the higher priority speaks");
  check(inZones({zoneOn(1, gravity, 5, 9.81f), zoneOn(2, gravity, 0, 0.0f)}, 0.0f)
                .height > 12.0f,
        "whichever was made first");
  check(inZones({zoneOn(2, gravity, 3, -9.81f), zoneOn(1, gravity, 3, 9.81f)}, 0.0f)
                .height > 12.0f,
        "and at equal priority the lower id does, whichever was made first");

  const uint32_t damping = ORBLIT_PHYSICS_ZONE_LINEAR_DAMPING;
  OrblitPhysicsZone thick = zoneOn(2, damping, 9, 0.0f);
  thick.damping[0] = 5.0f;
  check(inZones({zoneOn(1, gravity, 0, 0.0f)}, 5.0f).speed > 4.5f,
        "a zone that holds only gravity leaves damping as it was");
  const Outcome both = inZones({zoneOn(1, gravity, 0, 0.0f), thick}, 5.0f);
  check(near(both.height, 10.0f, 0.05f) && both.speed < 0.5f,
        "and each field takes its answer from its own zone");
}

void zoneEdits() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, triggerAt(1, 20.0f, 0.0f, 10.0f, 0.0f));
  OrblitPhysicsCommand napping = sphereAt(2, 0.25f, 0.0f, 10.0f, 0.0f);
  napping.asleep = true;
  submit(physics, napping);
  OrblitPhysicsZone lift = zoneOn(1, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, 9.81f);
  check(orblit_physics_zone(physics, &lift), "a zone is set on a trigger");

  run(physics, 1.0f);
  float motion[6] = {0};
  orblit_physics_velocity(physics, 2, motion);
  check(heightOf(physics, 2) > 12.0f && motion[1] > 8.0f,
        "a sleeper inside a new zone wakes and is lifted");

  OrblitPhysicsZone removed{};
  removed.body = 1;
  check(orblit_physics_zone(physics, &removed),
        "a zone is removed by naming none of its fields");
  check(!orblit_physics_zone(physics, &removed),
        "and removing it again says there was none");
  run(physics, 1.0f);
  orblit_physics_velocity(physics, 2, motion);
  check(motion[1] < 2.0f, "and what was in it stops being lifted");

  OrblitPhysicsZone solid = zoneOn(2, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, 0.0f);
  check(!orblit_physics_zone(physics, &solid), "a zone on a body that is not a trigger is refused");
  OrblitPhysicsZone nowhere = zoneOn(77, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, 0.0f);
  check(!orblit_physics_zone(physics, &nowhere), "and so is one on no body");
  OrblitPhysicsZone nan = zoneOn(1, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, std::nanf(""));
  check(!orblit_physics_zone(physics, &nan), "and one with NaN in it");
  check(!orblit_physics_zone(nullptr, &lift), "and a zone in no world is survivable");
  orblit_physics_destroy(physics);
}

/// A belt sixteen metres long with its top at half a metre and a box on it,
/// its surface moving at `along`. Nothing about the belt itself moves.
OrblitPhysics *beltWith(float friction, float along) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand belt = slabAt(1, 8.0f, 0.25f, 1.0f, 0.0f, 0.25f, 0.0f);
  belt.friction = friction;
  submit(physics, belt);
  OrblitPhysicsCommand box = boxAt(2, 0.5f, 0.0f, 1.0f, 0.0f);
  box.friction = friction;
  submit(physics, box);
  surfaceOf(physics, 1, along, 0.0f);
  return physics;
}

float alongOf(OrblitPhysics *physics, OrblitPhysicsId id) {
  float motion[6] = {0};
  orblit_physics_velocity(physics, id, motion);
  return motion[0];
}

void conveying() {
  OrblitPhysics *physics = beltWith(0.5f, 2.0f);
  run(physics, 2.0f);
  check(near(alongOf(physics, 2), 2.0f, 0.2f), "a box on a belt is carried at the belt's speed");
  check(coordinateOf(physics, 2, 0) > 1.5f, "and goes somewhere");
  check(near(heightOf(physics, 2), 1.0f, 0.02f), "without being lifted or sunk");

  surfaceOf(physics, 1, -2.0f, 0.0f);
  run(physics, 2.0f);
  check(near(alongOf(physics, 2), -2.0f, 0.2f), "and carried back when the belt is reversed");

  surfaceOf(physics, 1, 0.0f, 0.0f);
  run(physics, 2.0f);
  check(speedOf(physics, 2) < 0.1f, "and left at rest when it is stopped");
  orblit_physics_destroy(physics);

  OrblitPhysics *slippery = beltWith(0.0f, 2.0f);
  run(slippery, 1.0f);
  check(std::fabs(alongOf(slippery, 2)) < 0.2f, "a belt with no friction carries nothing");
  orblit_physics_destroy(slippery);

  // Only the part along the surface counts, so a belt that moves into itself
  // stands still.
  OrblitPhysics *into = beltWith(0.5f, 0.0f);
  surfaceOf(into, 1, 0.0f, 5.0f);
  run(into, 1.0f);
  check(near(heightOf(into, 2), 1.0f, 0.02f) && speedOf(into, 2) < 0.1f,
        "a surface speed straight into the surface does nothing");
  orblit_physics_destroy(into);
}

void wakingBelts() {
  OrblitPhysics *physics = beltWith(0.5f, 0.0f);
  run(physics, 3.0f);
  check(orblit_physics_asleep(physics, 2), "a box left on a belt that is stopped goes to sleep");
  surfaceOf(physics, 1, 2.0f, 0.0f);
  check(!orblit_physics_asleep(physics, 2), "and is woken by the belt starting");
  run(physics, 1.5f);
  check(alongOf(physics, 2) > 1.5f, "and carried off");
  orblit_physics_destroy(physics);
}

void pinnedBelts() {
  // A box a belt pushes into a wall is at rest against the world, but only
  // because the wall is in the way. Asleep it would stay where it is when the
  // wall went.
  OrblitPhysics *physics = beltWith(0.5f, 2.0f);
  submit(physics, slabAt(3, 0.25f, 1.0f, 1.0f, 2.0f, 1.0f, 0.0f));
  run(physics, 3.0f);
  check(speedOf(physics, 2) < 0.1f, "a box a belt pushes into a wall is held there");
  check(!orblit_physics_asleep(physics, 2), "and is not put to sleep by it");
  destroyBody(physics, 3);
  run(physics, 1.0f);
  check(alongOf(physics, 2) > 1.5f, "so when the wall goes the belt carries it on");
  orblit_physics_destroy(physics);
}

OrblitPhysicsRule ruleFor(OrblitPhysicsId a, OrblitPhysicsId b, uint32_t overrides) {
  OrblitPhysicsRule made{};
  made.a = a;
  made.b = b;
  made.overrides = overrides;
  made.moveScale[0] = 1.0f;
  made.moveScale[1] = 1.0f;
  return made;
}

/// A box given a shove across ground, with or without a rule that this box and
/// this ground have no friction, and the speed it still has a second later.
float slidBy(bool ruled) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.49f, 0.0f));
  drive(physics, 2, 3.0f, 0.0f, 0.0f, 0.0f);
  if (ruled) {
    OrblitPhysicsRule ice = ruleFor(1, 2, ORBLIT_PHYSICS_RULE_FRICTION);
    ice.friction = 0.0f;
    check(orblit_physics_rule(physics, &ice), "a rule between two bodies is made");
  }
  run(physics, 1.0f);
  const float speed = speedOf(physics, 2);
  orblit_physics_destroy(physics);
  return speed;
}

/// The highest a ball dropped from two metres gets on its second rise, with or
/// without a rule that it and the ground bounce.
float bouncedBy(bool ruled) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 2.0f, 0.0f));
  if (ruled) {
    OrblitPhysicsRule springy = ruleFor(2, 1, ORBLIT_PHYSICS_RULE_RESTITUTION);
    springy.restitution = 0.9f;
    orblit_physics_rule(physics, &springy);
  }
  run(physics, 0.7f);
  float highest = 0.0f;
  for (int i = 0; i < 72; ++i) {
    orblit_physics_step(physics, kStep);
    highest = std::fmax(highest, heightOf(physics, 2));
  }
  orblit_physics_destroy(physics);
  return highest;
}

/// Box 9 slid at 4 m/s into a box 4 at rest on ice, with box 9's move scale set to
/// `scale` for that pair. Box 9's id is the larger, so this also reads the
/// scales the way round they were given rather than the way round they are
/// keyed. Returns box 9's speed after the shove, and box 4's through `pushed`.
float shovedBy(float scale, float *pushed) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand ice = groundPlane(1);
  ice.friction = 0.0f;
  submit(physics, ice);
  for (const OrblitPhysicsId id : {OrblitPhysicsId{9}, OrblitPhysicsId{4}}) {
    OrblitPhysicsCommand box = boxAt(id, 0.5f, id == 9 ? 0.0f : 1.2f, 0.49f, 0.0f);
    box.friction = 0.0f;
    box.damping[0] = 0.0f;
    submit(physics, box);
  }
  drive(physics, 9, 4.0f, 0.0f, 0.0f, 0.0f);
  OrblitPhysicsRule heavy = ruleFor(9, 4, ORBLIT_PHYSICS_RULE_MOVE_SCALE);
  heavy.moveScale[0] = scale;
  orblit_physics_rule(physics, &heavy);
  run(physics, 0.5f);
  *pushed = alongOf(physics, 4);
  const float mover = alongOf(physics, 9);
  orblit_physics_destroy(physics);
  return mover;
}

void ruling() {
  check(slidBy(false) < 1.0f, "a box slides to a stop on grippy ground");
  check(slidBy(true) > 2.5f,
        "and does not once the friction of that one pair is nought");

  check(bouncedBy(false) < 0.6f, "a ball with no restitution lands dead");
  check(bouncedBy(true) > 1.0f,
        "and bounces when its pair's restitution is raised, named either way round");

  float pushed = 0.0f;
  check(shovedBy(1.0f, &pushed) < 3.0f, "a box that shoves another is slowed by it");
  const float held = shovedBy(0.0f, &pushed);
  check(held > 3.9f && pushed > 3.9f,
        "and is not when it takes none of the push in that pair, so the other takes all of it");
}

void ruleEdits() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  OrblitPhysicsRule made = ruleFor(1, 2, ORBLIT_PHYSICS_RULE_FRICTION);
  check(orblit_physics_rule(physics, &made), "a rule is made");
  OrblitPhysicsRule none = ruleFor(2, 1, 0);
  check(orblit_physics_rule(physics, &none),
        "and removed by naming none of its fields, in either order");
  check(!orblit_physics_rule(physics, &none), "and removing it again says there was none");

  OrblitPhysicsRule alone = ruleFor(2, 2, ORBLIT_PHYSICS_RULE_FRICTION);
  check(!orblit_physics_rule(physics, &alone), "a rule between a body and itself is refused");
  OrblitPhysicsRule nowhere = ruleFor(2, 55, ORBLIT_PHYSICS_RULE_FRICTION);
  check(!orblit_physics_rule(physics, &nowhere), "and so is one naming a body that is not there");
  OrblitPhysicsRule nan = ruleFor(1, 2, ORBLIT_PHYSICS_RULE_FRICTION);
  nan.friction = std::nanf("");
  check(!orblit_physics_rule(physics, &nan), "and one with NaN in it");

  // A new body with a dead one's id must not inherit what was said about it.
  orblit_physics_rule(physics, &made);
  destroyBody(physics, 2);
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  check(!orblit_physics_rule(physics, &none), "a rule ends with either body");
  check(!orblit_physics_rule(nullptr, &made), "and a rule in no world is survivable");
  orblit_physics_destroy(physics);
}

void hiding() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand floor = groundPlane(2);
  floor.size[3] = -3.0f;
  submit(physics, floor);
  submit(physics, triggerAt(1, 1.0f, 0.0f, 0.0f, 0.0f));

  OrblitPhysicsHit hit{};
  OrblitPhysicsCast down = rayFrom(0.0f, 10.0f, 0.0f, 0.0f, -1.0f, 0.0f, 100.0f);
  check(orblit_physics_cast(physics, &down, &hit) && hit.body == 2 &&
            near(hit.distance, 13.0f, 0.01f),
        "a ray passes through a trigger to what is behind it");
  down.triggers = true;
  check(orblit_physics_cast(physics, &down, &hit) && hit.body == 1 &&
            near(hit.distance, 9.0f, 0.01f),
        "and meets it only when asked for triggers");
  check(hitsAlong(physics, down, 4).size() == 1, "which are then all it meets");

  const OrblitPhysicsCast here = pointAt(0.0f, 0.0f, 0.0f);
  check(overlapping(physics, here, 4).empty(), "a point inside a trigger is in no solid body");
  OrblitPhysicsCast zones = here;
  zones.triggers = true;
  check(holds(overlapping(physics, zones, 4), {1}),
        "and is in the trigger when asked for triggers");
  orblit_physics_destroy(physics);

  // A character's own sweeps do not meet one either.
  OrblitPhysics *walked = orblit_physics_create(nullptr);
  submit(walked, groundPlane(1));
  OrblitPhysicsCommand wall = slabAt(2, 0.1f, 2.0f, 1.5f, 2.0f, 2.0f, 0.0f);
  wall.sensor = true;
  submit(walked, wall);
  addCharacter(walked, 3, 0.0f, 0.0f, 0.0f);
  walk(walked, 3, 2.0f, 0.0f, 1.5f);
  check(coordinateOf(walked, 3, 0) > 2.5f,
        "a character walks through a trigger as if it were not there");
  orblit_physics_destroy(walked);
}

void overlaps() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, slabAt(1, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f));
  OrblitPhysicsCommand ball = sphereAt(2, 0.5f, 1.4f, 0.0f, 0.0f);
  ball.motion = ORBLIT_PHYSICS_STATIC;
  submit(physics, ball);
  OrblitPhysicsCommand capsule = capsuleAt(3, 0.3f, 0.6f, 10.0f, 0.0f, 0.0f);
  capsule.motion = ORBLIT_PHYSICS_STATIC;
  submit(physics, capsule);
  OrblitPhysicsCommand floor = groundPlane(4);
  floor.size[3] = -50.0f;
  submit(physics, floor);

  check(holds(overlapping(physics, pointAt(0.0f, 0.0f, 0.0f), 8), {1}),
        "a point inside a box is in that box and nothing else");
  check(holds(overlapping(physics, pointAt(1.8f, 0.0f, 0.0f), 8), {2}),
        "and one inside the ball is in the ball");
  check(holds(overlapping(physics, pointAt(0.0f, 0.99f, 0.0f), 8), {1}),
        "however close to the surface, on the inside");
  check(overlapping(physics, pointAt(0.0f, 1.2f, 0.0f), 8).empty(),
        "and outside it is in nothing");
  check(holds(overlapping(physics, pointAt(10.0f, 0.0f, 0.0f), 8), {3}),
        "a point inside a capsule is in the capsule");
  check(holds(overlapping(physics, pointAt(0.0f, -60.0f, 0.0f), 8), {4}),
        "and one below a plane is in the plane");

  OrblitPhysicsCast ballQuery = pointAt(1.2f, 0.0f, 0.0f);
  ballQuery.shape = ORBLIT_PHYSICS_SPHERE;
  ballQuery.size[0] = 0.3f;
  check(overlapping(physics, ballQuery, 8).size() == 2, "a sphere between two bodies overlaps both");
  check(overlapping(physics, ballQuery, 1).size() == 1, "and stops at the number asked for");
  check(overlapping(physics, ballQuery, 0).empty(), "asking for none returns none");

  OrblitPhysicsCast boxQuery = ballQuery;
  boxQuery.shape = ORBLIT_PHYSICS_BOX;
  boxQuery.size[0] = 0.1f;
  boxQuery.size[1] = 0.1f;
  boxQuery.size[2] = 0.1f;
  boxQuery.from[0] = 10.0f;
  boxQuery.from[1] = 0.5f;
  check(holds(overlapping(physics, boxQuery, 8), {3}), "a box overlaps by its own shape");

  OrblitPhysicsCast ignoring = ballQuery;
  ignoring.ignore = 1;
  check(holds(overlapping(physics, ignoring, 8), {2}), "a body may be ignored");

  OrblitPhysicsCast flat = ballQuery;
  flat.shape = ORBLIT_PHYSICS_PLANE;
  flat.size[1] = 1.0f;
  check(overlapping(physics, flat, 8).empty(), "a half-space cannot be asked about");
  check(orblit_physics_overlap(nullptr, &ballQuery, nullptr, 0) == 0,
        "and asking no world is survivable");
  orblit_physics_destroy(physics);
}

void overlapLayers() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand shy = sphereAt(1, 0.5f, 0.0f, 0.0f, 0.0f);
  shy.motion = ORBLIT_PHYSICS_STATIC;
  shy.layerIs = 2;
  shy.layerCares = 0;
  submit(physics, shy);

  OrblitPhysicsCast blind = pointAt(0.0f, 0.0f, 0.0f);
  blind.layerIs = 4;
  blind.layerCares = 0;
  check(overlapping(physics, blind, 4).empty(),
        "a query and a body that care about nothing do not meet");
  blind.layerCares = 2;
  check(holds(overlapping(physics, blind, 4), {1}), "and do when the query cares");

  OrblitPhysicsCommand keen = shy;
  keen.id = 2;
  keen.layerCares = 4;
  submit(physics, keen);
  blind.layerCares = 0;
  check(holds(overlapping(physics, blind, 4), {2}), "or when the body does");
  orblit_physics_destroy(physics);
}

void askingAll() {
  // Three boxes in a row along +x, made in the reverse of the order a ray
  // meets them, so that meeting them in order is not an accident of storage.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  for (int i = 3; i >= 1; --i) {
    submit(physics, slabAt(i, 0.5f, 0.5f, 0.5f, 3.0f * static_cast<float>(i), 0.0f, 0.0f));
  }
  const OrblitPhysicsCast along = rayFrom(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);

  const std::vector<OrblitPhysicsHit> all = hitsAlong(physics, along, 8);
  check(all.size() == 3 && all[0].body == 1 && all[1].body == 2 && all[2].body == 3,
        "every body along a ray is met, nearest first");
  check(near(all[0].distance, 2.5f, 0.01f) && near(all[2].distance, 8.5f, 0.01f) &&
            near(all[1].normal[0], -1.0f, 0.01f),
        "each with its own distance and the normal of the face it met");
  const std::vector<OrblitPhysicsHit> two = hitsAlong(physics, along, 2);
  check(two.size() == 2 && two[1].body == 2, "a limit keeps the nearest ones");
  check(hitsAlong(physics, along, 0).empty(), "and none is none");

  OrblitPhysicsCast brief = along;
  brief.distance = 6.0f;
  check(hitsAlong(physics, brief, 8).size() == 2, "a short ray stops before the third");

  OrblitPhysicsCast skipping = along;
  skipping.ignore = 2;
  const std::vector<OrblitPhysicsHit> skipped = hitsAlong(physics, skipping, 8);
  check(skipped.size() == 2 && skipped[1].body == 3, "an ignored body is passed through");

  OrblitPhysicsCast within = along;
  within.from[0] = 3.0f;
  const std::vector<OrblitPhysicsHit> from = hitsAlong(physics, within, 8);
  check(from.size() == 3 && from[0].body == 1 && from[0].started &&
            from[0].distance == 0.0f && !from[1].started,
        "a ray that starts inside a body reports it as begun, and goes on to the rest");
  orblit_physics_destroy(physics);
}

void askingAny() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, slabAt(1, 0.5f, 0.5f, 0.5f, 3.0f, 0.0f, 0.0f));
  submit(physics, triggerAt(2, 0.5f, 6.0f, 0.0f, 0.0f));
  OrblitPhysicsCast along = rayFrom(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);

  check(orblit_physics_cast_any(physics, &along), "any-hit says a ray meets something");
  OrblitPhysicsCast back = along;
  back.direction[0] = -1.0f;
  check(!orblit_physics_cast_any(physics, &back), "and that a ray fired away meets nothing");
  along.distance = 2.0f;
  check(!orblit_physics_cast_any(physics, &along), "or one that stops short");
  along.distance = 3.0f;
  check(orblit_physics_cast_any(physics, &along), "or does when it just reaches");

  OrblitPhysicsCast beyond = rayFrom(4.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);
  check(!orblit_physics_cast_any(physics, &beyond), "a trigger alone is not a hit");
  beyond.triggers = true;
  check(orblit_physics_cast_any(physics, &beyond), "unless triggers are what is asked for");
  check(!orblit_physics_cast_any(nullptr, &along), "and asking no world is survivable");
  orblit_physics_destroy(physics);
}

/// The whole of M1 in one world: a ramp, a trigger, a zone and a belt with a
/// box on it, and each of the five questions asked of it.
void askingTheWorld() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, rampFrom(2, 10.0f, 0.5f));
  submit(physics, triggerAt(3, 1.0f, 4.0f, 1.0f, 0.0f));
  submit(physics, triggerAt(4, 2.0f, -20.0f, 2.0f, 0.0f));
  OrblitPhysicsZone weightless = zoneOn(4, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, 0.0f);
  check(orblit_physics_zone(physics, &weightless), "a zone is set on a trigger");
  submit(physics, slabAt(5, 4.0f, 0.25f, 1.0f, -5.0f, 0.25f, 0.0f));
  submit(physics, boxAt(6, 0.5f, -5.0f, 1.0f, 0.0f));
  submit(physics, sphereAt(7, 0.25f, -20.0f, 2.0f, 0.0f));
  surfaceOf(physics, 5, 2.0f, 0.0f);

  // Overlap and point: which trigger is this in, and is it in anything solid.
  OrblitPhysicsCast atTrigger = pointAt(4.0f, 1.0f, 0.0f);
  check(overlapping(physics, atTrigger, 4).empty(), "point: nothing solid is at the trigger");
  atTrigger.triggers = true;
  check(holds(overlapping(physics, atTrigger, 4), {3}), "point: the trigger is");
  OrblitPhysicsCast atZone = pointAt(-20.0f, 2.0f, 0.0f);
  atZone.triggers = true;
  check(holds(overlapping(physics, atZone, 4), {4}), "point: and the zone is where it was put");
  OrblitPhysicsCast small = pointAt(-5.0f, 1.0f, 0.0f);
  small.shape = ORBLIT_PHYSICS_SPHERE;
  small.size[0] = 0.1f;
  check(holds(overlapping(physics, small, 4), {6}),
        "overlap: a small sphere at the box is in the box");

  // Closest, all and any, down onto the ramp.
  const OrblitPhysicsCast down = rayFrom(13.0f, 10.0f, 0.0f, 0.0f, -1.0f, 0.0f, 100.0f);
  OrblitPhysicsHit hit{};
  check(orblit_physics_cast(physics, &down, &hit) && hit.body == 2,
        "closest: a ray fired down onto the ramp meets the ramp");
  check(near(hit.normal[0], -std::sin(0.5f), 0.01f) && near(hit.normal[1], std::cos(0.5f), 0.01f),
        "ray with normal: and the normal is the ramp's, tilted");
  const std::vector<OrblitPhysicsHit> both = hitsAlong(physics, down, 8);
  check(both.size() == 2 && both[0].body == 2 && both[1].body == 1 &&
            near(both[1].distance, 10.0f, 0.01f),
        "all: and goes on to the ground beneath it");
  OrblitPhysicsCast brief = down;
  brief.distance = 5.0f;
  check(!orblit_physics_cast_any(physics, &brief) && orblit_physics_cast_any(physics, &down),
        "any: and stops as soon as it knows");

  // The world, run: the belt carries its box and the zone holds its ball.
  run(physics, 2.0f);
  check(near(alongOf(physics, 6), 2.0f, 0.2f), "hook: a conveyor contact carries the box");
  check(near(heightOf(physics, 7), 2.0f, 0.05f), "zone: and the ball in the zone has not fallen");
  orblit_physics_destroy(physics);
}

// --- body controls -------------------------------------------------------- //
//
// Gravity scale, axis locks, speed caps, centre of mass, inertia, the switch
// between fixed, driven and free, and pairs that ignore each other.

OrblitPhysicsControls controlsOf(OrblitPhysicsId body) {
  OrblitPhysicsControls made{};
  made.body = body;
  made.gravityScale = 1.0f;
  return made;
}

bool control(OrblitPhysics *physics, const OrblitPhysicsControls &controls) {
  return orblit_physics_controls(physics, &controls);
}

/// A world with no gravity, so a body does what it was told and no more.
OrblitPhysics *spaceWorld() {
  OrblitPhysicsSettings settings;
  orblit_physics_defaults(&settings);
  settings.gravity[0] = 0.0f;
  settings.gravity[1] = 0.0f;
  settings.gravity[2] = 0.0f;
  return orblit_physics_create(&settings);
}

void impulseAt(OrblitPhysics *physics, OrblitPhysicsId id, float ix, float iy,
               float iz, float px, float py, float pz) {
  OrblitPhysicsCommand made{};
  made.kind = ORBLIT_PHYSICS_IMPULSE;
  made.id = id;
  made.vector[0] = ix;
  made.vector[1] = iy;
  made.vector[2] = iz;
  made.spin[0] = px;
  made.spin[1] = py;
  made.spin[2] = pz;
  submit(physics, made);
}

float velocityOf(OrblitPhysics *physics, OrblitPhysicsId id, int axis) {
  float motion[6] = {0};
  orblit_physics_velocity(physics, id, motion);
  return motion[axis];
}

/// Where a point given in the body's own frame is in the world. The transform
/// reports the body's origin, so this is how a test finds its centre of mass.
void worldPointOf(OrblitPhysics *physics, OrblitPhysicsId id,
                  const float local[3], float out[3]) {
  float t[7] = {0};
  orblit_physics_transform(physics, id, t);
  const float ux = t[3], uy = t[4], uz = t[5], w = t[6];
  const float tx = 2.0f * (uy * local[2] - uz * local[1]);
  const float ty = 2.0f * (uz * local[0] - ux * local[2]);
  const float tz = 2.0f * (ux * local[1] - uy * local[0]);
  out[0] = t[0] + local[0] + w * tx + (uy * tz - uz * ty);
  out[1] = t[1] + local[1] + w * ty + (uz * tx - ux * tz);
  out[2] = t[2] + local[2] + w * tz + (ux * ty - uy * tx);
}

void switchTo(OrblitPhysics *physics, OrblitPhysicsId id, uint32_t motion) {
  OrblitPhysicsCommand made{};
  made.kind = ORBLIT_PHYSICS_MOTION;
  made.id = id;
  made.motion = motion;
  submit(physics, made);
}

void weighing() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(1, 0.5f, 0.0f, 100.0f, 0.0f));
  submit(physics, sphereAt(2, 0.5f, 5.0f, 100.0f, 0.0f));
  submit(physics, sphereAt(3, 0.5f, 10.0f, 100.0f, 0.0f));
  submit(physics, sphereAt(4, 0.5f, 15.0f, 100.0f, 0.0f));
  OrblitPhysicsControls half = controlsOf(2);
  half.gravityScale = 0.5f;
  OrblitPhysicsControls none = controlsOf(3);
  none.gravityScale = 0.0f;
  OrblitPhysicsControls up = controlsOf(4);
  up.gravityScale = -1.0f;
  check(control(physics, half) && control(physics, none) && control(physics, up),
        "gravity scales are set");

  run(physics, 1.0f);
  const float full = -velocityOf(physics, 1, 1);
  check(full > 8.0f, "a body falls at the world's gravity by default");
  check(near(-velocityOf(physics, 2, 1) / full, 0.5f, 0.02f),
        "half the scale gives half the speed");
  check(near(velocityOf(physics, 3, 1), 0.0f, 0.001f), "no scale hangs in the air");
  check(velocityOf(physics, 4, 1) > 8.0f, "a negative scale rises");
  orblit_physics_destroy(physics);
}

void changingGravity() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  run(physics, 2.0f);
  check(orblit_physics_asleep(physics, 2), "a ball at rest goes to sleep");

  const float up[3] = {0.0f, 9.81f, 0.0f};
  check(orblit_physics_gravity(physics, up), "the world's gravity is set");
  check(!orblit_physics_asleep(physics, 2),
        "which wakes what was at rest, since it no longer is");
  run(physics, 1.0f);
  check(heightOf(physics, 2) > 3.0f, "and the ball rises");

  const float bad[3] = {std::nanf(""), 0.0f, 0.0f};
  check(!orblit_physics_gravity(physics, bad), "NaN gravity is refused");
  check(!orblit_physics_gravity(physics, nullptr), "and so is none");
  check(!orblit_physics_gravity(nullptr, up), "and gravity in no world is survivable");
  orblit_physics_destroy(physics);
}

void lockingToAPlane() {
  // The done-when: a body locked to a plane, at half gravity, under a cap.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(1, 0.5f, 0.0f, 50.0f, 0.0f));
  OrblitPhysicsControls plane = controlsOf(1);
  plane.locks = ORBLIT_PHYSICS_LOCK_MOVE_Z;
  plane.gravityScale = 0.5f;
  plane.maxSpeed = 4.0f;
  check(control(physics, plane),
        "a body is locked to a plane, at half gravity, under a cap");

  drive(physics, 1, 1.0f, 0.0f, 3.0f, 0.0f);
  check(velocityOf(physics, 1, 2) == 0.0f, "a velocity out of the plane is dropped");
  run(physics, 0.3f);
  check(near(velocityOf(physics, 1, 1), -1.45f, 0.1f), "it falls at half gravity");
  check(coordinateOf(physics, 1, 0) > 0.2f, "and keeps moving across the plane");

  float fastest = 0.0f;
  for (int i = 0; i < 120; ++i) {
    step(physics);
    const float vx = velocityOf(physics, 1, 0);
    const float vy = velocityOf(physics, 1, 1);
    fastest = std::fmax(fastest, std::sqrt(vx * vx + vy * vy));
  }
  check(fastest < 4.001f && fastest > 3.9f, "and is held to the cap, reaching it");
  check(coordinateOf(physics, 1, 2) == 0.0f, "and never leaves the plane");
  orblit_physics_destroy(physics);

  OrblitPhysics *pushed = spaceWorld();
  submit(pushed, boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  OrblitPhysicsControls flat = controlsOf(1);
  flat.locks = ORBLIT_PHYSICS_LOCK_MOVE_Z;
  control(pushed, flat);
  impulseAt(pushed, 1, 2.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f);
  check(near(velocityOf(pushed, 1, 0), 2.0f, 0.001f) && velocityOf(pushed, 1, 2) == 0.0f,
        "and so is the part of an impulse that points out of it");
  orblit_physics_destroy(pushed);
}

void lockedContacts() {
  // A tilted floor pushes partly along the locked axis. The ball still has to
  // rest on it, which only works if the solver treats that axis as immovable.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsCommand floor = groundPlane(1);
  floor.size[1] = 0.8f;
  floor.size[2] = 0.6f;
  submit(physics, floor);
  submit(physics, sphereAt(2, 0.5f, 0.0f, 0.7f, 0.0f));
  OrblitPhysicsControls plane = controlsOf(2);
  plane.locks = ORBLIT_PHYSICS_LOCK_MOVE_Z;
  control(physics, plane);

  run(physics, 3.0f);
  check(near(heightOf(physics, 2), 0.625f, 0.02f),
        "a body locked on one axis rests on a floor tilted across it");
  check(coordinateOf(physics, 2, 2) == 0.0f, "without moving along the lock");
  orblit_physics_destroy(physics);

  OrblitPhysics *swing = orblit_physics_create(nullptr);
  submit(swing, sphereAt(2, 0.25f, 2.0f, 5.0f, 0.0f));
  OrblitPhysicsControls hung = controlsOf(2);
  hung.locks = ORBLIT_PHYSICS_LOCK_MOVE_Z;
  control(swing, hung);
  join(swing, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 0, 0.0f, 5.0f, 0.0f));
  drive(swing, 2, 0.0f, 0.0f, 2.0f, 0.0f);
  run(swing, 2.0f);
  const float x = coordinateOf(swing, 2, 0);
  const float y = coordinateOf(swing, 2, 1) - 5.0f;
  check(coordinateOf(swing, 2, 2) == 0.0f && near(std::sqrt(x * x + y * y), 2.0f, 0.05f),
        "a locked body on a joint swings in its plane at the joint's length");
  orblit_physics_destroy(swing);
}

void lockingTurns() {
  // An impulse off the centre spins a box about x and y. With x and z locked,
  // only y is left, and it turns as fast as it did when nothing was locked.
  OrblitPhysics *loose = spaceWorld();
  submit(loose, boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  impulseAt(loose, 1, 0.0f, 0.0f, 5.0f, 0.5f, 0.5f, 0.0f);
  check(near(velocityOf(loose, 1, 3), 15.0f, 0.1f) &&
            near(velocityOf(loose, 1, 4), -15.0f, 0.1f),
        "an off-centre impulse spins a box about two axes");
  orblit_physics_destroy(loose);

  OrblitPhysics *locked = spaceWorld();
  submit(locked, boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  OrblitPhysicsControls upright = controlsOf(1);
  upright.locks = ORBLIT_PHYSICS_LOCK_TURN_X | ORBLIT_PHYSICS_LOCK_TURN_Z;
  control(locked, upright);
  impulseAt(locked, 1, 0.0f, 0.0f, 5.0f, 0.5f, 0.5f, 0.0f);
  check(velocityOf(locked, 1, 3) == 0.0f && velocityOf(locked, 1, 5) == 0.0f &&
            near(velocityOf(locked, 1, 4), -15.0f, 0.1f),
        "with two turns locked it spins about the one left, as fast as before");
  setMotion(locked, 1, 0.0f, 0.0f, 0.0f, 4.0f, 0.0f, 4.0f);
  check(velocityOf(locked, 1, 3) == 0.0f && velocityOf(locked, 1, 5) == 0.0f,
        "and a spin set on a locked axis is dropped");
  orblit_physics_destroy(locked);

  // A box tipped on its corner topples when free and holds when it cannot turn.
  OrblitPhysicsCommand tipped = boxAt(2, 0.5f, 0.0f, 2.0f, 0.0f);
  tipped.rotation[2] = 0.258819f;
  tipped.rotation[3] = 0.965926f;

  OrblitPhysics *falls = orblit_physics_create(nullptr);
  submit(falls, groundPlane(1));
  submit(falls, tipped);
  run(falls, 3.0f);
  check(turnOf(falls, 2) < 0.1f, "a box dropped at thirty degrees settles flat");
  orblit_physics_destroy(falls);

  OrblitPhysics *held = orblit_physics_create(nullptr);
  submit(held, groundPlane(1));
  submit(held, tipped);
  OrblitPhysicsControls stiff = controlsOf(2);
  stiff.locks = ORBLIT_PHYSICS_LOCK_TURN_X | ORBLIT_PHYSICS_LOCK_TURN_Y |
                ORBLIT_PHYSICS_LOCK_TURN_Z;
  control(held, stiff);
  run(held, 3.0f);
  check(near(turnOf(held, 2), 0.5236f, 0.01f),
        "and stays at thirty degrees when it cannot turn");
  check(heightOf(held, 2) > 0.6f, "resting on its corner");
  orblit_physics_destroy(held);
}

void capping() {
  OrblitPhysics *physics = spaceWorld();
  submit(physics, boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  OrblitPhysicsControls capped = controlsOf(1);
  capped.maxSpeed = 5.0f;
  capped.maxSpin = 3.0f;
  check(control(physics, capped), "caps are set");

  impulseAt(physics, 1, 100.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  step(physics);
  check(speedOf(physics, 1) < 5.001f && speedOf(physics, 1) > 4.9f,
        "a shove that would go faster is held to the speed cap");
  drive(physics, 1, 0.0f, 0.0f, 0.0f, 20.0f);
  step(physics);
  check(velocityOf(physics, 1, 4) < 3.001f && velocityOf(physics, 1, 4) > 2.9f,
        "and a spin faster than the cap is held to it");

  capped.maxSpeed = 0.0f;
  capped.maxSpin = 0.0f;
  control(physics, capped);
  drive(physics, 1, 50.0f, 0.0f, 0.0f, 20.0f);
  step(physics);
  check(speedOf(physics, 1) > 40.0f && velocityOf(physics, 1, 4) > 10.0f,
        "a cap of nothing means no cap");
  orblit_physics_destroy(physics);
}

/// A ball dropped onto a slab that lies over the ground. The slab is 3, the
/// ball 2. If `ignored`, the two are told to pass through each other.
OrblitPhysics *ballOverSlab(bool ignored) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, slabAt(3, 1.0f, 0.25f, 1.0f, 0.0f, 1.0f, 0.0f));
  submit(physics, sphereAt(2, 0.25f, 0.0f, 3.0f, 0.0f));
  if (ignored) {
    const OrblitPhysicsRule pass = ruleFor(3, 2, ORBLIT_PHYSICS_RULE_IGNORE);
    check(orblit_physics_rule(physics, &pass), "a pair is made to ignore each other");
  }
  return physics;
}

/// A ball dropped through a trigger, and how many times the trigger said it
/// came in.
int timesEntered(bool ignored) {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, triggerAt(1, 1.0f, 0.0f, 0.0f, 0.0f));
  submit(physics, sphereAt(2, 0.25f, 0.0f, 3.0f, 0.0f));
  if (ignored) {
    const OrblitPhysicsRule blind = ruleFor(1, 2, ORBLIT_PHYSICS_RULE_IGNORE);
    orblit_physics_rule(physics, &blind);
  }
  Watch watch(physics);
  watch.run(2.0f);
  const int entered = watch.count(ORBLIT_PHYSICS_ENTERED, 1, 2);
  orblit_physics_destroy(physics);
  return entered;
}

void ignoring() {
  OrblitPhysics *rests = ballOverSlab(false);
  run(rests, 2.0f);
  check(near(heightOf(rests, 2), 1.5f, 0.05f), "a ball dropped on a slab rests on it");
  orblit_physics_destroy(rests);

  OrblitPhysics *passes = ballOverSlab(true);
  Watch watch(passes);
  watch.run(2.0f);
  check(near(heightOf(passes, 2), 0.25f, 0.05f),
        "and falls through it to the ground when the two ignore each other");
  check(watch.count(ORBLIT_PHYSICS_TOUCH_BEGAN, 2, 3) == 0, "reporting no touch with it");

  const OrblitPhysicsRule ends = ruleFor(2, 3, 0);
  check(orblit_physics_rule(passes, &ends), "the rule is removed, named either way round");
  OrblitPhysicsCommand again{};
  again.kind = ORBLIT_PHYSICS_PLACE;
  again.id = 2;
  again.at[1] = 3.0f;
  again.rotation[3] = 1.0f;
  submit(passes, again);
  run(passes, 2.0f);
  check(near(heightOf(passes, 2), 1.5f, 0.05f), "and then the slab holds the ball again");
  orblit_physics_destroy(passes);

  check(timesEntered(false) == 1, "a trigger reports a ball falling through it");
  check(timesEntered(true) == 0, "and nothing for a ball it ignores");
}

void switching() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, sphereAt(1, 0.5f, 0.0f, 20.0f, 0.0f));
  drive(physics, 1, 3.0f, 0.0f, 0.0f, 0.0f);
  run(physics, 0.1f);
  switchTo(physics, 1, ORBLIT_PHYSICS_STATIC);
  const float stopped = heightOf(physics, 1);
  check(speedOf(physics, 1) == 0.0f, "a free body made fixed stops");
  run(physics, 1.0f);
  check(near(heightOf(physics, 1), stopped, 0.0001f), "and hangs where it was");

  OrblitPhysicsCommand crate = boxAt(2, 0.5f, 5.0f, 5.0f, 0.0f);
  crate.motion = ORBLIT_PHYSICS_STATIC;
  crate.mass = 2.0f;
  submit(physics, crate);
  switchTo(physics, 2, ORBLIT_PHYSICS_DYNAMIC);
  impulseAt(physics, 2, 2.0f, 0.0f, 0.0f, 5.0f, 5.0f, 0.0f);
  check(near(velocityOf(physics, 2, 0), 1.0f, 0.001f),
        "a fixed body made free has the mass it was made with");
  run(physics, 0.5f);
  check(heightOf(physics, 2) < 4.95f, "and falls");

  submit(physics, sphereAt(3, 0.5f, -5.0f, 10.0f, 0.0f));
  drive(physics, 3, 3.0f, 0.0f, 0.0f, 0.0f);
  switchTo(physics, 3, ORBLIT_PHYSICS_KINEMATIC);
  run(physics, 1.0f);
  check(near(coordinateOf(physics, 3, 0), -2.0f, 0.05f) &&
            near(heightOf(physics, 3), 10.0f, 0.0001f),
        "a free body made driven keeps its velocity and stops feeling gravity");

  switchTo(physics, 3, ORBLIT_PHYSICS_DYNAMIC);
  run(physics, 0.5f);
  check(heightOf(physics, 3) < 9.95f, "and a driven body made free falls again");

  submit(physics, triggerAt(4, 1.0f, 20.0f, 20.0f, 0.0f));
  switchTo(physics, 4, ORBLIT_PHYSICS_DYNAMIC);
  run(physics, 0.5f);
  check(near(heightOf(physics, 4), 20.0f, 0.0001f), "a trigger cannot be made free");

  switchTo(physics, 99, ORBLIT_PHYSICS_DYNAMIC);
  switchTo(physics, 1, 7);
  run(physics, 0.5f);
  check(orblit_physics_count(physics) == 4 && near(heightOf(physics, 1), stopped, 0.0001f),
        "a body that is not there and a motion that does not exist change nothing");
  orblit_physics_destroy(physics);
}

void centring() {
  // A ball weighted on one side rolls until the weight is underneath.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  OrblitPhysicsCommand ball = sphereAt(2, 0.5f, 0.0f, 0.5f, 0.0f);
  ball.damping[1] = 3.0f;
  submit(physics, ball);
  OrblitPhysicsControls weighted = controlsOf(2);
  weighted.centre[0] = 0.3f;
  check(control(physics, weighted), "a centre of mass is set");
  run(physics, 8.0f);
  const float local[3] = {0.3f, 0.0f, 0.0f};
  float centre[3];
  worldPointOf(physics, 2, local, centre);
  check(near(centre[0] - coordinateOf(physics, 2, 0), 0.0f, 0.03f) &&
            near(centre[1] - heightOf(physics, 2), -0.3f, 0.03f),
        "a ball weighted at one side rolls until the weight is underneath");
  orblit_physics_destroy(physics);

  // An impulse through the centre of mass does not spin a body. One through
  // the middle of the shape does, against an inertia that has grown by the
  // parallel axis theorem: a sixth of a box, plus the square of 0.4.
  OrblitPhysics *space = spaceWorld();
  submit(space, boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  submit(space, boxAt(2, 0.5f, 10.0f, 0.0f, 0.0f));
  OrblitPhysicsControls first = controlsOf(1);
  first.centre[0] = 0.4f;
  OrblitPhysicsControls second = controlsOf(2);
  second.centre[0] = 0.4f;
  control(space, first);
  control(space, second);
  impulseAt(space, 1, 0.0f, 0.0f, 1.0f, 0.4f, 0.0f, 0.0f);
  impulseAt(space, 2, 0.0f, 0.0f, 1.0f, 10.0f, 0.0f, 0.0f);
  check(near(velocityOf(space, 1, 4), 0.0f, 0.001f),
        "an impulse through the centre of mass does not spin it");
  check(near(velocityOf(space, 2, 4), 0.4f / (1.0f / 6.0f + 0.16f), 0.03f),
        "and one through the middle of the shape does");

  // With no point at all the push goes through the centre of mass.
  submit(space, boxAt(4, 0.5f, 30.0f, 0.0f, 0.0f));
  OrblitPhysicsControls unmarked = controlsOf(4);
  unmarked.centre[0] = 0.4f;
  control(space, unmarked);
  const float nowhere = std::nanf("");
  impulseAt(space, 4, 0.0f, 0.0f, 1.0f, nowhere, nowhere, nowhere);
  check(near(velocityOf(space, 4, 4), 0.0f, 0.001f) &&
            near(velocityOf(space, 4, 2), 1.0f, 0.001f),
        "an impulse with no point goes through the centre of mass");

  // Spinning in space, a body turns about its centre of mass, so its origin
  // goes round that.
  submit(space, boxAt(3, 0.5f, 20.0f, 0.0f, 0.0f));
  OrblitPhysicsControls pivot = controlsOf(3);
  pivot.centre[0] = 0.4f;
  control(space, pivot);
  setMotion(space, 3, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f);
  run(space, 0.5f);
  const float there[3] = {0.4f, 0.0f, 0.0f};
  float heavy[3];
  worldPointOf(space, 3, there, heavy);
  check(near(heavy[0], 20.4f, 0.03f) && near(heavy[2], 0.0f, 0.03f),
        "a body spinning in space turns about its centre of mass");
  check(turnOf(space, 3) > 0.5f && std::fabs(coordinateOf(space, 3, 2)) > 0.1f,
        "so its origin goes round it");
  orblit_physics_destroy(space);

  // A ball about its own middle has 0.1. About a point 0.3 away it has 0.19.
  OrblitPhysics *round = spaceWorld();
  submit(round, sphereAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  OrblitPhysicsControls shifted = controlsOf(1);
  shifted.centre[0] = 0.3f;
  control(round, shifted);
  impulseAt(round, 1, 0.0f, 0.0f, 1.0f, 0.8f, 0.0f, 0.0f);
  check(near(velocityOf(round, 1, 4), -0.5f / 0.19f, 0.05f),
        "the inertia about the centre of mass includes the distance to the middle");
  orblit_physics_destroy(round);
}

void centringJoined() {
  // A joint on the body's origin stays on the origin when the centre moves.
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, boxAt(2, 0.25f, 0.0f, 5.0f, 0.0f));
  join(physics, jointOf(1, ORBLIT_PHYSICS_JOINT_POINT, 2, 0, 0.0f, 5.0f, 0.0f));
  OrblitPhysicsControls weighted = controlsOf(2);
  weighted.centre[0] = 0.2f;
  control(physics, weighted);

  float widest = 0.0f;
  float turned = 0.0f;
  for (int i = 0; i < 90; ++i) {
    step(physics);
    const float x = coordinateOf(physics, 2, 0);
    const float y = coordinateOf(physics, 2, 1) - 5.0f;
    widest = std::fmax(widest, std::sqrt(x * x + y * y));
    turned = std::fmax(turned, turnOf(physics, 2));
  }
  check(widest < 0.05f, "a point joint on the origin stays on it when the centre moves");
  check(turned > 0.3f, "while the weight swings the body about it");
  orblit_physics_destroy(physics);
}

void spreading() {
  OrblitPhysics *physics = spaceWorld();
  submit(physics, boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f));
  submit(physics, boxAt(2, 0.5f, 5.0f, 0.0f, 0.0f));
  OrblitPhysicsControls heavy = controlsOf(2);
  heavy.inertia[0] = 10.0f;
  heavy.inertia[1] = 10.0f;
  heavy.inertia[2] = 10.0f;
  check(control(physics, heavy), "an inertia is set");
  impulseAt(physics, 1, 0.0f, 0.0f, 1.0f, 0.5f, 0.0f, 0.0f);
  impulseAt(physics, 2, 0.0f, 0.0f, 1.0f, 5.5f, 0.0f, 0.0f);
  check(near(velocityOf(physics, 1, 4), -3.0f, 0.05f),
        "a box takes its inertia from its shape");
  check(near(velocityOf(physics, 2, 4), -0.05f, 0.005f), "and from the one it is given");

  OrblitPhysicsControls partial = controlsOf(2);
  partial.inertia[0] = 10.0f;
  control(physics, partial);
  impulseAt(physics, 2, 0.0f, 0.0f, 1.0f, 5.5f, 0.0f, 0.0f);
  check(near(velocityOf(physics, 2, 4), -0.05f - 3.0f, 0.1f),
        "an inertia with an axis missing is not used, and the shape's is");
  orblit_physics_destroy(physics);
}

void controlling() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, slabAt(1, 0.5f, 0.5f, 0.5f, 5.0f, 5.0f, 0.0f));
  check(!control(physics, controlsOf(9)), "controls for a body that is not there are refused");
  OrblitPhysicsControls nan = controlsOf(1);
  nan.maxSpeed = std::nanf("");
  check(!control(physics, nan), "and so are controls with NaN in them");
  OrblitPhysicsControls centre = controlsOf(1);
  centre.centre[2] = std::nanf("");
  check(!control(physics, centre), "in the centre too");
  OrblitPhysicsControls negative = controlsOf(1);
  negative.maxSpin = -1.0f;
  check(!control(physics, negative), "a negative cap is refused");
  OrblitPhysicsControls unknown = controlsOf(1);
  unknown.locks = 0x100;
  check(!control(physics, unknown), "and a lock that does not exist");
  check(!orblit_physics_controls(physics, nullptr), "and none at all");
  check(!orblit_physics_controls(nullptr, &centre), "and controls in no world are survivable");

  OrblitPhysicsControls light = controlsOf(1);
  light.gravityScale = 0.0f;
  check(control(physics, light), "a fixed body takes controls");
  switchTo(physics, 1, ORBLIT_PHYSICS_DYNAMIC);
  run(physics, 1.0f);
  check(near(heightOf(physics, 1), 5.0f, 0.001f), "and keeps them when it is made free");
  orblit_physics_destroy(physics);
}

void wakingControls() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  run(physics, 2.0f);
  check(orblit_physics_asleep(physics, 2), "a ball at rest sleeps");
  OrblitPhysicsControls lift = controlsOf(2);
  lift.gravityScale = -1.0f;
  control(physics, lift);
  check(!orblit_physics_asleep(physics, 2), "and is woken by a change to how it moves");
  orblit_physics_destroy(physics);
}

// --- seeing what happened ------------------------------------------------- //

/// What a replay has to give back: where everything is, how it moves and
/// whether it sleeps, then what the step reported and what touched. Kept as
/// floats and compared as bytes, so a difference in the last bit of anything
/// is a difference here.
void record(OrblitPhysics *physics, const std::vector<OrblitPhysicsId> &ids,
            std::vector<float> &trace) {
  for (const OrblitPhysicsId id : ids) {
    float transform[7] = {0};
    float motion[6] = {0};
    orblit_physics_transform(physics, id, transform);
    orblit_physics_velocity(physics, id, motion);
    trace.insert(trace.end(), transform, transform + 7);
    trace.insert(trace.end(), motion, motion + 6);
    trace.push_back(orblit_physics_asleep(physics, id) ? 1.0f : 0.0f);
  }

  uint32_t count = 0;
  const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
  trace.push_back(static_cast<float>(count));
  for (uint32_t i = 0; i < count; ++i) {
    trace.push_back(static_cast<float>(events[i].kind));
    trace.push_back(static_cast<float>(events[i].a));
    trace.push_back(static_cast<float>(events[i].b));
    trace.insert(trace.end(), events[i].at, events[i].at + 3);
    trace.insert(trace.end(), events[i].normal, events[i].normal + 3);
    trace.push_back(events[i].force);
  }

  // Everything but the time, which is about the machine and not the world.
  OrblitPhysicsStats stats{};
  orblit_physics_stats(physics, &stats);
  for (const uint32_t counted :
       {stats.bodies, stats.staticBodies, stats.kinematicBodies, stats.dynamicBodies,
        stats.asleep, stats.triggers, stats.characters, stats.joints, stats.zones,
        stats.rules, stats.pairs, stats.touching, stats.points}) {
    trace.push_back(static_cast<float>(counted));
  }

  const uint32_t points = orblit_physics_contacts(physics, nullptr, 0);
  std::vector<OrblitPhysicsContact> contacts(points);
  orblit_physics_contacts(physics, contacts.data(), points);
  trace.push_back(static_cast<float>(points));
  for (const OrblitPhysicsContact &contact : contacts) {
    trace.push_back(static_cast<float>(contact.a));
    trace.push_back(static_cast<float>(contact.b));
    trace.insert(trace.end(), contact.at, contact.at + 3);
    trace.insert(trace.end(), contact.normal, contact.normal + 3);
    trace.push_back(contact.depth);
    trace.push_back(contact.impulse);
  }
}

bool same(const std::vector<float> &a, const std::vector<float> &b) {
  if (a.size() != b.size()) return false;
  return a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

/// A world with one of everything a snapshot has to hold: a stack, each shape,
/// a body asleep, a ball on ground that is not flat, two joints, a trigger
/// inside a zone, a rule and a character. Returns the ids worth watching.
std::vector<OrblitPhysicsId> richWorld(OrblitPhysics *physics) {
  // A hull whose origin is at its foot, so its weight is not where it is placed.
  const float foot[3] = {-0.3f, 0.0f, -0.3f};
  const float top[3] = {0.3f, 0.8f, 0.3f};
  layHull(physics, 70, cornersOf(foot, top));

  OrblitPhysicsCommand bottom = boxAt(2, 0.5f, 0.0f, 0.5f, 0.0f);
  bottom.stay = true;
  OrblitPhysicsCommand sleeper = sphereAt(7, 0.3f, 2.0f, 0.3f, 2.0f);
  sleeper.asleep = true;
  OrblitPhysicsCommand area = triggerAt(20, 1.5f, -3.0f, 2.0f, 0.0f);
  area.stay = true;

  const std::vector<OrblitPhysicsCommand> made = {
      groundPlane(1),
      bottom,
      boxAt(3, 0.5f, 0.05f, 1.6f, 0.0f),
      boxAt(4, 0.5f, -0.05f, 2.8f, 0.0f),
      sphereAt(5, 0.3f, 0.3f, 5.0f, 0.1f),
      capsuleAt(6, 0.25f, 0.5f, -0.4f, 4.0f, 0.2f),
      sleeper,
      sphereAt(8, 0.3f, 2.0f, 0.3f, -2.0f),
      boxAt(10, 0.25f, 4.0f, 2.0f, -3.0f),
      boxAt(12, 0.25f, -2.0f, 3.0f, 3.0f),
      boxAt(13, 0.25f, -2.0f, 3.6f, 3.0f),
      area,
      sphereAt(21, 0.25f, -3.0f, 6.0f, 0.0f),
      boxAt(22, 0.3f, -4.0f, 0.3f, -2.0f),
      sphereAt(40, 0.3f, 9.0f, 4.0f, 0.0f),
      hullAt(60, 70, -6.0f, 2.0f, -2.0f),
      cylinderAt(61, 0.3f, 0.4f, -6.0f, 4.0f, 2.0f),
      hullAt(62, 70, 11.0f, 5.0f, 1.0f),
      cylinderAt(63, 0.3f, 0.4f, 7.0f, 5.0f, 2.0f),
  };
  std::vector<OrblitPhysicsId> ids;
  for (const OrblitPhysicsCommand &command : made) {
    submit(physics, command);
    ids.push_back(command.id);
  }
  addCharacter(physics, 30, 4.0f, 0.0f, 3.0f);
  ids.push_back(30);

  drive(physics, 8, 0.0f, 0.0f, 4.0f, 0.0f);
  drive(physics, 10, 2.0f, 0.0f, 0.0f, 0.0f);
  drive(physics, 22, 3.0f, 0.0f, 0.0f, 0.0f);

  const std::vector<float> heights = sampled(9, 9, 1.0f, 6.0f, -4.0f, bumps);
  const OrblitPhysicsGround ground = groundOf(41, heights, 9, 9, 6.0f, -4.0f);
  const OrblitPhysicsZone lift = zoneOn(20, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, 2.0f);
  const OrblitPhysicsRule ice = ruleFor(1, 22, ORBLIT_PHYSICS_RULE_FRICTION);
  const bool built =
      orblit_physics_ground(physics, &ground) &&
      join(physics, jointOf(100, ORBLIT_PHYSICS_JOINT_POINT, 10, 0, 4.0f, 3.0f, -3.0f)) &&
      join(physics, jointOf(101, ORBLIT_PHYSICS_JOINT_FIXED, 12, 13, -2.0f, 3.3f, 3.0f)) &&
      orblit_physics_zone(physics, &lift) && orblit_physics_rule(physics, &ice);
  check(built, "the world to replay has its ground, joints, zone and rule");
  return ids;
}

/// One step of what a caller does: walks the character, and at fixed steps
/// makes and removes things, so the steps after a restore differ from the
/// steps before it in more than where things are.
void play(OrblitPhysics *physics, int at) {
  const OrblitPhysicsFooting footing = footingOf(physics, 30);
  drive(physics, 30, 1.0f, footing.velocity[1] - 9.81f * kStep, 0.0f, 0.0f);
  if (at == 55) submit(physics, sphereAt(50, 0.25f, 0.1f, 6.0f, 0.1f));
  if (at == 60) destroyBody(physics, 3);
  if (at == 75) destroyBody(physics, 2);
  step(physics);
}

void replaying() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const std::vector<OrblitPhysicsId> ids = richWorld(physics);
  for (int at = 0; at < 50; ++at) play(physics, at);

  OrblitPhysicsSnapshot *fifty = orblit_physics_snapshot(physics);
  check(fifty != nullptr, "a world gives a snapshot of itself");
  const bool asleepAtFifty = orblit_physics_asleep(physics, 7);
  const uint32_t bodiesAtFifty = orblit_physics_count(physics);
  std::vector<float> atFifty;
  record(physics, ids, atFifty);

  std::vector<float> first;
  for (int at = 50; at < 100; ++at) {
    play(physics, at);
    record(physics, ids, first);
  }
  std::vector<float> firstEnd;
  record(physics, ids, firstEnd);
  check(asleepAtFifty && !orblit_physics_asleep(physics, 7),
        "the run does something: a body asleep at step 50 is woken by step 100");
  check(orblit_physics_alive(physics, 50) && !orblit_physics_alive(physics, 3),
        "and changes what is there");

  check(orblit_physics_restore(physics, fifty), "a snapshot restores");
  check(orblit_physics_count(physics) == bodiesAtFifty &&
            !orblit_physics_alive(physics, 50) && orblit_physics_alive(physics, 3) &&
            orblit_physics_asleep(physics, 7),
        "and what was made since is gone, what was removed is back, and the sleeper sleeps");
  std::vector<float> restored;
  record(physics, ids, restored);
  check(same(atFifty, restored),
        "and before a step, what it reports and lists is what the snapshot's step did");

  std::vector<float> again;
  for (int at = 50; at < 100; ++at) {
    play(physics, at);
    record(physics, ids, again);
  }
  std::vector<float> againEnd;
  record(physics, ids, againEnd);
  check(same(firstEnd, againEnd),
        "100 steps, restored at step 50, reach an identical step 100");
  check(same(first, again),
        "and every step between, with its events and contacts, came out the same");

  orblit_physics_restore(physics, fifty);
  std::vector<float> thrice;
  for (int at = 50; at < 100; ++at) {
    play(physics, at);
    record(physics, ids, thrice);
  }
  check(same(first, thrice), "a snapshot is left as it was and restores again");

  // Into a world that was made differently, and after the first is gone: the
  // snapshot carries the settings and the ground, not a borrow of either.
  orblit_physics_destroy(physics);
  OrblitPhysicsSettings other;
  orblit_physics_defaults(&other);
  other.gravity[1] = -1.0f;
  other.velocitySteps = 2;
  other.positionSteps = 1;
  other.sleeping = false;
  OrblitPhysics *second = orblit_physics_create(&other);
  check(orblit_physics_restore(second, fifty), "a snapshot restores into another world");
  // Let go of the snapshot before the world steps, so the ground it was sharing
  // has to be the world's own by now.
  orblit_physics_snapshot_destroy(fifty);
  std::vector<float> elsewhere;
  for (int at = 50; at < 100; ++at) {
    play(second, at);
    record(second, ids, elsewhere);
  }
  check(same(first, elsewhere),
        "which then does what the first did, though it began with other settings");

  orblit_physics_destroy(second);
}

void refusingSnapshots() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  check(orblit_physics_snapshot(nullptr) == nullptr, "no world gives no snapshot");

  OrblitPhysicsSnapshot *empty = orblit_physics_snapshot(physics);
  check(empty != nullptr, "an empty world gives one");
  check(!orblit_physics_restore(nullptr, empty), "no world takes one");
  check(!orblit_physics_restore(physics, nullptr), "and no snapshot restores nothing");
  orblit_physics_snapshot_destroy(nullptr);

  submit(physics, groundPlane(1));
  submit(physics, sphereAt(2, 0.5f, 0.0f, 3.0f, 0.0f));
  run(physics, 0.2f);
  orblit_physics_restore(physics, empty);
  check(orblit_physics_count(physics) == 0 && !orblit_physics_alive(physics, 1),
        "restoring an empty world empties a busy one");
  check(orblit_physics_contacts(physics, nullptr, 0) == 0,
        "and its contacts with it");
  orblit_physics_snapshot_destroy(empty);

  OrblitPhysicsStats stats;
  std::memset(&stats, 0xFF, sizeof stats);
  orblit_physics_stats(nullptr, &stats);
  check(stats.bodies == 0 && stats.points == 0 && stats.stepMicroseconds == 0.0f,
        "the counts of no world are zero");
  orblit_physics_stats(physics, nullptr);
  check(orblit_physics_contacts(nullptr, nullptr, 0) == 0,
        "and the contacts of no world are none");
  orblit_physics_destroy(physics);
}

struct Listed {
  std::vector<OrblitPhysicsContact> points;
  uint32_t total;
};

Listed listContacts(OrblitPhysics *physics) {
  Listed listed;
  listed.total = orblit_physics_contacts(physics, nullptr, 0);
  listed.points.resize(listed.total);
  orblit_physics_contacts(physics, listed.points.data(), listed.total);
  return listed;
}

float impulseOf(const Listed &listed, OrblitPhysicsId a, OrblitPhysicsId b) {
  float sum = 0.0f;
  for (const OrblitPhysicsContact &point : listed.points) {
    if (point.a == a && point.b == b) sum += point.impulse;
  }
  return sum;
}

bool listsPair(const Listed &listed, OrblitPhysicsId a, OrblitPhysicsId b) {
  for (const OrblitPhysicsContact &point : listed.points) {
    if (point.a == a && point.b == b) return true;
  }
  return false;
}

void listingContacts() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  submit(physics, sphereAt(3, 0.5f, 3.0f, 0.5f, 0.0f));
  submit(physics, sphereAt(4, 0.5f, -3.0f, 0.5f, 0.0f));
  check(orblit_physics_contacts(physics, nullptr, 0) == 0,
        "before a step there is nothing to list");

  run(physics, 0.3f);
  const Listed listed = listContacts(physics);
  check(listed.total >= 3, "three bodies resting on ground make at least three points");

  bool ordered = true;
  bool facing = true;
  for (size_t i = 0; i < listed.points.size(); ++i) {
    const OrblitPhysicsContact &point = listed.points[i];
    facing = facing && point.a == 1 && point.b >= 2 && point.b <= 4 &&
             point.normal[1] < -0.9f;
    if (i == 0) continue;
    const OrblitPhysicsContact &before = listed.points[i - 1];
    ordered = ordered && (before.a < point.a ||
                          (before.a == point.a && before.b <= point.b));
  }
  check(ordered, "points come in the order of their pairs' ids");
  check(facing, "each names the ground first, with the normal out of the body and down");
  check(near(impulseOf(listed, 1, 3), 9.81f / 60.0f, 0.05f),
        "and the impulse over a step that holds a kilogram up is its weight times the step");

  OrblitPhysicsStats stats{};
  orblit_physics_stats(physics, &stats);
  check(stats.points == listed.total && stats.touching == 3,
        "the counts agree with the list");

  OrblitPhysicsContact room[3];
  std::memset(room, 0xAB, sizeof room);
  const uint32_t reported = orblit_physics_contacts(physics, room, 2);
  check(reported == listed.total && room[0].a == 1 && room[1].a == 1 &&
            room[2].a == 0xABABABABABABABABull,
        "asked for two, it writes two and says how many there were");

  destroyBody(physics, 2);
  const Listed after = listContacts(physics);
  check(!listsPair(after, 1, 2) && listsPair(after, 1, 3) && listsPair(after, 1, 4),
        "a body removed takes its pairs with it, and no other pair is lost when a row moves");
  check(impulseOf(after, 1, 4) == impulseOf(listed, 1, 4) &&
            near(after.points.back().normal[1], -1.0f, 0.1f),
        "and what is left is what it was");

  run(physics, 2.0f);
  check(orblit_physics_asleep(physics, 3) && orblit_physics_contacts(physics, nullptr, 0) == 0,
        "a pair that has gone to sleep is not listed, since nothing looked at it");
  orblit_physics_destroy(physics);
}

void counting() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  OrblitPhysicsStats stats{};
  orblit_physics_stats(physics, &stats);
  check(stats.bodies == 0 && stats.pairs == 0 && stats.stepMicroseconds == 0.0f,
        "an empty world counts nothing");

  OrblitPhysicsCommand mover = boxAt(4, 0.5f, 10.0f, 5.0f, 0.0f);
  mover.motion = ORBLIT_PHYSICS_KINEMATIC;
  submit(physics, groundPlane(1));
  submit(physics, boxAt(2, 0.5f, 0.0f, 0.5f, 0.0f));
  submit(physics, sphereAt(3, 0.5f, 3.0f, 0.5f, 0.0f));
  submit(physics, mover);
  submit(physics, triggerAt(5, 1.0f, -6.0f, 1.0f, 0.0f));
  submit(physics, boxAt(8, 0.25f, 20.0f, 5.0f, 0.0f));
  addCharacter(physics, 6, 6.0f, 0.0f, 6.0f);
  join(physics, jointOf(7, ORBLIT_PHYSICS_JOINT_POINT, 8, 0, 20.0f, 6.0f, 0.0f));
  const OrblitPhysicsZone zone = zoneOn(5, ORBLIT_PHYSICS_ZONE_GRAVITY, 0, 0.0f);
  const OrblitPhysicsRule rule = ruleFor(1, 2, ORBLIT_PHYSICS_RULE_FRICTION);
  orblit_physics_zone(physics, &zone);
  orblit_physics_rule(physics, &rule);

  run(physics, 0.3f);
  orblit_physics_stats(physics, &stats);
  check(stats.bodies == 7 && stats.staticBodies == 2 && stats.kinematicBodies == 2 &&
            stats.dynamicBodies == 3,
        "bodies are counted by what moves them, and the three add up");
  check(stats.triggers == 1 && stats.characters == 1 && stats.joints == 1 &&
            stats.zones == 1 && stats.rules == 1 && stats.asleep == 0,
        "and triggers, characters, joints, zones, rules and sleepers each by themselves");
  check(stats.pairs >= stats.touching && stats.touching >= 2 && stats.points >= stats.touching,
        "a step looked at pairs, some touched, and each made points");
  check(stats.stepMicroseconds > 0.0f, "and took a measurable time");

  OrblitPhysicsSnapshot *snapshot = orblit_physics_snapshot(physics);
  run(physics, 0.1f);
  OrblitPhysicsStats ran{};
  orblit_physics_stats(physics, &ran);
  orblit_physics_restore(physics, snapshot);
  orblit_physics_stats(physics, &stats);
  check(stats.bodies == 7 && stats.pairs > 0, "a restore brings the counts back");
  check(stats.stepMicroseconds > 0.0f && stats.stepMicroseconds == ran.stepMicroseconds,
        "while the time stays that of the last step on this handle");

  run(physics, 2.0f);
  orblit_physics_stats(physics, &stats);
  check(stats.asleep >= 2, "bodies that have come to rest are counted asleep");
  orblit_physics_snapshot_destroy(snapshot);
  orblit_physics_destroy(physics);
}

void endingInOrder() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  const int scrambled[] = {19, 4, 27, 2, 16, 9, 23, 6, 12, 3, 21, 8};
  for (const int id : scrambled) {
    submit(physics, sphereAt(static_cast<OrblitPhysicsId>(id), 0.5f, id * 2.0f, 0.5f, 0.0f));
  }
  run(physics, 0.2f);

  destroyBody(physics, 1);
  step(physics);
  uint32_t count = 0;
  const OrblitPhysicsEvent *events = orblit_physics_events(physics, &count);
  uint32_t ended = 0;
  bool sorted = true;
  OrblitPhysicsId last = 0;
  for (uint32_t i = 0; i < count; ++i) {
    if (events[i].kind != ORBLIT_PHYSICS_TOUCH_ENDED) continue;
    sorted = sorted && events[i].b > last;
    last = events[i].b;
    ended++;
  }
  check(ended == 12, "taking the ground away ends every touch on the one step");
  check(sorted, "and they are reported in the order of their pairs");
  orblit_physics_destroy(physics);
}

} // namespace

// --- convex shapes -------------------------------------------------------- //
//
// Cylinders and hulls: laid, made, rested on each kind of thing, rolled,
// weighed, cast, and put back by a snapshot. Every check goes through the
// header, so the narrowphase under it is judged by what the solver does with it.

OrblitPhysicsCommand fixedBody(OrblitPhysicsCommand command) {
  command.motion = ORBLIT_PHYSICS_STATIC;
  return command;
}

/// How many points a pair touched at.
uint32_t pointsOf(const Listed &listed, OrblitPhysicsId a, OrblitPhysicsId b) {
  uint32_t count = 0;
  for (const OrblitPhysicsContact &point : listed.points) {
    if (point.a == a && point.b == b) ++count;
  }
  return count;
}

/// The distance between the closest two points a pair touched at, which is how
/// well they hold a body up: four at the corners of a face hold it, four heaped
/// in one place hold nothing but one point.
float spreadOf(const Listed &listed, OrblitPhysicsId a, OrblitPhysicsId b) {
  float least = 1.0e9f;
  for (size_t i = 0; i < listed.points.size(); ++i) {
    for (size_t j = i + 1; j < listed.points.size(); ++j) {
      const OrblitPhysicsContact &p = listed.points[i];
      const OrblitPhysicsContact &q = listed.points[j];
      if (p.a != a || p.b != b || q.a != a || q.b != b) continue;
      const float dx = p.at[0] - q.at[0];
      const float dy = p.at[1] - q.at[1];
      const float dz = p.at[2] - q.at[2];
      least = std::fmin(least, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
  }
  return least;
}

void layingHulls() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const std::vector<float> cube = cubeOf(0.5f);
  check(layHull(physics, 1, cube), "a hull is laid from the corners of a cube");
  check(!layHull(physics, 1, cube), "and not again under the same id");
  check(!layHull(physics, 0, cube), "or under no id");
  check(!orblit_physics_hull(nullptr, 2, cube.data(), 8), "or into no world");
  check(!orblit_physics_hull(physics, 2, nullptr, 8), "or from no points");
  check(!orblit_physics_hull(physics, 2, cube.data(), 3), "or from fewer than four");

  const float square[12] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
  check(!orblit_physics_hull(physics, 2, square, 4), "or from points on one plane");
  const float row[12] = {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0};
  check(!orblit_physics_hull(physics, 2, row, 4), "or on one line");
  std::vector<float> broken = cube;
  broken[4] = std::nanf("");
  check(!layHull(physics, 2, broken), "or from a point that is not a number");
  std::vector<float> crowd;
  for (size_t i = 0; i < 3 * 100001; ++i) crowd.push_back(cube[i % cube.size()]);
  check(!layHull(physics, 2, crowd), "or from more than a hundred thousand points");
  check(!orblit_physics_alive(physics, 2), "and a refused hull makes no body either");

  submit(physics, hullAt(5, 99, 0.0f, 1.0f, 0.0f));
  check(!orblit_physics_alive(physics, 5), "a body that names a hull never laid is not made");
  submit(physics, hullAt(6, 1, 0.0f, 1.0f, 0.0f));
  check(orblit_physics_alive(physics, 6), "and one that names a hull that was is");

  check(!orblit_physics_hull_drop(physics, 1), "a hull a body is made from is not taken away");
  destroyBody(physics, 6);
  check(!orblit_physics_hull_drop(nullptr, 1) && !orblit_physics_hull_drop(physics, 77),
        "nor a hull in no world, nor one that was never laid");
  check(orblit_physics_hull_drop(physics, 1), "but once the body is gone it is");
  check(!orblit_physics_hull_drop(physics, 1), "and only once");
  submit(physics, hullAt(7, 1, 0.0f, 1.0f, 0.0f));
  check(!orblit_physics_alive(physics, 7), "a body that names a hull taken away is not made");
  check(layHull(physics, 1, cubeOf(0.25f)), "and the id is free to name another");
  orblit_physics_destroy(physics);
}

void refusingCylinders() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  const float nothing = std::nanf("");
  const float endless = HUGE_VALF;
  submit(physics, cylinderAt(1, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f));
  submit(physics, cylinderAt(2, 0.0f, 0.5f, 3.0f, 1.0f, 0.0f));
  submit(physics, cylinderAt(3, 0.5f, 0.0f, 6.0f, 1.0f, 0.0f));
  submit(physics, cylinderAt(4, -0.5f, 0.5f, 9.0f, 1.0f, 0.0f));
  submit(physics, cylinderAt(5, 0.5f, nothing, 12.0f, 1.0f, 0.0f));
  submit(physics, cylinderAt(6, endless, 0.5f, 15.0f, 1.0f, 0.0f));
  check(orblit_physics_alive(physics, 1), "a cylinder with a radius and a length is made");
  check(!orblit_physics_alive(physics, 2) && !orblit_physics_alive(physics, 3) &&
            !orblit_physics_alive(physics, 4),
        "one with no radius, no length or a negative one is not");
  check(!orblit_physics_alive(physics, 5) && !orblit_physics_alive(physics, 6),
        "nor one whose size is not a number or is not finite");

  OrblitPhysicsCast thin = rayFrom(-5.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);
  thin.shape = ORBLIT_PHYSICS_CYLINDER;
  thin.size[0] = 0.0f;
  thin.size[1] = 0.5f;
  OrblitPhysicsHit hit{};
  check(!orblit_physics_cast(physics, &thin, &hit), "a cast of one with no radius meets nothing");
  thin.size[0] = 0.25f;
  check(orblit_physics_cast(physics, &thin, &hit) && hit.body == 1,
        "and the same cast with a radius meets the cylinder");
  orblit_physics_destroy(physics);
}

void restingConvex() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  layHull(physics, 1, cubeOf(0.5f));
  const float foot[3] = {-0.5f, 0.0f, -0.5f};
  const float top[3] = {0.5f, 1.0f, 0.5f};
  layHull(physics, 2, cornersOf(foot, top));

  submit(physics, hullAt(2, 1, 0.0f, 0.51f, 0.0f));
  submit(physics, cylinderAt(3, 0.5f, 0.5f, 4.0f, 0.51f, 0.0f));
  OrblitPhysicsCommand lying = cylinderAt(4, 0.5f, 0.5f, -4.0f, 0.51f, 0.0f);
  layDown(lying);
  submit(physics, lying);
  submit(physics, hullAt(5, 2, 8.0f, 0.01f, 0.0f));

  run(physics, 0.3f);
  const Listed listed = listContacts(physics);
  check(pointsOf(listed, 1, 2) == 4 && spreadOf(listed, 1, 2) > 0.8f,
        "a hull cube on the ground rests on four points, one at each corner");
  check(pointsOf(listed, 1, 3) >= 3 && spreadOf(listed, 1, 3) > 0.5f,
        "a cylinder on its end rests on points spread round the rim");
  check(pointsOf(listed, 1, 4) >= 2 && spreadOf(listed, 1, 4) > 0.5f,
        "and on its side on the two ends of a line");
  bool downward = true;
  for (const OrblitPhysicsContact &point : listed.points) {
    downward = downward && point.a == 1 && point.normal[1] < -0.99f;
  }
  check(downward, "with the normal straight into the ground, never off to one side");

  run(physics, 3.0f);
  check(near(heightOf(physics, 2), 0.5f, 0.02f) && levelOf(physics, 2) > 0.999f,
        "the cube comes to rest on its face at the height of its half-size");
  check(near(heightOf(physics, 3), 0.5f, 0.02f) && levelOf(physics, 3) > 0.999f,
        "the cylinder on its end at its half-length");
  check(near(heightOf(physics, 4), 0.5f, 0.02f),
        "and on its side at its radius, which is the same for this one");
  check(near(heightOf(physics, 5), 0.0f, 0.02f) && levelOf(physics, 5) > 0.999f,
        "a hull whose origin is at its foot is placed by its foot");
  check(orblit_physics_asleep(physics, 2) && orblit_physics_asleep(physics, 3) &&
            orblit_physics_asleep(physics, 4) && orblit_physics_asleep(physics, 5),
        "and every one of them goes to sleep");
  orblit_physics_destroy(physics);
}

void restingOnConvex() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  layHull(physics, 1, cubeOf(0.5f));
  submit(physics, groundPlane(1));
  submit(physics, fixedBody(hullAt(2, 1, 0.0f, 0.5f, 0.0f)));
  submit(physics, fixedBody(cylinderAt(3, 0.5f, 0.5f, 6.0f, 0.5f, 0.0f)));
  submit(physics, fixedBody(boxAt(4, 1.0f, -6.0f, 1.0f, 0.0f)));

  // A cube turned half a radian about its own up, on another's top face.
  OrblitPhysicsCommand turned = hullAt(5, 1, 0.2f, 2.0f, 0.1f);
  turned.rotation[1] = std::sin(0.3f);
  turned.rotation[3] = std::cos(0.3f);
  submit(physics, turned);
  submit(physics, cylinderAt(6, 0.5f, 0.5f, 6.3f, 2.5f, 0.1f));
  submit(physics, cylinderAt(7, 0.5f, 0.5f, -5.7f, 4.0f, 0.1f));
  submit(physics, sphereAt(8, 0.2f, 6.2f, 4.0f, 0.0f));
  submit(physics, boxAt(9, 0.3f, 0.0f, 4.0f, 0.2f));
  submit(physics, hullAt(10, 1, -5.8f, 5.0f, 0.0f));
  run(physics, 4.0f);

  check(near(heightOf(physics, 5), 1.5f, 0.03f) && levelOf(physics, 5) > 0.999f,
        "a cube dropped twisted onto another lies flat on its top");
  check(near(coordinateOf(physics, 5, 0), 0.2f, 0.03f) && orblit_physics_asleep(physics, 5),
        "and stays where it landed");
  check(near(heightOf(physics, 6), 1.5f, 0.03f) && levelOf(physics, 6) > 0.999f,
        "a cylinder dropped onto a standing cylinder sits on its flat end");
  check(near(heightOf(physics, 7), 2.5f, 0.03f) && levelOf(physics, 7) > 0.999f,
        "and one dropped onto a box lies flat on the box's face");
  check(heightOf(physics, 8) > 1.9f,
        "and a ball dropped on the cylinder that sits on the other cylinder rests above it");
  check(near(heightOf(physics, 9), 0.3f, 0.03f) || heightOf(physics, 9) > 1.0f,
        "a box dropped beside the stack reaches the ground or the stack, not through it");
  check(heightOf(physics, 10) > 3.0f && levelOf(physics, 10) > 0.999f,
        "a hull dropped onto the cylinder that is on the box rests on top of it");
  orblit_physics_destroy(physics);
}

void restingOnGround() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  lay(physics, 1, incline);
  layHull(physics, 1, cubeOf(0.4f));
  // Set down flush with the slope. Dropped on their ends they would topple, and
  // a cylinder on its side would roll, which is the right answer to a different
  // question.
  const float lean = std::atan(0.3f);
  const float outX = -std::sin(lean), outY = std::cos(lean);
  OrblitPhysicsCommand cube = hullAt(2, 1, outX * 0.405f, outY * 0.405f, 0.0f);
  OrblitPhysicsCommand standing = cylinderAt(3, 0.4f, 0.4f, outX * 0.405f, outY * 0.405f, 3.0f);
  cube.rotation[2] = standing.rotation[2] = std::sin(lean / 2.0f);
  cube.rotation[3] = standing.rotation[3] = std::cos(lean / 2.0f);
  submit(physics, cube);
  submit(physics, standing);
  OrblitPhysicsCommand lying = cylinderAt(4, 0.4f, 0.4f, 0.0f, 1.0f, -3.0f);
  layDown(lying);
  submit(physics, lying);
  run(physics, 5.0f);

  // 0.3 in a metre: held by friction of one half, and level with the slope.
  const float tilt = 1.0f / std::sqrt(1.09f);
  check(speedOf(physics, 2) < 0.05f && near(levelOf(physics, 2), tilt, 0.01f),
        "a cube on a slope of three in ten lies along it and stays");
  check(speedOf(physics, 3) < 0.05f && near(levelOf(physics, 3), tilt, 0.01f),
        "a cylinder on its end does the same");
  check(near(coordinateOf(physics, 2, 0), -0.12f, 0.05f) &&
            near(coordinateOf(physics, 3, 0), -0.12f, 0.05f),
        "and neither has slid down it");
  check(heightOf(physics, 4) > -0.2f && heightOf(physics, 4) < 1.0f,
        "while one on its side is on the slope, not through it");
  orblit_physics_destroy(physics);

  // Through a crease, the way a box is, and across the seams of a flat field.
  OrblitPhysics *vale = orblit_physics_create(nullptr);
  lay(vale, 1, valley);
  layHull(vale, 1, cubeOf(0.5f));
  submit(vale, hullAt(2, 1, 0.0f, 3.0f, -1.7f));
  submit(vale, cylinderAt(3, 0.4f, 0.5f, 0.0f, 3.0f, 1.8f));
  run(vale, 4.0f);
  check(heightOf(vale, 2) > 0.4f && speedOf(vale, 2) < 0.05f && near(coordinateOf(vale, 2, 0), 0.0f, 0.05f),
        "a cube dropped in a valley sits across the crease, held up by both slopes");
  check(heightOf(vale, 3) > 0.4f && speedOf(vale, 3) < 0.05f && near(coordinateOf(vale, 3, 0), 0.0f, 0.05f),
        "and a cylinder does not catch on the crease or sink into either side");
  orblit_physics_destroy(vale);

  OrblitPhysics *seams = orblit_physics_create(nullptr);
  lay(seams, 1, flat);
  layHull(seams, 1, cubeOf(0.5f));
  submit(seams, hullAt(2, 1, -3.0f, 0.5f, 0.0f));
  submit(seams, cylinderAt(3, 0.5f, 0.5f, -3.0f, 0.5f, 2.0f));
  run(seams, 0.5f);
  drive(seams, 2, 4.0f, 0.0f, 0.0f, 0.0f);
  drive(seams, 3, 4.0f, 0.0f, 0.0f, 0.0f);
  float hopped = 0.0f;
  float tripped = 1.0f;
  for (int i = 0; i < 60; ++i) {
    step(seams);
    hopped = std::fmax(hopped, std::fmax(heightOf(seams, 2), heightOf(seams, 3)) - 0.5f);
    tripped = std::fmin(tripped, std::fmin(levelOf(seams, 2), levelOf(seams, 3)));
  }
  check(hopped < 0.02f && tripped > 0.999f && coordinateOf(seams, 2, 0) > -2.0f &&
            coordinateOf(seams, 3, 0) > -2.0f,
        "a cube and a cylinder slid across the seams of flat ground neither hop nor trip");
  orblit_physics_destroy(seams);
}

void rollingCylinders() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  OrblitPhysicsCommand roller = cylinderAt(2, 0.5f, 0.5f, 0.0f, 0.5f, 0.0f);
  layDown(roller);
  submit(physics, roller);
  submit(physics, cylinderAt(3, 0.5f, 0.5f, 4.0f, 0.5f, 0.0f));
  run(physics, 0.3f);

  // On its side it rolls without slipping at v = w r. On its end it slides and
  // friction stops it a few tenths of a metre on.
  setMotion(physics, 2, 0.0f, 0.0f, 2.0f, 4.0f, 0.0f, 0.0f);
  setMotion(physics, 3, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
  run(physics, 2.0f);
  check(coordinateOf(physics, 2, 2) > 3.0f && near(heightOf(physics, 2), 0.5f, 0.02f),
        "a cylinder on its side rolls on, at its radius off the ground the whole way");
  check(coordinateOf(physics, 3, 2) < 1.0f && speedOf(physics, 3) < 0.05f,
        "while one on its end, pushed as hard, slides to a stop");
  orblit_physics_destroy(physics);
}

void weighingConvex() {
  // Torque over inertia, so what a push does shows the inertia the shape got.
  // A unit cube is a sixth about any axis, however it was made.
  OrblitPhysics *space = spaceWorld();
  layHull(space, 1, cubeOf(0.5f));
  submit(space, boxAt(2, 0.5f, 0.0f, 0.0f, 0.0f));
  submit(space, hullAt(3, 1, 10.0f, 0.0f, 0.0f));
  impulseAt(space, 2, 0.0f, 0.0f, 1.0f, 0.5f, 0.0f, 0.0f);
  impulseAt(space, 3, 0.0f, 0.0f, 1.0f, 10.5f, 0.0f, 0.0f);
  check(near(velocityOf(space, 3, 4), velocityOf(space, 2, 4), 0.02f) &&
            near(velocityOf(space, 3, 4), -3.0f, 0.05f),
        "a hull cube turns under a push just as a box does");

  // The same cube given by its foot is placed by its foot and weighs from its
  // middle: a push through the origin is half a metre below the weight.
  const float foot[3] = {-0.5f, 0.0f, -0.5f};
  const float top[3] = {0.5f, 1.0f, 0.5f};
  layHull(space, 2, cornersOf(foot, top));
  submit(space, hullAt(4, 2, 20.0f, 0.0f, 0.0f));
  impulseAt(space, 4, 0.0f, 0.0f, 1.0f, 20.0f, 0.0f, 0.0f);
  check(near(velocityOf(space, 4, 3), -3.0f, 0.05f) && near(velocityOf(space, 4, 2), 1.0f, 0.02f),
        "and a push at the origin of one whose weight is above it turns it");
  run(space, 0.5f);
  float middle[3];
  const float inside[3] = {0.0f, 0.5f, 0.0f};
  worldPointOf(space, 4, inside, middle);
  // Run keeps whole steps, so half a second is 29 of them, and damping takes a
  // little more. The middle still keeps its height: it would not if the solid
  // swung about its origin.
  check(near(middle[2], 0.48f, 0.03f) && near(middle[1], 0.5f, 0.02f) &&
            near(middle[0], 20.0f, 0.02f),
        "about the middle of the solid, which goes straight on");

  // A cylinder of radius a and half-length h, mass one: half a squared about
  // its axis, and (3 a squared + 4 h squared) over twelve about a diameter.
  submit(space, cylinderAt(5, 0.5f, 0.5f, 30.0f, 0.0f, 0.0f));
  impulseAt(space, 5, 0.0f, 0.0f, 1.0f, 30.5f, 0.0f, 0.0f);
  check(near(velocityOf(space, 5, 4), -0.5f / 0.125f, 0.05f),
        "a cylinder turns about its axis against half its radius squared");
  submit(space, cylinderAt(6, 0.5f, 0.5f, 40.0f, 0.0f, 0.0f));
  impulseAt(space, 6, 0.0f, 0.0f, 1.0f, 40.0f, 0.5f, 0.0f);
  check(near(velocityOf(space, 6, 3), 0.5f / (1.75f / 12.0f), 0.05f),
        "and about a diameter against a twelfth of three radii squared and the length squared");
  orblit_physics_destroy(space);
}

void walkingOnConvex() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  const float foot[3] = {-0.5f, 0.0f, -0.5f};
  const float kerb[3] = {0.5f, 0.2f, 0.5f};
  const float wall[3] = {0.5f, 1.0f, 0.5f};
  layHull(physics, 1, cornersOf(foot, kerb));
  layHull(physics, 2, cornersOf(foot, wall));
  submit(physics, fixedBody(hullAt(20, 1, 2.0f, 0.0f, 0.0f)));
  submit(physics, fixedBody(cylinderAt(21, 0.5f, 0.1f, 2.0f, 0.1f, 3.0f)));
  submit(physics, fixedBody(hullAt(22, 2, 2.0f, 0.0f, 6.0f)));
  submit(physics, fixedBody(cylinderAt(23, 0.5f, 1.0f, 2.0f, 1.0f, 9.0f)));
  for (int lane = 0; lane < 4; ++lane) {
    addCharacter(physics, 10 + lane, 0.0f, 0.0f, 3.0f * static_cast<float>(lane));
  }

  walk(physics, 10, 1.5f, 0.0f, 1.4f);
  check(footingOf(physics, 10).ground == 20 && near(heightOf(physics, 10), 1.11f, 0.01f),
        "a character steps up onto a hull kerb and stands on it");
  walk(physics, 11, 1.5f, 0.0f, 1.4f);
  check(footingOf(physics, 11).ground == 21 && near(heightOf(physics, 11), 1.11f, 0.01f),
        "and onto a cylinder that is as low");
  walk(physics, 12, 1.5f, 0.0f, 2.0f);
  check(coordinateOf(physics, 12, 0) > 1.15f && coordinateOf(physics, 12, 0) < 1.22f &&
            near(heightOf(physics, 12), 0.91f, 0.01f),
        "but is stopped by a hull wall too high to climb, on the ground in front of it");
  walk(physics, 13, 1.5f, 0.0f, 2.0f);
  check(coordinateOf(physics, 13, 0) > 1.15f && coordinateOf(physics, 13, 0) < 1.22f &&
            near(heightOf(physics, 13), 0.91f, 0.01f),
        "and by a cylinder that tall");
  orblit_physics_destroy(physics);
}

void castingConvex() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  layHull(physics, 1, cubeOf(0.5f));
  submit(physics, fixedBody(hullAt(2, 1, 0.0f, 0.5f, 0.0f)));
  submit(physics, fixedBody(cylinderAt(3, 0.5f, 0.5f, 10.0f, 0.5f, 0.0f)));

  OrblitPhysicsHit hit{};
  OrblitPhysicsCast ray = rayFrom(0.1f, 5.0f, 0.2f, 0.0f, -1.0f, 0.0f, 20.0f);
  check(orblit_physics_cast(physics, &ray, &hit) && hit.body == 2 &&
            near(hit.distance, 4.0f, 0.01f) && near(hit.normal[1], 1.0f, 0.01f),
        "a ray down onto a hull meets its top face");
  ray = rayFrom(-5.0f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);
  check(orblit_physics_cast(physics, &ray, &hit) && hit.body == 2 &&
            near(hit.distance, 4.5f, 0.01f) && near(hit.normal[0], -1.0f, 0.01f),
        "and a ray along the ground meets a side face");

  ray = rayFrom(10.1f, 5.0f, 0.1f, 0.0f, -1.0f, 0.0f, 20.0f);
  check(orblit_physics_cast(physics, &ray, &hit) && hit.body == 3 &&
            near(hit.distance, 4.0f, 0.01f) && near(hit.normal[1], 1.0f, 0.01f),
        "a ray down onto a cylinder meets its flat end");
  ray = rayFrom(10.3f, 0.5f, -5.0f, 0.0f, 0.0f, 1.0f, 20.0f);
  check(orblit_physics_cast(physics, &ray, &hit) && hit.body == 3 &&
            near(hit.distance, 4.6f, 0.01f) && near(hit.normal[0], 0.6f, 0.01f) &&
            near(hit.normal[2], -0.8f, 0.01f),
        "and one across it meets the round side, with the normal out of the axis");
  ray = rayFrom(10.55f, 0.5f, -5.0f, 0.0f, 0.0f, 1.0f, 20.0f);
  check(!orblit_physics_cast(physics, &ray, &hit), "a ray past the side of it meets nothing");

  OrblitPhysicsCast ball = rayFrom(0.1f, 5.0f, 0.2f, 0.0f, -1.0f, 0.0f, 20.0f);
  ball.shape = ORBLIT_PHYSICS_SPHERE;
  ball.size[0] = 0.25f;
  check(orblit_physics_cast(physics, &ball, &hit) && near(hit.distance, 3.75f, 0.01f),
        "a ball cast down onto a hull stops a radius above it");

  OrblitPhysicsCast drum = rayFrom(0.0f, 5.0f, 0.0f, 0.0f, -1.0f, 0.0f, 20.0f);
  drum.shape = ORBLIT_PHYSICS_CYLINDER;
  drum.size[0] = 0.25f;
  drum.size[1] = 0.25f;
  check(orblit_physics_cast(physics, &drum, &hit) && near(hit.distance, 3.75f, 0.01f),
        "and so does a cylinder cast, by its half-length");

  OrblitPhysicsCast crate = rayFrom(-4.0f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 20.0f);
  crate.shape = ORBLIT_PHYSICS_HULL;
  crate.hull = 1;
  check(orblit_physics_cast(physics, &crate, &hit) && hit.body == 2 &&
            near(hit.distance, 3.0f, 0.01f) && near(hit.normal[0], -1.0f, 0.01f),
        "a hull cast along the ground stops where its face meets the hull's");
  crate.from[0] = 4.0f;
  check(orblit_physics_cast(physics, &crate, &hit) && hit.body == 3 &&
            near(hit.distance, 5.0f, 0.01f),
        "and meets a cylinder by its round side");
  crate.hull = 99;
  check(!orblit_physics_cast(physics, &crate, &hit) &&
            hitsAlong(physics, crate, 4).empty() && !orblit_physics_cast_any(physics, &crate),
        "a cast of a hull that was never laid meets nothing");

  OrblitPhysicsCast here = rayFrom(0.2f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
  here.shape = ORBLIT_PHYSICS_HULL;
  here.hull = 1;
  check(holds(overlapping(physics, here, 4), {2}),
        "a hull standing where one is overlaps it");
  here.from[0] = 5.0f;
  check(overlapping(physics, here, 4).empty(), "and not one standing clear of it");
  here.hull = 99;
  here.from[0] = 0.2f;
  check(overlapping(physics, here, 4).empty(), "nor one that was never laid");
  orblit_physics_destroy(physics);
}

void snapshottingHulls() {
  OrblitPhysics *physics = orblit_physics_create(nullptr);
  submit(physics, groundPlane(1));
  layHull(physics, 1, cubeOf(0.5f));
  submit(physics, hullAt(2, 1, 0.0f, 3.0f, 0.0f));
  run(physics, 0.5f);
  OrblitPhysicsSnapshot *before = orblit_physics_snapshot(physics);

  run(physics, 0.3f);
  const float landed = heightOf(physics, 2);
  destroyBody(physics, 2);
  check(orblit_physics_hull_drop(physics, 1), "a hull with no body left is taken away");
  check(orblit_physics_restore(physics, before), "a snapshot restores");
  check(orblit_physics_alive(physics, 2), "with the body that was made from the hull");

  check(!orblit_physics_hull_drop(physics, 1),
        "and the hull the body stands on, which the world had let go of");
  submit(physics, hullAt(3, 1, 4.0f, 3.0f, 0.0f));
  check(orblit_physics_alive(physics, 3), "so another body may be made from it");
  run(physics, 0.3f);
  check(near(heightOf(physics, 2), landed, 1.0e-5f),
        "and the body falls as it did before");

  // Into another world, after the first is gone: the snapshot holds the hull
  // itself, not a borrow of the world's.
  orblit_physics_destroy(physics);
  OrblitPhysics *other = orblit_physics_create(nullptr);
  check(orblit_physics_restore(other, before), "a snapshot restores into another world");
  orblit_physics_snapshot_destroy(before);
  run(other, 0.3f);
  check(near(heightOf(other, 2), landed, 1.0e-5f),
        "which, with the snapshot let go of, still has the hull to stand on");
  orblit_physics_destroy(other);
}

void compoundChecks();
void meshChecks();

using Triple = std::array<double, 3>;

Triple operator+(const Triple &a, const Triple &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Triple operator*(const Triple &a, double k) {
  return {a[0] * k, a[1] * k, a[2] * k};
}

double dot(const Triple &a, const Triple &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Triple cross(const Triple &a, const Triple &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

/// `v` turned by the unit quaternion `q`, given as x, y, z, w.
Triple turned(const double q[4], const Triple &v) {
  const Triple u = {q[0], q[1], q[2]};
  const Triple t = cross(u, v) * 2.0;
  return v + t * q[3] + cross(u, t);
}

/// A turn about a random axis by a random angle, as x, y, z, w, from a
/// generator whose sequence the standard fixes.
void randomTurn(std::mt19937 &dice, double q[4]) {
  const auto between = [&dice] { return dice() / 4294967295.0 * 2.0 - 1.0; };
  Triple axis = {between(), between(), between() + 0.01};
  axis = axis * (1.0 / std::sqrt(dot(axis, axis)));
  const double angle = 3.0 * between();
  for (int i = 0; i < 3; ++i) q[i] = axis[i] * std::sin(angle / 2.0);
  q[3] = std::cos(angle / 2.0);
}

/// How far a point is outside a box centred on the origin, nothing if inside.
/// The box's sides run along `axes` and reach `half` along each.
double outsideBox(const Triple &p, const Triple axes[3], const Triple &half) {
  double squared = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double past = std::fabs(dot(p, axes[i])) - half[i];
    if (past > 0.0) squared += past * past;
  }
  return std::sqrt(squared);
}

/// The shortest push that takes a segment out of a box it passes through. The
/// way out of that overlap is across one of six planes: a side of the box, or
/// one that holds the segment and an edge of the box.
double pushOutOfBox(const Triple &centre, const Triple &reach,
                    const Triple axes[3], const Triple &half) {
  double least = 1.0e9;
  for (int k = 0; k < 6; ++k) {
    Triple along = k < 3 ? axes[k] : cross(reach, axes[k - 3]);
    const double size = std::sqrt(dot(along, along));
    if (size < 1.0e-6) continue;
    along = along * (1.0 / size);
    double boxReach = 0.0;
    for (int i = 0; i < 3; ++i) boxReach += half[i] * std::fabs(dot(axes[i], along));
    const double overlap =
        std::fabs(dot(reach, along)) + boxReach - std::fabs(dot(centre, along));
    least = std::fmin(least, overlap);
  }
  return least;
}

/// How deep a capsule overlaps a box, or a negative number for how far apart
/// they are. Worked out from the shapes alone, a slow way and an exact one,
/// to be set against what the engine says.
double capsuleInBox(const Triple &centre, const Triple &reach, double radius,
                    const Triple axes[3], const Triple &half) {
  double nearest = 1.0e9;
  for (int i = 0; i <= 4000; ++i) {
    const double along = -1.0 + 2.0 * i / 4000.0;
    nearest = std::fmin(nearest, outsideBox(centre + reach * along, axes, half));
  }
  if (nearest > 1.0e-3) return radius - nearest;
  return radius + pushOutOfBox(centre, reach, axes, half);
}

void capsulesAgainstBoxes() {
  OrblitPhysicsSettings still;
  orblit_physics_defaults(&still);
  still.gravity[1] = 0.0f;
  still.sleeping = false;

  const double radius = 0.3;
  const double halfHeight = 0.6;
  const Triple half = {0.5, 0.4, 0.6};

  std::mt19937 dice(11);
  const auto between = [&dice] { return dice() / 4294967295.0 * 2.0 - 1.0; };

  int judged = 0;
  int wrong = 0;
  double worst = 0.0;
  for (int pair = 0; pair < 2000; ++pair) {
    double boxTurn[4];
    double capsuleTurn[4];
    randomTurn(dice, boxTurn);
    randomTurn(dice, capsuleTurn);
    const Triple at = {1.5 * between(), 1.5 * between(), 1.5 * between()};

    const Triple axes[3] = {turned(boxTurn, {1, 0, 0}), turned(boxTurn, {0, 1, 0}),
                            turned(boxTurn, {0, 0, 1})};
    const Triple reach = turned(capsuleTurn, {0, 1, 0}) * halfHeight;
    const double truth = capsuleInBox(at, reach, radius, axes, half);

    // A pair that only just touches could be called either way.
    if (std::fabs(truth) < 2.0e-3) continue;
    ++judged;

    OrblitPhysics *physics = orblit_physics_create(&still);
    OrblitPhysicsCommand box = boxAt(1, 0.5f, 0.0f, 0.0f, 0.0f);
    box.size[0] = static_cast<float>(half[0]);
    box.size[1] = static_cast<float>(half[1]);
    box.size[2] = static_cast<float>(half[2]);
    box.motion = ORBLIT_PHYSICS_STATIC;
    OrblitPhysicsCommand capsule =
        capsuleAt(2, static_cast<float>(radius), static_cast<float>(halfHeight),
                  static_cast<float>(at[0]), static_cast<float>(at[1]),
                  static_cast<float>(at[2]));
    for (int i = 0; i < 4; ++i) {
      box.rotation[i] = static_cast<float>(boxTurn[i]);
      capsule.rotation[i] = static_cast<float>(capsuleTurn[i]);
    }
    submit(physics, box);
    submit(physics, capsule);
    orblit_physics_step(physics, kStep);

    // The depth of the pair is its deepest point's, and no point means no touch.
    const Listed listed = listContacts(physics);
    double reported = -1.0;
    for (const OrblitPhysicsContact &point : listed.points) {
      reported = std::fmax(reported, point.depth);
    }
    orblit_physics_destroy(physics);

    const bool touching = truth > 0.0;
    const bool missed = touching != (reported >= 0.0);
    const double off = touching ? std::fabs(reported - truth) : 0.0;
    if (missed || off > 3.0e-3) {
      ++wrong;
      worst = std::fmax(worst, missed ? 1.0 : off);
    }
  }
  if (wrong > 0) {
    std::printf("      %d of %d pairs wrong, worst %.4f\n", wrong, judged, worst);
  }
  check(judged > 1500, "enough random capsules and boxes were judged to mean something");
  check(wrong == 0,
        "a capsule meets a box where the shapes meet, and as deep as they overlap");
}

int main() {
  compoundChecks();
  meshChecks();
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
  triggering();
  triggeringCharacters();
  staying();
  stayingInside();
  zoning();
  zoneEdits();
  conveying();
  wakingBelts();
  pinnedBelts();
  ruling();
  ruleEdits();
  hiding();
  overlaps();
  overlapLayers();
  askingAll();
  askingAny();
  askingTheWorld();
  weighing();
  changingGravity();
  lockingToAPlane();
  lockedContacts();
  lockingTurns();
  capping();
  ignoring();
  switching();
  centring();
  centringJoined();
  spreading();
  controlling();
  wakingControls();
  replaying();
  refusingSnapshots();
  listingContacts();
  counting();
  endingInOrder();
  layingHulls();
  refusingCylinders();
  restingConvex();
  restingOnConvex();
  restingOnGround();
  rollingCylinders();
  weighingConvex();
  walkingOnConvex();
  castingConvex();
  snapshottingHulls();
  capsulesAgainstBoxes();

  std::printf(failures == 0 ? "\nALL PASSED\n" : "\n%d FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
