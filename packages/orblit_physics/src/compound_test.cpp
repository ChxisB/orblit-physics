#include "compound.h"
#include "mass.h"

#include <cmath>
#include <stdexcept>
#include <memory>

namespace {

void equal(float actual, float expected) {
  if (std::fabs(actual - expected) > 1.0e-4f) {
    throw std::runtime_error("compound mass property differs");
  }
}

void compoundInertia() {
  using namespace orblit;
  const Shape box = Shape::box({0.5f, 0.5f, 0.5f});
  const Compound parts({{box, {-2, 0, 0}}, {box, {2, 0, 0}}}, {});
  equal(parts.centre().x, 0);
  equal(parts.inertia().row[0].x, 1.0f / 6.0f);
  equal(parts.inertia().row[1].y, 4.0f + 1.0f / 6.0f);
  equal(parts.inertia().row[2].z, 4.0f + 1.0f / 6.0f);
}

void scaledMass() {
  using namespace orblit;
  Shape sphere = Shape::sphere(1);
  sphere.affine = true;
  sphere.linear = Mat3::diagonal({2, 1, 0.5f});
  const Mat3 inertia = inertiaOf(sphere);
  equal(inertia.row[0].x, 0.25f);
  equal(inertia.row[1].y, 0.85f);
  equal(inertia.row[2].z, 1.0f);
  equal(volumeOf(sphere), 4.0f / 3.0f * kPi);
}

void weightedCentre() {
  using namespace orblit;
  const Compound parts({{Shape::box({0.5f, 0.5f, 0.5f}), {0, 0, 0}},
                        {Shape::box({1, 1, 1}), {9, 0, 0}}}, {});
  equal(parts.centre().x, 8);
  const MassProperties mass = massOf(Shape::compound(&parts), true, 9, {});
  equal(mass.offset.x, 8);
  equal(mass.inverseMass, 1.0f / 9.0f);
}

void compoundWorld() {
  OrblitPhysicsSettings settings{};
  orblit_physics_defaults(&settings);
  using Physics = std::unique_ptr<OrblitPhysics, decltype(&orblit_physics_destroy)>;
  using Snapshot = std::unique_ptr<OrblitPhysicsSnapshot, decltype(&orblit_physics_snapshot_destroy)>;
  Physics world(orblit_physics_create(&settings), orblit_physics_destroy);
  OrblitPhysicsPart parts[2]{};
  for (int i = 0; i < 2; ++i) {
    parts[i].shape = ORBLIT_PHYSICS_BOX;
    parts[i].size[0] = parts[i].size[1] = parts[i].size[2] = 0.5f;
    parts[i].at[0] = i == 0 ? -2.0f : 2.0f;
    parts[i].linear[0] = parts[i].linear[4] = parts[i].linear[8] = 1.0f;
  }
  if (!orblit_physics_compound(world.get(), 7, parts, 2)) throw std::runtime_error("compound registration failed");
  OrblitPhysicsCommand body{};
  body.kind = ORBLIT_PHYSICS_CREATE;
  body.shape = ORBLIT_PHYSICS_COMPOUND;
  body.hull = 7;
  body.id = 1;
  body.rotation[3] = 1;
  body.layerIs = 1;
  body.layerCares = 0xffffffff;
  orblit_physics_submit(world.get(), &body, 1);
  OrblitPhysicsCast ray{};
  ray.from[0] = 2;
  ray.from[2] = 3;
  ray.direction[2] = -1;
  ray.distance = 6;
  ray.layerIs = 1;
  ray.layerCares = 0xffffffff;
  OrblitPhysicsHit hit{};
  if (!orblit_physics_cast(world.get(), &ray, &hit)) throw std::runtime_error("compound ray missed");
  equal(hit.distance, 2.5f);
  Snapshot saved(orblit_physics_snapshot(world.get()), orblit_physics_snapshot_destroy);
  world.reset(orblit_physics_create(&settings));
  orblit_physics_restore(world.get(), saved.get());
  saved.reset();
  if (!orblit_physics_cast(world.get(), &ray, &hit)) throw std::runtime_error("compound snapshot lost geometry");
  equal(hit.distance, 2.5f);
}

} // namespace

void compoundChecks() {
  compoundInertia();
  scaledMass();
  weightedCentre();
  compoundWorld();
}
