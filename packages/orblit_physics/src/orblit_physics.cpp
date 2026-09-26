// The C ABI, and nothing else.
//
// Every function here is a hand-off: check the pointer, turn the caller's
// numbers into the engine's own, call one method. It is this thin on purpose.
// This is the one file whose shape is fixed — a header that has shipped
// cannot change its mind — so anything with a decision in it belongs behind
// the wall, where it still can.

#include "orblit_physics.h"

#include <new>

#include "world.h"

struct OrblitPhysics {
  explicit OrblitPhysics(const OrblitPhysicsSettings &settings)
      : world(settings) {}
  orblit::World world;
};

void orblit_physics_defaults(OrblitPhysicsSettings *out) {
  orblit::World::defaults(out);
}

OrblitPhysics *orblit_physics_create(const OrblitPhysicsSettings *settings) {
  OrblitPhysicsSettings use;
  orblit::World::defaults(&use);
  if (settings != nullptr) use = *settings;
  // A world that could not be made is a null the caller can check, not an
  // exception crossing a C boundary that has no way to carry one.
  return new (std::nothrow) OrblitPhysics(use);
}

void orblit_physics_destroy(OrblitPhysics *physics) { delete physics; }

void orblit_physics_submit(OrblitPhysics *physics,
                           const OrblitPhysicsCommand *commands,
                           uint32_t count) {
  if (physics == nullptr) return;
  physics->world.submit(commands, count);
}

bool orblit_physics_ground(OrblitPhysics *physics,
                           const OrblitPhysicsGround *ground) {
  if (physics == nullptr || ground == nullptr) return false;
  return physics->world.ground(*ground);
}

void orblit_physics_step(OrblitPhysics *physics, float delta) {
  if (physics == nullptr) return;
  physics->world.step(delta);
}

uint32_t orblit_physics_read(OrblitPhysics *physics, const OrblitPhysicsId *ids,
                             uint32_t count, float *out, uint32_t stride,
                             uint32_t offset) {
  if (physics == nullptr) return 0;
  return physics->world.read(ids, count, out, stride, offset);
}

const OrblitPhysicsEvent *orblit_physics_events(const OrblitPhysics *physics,
                                                uint32_t *count) {
  if (count != nullptr) *count = 0;
  if (physics == nullptr) return nullptr;

  const std::vector<OrblitPhysicsEvent> &events = physics->world.events();
  if (count != nullptr) *count = static_cast<uint32_t>(events.size());
  return events.empty() ? nullptr : events.data();
}

uint32_t orblit_physics_count(const OrblitPhysics *physics) {
  return physics == nullptr ? 0 : physics->world.bodies().count();
}

bool orblit_physics_alive(const OrblitPhysics *physics, OrblitPhysicsId id) {
  return physics != nullptr &&
         physics->world.bodies().rowOf(id) != orblit::Bodies::kNone;
}

bool orblit_physics_asleep(const OrblitPhysics *physics, OrblitPhysicsId id) {
  if (physics == nullptr) return false;
  const orblit::Bodies &bodies = physics->world.bodies();
  const uint32_t row = bodies.rowOf(id);
  return row != orblit::Bodies::kNone && bodies.asleep(row);
}

bool orblit_physics_transform(const OrblitPhysics *physics, OrblitPhysicsId id,
                              float *out) {
  if (physics == nullptr || out == nullptr) return false;
  const orblit::Bodies &bodies = physics->world.bodies();
  const uint32_t row = bodies.rowOf(id);
  if (row == orblit::Bodies::kNone) return false;

  const orblit::Vec3 &at = bodies.at(row);
  const orblit::Quat &rotation = bodies.rotation(row);
  out[0] = at.x;
  out[1] = at.y;
  out[2] = at.z;
  out[3] = rotation.x;
  out[4] = rotation.y;
  out[5] = rotation.z;
  out[6] = rotation.w;
  return true;
}

bool orblit_physics_cast(const OrblitPhysics *physics,
                         const OrblitPhysicsCast *cast, OrblitPhysicsHit *out) {
  if (physics == nullptr || cast == nullptr || out == nullptr) return false;
  return physics->world.cast(*cast, *out);
}

bool orblit_physics_velocity(const OrblitPhysics *physics, OrblitPhysicsId id,
                             float *out) {
  if (physics == nullptr || out == nullptr) return false;
  const orblit::Bodies &bodies = physics->world.bodies();
  const uint32_t row = bodies.rowOf(id);
  if (row == orblit::Bodies::kNone) return false;

  const orblit::Vec3 &velocity = bodies.velocity(row);
  const orblit::Vec3 &spin = bodies.spin(row);
  out[0] = velocity.x;
  out[1] = velocity.y;
  out[2] = velocity.z;
  out[3] = spin.x;
  out[4] = spin.y;
  out[5] = spin.z;
  return true;
}

uint32_t orblit_physics_footing(const OrblitPhysics *physics,
                                const OrblitPhysicsId *ids, uint32_t count,
                                OrblitPhysicsFooting *out) {
  if (physics == nullptr || ids == nullptr || out == nullptr) return 0;
  return physics->world.footing(ids, count, out);
}

bool orblit_physics_join(OrblitPhysics *physics, const OrblitPhysicsJoint *joint) {
  if (physics == nullptr || joint == nullptr) return false;
  return physics->world.join(*joint);
}

bool orblit_physics_unjoin(OrblitPhysics *physics, OrblitPhysicsId joint) {
  if (physics == nullptr) return false;
  return physics->world.unjoin(joint);
}

bool orblit_physics_joint(const OrblitPhysics *physics, OrblitPhysicsId joint,
                          OrblitPhysicsJointState *out) {
  if (physics == nullptr || out == nullptr) return false;
  return physics->world.joint(joint, *out);
}
