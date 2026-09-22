# orblit-physics

Rigid body physics for [Orblit](https://github.com/ChxisB/orblit). A sequential
impulse solver written in C++, driven from Dart.

```sh
cd packages/orblit_physics && dart test
```

```sh
./tool/check.sh          # the C++ checks, then analyze and test
```

## The shape of it

A world is a queue and a step. Dart describes what it wants — add this body,
shove that one, drive the platform — and those commands sit in one native
buffer until the next `step` applies them in order. Reading goes the same way
round: `readInto` fills a caller's float buffer for every body it was asked
about, at whatever stride that buffer already uses. A tick costs two crossings
of the FFI boundary, not two per body.

```dart
final physics = Physics();

physics.add(ground, shape: const Shape.plane(0, 1, 0), motion: PhysicsMotion.fixed);
physics.add(ball, shape: const Shape.sphere(0.5), at: [0, 3, 0], restitution: 0.6);

physics.step(1 / 60);

for (final event in physics.events) {
  if (event.kind == PhysicsEventKind.touchBegan) playThud(event.force);
}
```

Bodies are named by whatever integer the caller already uses for them, usually
an Orblit entity handle. The solver never looks inside one, which is why this
package does not depend on `orblit_core` — or on anything else in the engine.
It is a physics library that Orblit happens to use.

### Inside

A step is: find contacts, integrate velocity, solve velocity, integrate
position, solve position, sleep. Velocity and position are solved separately
because they answer different questions — restitution and friction are about
how things are moving, overlap is about where they are — and the position pass
runs after integration so it is not correcting a gap that integration is about
to reopen.

Contacts come from a sweep along X and a narrowphase of spheres, boxes and
planes, with separating-axis box–box and face clipping. Manifolds persist
between ticks, so each pass starts from the impulse that worked last time
rather than from zero. Bodies that stop moving go to sleep individually and are
woken by anything that touches them.

## What it does not do

**Cross-platform determinism.** Two machines running the same inputs will not
produce bit-identical results, and this is a decision rather than an omission.
Getting there means constraining floating point everywhere — no fast-math, no
vectorised reassociation, a fixed order for every accumulation — and it makes
the solver slower on every platform to buy something only lockstep networking
needs. Orblit's multiplayer is state-synchronised, so it does not need it. A
single machine replaying the same inputs at the same fixed step does reproduce
itself; that is a different and much cheaper guarantee, and it is the one on
offer.

**Continuous collision.** Fast, small things tunnel. A fixed step small enough
for the speeds in play is the answer for now.

**The web.** The solver is C++ behind a build hook, so it goes where a native
toolchain goes. Whether it follows Orblit to the web later is open.

## Licence

MPL-2.0, © 2026 Chris Beckett. That is the Mozilla Public License, and it is
open source. Use it, fork it and ship games with it, including commercial ones.
Your game stays yours, and the licence does not reach into it. What it asks is
that changes to this repository's own files ship under the same licence, so
engine work stays in the open.
