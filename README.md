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

Contacts come from a sweep along X and a narrowphase of spheres, boxes,
capsules and planes, with separating-axis box–box and face clipping. Where two
shapes meet along a line rather than at a point — a capsule lying on the floor,
or across a crate — the contact is reported at both ends, because one contact in
the middle of a line is something to roll off. Manifolds persist between ticks,
so each pass starts from the impulse that worked last time rather than from
zero. Bodies that stop moving go to sleep individually and are woken by
anything that touches them.

### Casting

`cast` fires a shape along a direction and answers the first body it meets.
With no shape it fires a point, which is the ray cast everything else is named
after and the cheap case; with one it is how a character finds the floor under
its feet, or a camera the wall behind the player.

```dart
final ground = physics.cast(
  from: feet,
  direction: const [0, -1, 0],
  distance: 1.5,
  shape: const Shape.sphere(0.3),
  ignore: player,
);
```

It advances the shape along the line by however far it can prove is safe, over
and over, until there is no gap left. Every step is a lower bound on the real
distance between the two shapes, so the loop can stop short of an impact but
can never step past one. A cast that starts inside a body says so rather than
reporting a distance of nothing and leaving the caller to guess — a character
who begins in a wall wants pushing out of it, not stopping where they already
are.

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
for the speeds in play is the answer for now, and a cast along where something
is about to go is the answer for the few cases that cannot afford one.

**The web.** The solver is C++ behind a build hook, so it goes where a native
toolchain goes. Whether it follows Orblit to the web later is open.

## Licence

MPL-2.0, © 2026 Chris Beckett. That is the Mozilla Public License, and it is
open source. Use it, fork it and ship games with it, including commercial ones.
Your game stays yours, and the licence does not reach into it. What it asks is
that changes to this repository's own files ship under the same licence, so
engine work stays in the open.
