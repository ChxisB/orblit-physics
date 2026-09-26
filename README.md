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
an Orblit entity handle. The solver never looks inside one, which is why
`orblit_physics` does not depend on `orblit_core` — or on anything else in the
engine. It is a physics library that Orblit happens to use.

## In a scene

`orblit_physics_scene` is one of the two packages here that know about Orblit;
the other lays its terrain, [below](#on-terrain). This one simulates the
entities in a scene document that have a body component, and answers each step
with the same kind of diff an edit makes, so whatever already draws a document
draws the simulation.

```dart
final scene = ScenePhysics(document);

// Every frame.
view.apply(scene.advance(elapsed));

// When the author drags a crate.
scene.apply(edit);
```

`advance` steps the world at a fixed sixtieth however unevenly it is called,
and writes back only the entities whose bodies moved, parents before children.
`apply` rebuilds the bodies an edit touches where the document now puts them.
That is a teleport: a crate that is dragged mid-fall stops falling. Entities
are named by strings and bodies by numbers, so the bridge keeps the pairing
(`bodyOf`, `entityOf`) and never gives a number to a second entity. An entity
with a joint component joins two bodies, [below](#joints), and `jointOf` and
`entityOfJoint` pair joints with entities the same way. The world itself is
`scene.physics`, for pushing and casting. What touched what is `scene.events`,
gathered over every step the last `advance` took; the world's own list keeps
only its last step.

It depends on `orblit_scene` over git. To work against a checkout of the engine
beside this one:

```sh
./tool/link_local.sh     # or: ./tool/link_local.sh path/to/orblit
```

### Inside

A step is: find contacts, integrate velocity, solve velocity, integrate
position, solve position, break joints, move characters, sleep. Velocity and
position are solved separately
because they answer different questions — restitution and friction are about
how things are moving, overlap is about where they are — and the position pass
runs after integration so it is not correcting a gap that integration is about
to reopen.

Contacts come from a sweep along X and a narrowphase of spheres, boxes,
capsules, planes and ground, with separating-axis box–box and face clipping. Where two
shapes meet along a line rather than at a point — a capsule lying on the floor,
or across a crate — the contact is reported at both ends, because one contact in
the middle of a line is something to roll off. Manifolds persist between ticks,
so each pass starts from the impulse that worked last time rather than from
zero. Bodies that stop moving go to sleep individually and are woken by
anything that touches them, except that bodies joined together sleep and wake
as one: half a chain asleep is half a chain that has stopped being held.

### Casting

`cast` fires a shape along a direction and answers the first body it meets.
With no shape it fires a point, which is the ray cast everything else is named
after and the cheap case; with one it is how a camera finds the wall behind the
player, or a thrown thing where it will land.

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

### Characters

A character is a body that walks. The solver would bounce it off walls, catch
its feet on every step and slide it down every slope, so it is not solved: it
is swept through the world along what it asks for, and slides along whatever it
meets.

```dart
physics.addCharacter(player, at: [0, 0.91, 0]);

// Every step.
final left = physics.footingOf(player)!.velocity;
physics.drive(player, velocity: [walkX, left[1] - 9.81 / 60, walkZ]);
physics.step(1 / 60);
```

It climbs anything up to `stepHeight`, stands on slopes up to `steepest` and
slides down steeper ones, and rides what it stands on: a lift carries it up, a
turntable turns it, and `footingOf` says by how much, apart from the walk it
asked for, so an animation plays the walk and not the ride. It pushes free
bodies it walks into, no harder than `strength`, and is pushed by nothing but
driven ones. A crate cannot shove a player off a ledge; a closing door can.

It does not fall on its own. Gravity is part of what it asks for, which is what
leaves a game to decide what a jump is, how long a player hangs at the top of
one and whether they can steer in the air. The footing is what survived of the
last request — a floor takes away the fall, a wall takes away the part going
into it — so each step asks for that plus a step of gravity, and a character
standing still does not pile up a fall it is not taking.

Standing is decided by looking down, not by what it bumped into. A rounded base
touching the corner of a step leans by however far round its curve the corner
is, which says nothing about whether the step is a floor, so a contact that
leans too far to stand on is looked past, to the top of whatever it touched.

### Ground

Ground is heights on a grid, laid as one fixed body. Each square of four
samples is two triangles, and everything under the surface is solid: a crate
that has sunk in, or had the ground raised under it while it slept, is pushed
up and out rather than through. A height that is not a number is a hole.

```dart
physics.layGround(
  hill,
  heights: heights, // columns * rows of them, row after row
  columns: 65,
  rows: 65,
  spacing: 0.5,
  at: [-16, 0, -16],
);
```

Laying it again under the same id replaces it and wakes whatever was over the
old ground or the new, which is how an edit reaches the world. Taking it away
wakes nothing, as taking any body away does, so a crate asleep on ground that
has streamed out stays put until the ground comes back.

A line between two triangles is a seam, not an edge: a ball rolls across flat
ground without hopping at every cell, and a box sits in a crease between two
slopes without being shoved off either. The edge of the grid is a seam too,
because ground big enough to stream is laid in pieces, side by side. Laid with
a `margin`, a piece's outer ring of samples is its neighbours', never stood on,
and a ridge along the line between two pieces holds a ball up the way one
piece would.

### Joints

A joint holds two bodies together, or one to the world. There are seven kinds,
and each is only which of the six ways the second body can move relative to
the first it holds, and how far: along the three axes of the joint's frame and
about them. A hinge holds five of them and leaves the turn about x free,
perhaps within a limit. The rows are solved in the same passes as the
contacts, so a door leaning on a crate and the crate leaning on the floor are
one problem rather than two that take turns being wrong.

```dart
physics.join(
  1,
  const Joint.hinge(limit: JointLimit(0, 1.6), speed: 1, strength: 50),
  a: frame,
  b: door,
  at: [0.5, 1, 0],
  rotation: upright, // the frame's x axis is the one it turns about
);
```

`fixed` welds, `point` is a ball and socket, `hinge` turns about x, `slider`
slides along it, `distance` keeps two points apart — a rod, or with a range a
rope — `cone` lets x swing within a cone and twist within a range, and
`sixAxis` sets each of the six on its own. A hinge and a slider take a motor,
which drives at a speed no harder than a strength; a motor with no speed is
friction in the joint.

It is made where the bodies stand, and what it holds is how they stood, so a
hinge made with its door shut reads nought shut. Every measure is the second
body as the first sees it, so which body is first decides the sense: with the
world first a door's angle is the door's, and with the world second it is the
world's as the door sees it, the other way round. `jointStateOf` reads it
back, and says how hard it held on the last step, which is the number to look
at before choosing `breakingForce` or `breakingTorque`. Past either it breaks,
and a `broke` event names it. Taking a body out of the world takes its joints
with it, silently, since nothing broke; laying ground again keeps them.

A range solves only its nearer end, and pushes only once the joint would reach
it within the step, so a hinge swinging in the middle of its range is held by
its five locked rows and nothing else. Turning is measured as a twist about x
and then a swing, which is the angle it looks like to within a quarter turn
either way and stops meaning anything close to a half turn; a joint asked to
bend that far wants a limit that stops it first.

In a scene document a joint is a `JointComponent`, and which two bodies it
holds is where its entity sits in the tree: the nearest body at or above it,
held to the nearest body above that, or to the world when there is none. An
id named in a component is one every copy, paste and prefab has to find and
change; a parent is one they already do. The joint's point and frame are the
entity's own, and its angles are in degrees, as a transform's are. An edit
that rebuilds either body makes the joint again as things then stand, and one
that has broken stays broken, in `scene.broken`, until its own entity is
edited — dragging the door does not mend the hinge it tore off.

## On terrain

`orblit_physics_terrain` lays an Orblit terrain as ground, a region to a piece,
and only the regions near a point.

```dart
final ground = TerrainPhysics(physics, terrain);

// Every frame, or whenever the camera moves.
ground.sync(x: camera.x, z: camera.z, radius: 64);
```

`sync` lays the regions within the radius, takes up the ones a region's width
beyond it, and lays a region again when its heights, holes or any neighbour's
have changed. The ground it lays is the ground the renderer draws, triangle for
triangle, holes included. An editor passes `refresh: false` while a stroke is
being drawn and syncs normally once it is done, so a brush does not relay the
ground under it every frame. Its bodies are numbered far below zero, out of the
way of entity handles and of the small negative numbers a game gives its own
bodies; `regionOf` turns one back into the region it is.

Like the scene bridge, it depends on the engine over git, and
`./tool/link_local.sh` points it at a checkout instead.

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
