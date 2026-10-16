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

`orblit_physics_scene` is one of three packages here that know about Orblit;
the others lay its terrain, [below](#on-terrain), and give a rig a body for each
limb, [below that](#on-a-rig). This one simulates the
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
`entityOfJoint` pair joints with entities the same way. A body's `trigger`,
`stay` and `surface`, and a zone component on an entity with a body, are read
the same way, [below](#questions-and-places). The world itself is
`scene.physics`, for pushing and casting. What touched what is `scene.events`,
gathered over every step the last `advance` took; the world's own list keeps
only its last step.

On a screen faster than 60 Hz a body stands still on the frames that take no
step. `ScenePhysics(document, smooth: true)` shows each body blended between
its last two steps, so it moves on every frame. The document is then up to one
step behind `scene.physics`, which still answers casts and events for where
bodies are now. After a move made straight on the world, call
`scene.resetSmoothing(entity)` so the body is not drawn crossing the gap. An
edit through `apply` needs nothing.

It depends on `orblit_scene` over git. To work against a checkout of the engine
beside this one:

```sh
./tool/link_local.sh     # or: ./tool/link_local.sh path/to/orblit
```

### Inside

A step is: find contacts, solve, move characters, report touches, sleep. The
solve cuts the step into sub-steps, six by default. Each sub-step integrates
velocity, corrects velocity against the contacts and the joints, moves the
bodies, and corrects once more with no push in it. That last pass takes back
the speed the push added, so a stack settles and does not bounce. Contacts are
found once a step, and each sub-step reads where its bodies have got to from
those. A short step has a small overlap to correct, and a small correction puts
little energy into a stack.

A contact is a spring at `contactHertz` and not a wall. The push starts at the
surface, and the speed it pushes with is capped. After the sub-steps, a bounce
turns round the bodies that met, at the speed they met at.

Contacts come from a tree of the fixed bodies and a sweep along X of the rest.
The narrowphase covers spheres, boxes, capsules, planes and ground, with
separating-axis box to box and face clipping. Where two shapes meet along a
line, as a capsule does lying on the floor or across a crate, the contact is
reported at both ends, because one contact in the middle of a line is something
to roll off. A box face on a box face keeps the four corners that spread
furthest. Manifolds persist between ticks, so each pass starts from the impulse
that worked last time rather than from zero.

A pair that will cross the gap this step gets a contact before it touches, so
a fast body does not tunnel through a thin wall. The margin is sized by speed
and is zero under about 3 m/s. `PhysicsControls.continuous` adds a cast along
the body's move against fixed bodies and ground, for what the margin misses: a
small body at a walking pace against a thin wall.

Bodies that touch sleep and wake as one island, so a stack goes to sleep whole
and a heap of 1,000 boxes is asleep within six seconds. Bodies joined together
sleep and wake as one as well: half a chain asleep is half a chain that has
stopped being held. A body is put to sleep by the distance it walked as well as
by its speed.

### Settings and the benchmark

`subSteps` is 6, `contactHertz` is 50, `velocitySteps` is 1 for each sub-step,
`positionSteps` is 2 for joints and `slop` is 0.02. They are tuned for 60 steps
a second. At 30 a second a wall of 25 by 20 boxes grows a mode and comes down to
half its height, as it did before sub-steps. The heap sleeps whole in 6 of 8
seeds there.

`./tool/benchmark.sh` times a step on the scenes in `bench/benchmark.cpp`.
These are from one run on an Apple M4 Pro at 60 steps a second, in microseconds
a step.

| Scene | Mean | Median | Asleep at the end |
| --- | ---: | ---: | --- |
| Scatter, 2,000 spheres | 1,826 | 2,100 | 947 of 2,000 |
| Heap, 1,000 boxes | 1,923 | 389 | all, by step 323 |
| Wall, 500 boxes | 437 | 27 | all, by step 50 |
| Wall, 2,000 boxes | 15,255 | 14,447 | 0 of 2,000 |
| 5,000 fixed boxes and 200 spheres | 149 | 135 | 148 of 200 |

A ray cast against 5,000 static boxes takes 2.3 µs. A 0.2 m sphere fired at a
0.1 m wall crosses from none of 16 starts at any speed from 5 to 320 m/s.

The wall of 2,000 boxes, 50 wide and 40 high, does not sleep. A mode grows at
the top row's edge until the wall comes down, and more sub-steps only delay it.
The median of the heap is the point it falls asleep, so it moves between runs.
The mean is the figure to compare.

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

### Questions and places

`castAll`, `castAny` and `overlap` join `cast`. They ask for every body along a
line, nearest first, whether anything is there at all, and what a shape or a
point is inside. Each takes the same filters, `layers` and `ignore`, and a way
to stop early: `castAny` stops at the first body it finds and the others take a
`limit`. Nothing they ask moves or wakes anything.

A body added with `trigger` is a place. Nothing bounces off it, and solid casts,
overlaps and characters do not see it. The world reports `entered` and
`exited` for each body that comes in and goes out, and `inside` every step when
either asked for `stay`. A trigger reads positions, not touches, so a body
asleep inside one is still inside. It has to stay where it is, because the
solver has to move a free body. `triggers: true` on a question asks about
triggers and nothing else.

A zone is a trigger that changes the gravity and damping of the free bodies
inside it. Each field is left to the body unless the zone sets it, and where
zones overlap each field goes to the highest `priority`, then the lower id. A
belt is `setSurface`: a velocity in the world that friction drags whatever
stands on it towards, along the surface it touches and no faster.

`setRule` is the contact hook. It changes the friction, restitution and move
scales of one pair of bodies. It is data rather than a callback, because a Dart
function called from inside the solver for every contact on every step would
put a trip across the boundary in the hottest loop there is. A move scale says
how much of a contact's push a body takes, so a lift carries a crate without
the crate slowing it.

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
driven ones and other characters. A crate cannot shove a player off a ledge; a
closing door can. A character that walks into another moves it out of the way
at half the speed they close at, and never through a wall.

`setStance` changes its shape in place, with the feet kept where they were: a
crouch comes down from the top. It answers whether the new shape fits, so a
character under a beam that asks to stand gets false and stays as it was.
`footingOf` also says whether the last walk met a `wall` or a `ceiling`, beside
the ground it stands on.

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

## On a rig

`orblit_physics_rig` gives a skeleton from `orblit_rig` a body for each limb
and a joint to its parent. A `RagdollSpec` says which bones get a body, how
fat it is, and whether its joint is a hinge or a ball.

```dart
final ragdoll = Ragdoll.build(
  physics: physics,
  pose: pose,
  spec: spec,
  modelToWorld: model,
  ignore: characterLayer,
);
stack.add(RagdollPose(ragdoll));

// Every frame, after the stack has run.
physics.step(dt);
```

Each body is `animated`, `powered` or `limp`. An animated body is given the
velocity that reaches the pose in one step, so it pushes what it meets and is
not pushed back. A powered body is held to the pose's angles by the joint's
springs and gives under a blow. A limp body has only the joint's limits.

`hit` makes the struck bone limp at once and its neighbours one joint at a time,
`spread` seconds apart. `setFrom` makes a limb go from a bone down. An animated
body below a free one holds that one's far end where the pose has it, as a
hand holds a rail, so a limb that should fall has to go limp from its top.

`RagdollPose` writes the bones of free bodies back into the pose and leaves the
rest to the animation. Only the root is moved. Every other bone is turned and
stays on its parent, so bones keep the length the rig gave them however far a
joint has been pulled. `recover` mixes from the last drawn fall to the animated
pose over a time, so a get-up does not jump.

`RagdollCharacter` hands a character between its capsule and its bodies. The
capsule and the ragdoll sit in layers that ignore each other. `knock` takes the
capsule out, and `standUp` finds the ground under the root, puts the capsule
there and makes every body animated again. The model is the game's to move.

Like the scene bridge, it depends on `orblit_rig` over git, and
`./tool/link_local.sh` points it at a checkout instead.

## On wheels

`orblit_physics_vehicle` makes a car: a chassis body whose wheels are casts.
Each wheel hangs from a spring and a damper, written as a frequency and a
damping ratio so the same numbers suit a kart and a truck.

```dart
final car = Vehicle(
  physics: physics,
  id: 1,
  spec: VehicleSpec(
    chassis: const Shape.box(0.9, 0.3, 2.0),
    mass: 1200,
    antiRoll: 8000,
    wheels: [
      for (final x in const [-0.8, 0.8])
        for (final z in const [-1.2, 1.2])
          WheelSpec(mount: Vector3(x, 0, z), radius: 0.35),
    ],
  ),
  at: [0, 0.6, 0],
);

// Every step, before the step itself.
car.update(dt);
physics.step(dt);
```

Each wheel has a tyre that holds the car to the ground. A tyre bends like
rubber, so a parked car stays parked on a slope and a turning one gives a
steady side force. `VehicleInput` steers, brakes and pulls the handbrake. The
wheels that steer turn by Ackermann geometry, the inner one more than the
outer. A brake that can hold a wheel locks it, and a locked tyre grips less
than a rolling one. `Vehicle(surface:)` takes a function from the body under a
wheel to its grip, so ice and tarmac can share a world.

```dart
car.setInput(VehicleInput(steer: 0.5, brake: 0.2));
```

A car with a `drive` has an engine. `EngineSpec` is a torque curve. `DriveSpec`
adds the gears, the final drive and a centrifugal clutch that holds nothing at
idle and grips harder as the engine speeds up, so a car pulls away without
stalling and does not creep. Wheels that `drives` share the torque, evenly
between axles, then between the two wheels of an axle through a differential.
`lock` is the torque that differential may move to the slower wheel: zero is
open and infinity is locked. The box shifts by itself, or holds the
`VehicleInput(gear:)` the driver picks, with reverse as minus one.

```dart
car.setInput(VehicleInput(throttle: 1));
print('${car.gear} ${car.rpm}');
```

The clutch is solved against what each tyre will do with a torque. A wheel on
the road feels the whole car behind it. A wheel in the air spins up. Wheelspin
on ice is smooth and does not flip from step to step.

The wheels have no mass. The chassis still collides, so a car that bottoms
out rests on its floor. A parked car never sleeps, because its springs push
it every step. A scene component, wheel entities that follow the springs and
an input action to drive from are not there yet.

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

**Continuous collision between free bodies.** A body that names `continuous`
is cast against fixed bodies and ground only. Two free bodies rely on the
contacts made before they touch. A step short enough for the speeds in play is
the answer past that.

**The web.** The solver is C++ behind a build hook, so it goes where a native
toolchain goes. Whether it follows Orblit to the web later is open.

## Licence

MPL-2.0, © 2026 Chris Beckett. That is the Mozilla Public License, and it is
open source. Use it, fork it and ship games with it, including commercial ones.
Your game stays yours, and the licence does not reach into it. What it asks is
that changes to this repository's own files ship under the same licence, so
engine work stays in the open.

## Compound and scaled shapes

`layCompound(id, parts: [...])` keeps 1 to 64 `ShapePart` values under an id
for `Shape.compound(id)` to name. Each part is a convex primitive or a laid
hull, with local `at`, `rotation` in xyzw order, and positive `scale`.
Scale is applied before turning and placement. An outer `scale` stretches the
whole compound, including rotated parts and their offsets. One part provides
a scaled or offset shape. Nonuniform scaling stretches rounded geometry
exactly. The gaps between parts stay empty for collisions and queries.

Parts share one body, its material and its events. Mass is distributed by
volume at uniform density, with a combined inertia tensor. Overlaps count
mass twice. Four contact points remain the budget for each pair of bodies.
Planes, height fields, meshes and nested compounds cannot be parts.

Remove bodies before `dropCompound`. A hull retained by a compound cannot be
dropped until that compound goes. Snapshots retain both assets independently.

In a scene, `BodyShape.compound` reads `BodyComponent.parts`, a list of
`BodyPart` values with local centre, quaternion rotation and scale. The
body's `shapeScale` stretches its collision geometry independently of its
visual transform. Compounds and explicit shapeScale use exact scaling.
Ordinary rounded bodies with default shapeScale retain their previous
entity sizing rules. Invalid parts remain editable as an inert body.


## Static model surfaces

`layMesh(id, vertices: [...], indices: [...])` copies indexed triangles,
welds coincident coordinates and builds a spatial tree with seam information.
`Shape.mesh(id)` names the result on a fixed body. Rays, convex shape casts,
overlaps, contacts and characters meet the same two-sided surfaces. Shared
flat edges produce no rim impulse, including duplicated exporter vertices.
Outer and nonmanifold edges stay rims; separate assets do not share seams.

A mesh has no solid interior and cannot move or be a compound part. Use a
convex hull or convex compound for something the solver moves. Invalid inputs
and all-degenerate meshes return false. Remove its bodies before `dropMesh`;
snapshots retain their own references.

`CollisionMesh.fromGltf` in `orblit_asset` bakes an active static model scene
into indexed triangles, or model settings `collision: true` cook a
`collision.json` sidecar. A scene uses `BodyShape.mesh` with `meshVertices`
and `meshIndices`; the bridge applies entity and shapeScale, forces fixed
motion and owns the asset through edits, removal and snapshot restoration.
