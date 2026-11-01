# orblit-physics

Rigid body physics for [Orblit](https://github.com/ChxisB/orblit). A sequential
impulse solver written in C++, driven from C#.

```sh
git clone https://github.com/ChxisB/orblit.git       # beside this one
orblit/tool/build_native.sh                            # the engine's native core
orblit/tool/pack_local.sh                              # packs the engine into ~/.orblit/feed
./tool/check_dotnet.sh                                 # the solver, then build, test and format
```

It needs the .NET 10 SDK and CMake. `ORBLIT_FEED` names another feed folder.

## The shape of it

A world is a queue and a step. The caller says what it wants. Add this body,
shove that one, drive the platform. Those commands sit in one native buffer
until the next `Step` applies them in order. Reading goes the same way round.
`ReadInto` fills a caller's float buffer for every body it was asked about, at
whatever stride that buffer already uses. A tick costs two calls across the
native boundary, not two per body.

```csharp
using var physics = new Physics();

physics.Add(ground, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
physics.Add(ball, Shape.Sphere(0.5), at: [0, 3, 0], restitution: 0.6);

physics.Step(1.0 / 60);

foreach (var change in physics.Events)
{
    if (change.Kind == PhysicsEventKind.TouchBegan) PlayThud(change.Force);
}
```

Bodies are named by whatever integer the caller already uses for them, usually
an Orblit entity handle. The solver never looks inside one, which is why
`Orblit.Physics` does not depend on `Orblit.Core`. Its one engine dependency is
`Orblit.Interop`, which loads native libraries. It is a physics library that
Orblit happens to use.

## In a scene

`Orblit.Physics.Scene` is one of three packages here that know about Orblit;
the others lay its terrain, [below](#on-terrain), and give a rig a body for each
limb, [below that](#on-a-rig). This one simulates the
entities in a scene document that have a body component, and answers each step
with the same kind of diff an edit makes, so whatever already draws a document
draws the simulation.

```csharp
var scene = new ScenePhysics(document);

// Every frame.
view.Apply(scene.Advance(elapsed));

// When the author drags a crate.
scene.Apply(edit);
```

`Advance` steps the world at a fixed sixtieth however unevenly it is called,
and writes back only the entities whose bodies moved, parents before children.
`Apply` rebuilds the bodies an edit touches where the document now puts them.
That is a teleport: a crate that is dragged mid-fall stops falling. Entities
are named by strings and bodies by numbers, so the bridge keeps the pairing
(`BodyOf`, `EntityOf`) and never gives a number to a second entity. An entity
with a joint component joins two bodies, [below](#joints), and `JointOf` and
`EntityOfJoint` pair joints with entities the same way. A body's `Trigger`,
`Stay` and `Surface`, and a zone component on an entity with a body, are read
the same way, [below](#questions-and-places). The world itself is
`scene.Physics`, for pushing and casting. What touched what is `scene.Events`,
gathered over every step the last `Advance` took; the world's own list keeps
only its last step.

On a screen faster than 60 Hz a body stands still on the frames that take no
step. `new ScenePhysics(document, smooth: true)` shows each body blended between
its last two steps, so it moves on every frame. The document is then up to one
step behind `scene.Physics`, which still answers casts and events for where
bodies are now. After a move made straight on the world, call
`scene.ResetSmoothing(entity)` so the body is not drawn crossing the gap. An
edit through `Apply` needs nothing.

It depends on `Orblit.Scene`, which comes from the local feed. To work against
a change in the engine, pack the engine again with `orblit/tool/pack_local.sh`.

### Inside

A step is: find contacts, solve, move characters, report touches, sleep. The
solve cuts the step into sub-steps, six by default. Each sub-step integrates
velocity, corrects velocity against the contacts and the joints, moves the
bodies, and corrects once more with no push in it. That last pass takes back
the speed the push added, so a stack settles and does not bounce. Contacts are
found once a step, and each sub-step reads where its bodies have got to from
those. A short step has a small overlap to correct, and a small correction puts
little energy into a stack.

A contact is a spring at `ContactHertz` and not a wall. The push starts at the
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
and is zero under about 3 m/s. `PhysicsControls.Continuous` adds a cast along
the body's move against fixed bodies and ground, for what the margin misses: a
small body at a walking pace against a thin wall.

Bodies that touch sleep and wake as one island, so a stack goes to sleep whole
and a heap of 1,000 boxes is asleep within eight seconds. Bodies joined together
sleep and wake as one as well: half a chain asleep is half a chain that has
stopped being held. A body is put to sleep by the distance it walked as well as
by its speed.

### Settings and the benchmark

`SubSteps` is 6, `ContactHertz` is 50, `VelocitySteps` is 1 for each sub-step,
`PositionSteps` is 2 for joints and `Slop` is 0.02. They are tuned for 60 steps
a second. At 30 a second a wall of 25 by 20 boxes grows a mode and comes down to
half its height, as it did before sub-steps. The heap sleeps whole in 6 of 8
seeds there.

`./tool/benchmark.sh` times a step on the scenes in `bench/benchmark.cpp`.
These are from one run on an Apple M4 Pro at 60 steps a second, in microseconds
a step. Each scene ran on the solver of 0.15.2 and then on this one, from the
same build of the benchmark, one after the other.

| Scene | Mean before | Mean after | Median before | Median after | Asleep at the end, before and after |
| --- | ---: | ---: | ---: | ---: | --- |
| Scatter, 2,000 spheres | 1,848 | 1,450 | 2,110 | 1,634 | 947 of 2,000 both |
| Scatter, 2,000 continuous spheres | 2,931 | 2,535 | 3,411 | 2,902 | 957 of 2,000 both |
| Heap, 1,000 boxes | 1,901 | 1,326 | 310 | 260 | all, by step 343 and by step 445 |
| Wall, 500 boxes | 443 | 304 | 27 | 27 | all, by step 50 both |
| Wall, 2,000 boxes, 50 wide and 40 high | 15,556 | 10,457 | 14,687 | 9,958 | 0 of 2,000 both |
| Wall, 2,000 boxes, 100 wide and 20 high | 2,602 | 1,759 | 121 | 124 | all, by step 50 both |
| 5,000 fixed boxes and 200 spheres | 152 | 122 | 137 | 111 | 143 of 200 both |

A ray cast against 5,000 static boxes takes 2.3 µs. A 0.2 m sphere fired at a
0.1 m wall crosses from none of 16 starts at any speed from 5 to 320 m/s.

Where the time went, on the wall of 2,000 boxes: more than nine tenths of a
step is the solver. The contact search takes about 1.1 ms of the 15. The solver
is bound by arithmetic, not by memory, so its cost is the number of contact
passes and the multiplies in each one. Two changes cut it. A contact's lever
arms are turned into the velocity change each push makes once a step, not in
every pass. This moves answers by rounding. The sweeps over a manifold's points
run for the manifold's own number of points, which lets the compiler unroll
them. This changes no result: a hash of every body after a run is equal before
and after it. Nothing here uses a second thread. A pool of threads over islands
would help a scene of many separate piles. It would not help a wall or a heap,
which are one island each.

Whether a tall wall stands comes down to rounding. A wall of 2,000 boxes 100
wide and 20 high stands and sleeps. So does every wall up to 20 high that was
tried, from 5 to 50 wide, and a wall 5 wide up to 40 high. A wall 50 wide and
40 high falls. Between those, from 25 to 40 high, some walls stand and some
fall in no order, and the set differs between the solver of 0.15.2 and this
one, and again with the compiler's float contraction switched off. More
sub-steps hold more of them and cost a third more on every scene with
contacts, so the default stays at six. The mean is the figure to compare. The
median of the heap is the point it falls asleep, so it moves between runs.
Across twelve seeds the heap was all asleep between step 285 and step 445, and
between step 278 and step 361 before, so it may sleep a little later.

### Casting

`Cast` fires a shape along a direction and answers the first body it meets.
With no shape it fires a point, which is the ray cast everything else is named
after and the cheap case; with one it is how a camera finds the wall behind the
player, or a thrown thing where it will land.

```csharp
var ground = physics.Cast(
    feet,
    [0, -1, 0],
    1.5,
    shape: Shape.Sphere(0.3),
    ignore: player);
```

It advances the shape along the line by however far it can prove is safe, over
and over, until there is no gap left. Every step is a lower bound on the real
distance between the two shapes, so the loop can stop short of an impact but
can never step past one. A cast that starts inside a body says so rather than
reporting a distance of nothing and leaving the caller to guess — a character
who begins in a wall wants pushing out of it, not stopping where they already
are.

### Questions and places

`CastAll`, `CastAny` and `Overlap` join `Cast`. They ask for every body along a
line, nearest first, whether anything is there at all, and what a shape or a
point is inside. Each takes the same filters, `layers` and `ignore`, and a way
to stop early: `CastAny` stops at the first body it finds and the others take a
`limit`. Nothing they ask moves or wakes anything.

A body added with `trigger` is a place. Nothing bounces off it, and solid casts,
overlaps and characters do not see it. The world reports `Entered` and
`Exited` for each body that comes in and goes out, and `Inside` every step when
either asked for `stay`. A trigger reads positions, not touches, so a body
asleep inside one is still inside. It has to stay where it is, because the
solver has to move a free body. `triggers: true` on a question asks about
triggers and nothing else.

A zone is a trigger that changes the gravity and damping of the free bodies
inside it. Each field is left to the body unless the zone sets it, and where
zones overlap each field goes to the highest `Priority`, then the lower id. A
belt is `SetSurface`: a velocity in the world that friction drags whatever
stands on it towards, along the surface it touches and no faster.

`SetRule` is the contact hook. It changes the friction, restitution and move
scales of one pair of bodies. It is data rather than a callback, because a C#
delegate called from inside the solver for every contact on every step would
put a trip across the native boundary in the hottest loop there is. A move
scale says how much of a contact's push a body takes, so a lift carries a
crate without the crate slowing it.

### Characters

A character is a body that walks. The solver would bounce it off walls, catch
its feet on every step and slide it down every slope, so it is not solved: it
is swept through the world along what it asks for, and slides along whatever it
meets.

```csharp
physics.AddCharacter(player, at: [0, 0.91, 0]);

// Every step.
var left = physics.FootingOf(player)!.Velocity;
physics.Drive(player, velocity: [walkX, left[1] - 9.81 / 60, walkZ]);
physics.Step(1.0 / 60);
```

It climbs anything up to `stepHeight`, stands on slopes up to `steepest` and
slides down steeper ones, and rides what it stands on: a lift carries it up, a
turntable turns it, and `FootingOf` says by how much, apart from the walk it
asked for, so an animation plays the walk and not the ride. It pushes free
bodies it walks into, no harder than `strength`, and is pushed by nothing but
driven ones and other characters. A crate cannot shove a player off a ledge; a
closing door can. A character that walks into another moves it out of the way
at half the speed they close at, and never through a wall.

`SetStance` changes its shape in place, with the feet kept where they were: a
crouch comes down from the top. It answers whether the new shape fits, so a
character under a beam that asks to stand gets false and stays as it was.
`FootingOf` also says whether the last walk met a `Wall` or a `Ceiling`, beside
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

```csharp
physics.LayGround(
    hill,
    heights, // columns * rows of them, row after row
    columns: 65,
    rows: 65,
    spacing: 0.5,
    at: [-16, 0, -16]);
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

```csharp
physics.Join(
    1,
    Joint.Hinge(limit: new JointLimit(0, 1.6), speed: 1, strength: 50),
    a: frame,
    at: [0.5, 1, 0],
    b: door,
    rotation: upright); // the frame's x axis is the one it turns about
```

`Fixed` welds, `Point` is a ball and socket, `Hinge` turns about x, `Slider`
slides along it, `Distance` keeps two points apart, as a rod or, with a range, a
rope. `Cone` lets x swing within a cone and twist within a range, and
`SixAxis` sets each of the six on its own. A hinge and a slider take a motor,
which drives at a speed no harder than a strength; a motor with no speed is
friction in the joint.

It is made where the bodies stand, and what it holds is how they stood, so a
hinge made with its door shut reads nought shut. Every measure is the second
body as the first sees it, so which body is first decides the sense: with the
world first a door's angle is the door's, and with the world second it is the
world's as the door sees it, the other way round. `JointStateOf` reads it
back, and says how hard it held on the last step, which is the number to look
at before choosing `breakingForce` or `breakingTorque`. Past either it breaks,
and a `Broke` event names it. Taking a body out of the world takes its joints
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
that has broken stays broken, in `scene.Broken`, until its own entity is
edited — dragging the door does not mend the hinge it tore off.

## On terrain

`Orblit.Physics.Terrain` lays an Orblit terrain as ground, a region to a piece,
and only the regions near a point.

```csharp
var ground = new TerrainPhysics(physics, terrain);

// Every frame, or whenever the camera moves.
ground.Sync(camera.X, camera.Z, radius: 64);
```

`Sync` lays the regions within the radius, takes up the ones a region's width
beyond it, and lays a region again when its heights, holes or any neighbour's
have changed. The ground it lays is the ground the renderer draws, triangle for
triangle, holes included. An editor passes `refresh: false` while a stroke is
being drawn and syncs normally once it is done, so a brush does not relay the
ground under it every frame. Its bodies are numbered far below zero, out of the
way of entity handles and of the small negative numbers a game gives its own
bodies; `RegionOf` turns one back into the region it is.

Like the scene bridge, it takes the engine packages from the local feed.

## On a rig

`Orblit.Physics.Rig` gives a skeleton from `Orblit.Rig` a body for each limb
and a joint to its parent. A `RagdollSpec` says which bones get a body, how
fat it is, and whether its joint is a hinge or a ball.

```csharp
var ragdoll = Ragdoll.Build(
    physics,
    pose,
    spec,
    model,
    ignore: characterLayer);
stack.Add("ragdoll", new RagdollPose(ragdoll));

// Every frame, after the stack has run.
physics.Step(dt);
```

Each body is `Animated`, `Powered` or `Limp`. An animated body is given the
velocity that reaches the pose in one step, so it pushes what it meets and is
not pushed back. A powered body is held to the pose's angles by the joint's
springs and gives under a blow. A limp body has only the joint's limits.

`Hit` makes the struck bone limp at once and its neighbours one joint at a time,
`spread` seconds apart. `SetFrom` makes a limb go from a bone down. An animated
body below a free one holds that one's far end where the pose has it, as a
hand holds a rail, so a limb that should fall has to go limp from its top.

`RagdollPose` writes the bones of free bodies back into the pose and leaves the
rest to the animation. Only the root is moved. Every other bone is turned and
stays on its parent, so bones keep the length the rig gave them however far a
joint has been pulled. `Recover` mixes from the last drawn fall to the animated
pose over a time, so a get-up does not jump.

`RagdollCharacter` hands a character between its capsule and its bodies. The
capsule and the ragdoll sit in layers that ignore each other. `Knock` takes the
capsule out, and `StandUp` finds the ground under the root, puts the capsule
there and makes every body animated again. The model is the game's to move.

Like the scene bridge, it takes `Orblit.Rig` from the local feed.

## On wheels

`Orblit.Physics.Vehicle` makes a car: a chassis body whose wheels are casts.
Each wheel hangs from a spring and a damper, written as a frequency and a
damping ratio so the same numbers suit a kart and a truck.

```csharp
var car = new Vehicle(
    physics,
    id: 1,
    chassis: Shape.Box(0.9, 0.3, 2.0),
    spec: new VehicleSpec(
        mass: 1200,
        antiRoll: 8000,
        wheels:
        [
            new WheelSpec(new Vector3(-0.8, 0, -1.2), radius: 0.35),
            new WheelSpec(new Vector3(0.8, 0, -1.2), radius: 0.35),
            new WheelSpec(new Vector3(-0.8, 0, 1.2), radius: 0.35),
            new WheelSpec(new Vector3(0.8, 0, 1.2), radius: 0.35),
        ]),
    at: [0, 0.6, 0]);

// Every step, before the step itself.
car.Update(dt);
physics.Step(dt);
```

Each wheel has a tyre that holds the car to the ground. A tyre bends like
rubber, so a parked car stays parked on a slope and a turning one gives a
steady side force. `VehicleInput` steers, brakes and pulls the handbrake. The
wheels that steer turn by Ackermann geometry, the inner one more than the
outer. A brake that can hold a wheel locks it, and a locked tyre grips less
than a rolling one. `Vehicle(surface:)` takes a function from the body under a
wheel to its grip, so ice and tarmac can share a world.

```csharp
car.SetInput(new VehicleInput(steer: 0.5, brake: 0.2));
```

A car with a `drive` has an engine. `EngineSpec` is a torque curve. `DriveSpec`
adds the gears, the final drive and a centrifugal clutch that holds nothing at
idle and grips harder as the engine speeds up, so a car pulls away without
stalling and does not creep. Wheels that `drives` share the torque, evenly
between axles, then between the two wheels of an axle through a differential.
`lock` is the torque that differential may move to the slower wheel: zero is
open and infinity is locked. The box shifts by itself, or holds the
`VehicleInput(gear:)` the driver picks, with reverse as minus one.

```csharp
car.SetInput(new VehicleInput(throttle: 1));
Console.WriteLine($"{car.Gear} {car.Rpm}");
```

The clutch is solved against what each tyre will do with a torque. A wheel on
the road feels the whole car behind it. A wheel in the air spins up. Wheelspin
on ice is smooth and does not flip from step to step.

The wheels have no mass. The chassis still collides, so a car that bottoms
out rests on its floor. A parked car never sleeps, because its springs push
it every step.

In a scene a car is a `VehicleComponent` on a free body, with a `WheelComponent`
on each of its direct children. Place each wheel entity where the wheel
sits at rest. `ScenePhysics` finds the spring from it, updates the car before
every step and writes the springs and the roll and steer of each wheel back to
its entity, so a model hung on the entity follows. The body must be at a scale
of one. A document that breaks that keeps its data and the body is just a body.

```csharp
scene.Pilot("car", new VehicleInput(throttle: 1, steer: -0.3));
var gear = scene.VehicleOf("car")!.Gear;
```

The scene has no input actions of its own. `Pilot` is the one place a game
hands over what a driver asks for, every step, and `Pilot("car", null)` lets go
of the controls. A snapshot does not hold a car's tyres, so a restored car
starts with straight bristles and the driver the game gave it.

## On cloth

`Orblit.Physics.Soft` makes cloth and rope: particles joined by links that
hold a distance, solved as position constraints in sub-steps of 1/480 of a
second. It is stepped beside a world and asks the world what it touches.

```csharp
var flag = SoftBody.Cloth(
    columns: 20,
    rows: 12,
    origin: new Vector3(0, 3.4, 0),
    across: new Vector3(0.08, 0, 0),
    down: new Vector3(0, -0.08, 0),
    surface: new PhysicsSurface(physics),
    air: new PhysicsWind(physics, new Dictionary<ulong, Vector3> { [gustZone] = new Vector3(0, 0, 8) }));
for (var row = 0; row < 12; row++)
{
    flag.Pin(row * 20);
}

physics.Step(dt);
flag.Step(dt);
```

`PhysicsSurface` keeps the particles off the bodies of a world by casting a
sphere, and a floor must be a box because a world cannot cast a plane.
`Attach` holds a particle to a point that moves, such as `BodyPoint` on a
body, and lets it go when the pull passes a `limit`. `PhysicsWind` blows the
cloth from the trigger bodies of a world.

The solver draws nothing. `ParticleSheet` in `Orblit.Stage` draws a
cloth of up to 256 particles as a skinned sheet with a joint for each
particle. The gallery has a flag.

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

**Continuous collision between free bodies.** A body that names `Continuous`
is cast against fixed bodies and ground only. Two free bodies rely on the
contacts made before they touch. A step short enough for the speeds in play is
the answer past that.

**The web.** The solver is C++ built by CMake, so it goes where a native
toolchain goes. Whether it follows Orblit to the web later is open.

**Cloth meeting cloth.** A soft body meets the world and not itself, so a cloth
folded over passes through. It holds no volume either. A pillow or a balloon
would need a constraint that is not there.

## Licence

MPL-2.0, © 2026 Chris Beckett. That is the Mozilla Public License, and it is
open source. Use it, fork it and ship games with it, including commercial ones.
Your game stays yours, and the licence does not reach into it. What it asks is
that changes to this repository's own files ship under the same licence, so
engine work stays in the open.

## Compound and scaled shapes

`LayCompound(id, parts)` keeps 1 to 64 `ShapePart` values under an id
for `Shape.Compound(id)` to name. Each part is a convex primitive or a laid
hull, with local `at`, `rotation` in xyzw order, and positive `scale`.
Scale is applied before turning and placement. An outer `scale` stretches the
whole compound, including rotated parts and their offsets. One part provides
a scaled or offset shape. Nonuniform scaling stretches rounded geometry
exactly. The gaps between parts stay empty for collisions and queries.

Parts share one body, its material and its events. Mass is distributed by
volume at uniform density, with a combined inertia tensor. Overlaps count
mass twice. Four contact points remain the budget for each pair of bodies.
Planes, height fields, meshes and nested compounds cannot be parts.

Remove bodies before `DropCompound`. A hull retained by a compound cannot be
dropped until that compound goes. Snapshots retain both assets independently.

In a scene, `BodyShape.Compound` reads `BodyComponent.Parts`, a list of
`BodyPart` values with local centre, quaternion rotation and scale. The
body's `ShapeScale` stretches its collision geometry independently of its
visual transform. Compounds and explicit `ShapeScale` use exact scaling.
Ordinary rounded bodies with default `ShapeScale` retain their previous
entity sizing rules. Invalid parts remain editable as an inert body.


## Static model surfaces

`LayMesh(id, vertices, indices)` copies indexed triangles,
welds coincident coordinates and builds a spatial tree with seam information.
`Shape.Mesh(id)` names the result on a fixed body. Rays, convex shape casts,
overlaps, contacts and characters meet the same two-sided surfaces. Shared
flat edges produce no rim impulse, including duplicated exporter vertices.
Outer and nonmanifold edges stay rims; separate assets do not share seams.

A mesh has no solid interior and cannot move or be a compound part. Use a
convex hull or convex compound for something the solver moves. Invalid inputs
and all-degenerate meshes return false. Remove its bodies before `DropMesh`;
snapshots retain their own references.

`CollisionMesh.FromGltfAsync` in `Orblit.Asset` bakes an active static model scene
into indexed triangles, or model settings `collision: true` cook a
`collision.json` sidecar. A scene uses `BodyShape.Mesh` with `MeshVertices`
and `MeshIndices`; the bridge applies entity and `ShapeScale`, forces fixed
motion and owns the asset through edits, removal and snapshot restoration.
