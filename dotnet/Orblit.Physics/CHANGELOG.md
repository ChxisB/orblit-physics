# Changelog

## 0.18.0

`OverlapInto` writes the ids `Overlap` would list into a buffer the caller
owns. It takes a `Span<ulong>` after the point and answers how many ids it
wrote, up to the length of the buffer. The rest of the buffer is left as it
was. `Overlap` still makes a new array on each call, and now calls this one.
`DefaultOverlapLimit` names the 32 that `Overlap` lists when no limit is given.

## 0.17.0

The solver is packed at `runtimes/<rid>/native`, where NuGet looks for it.

Loads the solver and makes a world from its settings. The version is the one
the package had when the port began.

Adds bodies, characters and height ground. A world steps, and answers each
body's transform, velocity, footing and events, and the first thing a ray or
shape meets.

Adds joints. A `Joint` says which of the six ways its second body may move,
and `Physics.Join` says where it goes. Eleven kinds hold bodies together:
fixed, point, hinge, slider, distance, cone, six-axis, gear, rack, pulley and
path. A motor, a goal, springs and a soft limit give a joint some play, and
`Aim` moves them without making the joint again. A joint breaks past a force
or a torque, and `JointStateOf` reads how it stands.

Adds asking the world. A trigger can wear a `PhysicsZone` that changes the
gravity or the damping of what is inside it, or makes a fluid that floats a
light body and sinks a heavy one. A `PhysicsRule` changes the contact between
two named bodies: its friction, its restitution, how much of the push each
takes, passing through, or a one-way platform. `SetSurface` makes a belt.
`CastAll`, `CastAny` and `Overlap` ask what a ray or a shape meets, in order,
at all, or where it stands.

Adds body controls. `SetControls` sets how a body moves apart from its shape:
which of the six ways are locked, how much gravity it feels, a cap on its
speed and on its spin, where its weight is, its inertia, and whether it is
stopped at a thin wall. `SetMotion` makes a body fixed, driven or free, and
`SetGravity` sets the world's gravity.

Adds seeing what happened. `Snapshot` copies the whole world and `Restore`
makes a world that copy again, in the same world or in another. `Contacts`
lists the points where bodies touched in the last step, and `Stats` says how
much a world holds and how much its last step looked at.

Adds hulls, compounds and meshes. `LayHull` cooks a cloud of points into a
convex solid that any number of bodies name with `Shape.Hull`. A `ShapePart`
is one convex shape with an offset, a turn and a scale, and `LayCompound`
joins up to 64 of them into one body named with `Shape.Compound`. `LayMesh`
takes indexed triangles for fixed ground and walls, named with `Shape.Mesh`.
Each asset stays while a body names it, and `DropHull`, `DropCompound` and
`DropMesh` take an unused one away.
