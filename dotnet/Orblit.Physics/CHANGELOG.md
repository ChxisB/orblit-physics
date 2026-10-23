# Changelog

## 0.17.0

Loads the solver and makes a world from its settings. The version is the one
the Dart package had when the port began.

Adds bodies, characters and height ground. A world steps, and answers each
body's transform, velocity, footing and events, and the first thing a ray or
shape meets. The other casts, body controls and the convex shapes follow in
later parts.

Adds joints. A `Joint` says which of the six ways its second body may move,
and `Physics.Join` says where it goes. Eleven kinds hold bodies together:
fixed, point, hinge, slider, distance, cone, six-axis, gear, rack, pulley and
path. A motor, a goal, springs and a soft limit give a joint some play, and
`Aim` moves them without making the joint again. A joint breaks past a force
or a torque, and `JointStateOf` reads how it stands.
