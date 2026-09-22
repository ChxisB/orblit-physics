## 0.2.0

- Capsules: `Shape.capsule`, against every other shape and against each other.
  Where the contact is a line rather than a point it is reported at both ends,
  so a capsule resting on its side settles instead of rocking.
- Shape casting: `Physics.cast` fires a point, sphere, box or capsule along a
  direction and answers the first body it meets, with a layer filter and a body
  to ignore. A cast that begins inside something says so.

## 0.1.0

- First release. A sequential impulse solver in C++ with spheres, boxes and
  planes, friction, restitution, static, driven and free bodies, layer
  filtering, per-body sleeping, and touch, sleep and wake events.
- A Dart API over it: commands queued into one native buffer, a whole world
  read back into a caller's float buffer at its own stride.
