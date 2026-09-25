## 0.3.0

- Characters: `Physics.addCharacter` makes a body that walks. It is swept
  through the world rather than solved, so it slides along walls, climbs steps
  up to a height, stands on slopes up to an angle and slides down steeper ones,
  rides whatever it stands on, pushes free bodies no harder than it is allowed
  to, and is pushed by nothing but driven ones. `drive` is what it asks for,
  gravity included; `footingOf` and `footingsOf` read back what it stood on and
  what survived of the request.
- A ray landing on a face away from its middle reports the face's own normal.
  It used to lean by however the last bits of the float fell.
- A shape cast alongside a surface it is already touching, and not closing on
  it, misses it. It used to be stopped dead against it.

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
