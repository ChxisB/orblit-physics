## 0.6.0

- Triggers: `add(trigger: true)` makes a place rather than a thing. Nothing
  collides with it and it pushes nothing. It reports `entered` and `exited`
  with the trigger named first, and `inside` every step when it has `stay`.
  Solid casts, overlaps and characters do not see it. Only a fixed or driven
  body can be one.
- `stay` on either body of a contact adds `touchStay` every step the touch
  goes on, between `touchBegan` and `touchEnded`. A pair that has gone to
  sleep has its touch ended, so the stay stops with it.
- Zones: `setZone` puts a gravity, a linear damping and an angular damping
  over a trigger's region, each optional, with a priority. Each field is
  decided on its own. The highest priority wins and the lower id wins a tie.
  A zone acts on free bodies that are awake. `removeZone` takes it off.
- Contact rules: `setRule` changes the friction and restitution of one pair
  of bodies, and how much of the contact's push each takes. `removeRule`
  clears it. This is the contact hook as data rather than a callback. A rule
  ends when either body is removed.
- `setSurface` sets how fast a body's surface moves while the body stays put:
  a conveyor belt. What stands on it is carried to that speed and no more.
- Queries: `castAll` gives every hit along a ray or a swept shape, nearest
  first, up to a `limit`. `castAny` says whether there is one. `overlap` says
  which bodies touch a shape, or a point when it has none. `triggers: true`
  asks about triggers and nothing solid.
- The ABI gains `OrblitPhysicsZone`, `OrblitPhysicsRule`,
  `orblit_physics_zone`, `_rule`, `_cast_all`, `_cast_any` and `_overlap`.
  `OrblitPhysicsCommand` gains `sensor` and `stay` and the kind `SURFACE`.
  `OrblitPhysicsCast` gains `triggers`. The events `ENTERED`, `EXITED`,
  `TOUCH_STAY` and `INSIDE` are new.

## 0.5.0

- Joints: `Physics.join` holds two bodies together, or one to the world, as a
  `Joint` says — `fixed`, `point`, `hinge`, `slider`, `distance`, `cone` or
  `sixAxis`. Each is which of the six ways the second body can move relative
  to the first it holds, and how far, solved in the same passes as the
  contacts. A range solves only its nearer end, a hinge and a slider take a
  motor, and a joint made with its door shut reads nought shut.
- Either end may be the world, and which one decides the sense: every measure
  is the second body as the first sees it.
- `jointStateOf` reads back how a joint stands and how hard it held on the
  last step. Past `breakingForce` or `breakingTorque` it breaks, and a
  `PhysicsEventKind.broke` event names it.
- Bodies joined together sleep and wake as one. Removing a body takes its
  joints with it, silently, and wakes what they held; laying ground again
  keeps them. `unjoin` takes one away.
- The ABI gains `OrblitPhysicsJoint`, `OrblitPhysicsJointState` and
  `orblit_physics_join`, `_unjoin` and `_joint`.

## 0.4.0

- Ground: `Physics.layGround` lays heights on a grid as one fixed body. Each
  square is two triangles, everything under the surface is solid, so ground
  can be raised under a sleeping crate and push it up and out, and a height
  that is not a number is a hole. Laying it again under the same id replaces
  it and wakes whatever is over the old ground or the new.
- Ground streamed in pieces behaves as one ground. The edge of a field is a
  seam, not a cliff, and a field laid with a `margin` knows its neighbours'
  samples, so a ridge or a valley along the line between two pieces holds a
  ball the way one field would.
- A seam between two triangles is not an edge: a ball rolls across flat ground
  without hopping at the line between cells, and a box sits in a crease
  between two slopes without being shoved off it. Every contact point now
  carries its own normal, which is what lets one pair touch two slopes at
  once.
- Casts, characters and the solver all meet ground: a character walks over
  it, climbs it and slides down what is too steep, and a cast stops on it.

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
