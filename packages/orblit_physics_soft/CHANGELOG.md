## 0.2.0

- `SoftBody.balloon` makes a closed triangle mesh whose edges are links and
  whose volume is held as a constraint, so a balloon or a pillow squashes
  under a load without going flat. `volumeCompliance` lets the volume give.
  A surface with a hole, or wound inside out, throws `ArgumentError`.
- `SoftBody.volume` reads what a balloon encloses, or null for a body that is
  not closed.
- A balloon holds no bend links, so it wrinkles when squeezed.

## 0.1.0

- `SoftBody.cloth` and `SoftBody.rope` make particles joined by distance
  links. A cloth has links along its rows and columns, along both diagonals
  and two apart.
- The links are solved as position constraints with a compliance, in
  sub-steps no longer than `SoftSpec.substepLength`. A body hangs the same at
  30 and at 60 frames a second.
- `pin`, `release` and `place` hold a particle where the caller puts it.
- A body that has been still for a second stops being stepped until a pin
  moves.
- A `SoftSurface` makes the particles stay off the world. `PhysicsSurface`
  reads a `Physics` world with a ball cast, once a step for each particle.
  `SoftSpec.thickness` is the skin and `SoftSpec.friction` the grip.
- A cloth dropped from any height is stopped by a surface it can reach in a
  step. A world cannot cast a plane, so a floor must be a box.
- `attach` holds a particle to a point a closure answers, asked at the start
  of each step. `bodyPoint` gives the point for a place on a physics body, and
  a bone is any closure of the caller's own. `pullOf` reads the force the body
  puts on the particle. With a `limit` in newtons the particle lets go when
  the pull, averaged over a step, passes it. `detach` and `isAttached` finish
  the set.
- A `SoftAir` blows the body. `PhysicsWind` reads it from trigger bodies of a
  `Physics` world, asked at the middle of the body once a step.
  `SoftSpec.windDrag` is how fast the air takes a particle up to its speed.
