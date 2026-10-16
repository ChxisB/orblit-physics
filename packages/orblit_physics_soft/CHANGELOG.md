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
