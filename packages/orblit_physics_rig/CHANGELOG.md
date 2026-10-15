## 0.1.0

- First release. `Ragdoll.build` makes a body for each bone a `RagdollSpec`
  names and a joint to its parent, in a group that does not collide with
  itself. A bone is `animated`, `powered` or `limp`: an animated body follows
  the pose by velocity, a powered one is held to the pose's angles by springs
  and gives under a hit, a limp one is only its joint's limits. `hit` makes a
  bone limp at once and its neighbours one hop at a time, so a blow spreads
  out from where it landed. `setMode` changes one body and `setFrom` a body
  and everything that hangs below it.
- `RagdollPose` is a `Modifier`: it drives the bodies from the animated pose
  and writes the limp and powered bones back from the bodies. `recover`
  blends from the limp pose to the animation over a time.
- `RagdollCharacter` hands a character's capsule over to the ragdoll and back:
  `knock` takes the capsule out and lets the bodies go, `standUp` finds the
  floor under the root, puts the capsule there and starts the blend.
- `turnAngles` measures a turn as a joint does, so a pose's angles can be
  turned into the places a joint's springs pull to.
