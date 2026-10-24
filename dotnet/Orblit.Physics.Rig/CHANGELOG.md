# Changelog

## 0.1.0

The version is the one the package had when the port began.

- `Ragdoll.Build` makes a body for each bone a `RagdollSpec` names, with a
  joint to its parent. The bodies share one layer that does not collide with
  itself, so a forearm passes through the thigh it hangs beside.
- Each body is `Animated`, `Powered` or `Limp`. An animated body follows the
  pose by velocity. A powered body is held to the pose's angles by springs
  and gives under a hit. A limp body has only its joint's limits.
- `Ragdoll.Hit` limps a bone at once and its neighbours one joint at a time,
  so a blow runs out from where it landed. `SetMode`, `SetFrom` and `SetAll`
  change modes by hand.
- `RagdollPose` is a modifier. It drives the bodies from the animated pose
  and writes the free bones back. `Recover` mixes the output from the last
  fallen pose to the animation.
- `RagdollCharacter` hands a walking capsule over to the ragdoll with
  `Knock` and takes it back with `StandUp`. `StandUp` finds the floor under
  the root, stands the capsule there and starts the mix.
- A turn is measured as a joint measures it, so a pose's angles become the
  places the springs pull to. The measure is internal to the package.
- `Ragdoll` and `RagdollCharacter` are `IDisposable`. `Dispose` takes their
  bodies out of the world.
