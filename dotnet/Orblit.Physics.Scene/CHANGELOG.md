# Changelog

## 0.13.0

The version is the one the package had when the port began.

- `ScenePhysics` runs a `SceneDocument` in a `Physics` world. It makes a
  body for each entity with a body component and a joint for each joint
  component. `Advance` takes the time owed in whole steps, at most
  `maxSteps` of them, and answers with the diff that writes the moved
  bodies back into the document.
- `Apply` takes a `SceneDiff` and rebuilds only the bodies, joints and cars
  it touches, with everything below them in the tree.
- A body hung from another is written back against where its parent went. A
  scaled or mirrored entity is written back by where its origin went, not
  its centre.
- Joints are numbered nearest the root first. A joint with a breaking force
  that breaks is listed in `Broken` and is not made again until its
  component is edited.
- `Ignore` and `Unignore` stop a pair of bodies colliding. `Crouch` shrinks
  a character's capsule to its crouch height.
- `Walk` hands a character to an `ICharacterWalk`. It is asked once a step
  and is told what the character stands on. `RootMotionWalk` walks a
  character along the root motion of a clip.
- A vehicle component on a body with wheel entities below it becomes a car.
  `Pilot` sets its input and `VehicleOf` reads it. A car sags on the gravity
  of the scene's settings.
- A body component can be a box, sphere, capsule, cylinder, plane, hull,
  mesh or compound. A mesh with indices outside the range of an `int` is
  refused.
- With `smooth` set, `Advance` writes the pose blended between the last two
  steps. `StopSmoothing`, `StartSmoothing` and `ResetSmoothing` choose which
  bodies are shown sharp.
- `Snapshot` copies the world, the document and every number the scene has
  handed out. `Restore` goes back to it and answers with the diff to the
  document that was restored. A snapshot taken by a scene that smooths does
  not go into one that does not.
- `ScenePhysics` and `ScenePhysicsSnapshot` are `IDisposable`.
