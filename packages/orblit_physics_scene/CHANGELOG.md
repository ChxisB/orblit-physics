## 0.5.0

- Snapshots: `snapshot()` copies the simulation and `restore(snapshot)` goes
  back to it, for an undo or a rollback that resimulates from a known frame.
  It holds the world's snapshot and what the scene keeps beside it: the
  document, which body each entity is, the joints, the broken ones, the
  pairs told to pass through each other, the time still owed and the events
  of the last `advance`.
- `restore` answers with the diff that takes the document from what it was
  to the snapshot's, so whatever draws the scene can follow. The diff is
  already in `document`.
- The next body and joint numbers go back with the snapshot. An entity added
  after it gets the same number again, which a replay needs: the world
  orders its events and contacts by number.
- A snapshot is native memory and is let go of with `dispose`. Restoring a
  snapshot that has been disposed throws `StateError` and changes nothing.
  One restored into a scene with a different `step` does not replay.
- Needs `orblit_scene` 0.11.0 and `orblit_physics` 0.8.0. What the world
  saw in its last step is `physics.contacts` and `physics.stats`.

## 0.4.0

- Body controls: a body's `locks`, `gravityScale`, `maxSpeed`, `maxSpin`,
  `centreOfMass` and `inertia` are sent to the world with it. The centre of
  mass grows with the entity's scale and the inertia does not. A body with
  none of them set sends nothing. A negative cap or inertia from a hand
  edited file is read as none, so the locks still apply.
- A body added with `startsAsleep` stays asleep with its controls.
- `ignore` and `unignore` make two entities pass through each other, and
  restore contact. They take entity ids, in either order. The pair is kept
  when an edit rebuilds a body, and forgotten when an entity loses its body
  or leaves the document. A character does not read the rule.
- Needs `orblit_scene` 0.11.0, for the body fields, and `orblit_physics`
  0.7.0.

## 0.3.0

- Triggers: a body with `BodyComponent.trigger` is a place. What enters and
  leaves it is heard in `events` as `entered` and `exited`, with the
  trigger's number first. `stay` adds `touchStay` and `inside`. The flag is
  ignored for a free body, which the solver has to move.
- Zones: an entity with a `ZoneComponent` and a body is a trigger whose
  region changes how the free bodies in it move. Editing or removing the
  zone makes the world agree, and what was inside is woken.
- Belts: `BodyComponent.surface` carries what stands on the body.
- Needs `orblit_scene` 0.10.0, for `ZoneComponent` and the new body fields,
  and `orblit_physics` 0.6.0.

## 0.2.0

- Joints: an entity with a `JointComponent` joins the nearest body at or above
  it to the nearest body above that, or to the world, at the entity's own
  place and frame. Angles in the document are degrees, as a transform's are.
- An edit that rebuilds either body, or touches the joint entity, makes the
  joint again as things then stand. A joint that breaks stays broken, and in
  `broken`, until its own entity is edited.
- `jointOf` and `entityOfJoint` pair joint entities with the world's joint
  numbers, which are counted apart from the bodies'.
- Needs `orblit_scene` 0.9.0, for `JointComponent`, and `orblit_physics`
  0.5.0.

## 0.1.1

- A body whose `centre` is off its entity's origin leaves the entity where it
  should be once it has turned. The offset was turned the opposite way when
  the entity was read back from the body, so a turned body moved its entity
  sideways by up to twice that offset.

## 0.1.0

- First release. `ScenePhysics` builds a physics world from a scene document's
  body components, placed where the document puts each entity in the world,
  sized by its world scale and offset by the body's centre. `advance` steps it
  at a fixed rate and answers with a `SceneDiff` that moves every entity whose
  body moved — and every body under one that did — to where the world has it,
  with its rotation written as the angles nearest the ones it had. `apply`
  takes an edit the other way and rebuilds the bodies it touched, and the ones
  under them, where the document now puts them.
- `events` is everything that happened in the steps the last `advance` took,
  so a slow frame that owed several steps hears all of them and a frame that
  owed none hears nothing.
