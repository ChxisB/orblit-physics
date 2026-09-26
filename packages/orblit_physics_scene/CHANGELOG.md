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
