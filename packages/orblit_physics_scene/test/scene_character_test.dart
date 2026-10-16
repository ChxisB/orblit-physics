import 'package:orblit_motion/orblit_motion.dart';
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// Two metres along Z a second, looping, with the hips doing the walking.
ClipDocument walkClip() => ClipDocument(
  name: 'Walk',
  duration: 1,
  whenDone: WhenDone.loop,
  rootMotion: RootMotion(bone: 'hips'),
  channels: [
    ClipChannel<Vector3>(
      target: '',
      bone: 'hips',
      property: 'position',
      kind: ChannelKind.vector,
      keys: [
        Key(0, Vector3(0, 1, 0), hold: Hold.linear),
        Key(1, Vector3(0, 1, 2)),
      ],
    ),
    ClipChannel<Quaternion>(
      target: '',
      bone: 'hips',
      property: 'rotation',
      kind: ChannelKind.rotation,
      keys: [
        Key(0, Quaternion.identity(), hold: Hold.linear),
        Key(1, Quaternion.identity()),
      ],
    ),
  ],
);

SceneEntity solid(String id, Vector3 size, Vector3 at) => SceneEntity(
  id: id,
  name: id,
  components: {
    SceneComponents.transform: TransformComponent(position: at),
    SceneComponents.body: BodyComponent(size: size, motion: BodyMotion.fixed),
  },
);

/// A floor, a step 0.2 m tall from z 1 to 2, a beam from z 3 to 6 with its
/// underside 1.2 m up, and a hero at the origin, written out and read back as
/// a file so that nothing here is only in memory.
SceneDocument world({double crouchHeight = 1.0}) {
  final built = SceneDocument(
    name: 'Scene',
    entities: [
      SceneEntity(
        id: 'floor',
        name: 'Floor',
        components: {
          SceneComponents.transform: TransformComponent(),
          SceneComponents.body: BodyComponent(
            shape: BodyShape.plane,
            motion: BodyMotion.fixed,
          ),
        },
      ),
      solid('step', Vector3(2, 0.2, 1), Vector3(0, 0.1, 1.5)),
      solid('beam', Vector3(2, 0.2, 3), Vector3(0, 1.3, 4.5)),
      SceneEntity(
        id: 'hero',
        name: 'Hero',
        components: {
          SceneComponents.transform: TransformComponent(
            position: Vector3(0, 0.01, 0),
          ),
          SceneComponents.body: BodyComponent(
            shape: BodyShape.capsule,
            motion: BodyMotion.character,
            radius: 0.3,
            height: 1.8,
            centre: Vector3(0, 0.9, 0),
            crouchHeight: crouchHeight,
          ),
        },
      ),
    ],
  );
  final load = SceneDocument.decode(built.encode());
  expect(load.hasProblems, isFalse, reason: '${load.problems}');
  return load.document;
}

double feetOf(ScenePhysics scene) {
  final transform =
      scene.document['hero']![SceneComponents.transform]! as TransformComponent;
  return transform.position.y;
}

double depthOf(ScenePhysics scene) {
  final transform =
      scene.document['hero']![SceneComponents.transform]! as TransformComponent;
  return transform.position.z;
}

/// Frame by frame, because one call takes no more than `maxSteps` steps.
void run(ScenePhysics scene, double seconds) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
  }
}

void main() {
  late ScenePhysics scene;

  tearDown(() => scene.dispose());

  test('a character in a file is swept, and the file keeps its numbers', () {
    scene = ScenePhysics(world());
    final body =
        scene.document['hero']![SceneComponents.body]! as BodyComponent;

    expect(body.motion, BodyMotion.character);
    expect(body.crouchHeight, 1.0);
    expect(scene.physics.footingOf(scene.bodyOf('hero')!), isNotNull);
    expect(scene.physics.footingOf(scene.bodyOf('step')!), isNull);
  });

  test('root motion takes it up a step, then under a beam where it cannot '
      'stand', () {
    scene = ScenePhysics(world());
    final walk = RootMotionWalk(ClipPlayer(walkClip())..play());

    expect(scene.crouch('hero', true), isTrue);
    expect(scene.isCrouching('hero'), isTrue);
    scene.walk('hero', walk);

    // Two metres a second, so three quarters of one is the step's middle.
    run(scene, 0.75);
    expect(depthOf(scene), closeTo(1.5, 0.15), reason: 'on the step');
    expect(feetOf(scene), closeTo(0.2, 0.03), reason: 'feet on its top');

    run(scene, 1.25);
    expect(depthOf(scene), greaterThan(3.2), reason: 'under the beam');
    expect(feetOf(scene), closeTo(0.0, 0.03), reason: 'off the step again');
    expect(
      scene.crouch('hero', false),
      isFalse,
      reason: 'a 1.8 m capsule is not 1.2 m tall',
    );
    expect(scene.isCrouching('hero'), isTrue);

    run(scene, 1.5);
    expect(depthOf(scene), greaterThan(6.3), reason: 'out the far side');
    expect(scene.crouch('hero', false), isTrue);
    expect(scene.isCrouching('hero'), isFalse);
    expect(feetOf(scene), closeTo(0.0, 0.03), reason: 'feet stayed put');
  });

  test('a standing character is stopped by the beam it cannot go under', () {
    scene = ScenePhysics(world());
    scene.walk('hero', RootMotionWalk(ClipPlayer(walkClip())..play()));

    run(scene, 4.0);

    expect(depthOf(scene), lessThan(2.8), reason: 'stopped short of the beam');
    final footing = scene.physics.footingOf(scene.bodyOf('hero')!)!;
    expect(footing.wall, isTrue);
  });

  test('a body that is not a character has no stance to take', () {
    scene = ScenePhysics(world());

    expect(scene.crouch('step', true), isFalse);
    expect(scene.crouch('nobody', true), isFalse);
  });

  test('a character with no crouch height cannot crouch', () {
    scene = ScenePhysics(world(crouchHeight: 0));

    expect(scene.crouch('hero', true), isFalse);
    expect(scene.isCrouching('hero'), isFalse);
  });

  test('a snapshot keeps a crouch, and a restore stands it where it was', () {
    scene = ScenePhysics(world());
    scene.crouch('hero', true);
    final snapshot = scene.snapshot();
    scene.crouch('hero', false);
    expect(scene.isCrouching('hero'), isFalse);

    scene.restore(snapshot);
    snapshot.dispose();

    expect(scene.isCrouching('hero'), isTrue);
  });
}
