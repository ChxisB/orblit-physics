import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

SceneEntity entityOf(
  String id, {
  required Vector3 at,
  required BodyComponent body,
  Vector3? scale,
}) => SceneEntity(
  id: id,
  name: id,
  components: {
    SceneComponents.transform: TransformComponent(position: at, scale: scale),
    SceneComponents.body: body,
  },
);

SceneEntity floor() => entityOf(
  'floor',
  at: Vector3.zero(),
  body: BodyComponent(shape: BodyShape.plane, motion: BodyMotion.fixed),
);

/// The eight corners of a cube [half] either side of its middle.
List<double> cube(double half) => [
  for (final x in [-half, half])
    for (final y in [-half, half])
      for (final z in [-half, half]) ...[x, y, z],
];

BodyComponent hullOf(List<double> corners) =>
    BodyComponent(shape: BodyShape.hull, hull: corners);

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

double heightOf(ScenePhysics scene, String entity) =>
    scene.physics.transformOf(scene.bodyOf(entity)!)![1];

void run(ScenePhysics scene, double seconds) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
  }
}

void remove(ScenePhysics scene, String entity) =>
    scene.apply(SceneDiff([RemoveEntity(scene.document[entity]!.toJson())]));

/// Whether the world has a hull under [id], told by trying to lay one there.
/// When it has none the try lays one, so ask last.
bool hasHull(ScenePhysics scene, int id) =>
    !scene.physics.layHull(id, points: cube(1));

/// How far into the ground the solver lets a body rest, which is the same for
/// every shape: a box and a ball sink as far as these do.
const slop = 0.03;

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  group('a cylinder body', () {
    test('stands on its flat end, as tall as it says', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'barrel',
            at: Vector3(0, 3, 0),
            body: BodyComponent(
              shape: BodyShape.cylinder,
              radius: 0.4,
              height: 1.2,
            ),
          ),
        ]),
      );
      run(scene, 2);

      expect(heightOf(scene, 'barrel'), closeTo(0.6, slop));
    });

    test('grows with its entity, the sideways scale for the radius', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'barrel',
            at: Vector3(0, 4, 0),
            scale: Vector3(2, 3, 1),
            body: BodyComponent(
              shape: BodyShape.cylinder,
              radius: 0.4,
              height: 1.2,
            ),
          ),
        ]),
      );
      run(scene, 2);

      // Three times as tall, so 3.6 metres from end to end.
      expect(heightOf(scene, 'barrel'), closeTo(1.8, slop));
    });
  });

  group('a hull body', () {
    test('rests on the ground as the solid round its corners', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf('rock', at: Vector3(0, 3, 0), body: hullOf(cube(0.5))),
        ]),
      );
      run(scene, 2);

      expect(heightOf(scene, 'rock'), closeTo(0.5, slop));
    });

    test('is stretched by its entity along each axis', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'slab',
            at: Vector3(0, 4, 0),
            scale: Vector3(1, 0.2, 3),
            body: hullOf(cube(0.5)),
          ),
        ]),
      );
      run(scene, 2);

      expect(heightOf(scene, 'slab'), closeTo(0.1, slop));
    });

    test('is placed from its centre, where the corners are measured', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'rock',
            at: Vector3(0, 3, 0),
            body: hullOf(cube(0.5)).copyWith(centre: Vector3(0, 0.5, 0)),
          ),
        ]),
      );

      expect(heightOf(scene, 'rock'), closeTo(3.5, 1e-4));
    });

    test('that encloses nothing makes no body, and the entity keeps its '
        'number', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'flat',
            at: Vector3(0, 3, 0),
            body: hullOf([0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1]),
          ),
          entityOf('ragged', at: Vector3(4, 3, 0), body: hullOf([0, 1, 2, 3])),
        ]),
      );

      expect(scene.bodyOf('flat'), isNotNull);
      expect(scene.physics.velocityOf(scene.bodyOf('flat')!), isNull);
      expect(scene.physics.velocityOf(scene.bodyOf('ragged')!), isNull);
      expect(scene.advance(1 / 60).isEmpty, isTrue);
    });
  });

  group('the hulls a scene lays', () {
    SceneDocument crates() => sceneOf([
      floor(),
      entityOf('a', at: Vector3(0, 2, 0), body: hullOf(cube(0.5))),
      entityOf('b', at: Vector3(3, 2, 0), body: hullOf(cube(0.5))),
      entityOf(
        'c',
        at: Vector3(6, 2, 0),
        scale: Vector3(2, 2, 2),
        body: hullOf(cube(0.5)),
      ),
    ]);

    test('are shared by bodies cut from the same corners at the same size', () {
      scene = ScenePhysics(crates());

      // Two hulls, not three: the bridge numbers its own from one.
      expect(hasHull(scene, 1), isTrue);
      expect(hasHull(scene, 2), isTrue);
      expect(hasHull(scene, 3), isFalse);
    });

    test('stay while a body is made of them, and go with the last', () {
      scene = ScenePhysics(crates());
      remove(scene, 'a');
      expect(hasHull(scene, 1), isTrue);

      remove(scene, 'b');
      expect(hasHull(scene, 2), isTrue);
      expect(hasHull(scene, 1), isFalse);
    });

    test('change when the body is edited to other corners', () {
      scene = ScenePhysics(crates());
      final before = scene.document['c']![SceneComponents.body]!;
      scene.apply(
        SceneDiff([
          SetComponent(
            'c',
            SceneComponents.body,
            from: before.toJson(),
            to: hullOf(cube(0.3)).toJson(),
          ),
        ]),
      );

      // The cube at twice the size is gone, and the one at the new size is
      // laid. The two unedited bodies still have theirs.
      expect(hasHull(scene, 1), isTrue);
      expect(hasHull(scene, 3), isTrue);
      expect(hasHull(scene, 2), isFalse);
    });

    test('come back with a snapshot, and go again with the last body', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf('rock', at: Vector3(0, 3, 0), body: hullOf(cube(0.5))),
        ]),
      );
      final snapshot = scene.snapshot();
      remove(scene, 'rock');
      expect(hasHull(scene, 1), isFalse);

      scene.restore(snapshot);
      run(scene, 2);
      expect(heightOf(scene, 'rock'), closeTo(0.5, slop));

      remove(scene, 'rock');
      expect(hasHull(scene, 1), isFalse);
      snapshot.dispose();
    });
  });
}
