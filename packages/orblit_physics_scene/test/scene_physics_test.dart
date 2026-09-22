import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// Ground through the origin, facing up.
SceneEntity floor({double height = 0}) => SceneEntity(
  id: 'floor',
  name: 'Floor',
  components: {
    SceneComponents.transform: TransformComponent(
      position: Vector3(0, height, 0),
    ),
    SceneComponents.body: BodyComponent(
      shape: BodyShape.plane,
      motion: BodyMotion.fixed,
    ),
  },
);

/// A one-metre crate, dropped from [at] unless it is told otherwise.
SceneEntity crate({
  String id = 'crate',
  String? parent,
  Vector3? at,
  Vector3? rotation,
  Vector3? scale,
  BodyComponent? body,
}) => SceneEntity(
  id: id,
  name: id,
  parent: parent,
  components: {
    SceneComponents.transform: TransformComponent(
      position: at ?? Vector3(0, 3, 0),
      rotation: rotation,
      scale: scale,
    ),
    SceneComponents.body: body ?? BodyComponent(),
  },
);

/// How far a resting body can be from touching exactly: the solver lets it sink
/// up to its slop, two centimetres, so a contact is not lost and found again
/// every step, and a little more for the bounce still settling.
const resting = 0.03;

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

TransformComponent transformOf(SceneDocument document, String id) =>
    document[id]![SceneComponents.transform]! as TransformComponent;

/// Runs [seconds] of simulation a sixtieth at a time, as a frame loop would,
/// and answers with every diff it made.
List<SceneDiff> run(ScenePhysics scene, double seconds) => [
  for (var i = 0; i < (seconds * 60).round(); i++) scene.advance(1 / 60),
];

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  group('a body in a document', () {
    test('falls, lands and says where it landed', () {
      scene = ScenePhysics(sceneOf([floor(), crate()]));
      run(scene, 3);

      final rested = transformOf(scene.document, 'crate');
      expect(rested.position.y, closeTo(0.5, resting));
      expect(rested.position.x, closeTo(0, 0.01));
      expect(rested.rotation.x, closeTo(0, 0.5));
      expect(rested.rotation.z, closeTo(0, 0.5));
      expect(rested.scale, Vector3.all(1));
    });

    test('the diffs are the simulation: applied in turn they reach the '
        'document it ended with', () {
      final start = sceneOf([floor(), crate()]);
      scene = ScenePhysics(start);

      var followed = start;
      for (final diff in run(scene, 2)) {
        followed = diff.applyTo(followed);
      }

      expect(followed.encode(), scene.document.encode());
      expect(followed.encode(), isNot(start.encode()));
    });

    test('each diff undoes', () {
      scene = ScenePhysics(sceneOf([floor(), crate()]));
      run(scene, 0.5);
      final before = scene.document;
      final diff = scene.advance(1 / 60);

      expect(diff.isEmpty, isFalse);
      expect(diff.inverse.applyTo(scene.document).encode(), before.encode());
    });

    test('nothing that cannot move is ever written', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          crate(
            id: 'shelf',
            at: Vector3(0, 2, 0),
            rotation: Vector3(10, 37, 5),
            body: BodyComponent(motion: BodyMotion.fixed),
          ),
        ]),
      );

      expect(run(scene, 1).every((diff) => diff.isEmpty), isTrue);
    });

    test('less than a step owed is no step and no diff', () {
      scene = ScenePhysics(sceneOf([floor(), crate()]));

      expect(scene.advance(0.001).isEmpty, isTrue);
      expect(transformOf(scene.document, 'crate').position.y, 3);
    });

    test('a late frame catches up only so far', () {
      scene = ScenePhysics(sceneOf([floor(), crate()]), maxSteps: 2);
      scene.advance(10);

      // Two sixtieths of falling from rest is a few millimetres, not a crate
      // that has already landed.
      final y = transformOf(scene.document, 'crate').position.y;
      expect(y, lessThan(3));
      expect(y, greaterThan(2.9));
    });
  });

  group('events', () {
    /// What [scene] heard over [frames] advances of [seconds] each, as
    /// entity ids so two runs can be compared.
    List<(PhysicsEventKind, String?, String?)> heard(
      ScenePhysics scene,
      int frames,
      double seconds,
    ) => [
      for (var i = 0; i < frames; i++)
        for (final event in (scene..advance(seconds)).events)
          (event.kind, scene.entityOf(event.a), scene.entityOf(event.b)),
    ];

    test('a slow frame hears every step it took, not only its last', () {
      // A sixty-fourth, so three of them add up exactly and both runs take
      // the same steps.
      const step = 1 / 64;
      final smooth = ScenePhysics(
        sceneOf([floor(), crate(at: Vector3(0, 1, 0))]),
        step: step,
      );
      scene = ScenePhysics(
        sceneOf([floor(), crate(at: Vector3(0, 1, 0))]),
        step: step,
      );

      final everyStep = heard(smooth, 192, step);
      final everyThird = heard(scene, 64, 3 * step);
      smooth.dispose();

      expect(
        everyStep,
        contains((PhysicsEventKind.touchBegan, 'floor', 'crate')),
      );
      expect(everyThird, everyStep);
    });

    test('a frame that took no step hears nothing', () {
      scene = ScenePhysics(sceneOf([floor(), crate(at: Vector3(0, 1, 0))]));
      for (var i = 0; i < 600 && scene.events.isEmpty; i++) {
        scene.advance(1 / 60);
      }
      expect(scene.events, isNotEmpty);

      scene.advance(0.001);
      expect(scene.events, isEmpty);
    });
  });

  group('shapes and sizes', () {
    test('a crate scaled to two is a body twice the size', () {
      scene = ScenePhysics(sceneOf([floor(), crate(scale: Vector3.all(2))]));
      run(scene, 3);

      final rested = transformOf(scene.document, 'crate');
      expect(rested.position.y, closeTo(1, resting));
      expect(rested.scale, Vector3.all(2));
    });

    test('a ball stretched upright grows rather than stretches', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          crate(
            scale: Vector3(1, 3, 1),
            body: BodyComponent(shape: BodyShape.sphere),
          ),
        ]),
      );
      run(scene, 3);

      expect(
        transformOf(scene.document, 'crate').position.y,
        closeTo(1.5, resting),
      );
    });

    test(
      'a capsule round the waist leaves its entity standing on the floor',
      () {
        scene = ScenePhysics(
          sceneOf([
            floor(),
            crate(
              id: 'hero',
              at: Vector3(0, 2, 0),
              body: BodyComponent(
                shape: BodyShape.capsule,
                radius: 0.5,
                height: 2,
                centre: Vector3(0, 1, 0),
                angularDamping: 1,
              ),
            ),
          ]),
        );
        run(scene, 3);

        expect(
          transformOf(scene.document, 'hero').position.y,
          closeTo(0, resting),
        );
      },
    );

    test('a raised plane is raised ground', () {
      scene = ScenePhysics(sceneOf([floor(height: 1), crate()]));
      run(scene, 3);

      expect(
        transformOf(scene.document, 'crate').position.y,
        closeTo(1.5, resting),
      );
    });

    test('a crate turned about its upright stays turned', () {
      scene = ScenePhysics(
        sceneOf([floor(), crate(rotation: Vector3(0, 30, 0))]),
      );
      run(scene, 3);

      final rested = transformOf(scene.document, 'crate');
      expect(rested.rotation.y, closeTo(30, 0.5));
      expect(rested.rotation.x, closeTo(0, 0.5));
      expect(rested.rotation.z, closeTo(0, 0.5));
    });

    test('a crate turned a quarter keeps the angles it was given', () {
      scene = ScenePhysics(
        sceneOf([floor(), crate(rotation: Vector3(0, 90, 0))]),
      );
      run(scene, 3);

      final rested = transformOf(scene.document, 'crate');
      expect(rested.rotation.x, closeTo(0, 0.5));
      expect(rested.rotation.y, closeTo(90, 0.5));
      expect(rested.rotation.z, closeTo(0, 0.5));
    });
  });

  group('parents', () {
    SceneEntity cart() => SceneEntity(
      id: 'cart',
      name: 'Cart',
      components: {
        SceneComponents.transform: TransformComponent(
          position: Vector3(10, 0, 0),
          rotation: Vector3(0, 90, 0),
        ),
      },
    );

    test('a body under a parent is written relative to it', () {
      scene = ScenePhysics(
        sceneOf([floor(), cart(), crate(parent: 'cart', at: Vector3(0, 3, 0))]),
      );
      run(scene, 3);

      final rested = transformOf(scene.document, 'crate');
      expect(rested.position.x, closeTo(0, 0.01));
      expect(rested.position.y, closeTo(0.5, resting));
      expect(rested.position.z, closeTo(0, 0.01));
      expect(rested.rotation.y, closeTo(0, 0.5));
    });

    test('a body under a falling body is carried and rewritten', () {
      // The rider is fixed, so the world never moves it; but what it is
      // relative to falls, and to stay where the world has it its transform
      // has to change.
      scene = ScenePhysics(
        sceneOf([
          floor(),
          crate(),
          crate(
            id: 'rider',
            parent: 'crate',
            at: Vector3(0, 5, 0),
            body: BodyComponent(motion: BodyMotion.fixed),
          ),
        ]),
      );
      run(scene, 3);

      final rider = transformOf(scene.document, 'rider');
      final crateAt = transformOf(scene.document, 'crate').position;
      expect(rider.position.y + crateAt.y, closeTo(8, resting));
    });
  });

  group('edits', () {
    test('moving an entity moves its body and keeps its number', () {
      final start = sceneOf([floor(), crate()]);
      scene = ScenePhysics(start);
      final number = scene.bodyOf('crate');
      run(scene, 3);

      final lifted = TransformComponent(position: Vector3(4, 6, 0));
      scene.apply(
        SceneDiff([
          SetComponent(
            'crate',
            SceneComponents.transform,
            from: transformOf(scene.document, 'crate').toJson(),
            to: lifted.toJson(),
          ),
        ]),
      );

      expect(scene.bodyOf('crate'), number);
      expect(scene.physics.transformOf(number!)![1], closeTo(6, 1e-6));

      run(scene, 3);
      final rested = transformOf(scene.document, 'crate');
      expect(rested.position.x, closeTo(4, 0.01));
      expect(rested.position.y, closeTo(0.5, resting));
    });

    test('a body taken away leaves the world, and never comes back as the '
        'same number', () {
      scene = ScenePhysics(sceneOf([floor(), crate()]));
      final number = scene.bodyOf('crate')!;
      final body = scene.document['crate']![SceneComponents.body]!;

      scene.apply(
        SceneDiff([
          SetComponent(
            'crate',
            SceneComponents.body,
            from: body.toJson(),
            to: null,
          ),
        ]),
      );
      expect(scene.bodyOf('crate'), isNull);
      expect(scene.entityOf(number), isNull);
      expect(scene.physics.transformOf(number), isNull);
      expect(run(scene, 1).every((diff) => diff.isEmpty), isTrue);

      scene.apply(
        SceneDiff([
          SetComponent(
            'crate',
            SceneComponents.body,
            from: null,
            to: body.toJson(),
          ),
        ]),
      );
      expect(scene.bodyOf('crate'), isNot(number));
      expect(scene.entityOf(scene.bodyOf('crate')!), 'crate');
    });

    test('a removed entity takes its body with it', () {
      scene = ScenePhysics(sceneOf([floor(), crate()]));
      final number = scene.bodyOf('crate')!;
      final entity = scene.document['crate']!;

      scene.apply(SceneDiff([RemoveEntity(entity.toJson())]));

      expect(scene.bodyOf('crate'), isNull);
      expect(scene.physics.transformOf(number), isNull);
    });

    test('an added entity with a body is simulated', () {
      scene = ScenePhysics(sceneOf([floor()]));
      scene.apply(SceneDiff([AddEntity(crate().toJson())]));

      expect(scene.bodyOf('crate'), isNotNull);
      run(scene, 3);
      expect(
        transformOf(scene.document, 'crate').position.y,
        closeTo(0.5, resting),
      );
    });

    test('moving a parent moves the bodies under it', () {
      final folder = SceneEntity(
        id: 'folder',
        name: 'Folder',
        components: {SceneComponents.transform: TransformComponent()},
      );
      scene = ScenePhysics(sceneOf([floor(), folder, crate(parent: 'folder')]));
      final number = scene.bodyOf('crate')!;

      scene.apply(
        SceneDiff([
          SetField(
            'folder',
            SceneComponents.transform,
            'position',
            from: Values.vectorToJson(Vector3.zero()),
            to: Values.vectorToJson(Vector3(7, 0, 0)),
          ),
        ]),
      );

      expect(scene.physics.transformOf(number)![0], closeTo(7, 1e-6));
    });
  });
}
