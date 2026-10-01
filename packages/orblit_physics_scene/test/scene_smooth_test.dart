import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// A 144 Hz render over 60 Hz physics.
const frame = 1 / 144;
const step = 1 / 60;

SceneEntity floor() => SceneEntity(
  id: 'floor',
  name: 'Floor',
  components: {
    SceneComponents.transform: TransformComponent(),
    SceneComponents.body: BodyComponent(
      shape: BodyShape.plane,
      motion: BodyMotion.fixed,
    ),
  },
);

/// A crate that falls from [at].
SceneEntity crate(String id, Vector3 at, {String? parent}) => SceneEntity(
  id: id,
  name: id,
  parent: parent,
  components: {
    SceneComponents.transform: TransformComponent(position: at),
    SceneComponents.body: BodyComponent(),
  },
);

/// An entity that is only a place in the tree, with no body of its own.
SceneEntity folder(String id) => SceneEntity(
  id: id,
  name: id,
  components: {SceneComponents.transform: TransformComponent()},
);

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

TransformComponent transformOf(SceneDocument document, String id) =>
    document[id]![SceneComponents.transform]! as TransformComponent;

double heightOf(ScenePhysics scene, String id) =>
    transformOf(scene.document, id).position.y;

double sidewaysOf(ScenePhysics scene, String id) =>
    transformOf(scene.document, id).position.x;

/// Where the world has the entity's body, which is not where the document
/// shows it when the scene smooths.
double truthOf(ScenePhysics scene, String id) =>
    scene.physics.transformOf(scene.bodyOf(id)!)![1];

SceneDiff teleport(ScenePhysics scene, String id, Vector3 to) => SceneDiff([
  SetComponent(
    id,
    SceneComponents.transform,
    from: transformOf(scene.document, id).toJson(),
    to: TransformComponent(position: to).toJson(),
  ),
]);

void frames(ScenePhysics scene, int count) {
  for (var i = 0; i < count; i++) {
    scene.advance(frame);
  }
}

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  /// A second world over the same document, kept as the first would be
  /// without smoothing.
  ScenePhysics twin(SceneDocument document) {
    final world = ScenePhysics(document);
    addTearDown(world.dispose);
    return world;
  }

  final falling = sceneOf([floor(), crate('crate', Vector3(0, 60, 0))]);
  final pair = sceneOf([
    floor(),
    crate('a', Vector3(0, 60, 0)),
    crate('b', Vector3(5, 60, 0)),
  ]);

  group('a scene that smooths', () {
    test('moves a body on every frame of a 144 Hz render', () {
      scene = ScenePhysics(falling, smooth: true);

      final shown = [heightOf(scene, 'crate')];
      for (var i = 0; i < 288; i++) {
        scene.advance(frame);
        shown.add(heightOf(scene, 'crate'));
      }

      // The first step ends on frame three. Until it has, there is nothing to
      // blend towards.
      for (var i = 3; i < shown.length; i++) {
        expect(shown[i], lessThan(shown[i - 1]), reason: 'frame $i');
      }
    });

    test('says so in the diff on a frame that took no step', () {
      scene = ScenePhysics(falling, smooth: true);
      frames(scene, 3);

      for (var i = 0; i < 100; i++) {
        expect(scene.advance(frame).isEmpty, isFalse, reason: 'frame $i');
      }
    });

    test('shows a body a step behind the world, by the time owed', () {
      scene = ScenePhysics(falling, smooth: true);
      final world = twin(falling);
      final truth = [heightOf(world, 'crate')];
      for (var k = 0; k < 130; k++) {
        world.advance(step);
        truth.add(heightOf(world, 'crate'));
      }

      for (var n = 1; n <= 288; n++) {
        scene.advance(frame);

        // How many steps ago the render is showing.
        final behind = n * frame / step - 1;
        final shown = heightOf(scene, 'crate');
        if (behind < 0) {
          expect(shown, closeTo(truth[0], 1e-4), reason: 'frame $n');
          continue;
        }
        final k = behind.floor();
        final expected = truth[k] + (truth[k + 1] - truth[k]) * (behind - k);
        expect(shown, closeTo(expected, 1e-4), reason: 'frame $n');
      }
    });

    test('leaves the world exactly as it would have been', () {
      scene = ScenePhysics(falling, smooth: true);
      final world = twin(falling);

      // 41 whole steps in a hundred frames, clear of a step landing on a frame.
      frames(scene, 100);
      for (var k = 0; k < 41; k++) {
        world.advance(step);
      }

      expect(
        scene.physics.transformOf(scene.bodyOf('crate')!),
        world.physics.transformOf(world.bodyOf('crate')!),
      );
    });

    test('keeps the world ahead of what is shown', () {
      scene = ScenePhysics(falling, smooth: true);
      frames(scene, 100);

      expect(truthOf(scene, 'crate'), lessThan(heightOf(scene, 'crate')));
    });

    test('does not move a body at rest', () {
      scene = ScenePhysics(
        sceneOf([floor(), crate('crate', Vector3(0, 0.5, 0))]),
        smooth: true,
      );
      frames(scene, 576);
      final settled = scene.document.encode();

      for (var i = 0; i < 288; i++) {
        expect(scene.advance(frame).isEmpty, isTrue, reason: 'frame $i');
      }
      expect(scene.document.encode(), settled);
    });

    test('shows a late frame a step behind', () {
      scene = ScenePhysics(
        sceneOf([floor(), crate('crate', Vector3(0, 100, 0))]),
        smooth: true,
      );

      scene.advance(10);

      expect(heightOf(scene, 'crate'), lessThan(100));
      expect(heightOf(scene, 'crate'), greaterThan(truthOf(scene, 'crate')));
    });

    test('shows a body under a body where its own blend puts it', () {
      final nested = sceneOf([
        floor(),
        crate('parent', Vector3(0, 60, 0)),
        crate('child', Vector3(3, 20, 0), parent: 'parent'),
      ]);
      scene = ScenePhysics(nested, smooth: true);
      final world = twin(nested);
      final truth = [heightOf(world, 'child') + heightOf(world, 'parent')];
      for (var k = 0; k < 60; k++) {
        world.advance(step);
        truth.add(heightOf(world, 'child') + heightOf(world, 'parent'));
      }

      // No step has ended yet, so the frames before frame three show the start.
      frames(scene, 2);
      for (var n = 3; n <= 100; n++) {
        scene.advance(frame);
        final behind = n * frame / step - 1;
        final k = behind.floor();
        final expected = truth[k] + (truth[k + 1] - truth[k]) * (behind - k);
        // The child's transform is under the parent's, so what the document
        // makes of it in the world is the sum.
        final shown = heightOf(scene, 'child') + heightOf(scene, 'parent');
        expect(shown, closeTo(expected, 1e-3), reason: 'frame $n');
      }
    });
  });

  group('a scene that does not smooth', () {
    test('stands still between steps, as it did', () {
      scene = ScenePhysics(falling);

      var still = 0;
      for (var i = 0; i < 288; i++) {
        if (scene.advance(frame).isEmpty) still++;
      }

      // 120 steps in 288 frames leaves at least 168 with nothing to show.
      expect(still, greaterThanOrEqualTo(168));
    });

    test('does nothing with the calls about smoothing', () {
      scene = ScenePhysics(falling);

      expect(scene.stopSmoothing('crate'), isTrue);
      expect(scene.resetSmoothing('crate'), isTrue);
      expect(scene.startSmoothing('crate'), isTrue);
      scene.advance(step);
      expect(heightOf(scene, 'crate'), lessThan(60));
    });
  });

  group('a teleport', () {
    test('by an edit does not streak', () {
      scene = ScenePhysics(falling, smooth: true);
      frames(scene, 30);

      scene.apply(teleport(scene, 'crate', Vector3(100, 60, 0)));
      expect(sidewaysOf(scene, 'crate'), 100);

      for (var i = 0; i < 20; i++) {
        scene.advance(frame);
        expect(sidewaysOf(scene, 'crate'), closeTo(100, 1e-3), reason: '$i');
      }
    });

    test('through the world does not streak once it is reset', () {
      scene = ScenePhysics(falling, smooth: true);
      frames(scene, 30);

      scene.physics.place(scene.bodyOf('crate')!, at: [100, 60, 0]);
      expect(scene.resetSmoothing('crate'), isTrue);

      for (var i = 0; i < 20; i++) {
        scene.advance(frame);
        expect(sidewaysOf(scene, 'crate'), closeTo(100, 1e-3), reason: '$i');
      }
    });

    test('through the world slides there until it is reset', () {
      scene = ScenePhysics(falling, smooth: true);
      frames(scene, 30);

      scene.physics.place(scene.bodyOf('crate')!, at: [100, 60, 0]);

      final seen = <double>[];
      for (var i = 0; i < 6; i++) {
        scene.advance(frame);
        seen.add(sidewaysOf(scene, 'crate'));
      }
      expect(seen.any((x) => x > 1 && x < 99), isTrue, reason: '$seen');
    });

    test('of one body leaves the blend of the others alone', () {
      scene = ScenePhysics(pair, smooth: true);
      final other = ScenePhysics(pair, smooth: true);
      addTearDown(other.dispose);
      frames(scene, 30);
      frames(other, 30);

      scene.physics.place(scene.bodyOf('a')!, at: [100, 60, 0]);
      scene.resetSmoothing('a');
      for (var i = 0; i < 30; i++) {
        scene.advance(frame);
        other.advance(frame);
        expect(heightOf(scene, 'b'), heightOf(other, 'b'), reason: '$i');
      }
    });

    test('is not reset for an entity the document does not have', () {
      scene = ScenePhysics(falling, smooth: true);

      expect(scene.resetSmoothing('nothing'), isFalse);
    });
  });

  group('stopSmoothing', () {
    test('shows a body as the world has it, from that moment', () {
      scene = ScenePhysics(pair, smooth: true);
      frames(scene, 20);
      expect(heightOf(scene, 'a'), greaterThan(truthOf(scene, 'a')));

      expect(scene.stopSmoothing('a'), isTrue);
      scene.advance(0);
      expect(heightOf(scene, 'a'), closeTo(truthOf(scene, 'a'), 1e-4));

      var behind = 0;
      for (var i = 0; i < 100; i++) {
        scene.advance(frame);
        expect(heightOf(scene, 'a'), closeTo(truthOf(scene, 'a'), 1e-4));
        if (heightOf(scene, 'b') > truthOf(scene, 'b') + 1e-4) behind++;
      }
      // The body beside it is still blended.
      expect(behind, greaterThan(50));
    });

    test('takes everything under the entity with it', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          folder('group'),
          crate('inside', Vector3(0, 60, 0), parent: 'group'),
          crate('outside', Vector3(5, 60, 0)),
        ]),
        smooth: true,
      );
      frames(scene, 20);

      expect(scene.stopSmoothing('group'), isTrue);
      frames(scene, 20);

      expect(
        heightOf(scene, 'inside'),
        closeTo(truthOf(scene, 'inside'), 1e-4),
      );
      expect(
        heightOf(scene, 'outside'),
        greaterThan(truthOf(scene, 'outside') + 1e-4),
      );
    });

    test('is undone by startSmoothing, once, without a jump', () {
      scene = ScenePhysics(pair, smooth: true);
      scene.stopSmoothing('a');
      frames(scene, 30);

      expect(scene.startSmoothing('a'), isTrue);
      expect(scene.startSmoothing('a'), isFalse);

      var last = heightOf(scene, 'a');
      var behind = 0;
      for (var i = 0; i < 30; i++) {
        scene.advance(frame);
        final shown = heightOf(scene, 'a');
        expect(shown, lessThanOrEqualTo(last), reason: 'frame $i');
        last = shown;
        if (shown > truthOf(scene, 'a') + 1e-4) behind++;
      }
      expect(behind, greaterThan(10));
    });

    test('says no to an entity the document does not have', () {
      scene = ScenePhysics(pair, smooth: true);

      expect(scene.stopSmoothing('nothing'), isFalse);
      expect(scene.startSmoothing('a'), isFalse);
    });

    test('ends with the entity', () {
      scene = ScenePhysics(pair, smooth: true);
      scene.stopSmoothing('a');
      final entity = scene.document['a']!;

      scene.apply(SceneDiff([RemoveEntity(entity.toJson())]));
      scene.apply(SceneDiff([AddEntity(entity.toJson())]));

      expect(scene.startSmoothing('a'), isFalse);
    });
  });

  group('a snapshot of a scene that smooths', () {
    List<String> replay(ScenePhysics scene) => [
      for (var i = 0; i < 60; i++) (scene..advance(frame)).document.encode(),
    ];

    test('replays the same shown poses, however often it is restored', () {
      scene = ScenePhysics(pair, smooth: true);
      // After 41 frames the next call takes no step, so the first frame
      // replayed is a blend that reads where the bodies were a step ago. A
      // snapshot that shared that with the scene would show it moved by the
      // first replay.
      frames(scene, 41);
      final snapshot = scene.snapshot();
      addTearDown(snapshot.dispose);

      final first = replay(scene);
      scene.restore(snapshot);
      expect(replay(scene), first);
      scene.restore(snapshot);
      expect(replay(scene), first);
    });

    test('goes back to the poses it was taken at', () {
      scene = ScenePhysics(pair, smooth: true);
      frames(scene, 40);
      final shown = scene.document.encode();
      final snapshot = scene.snapshot();
      addTearDown(snapshot.dispose);

      frames(scene, 40);
      scene.restore(snapshot);

      expect(scene.document.encode(), shown);
    });

    test('keeps what was switched off, and what was not', () {
      scene = ScenePhysics(pair, smooth: true);
      scene.stopSmoothing('a');
      final stopped = scene.snapshot();
      addTearDown(stopped.dispose);
      scene.startSmoothing('a');
      final blended = scene.snapshot();
      addTearDown(blended.dispose);

      scene.restore(stopped);
      expect(scene.startSmoothing('a'), isTrue);
      scene.stopSmoothing('a');
      scene.restore(blended);
      expect(scene.startSmoothing('a'), isFalse);
    });

    test('does not go into a scene that smooths differently', () {
      scene = ScenePhysics(pair, smooth: true);
      final plain = ScenePhysics(pair);
      addTearDown(plain.dispose);
      final smoothed = scene.snapshot();
      addTearDown(smoothed.dispose);
      final unsmoothed = plain.snapshot();
      addTearDown(unsmoothed.dispose);
      frames(plain, 10);
      final before = plain.document.encode();

      expect(() => plain.restore(smoothed), throwsArgumentError);
      expect(() => scene.restore(unsmoothed), throwsArgumentError);
      expect(plain.document.encode(), before);
    });
  });
}
