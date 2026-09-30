import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

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

SceneEntity crate(String id, Vector3 at) => SceneEntity(
  id: id,
  name: id,
  components: {
    SceneComponents.transform: TransformComponent(position: at),
    SceneComponents.body: BodyComponent(),
  },
);

/// An entity that is only a weld, held to the crate it is under.
SceneEntity weld(String parent, double breakingForce, {String id = 'weld'}) =>
    SceneEntity(
      id: id,
      name: id,
      parent: parent,
      components: {
        SceneComponents.transform: TransformComponent(),
        SceneComponents.joint: JointComponent(
          kind: JointKind.fixed,
          breakingForce: breakingForce,
        ),
      },
    );

/// Takes the breaking force off the weld entity. Editing a broken joint's own
/// entity is how it is asked for back.
const mend = SceneDiff([
  SetField('weld', SceneComponents.joint, 'breakingForce', from: 5, to: 0),
]);

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

double heightOf(ScenePhysics scene, String id) =>
    (scene.document[id]![SceneComponents.transform]! as TransformComponent)
        .position
        .y;

void run(ScenePhysics scene, double seconds) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
  }
}

/// Everything a replay has to reproduce: the document, how much of it the
/// advance wrote, which body each entity is, and what happened.
String record(ScenePhysics scene, SceneDiff diff) {
  final numbers = [
    for (final id in ['a', 'b', 'late']) '$id ${scene.bodyOf(id)}',
  ];
  final events = [for (final e in scene.events) '${e.kind.name} ${e.a} ${e.b}'];
  return [
    scene.document.encode(),
    diff.operations.length,
    ...numbers,
    ...events,
  ].join('\n');
}

/// Frames [from] up to [to] of a script that keeps editing the scene: an
/// entity arrives at frame 35 and another leaves at frame 50. Edits are keyed
/// on the frame, so a replay does what the first run did.
List<String> play(ScenePhysics scene, int from, int to) {
  final frames = <String>[];
  for (var frame = from; frame < to; frame++) {
    if (frame == 35) {
      scene.apply(
        SceneDiff([AddEntity(crate('late', Vector3(0.2, 2, 0)).toJson())]),
      );
    }
    if (frame == 50) {
      scene.apply(SceneDiff([RemoveEntity(scene.document['b']!.toJson())]));
    }
    frames.add(record(scene, scene.advance(1 / 60)));
  }
  return frames;
}

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  ScenePhysics stacked() => scene = ScenePhysics(
    sceneOf([
      floor(),
      crate('a', Vector3(0, 3, 0)),
      crate('b', Vector3(0.3, 5, 0)),
    ]),
  );

  ScenePhysics welded() => scene = ScenePhysics(
    sceneOf([crate('a', Vector3(0, 3, 0)), weld('a', 5)]),
  );

  /// One crate on the floor and one dropped on it from above.
  ScenePhysics aboveAnother() => scene = ScenePhysics(
    sceneOf([
      floor(),
      crate('low', Vector3(0, 0.5, 0)),
      crate('high', Vector3(0, 3, 0)),
    ]),
  );

  group('a snapshot of a scene', () {
    test('restores to the frame it was taken at, and replays to the same '
        'frames', () {
      stacked();
      play(scene, 0, 30);
      final snapshot = scene.snapshot();
      final atThirty = scene.document.encode();
      final first = play(scene, 30, 90);

      scene.restore(snapshot);
      expect(scene.document.encode(), atThirty);
      expect(play(scene, 30, 90), first);

      // A snapshot is not used up, and what the replays did to the scene did
      // not leak into it.
      scene.restore(snapshot);
      expect(play(scene, 30, 90), first);
      snapshot.dispose();
    });

    test('hands the same body numbers out again', () {
      stacked();
      final snapshot = scene.snapshot();
      play(scene, 0, 40);
      final late = scene.bodyOf('late');
      expect(late, isNotNull);

      scene.restore(snapshot);
      expect(scene.bodyOf('late'), isNull);
      play(scene, 0, 40);
      expect(scene.bodyOf('late'), late);
      snapshot.dispose();
    });

    test('answers with the diff that takes the document back', () {
      stacked();
      play(scene, 0, 20);
      final snapshot = scene.snapshot();
      final atTwenty = scene.document.encode();
      play(scene, 20, 60);
      final before = scene.document;

      final diff = scene.restore(snapshot);

      expect(diff.isEmpty, isFalse);
      expect(diff.applyTo(before).encode(), atTwenty);
      expect(scene.document.encode(), atTwenty);
      snapshot.dispose();
    });

    test('undoes an edit, removals included', () {
      stacked();
      play(scene, 0, 10);
      final snapshot = scene.snapshot();
      scene.apply(SceneDiff([RemoveEntity(scene.document['a']!.toJson())]));
      expect(scene.bodyOf('a'), isNull);

      scene.restore(snapshot);

      final body = scene.bodyOf('a');
      expect(body, isNotNull);
      expect(scene.physics.transformOf(body!), isNotNull);
      expect(scene.entityOf(body), 'a');
      snapshot.dispose();
    });

    test('keeps the time a frame still owes the next', () {
      stacked();
      scene.advance(0.01);
      final snapshot = scene.snapshot();
      scene.advance(0.01);

      scene.restore(snapshot);

      // A step is 0.0167 s. The ten milliseconds owed and these seven add up
      // to one, and the seven alone would not.
      expect(scene.advance(0.0067).isEmpty, isFalse);
      snapshot.dispose();
    });

    test('is not disturbed by where a body went after it', () {
      scene = ScenePhysics(sceneOf([floor(), crate('a', Vector3(0, 3, 0))]));
      run(scene, 4);
      final snapshot = scene.snapshot();

      // The second round runs on what the first restore gave the scene. If
      // that were the snapshot's own copy, the run would change it.
      for (var round = 0; round < 2; round++) {
        scene.physics.push(scene.bodyOf('a')!, impulse: [5, 0, 0]);
        run(scene, 1);
        scene.restore(snapshot);
      }

      // The crate is asleep where the snapshot left it. Writing it again
      // would mean the snapshot had been moved along with the scene.
      expect(scene.advance(1 / 60).isEmpty, isTrue);
      snapshot.dispose();
    });
  });

  group('a snapshot of a scene with joints and pairs', () {
    test('mends a joint that broke after it, and lets it break again', () {
      welded();
      final number = scene.jointOf('weld')!;
      final snapshot = scene.snapshot();
      scene.advance(1 / 60);
      expect(scene.broken, {'weld'});
      expect(scene.physics.jointStateOf(number), isNull);

      scene.restore(snapshot);

      expect(scene.broken, isEmpty);
      expect(scene.physics.jointStateOf(number), isNotNull);
      scene.advance(1 / 60);
      expect(scene.broken, {'weld'});
      expect(
        scene.events.where((event) => event.kind == PhysicsEventKind.broke),
        hasLength(1),
      );
      snapshot.dispose();
    });

    test('keeps a joint broken that had broken before it', () {
      welded();
      final number = scene.jointOf('weld')!;
      scene.advance(1 / 60);
      final snapshot = scene.snapshot();
      scene.apply(mend);
      expect(scene.broken, isEmpty);

      scene.restore(snapshot);

      expect(scene.broken, {'weld'});
      expect(scene.physics.jointStateOf(number), isNull);
      snapshot.dispose();
    });

    test('brings back a joint taken out after it, and edits it as one', () {
      welded();
      final number = scene.jointOf('weld')!;
      final snapshot = scene.snapshot();
      scene.apply(SceneDiff([RemoveEntity(scene.document['weld']!.toJson())]));
      expect(scene.jointOf('weld'), isNull);

      scene.restore(snapshot);

      expect(scene.jointOf('weld'), number);
      expect(scene.entityOfJoint(number), 'weld');
      // Editing it takes the world's joint out before making it again. One
      // the scene had forgotten it held would stay as it was, and still break
      // at the old force.
      scene.apply(mend);
      scene.advance(1 / 60);
      expect(scene.broken, isEmpty);
      expect(scene.physics.jointStateOf(number), isNotNull);
      snapshot.dispose();
    });

    test('hands the same joint numbers out again', () {
      welded();
      final snapshot = scene.snapshot();
      final added = SceneDiff([AddEntity(weld('a', 0, id: 'second').toJson())]);
      scene.apply(added);
      final second = scene.jointOf('second');
      expect(second, isNotNull);

      scene.restore(snapshot);
      expect(scene.jointOf('second'), isNull);
      scene.apply(added);

      expect(scene.jointOf('second'), second);
      snapshot.dispose();
    });

    test('gives back the events of the advance it was taken after', () {
      welded();
      scene.advance(1 / 60);
      final snapshot = scene.snapshot();
      scene.advance(1 / 60);
      expect(scene.events, isEmpty);

      scene.restore(snapshot);

      expect(scene.events.map((event) => event.kind), [PhysicsEventKind.broke]);
      snapshot.dispose();
    });

    test('forgets a pair told to pass through each other after it', () {
      aboveAnother();
      final snapshot = scene.snapshot();
      scene.ignore('low', 'high');

      scene.restore(snapshot);

      expect(scene.unignore('low', 'high'), isFalse);
      run(scene, 2);
      expect(heightOf(scene, 'high'), closeTo(1.5, 0.05));
      snapshot.dispose();
    });

    test('keeps a pair told to pass through each other before it', () {
      aboveAnother();
      scene.ignore('low', 'high');
      final snapshot = scene.snapshot();
      scene.unignore('low', 'high');

      scene.restore(snapshot);

      run(scene, 2);
      expect(heightOf(scene, 'high'), closeTo(0.5, 0.05));
      expect(scene.unignore('low', 'high'), isTrue);
      snapshot.dispose();
    });
  });

  group('a snapshot that has been let go of', () {
    test('cannot be restored, and the scene is as it was', () {
      stacked();
      final snapshot = scene.snapshot();
      play(scene, 0, 20);
      final before = record(scene, SceneDiff.none);
      snapshot.dispose();

      expect(snapshot.disposed, isTrue);
      expect(() => scene.restore(snapshot), throwsStateError);
      expect(record(scene, SceneDiff.none), before);
    });

    test('can be let go of twice', () {
      stacked();
      final snapshot = scene.snapshot()..dispose();
      expect(snapshot.dispose, returnsNormally);
    });
  });
}
