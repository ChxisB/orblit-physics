import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

SceneEntity entityOf(
  String id, {
  required Vector3 at,
  required BodyComponent body,
  ZoneComponent? zone,
}) => SceneEntity(
  id: id,
  name: id,
  components: {
    SceneComponents.transform: TransformComponent(position: at),
    SceneComponents.body: body,
    SceneComponents.zone: ?zone,
  },
);

/// Ground through the origin, facing up.
SceneEntity floor() => entityOf(
  'floor',
  at: Vector3.zero(),
  body: BodyComponent(shape: BodyShape.plane, motion: BodyMotion.fixed),
);

/// A one-metre crate at [at].
SceneEntity crate(Vector3 at, {BodyComponent? body}) =>
    entityOf('crate', at: at, body: body ?? BodyComponent());

/// A fixed box [size] across: a place, or a wall when it is not a trigger.
BodyComponent place(Vector3 size, {bool trigger = false, bool stay = false}) =>
    BodyComponent(
      size: size,
      motion: BodyMotion.fixed,
      trigger: trigger,
      stay: stay,
    );

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

Vector3 middleOf(ScenePhysics scene, String entity) {
  final pose = scene.physics.transformOf(scene.bodyOf(entity)!)!;
  return Vector3(pose[0], pose[1], pose[2]);
}

/// Runs [seconds] a sixtieth at a time, as a frame loop would, and answers
/// with everything that was heard.
List<PhysicsEvent> run(ScenePhysics scene, double seconds) {
  final seen = <PhysicsEvent>[];
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
    seen.addAll(scene.events);
  }
  return seen;
}

Iterable<PhysicsEvent> only(List<PhysicsEvent> seen, PhysicsEventKind kind) =>
    seen.where((event) => event.kind == kind);

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  group('a trigger in a document', () {
    test('lets a body fall through and says when it came and went', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'gate',
            at: Vector3(0, 2, 0),
            body: place(Vector3(3, 1, 3), trigger: true),
          ),
          crate(Vector3(0, 5, 0)),
        ]),
      );
      final seen = run(scene, 2);

      final entered = only(seen, PhysicsEventKind.entered).single;
      expect(scene.entityOf(entered.a), 'gate');
      expect(scene.entityOf(entered.b), 'crate');
      expect(only(seen, PhysicsEventKind.exited), hasLength(1));
      expect(only(seen, PhysicsEventKind.touchBegan), hasLength(1));
      expect(middleOf(scene, 'crate').y, closeTo(0.5, 0.05));
    });

    test('is a wall when it is not one', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'shelf',
            at: Vector3(0, 2, 0),
            body: place(Vector3(3, 1, 3)),
          ),
          crate(Vector3(0, 5, 0)),
        ]),
      );
      run(scene, 2);

      expect(middleOf(scene, 'crate').y, closeTo(3, 0.05));
    });

    test('is ignored for a body that falls', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          crate(Vector3(0, 3, 0), body: BodyComponent(trigger: true)),
        ]),
      );
      run(scene, 2);

      expect(middleOf(scene, 'crate').y, closeTo(0.5, 0.05));
    });

    test('hears a body inside it every step when it asks to', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'pool',
            at: Vector3(0, 1, 0),
            body: place(Vector3(4, 2, 4), trigger: true, stay: true),
          ),
          crate(
            Vector3(0, 1, 0),
            body: BodyComponent(motion: BodyMotion.driven),
          ),
        ]),
      );
      final seen = run(scene, 0.5);

      expect(only(seen, PhysicsEventKind.inside).length, greaterThan(20));
    });

    test('says nothing every step when it does not ask', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'pool',
            at: Vector3(0, 1, 0),
            body: place(Vector3(4, 2, 4), trigger: true),
          ),
          crate(
            Vector3(0, 1, 0),
            body: BodyComponent(motion: BodyMotion.driven),
          ),
        ]),
      );
      final seen = run(scene, 0.5);

      expect(only(seen, PhysicsEventKind.inside), isEmpty);
      expect(only(seen, PhysicsEventKind.entered), hasLength(1));
    });

    test('a body that asks hears its contact every step it is awake', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          crate(Vector3(0, 0.5, 0), body: BodyComponent(stay: true)),
        ]),
      );
      final seen = run(scene, 0.25);

      expect(only(seen, PhysicsEventKind.touchStay).length, greaterThan(10));
    });

    test('a body that does not ask hears no stay', () {
      scene = ScenePhysics(sceneOf([floor(), crate(Vector3(0, 0.5, 0))]));
      final seen = run(scene, 0.25);

      expect(only(seen, PhysicsEventKind.touchBegan), isNotEmpty);
      expect(only(seen, PhysicsEventKind.touchStay), isEmpty);
    });
  });

  group('a zone in a document', () {
    SceneDocument shaft({ZoneComponent? zone}) => sceneOf([
      entityOf(
        'shaft',
        at: Vector3(0, 3, 0),
        body: place(Vector3(4, 4, 4)),
        zone: zone,
      ),
      crate(Vector3(0, 2, 0)),
    ]);

    test('makes the body under it a trigger without being asked', () {
      scene = ScenePhysics(shaft(zone: ZoneComponent(gravity: Vector3.zero())));
      final seen = run(scene, 0.5);

      expect(only(seen, PhysicsEventKind.entered), hasLength(1));
      expect(only(seen, PhysicsEventKind.touchBegan), isEmpty);
    });

    test('holds a body where gravity is nought', () {
      scene = ScenePhysics(shaft(zone: ZoneComponent(gravity: Vector3.zero())));
      run(scene, 1);

      expect(middleOf(scene, 'crate').y, closeTo(2, 0.01));
    });

    test('lifts a body where gravity points up', () {
      scene = ScenePhysics(
        shaft(zone: ZoneComponent(gravity: Vector3(0, 20, 0))),
      );
      run(scene, 0.5);

      expect(middleOf(scene, 'crate').y, greaterThan(3));
    });

    test('does nothing to a body when it is not there', () {
      scene = ScenePhysics(shaft());
      run(scene, 0.5);

      expect(middleOf(scene, 'crate').y, lessThan(1.5));
    });

    test('lets go when the zone is taken off', () {
      final held = shaft(zone: ZoneComponent(gravity: Vector3.zero()));
      scene = ScenePhysics(held);
      run(scene, 1);
      scene.apply(SceneDiff.between(held, shaft()));
      run(scene, 0.5);

      expect(middleOf(scene, 'crate').y, lessThan(1.5));
    });

    test('follows an edit to the zone', () {
      final held = shaft(zone: ZoneComponent(gravity: Vector3.zero()));
      scene = ScenePhysics(held);
      run(scene, 1);
      scene.apply(
        SceneDiff.between(
          held,
          shaft(zone: ZoneComponent(gravity: Vector3(0, 20, 0))),
        ),
      );
      run(scene, 0.5);

      expect(middleOf(scene, 'crate').y, greaterThan(3));
    });

    test('slows a body with damping alone', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'water',
            at: Vector3(0, 3, 0),
            body: place(Vector3(4, 4, 4)),
            zone: ZoneComponent(linearDamping: 8),
          ),
          crate(Vector3(0, 4, 0)),
        ]),
      );
      run(scene, 0.5);
      final wet = middleOf(scene, 'crate').y;
      scene.dispose();

      scene = ScenePhysics(sceneOf([crate(Vector3(0, 4, 0))]));
      run(scene, 0.5);

      expect(wet, greaterThan(middleOf(scene, 'crate').y));
    });

    test('where two overlap, the higher priority wins', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'low',
            at: Vector3(0, 3, 0),
            body: place(Vector3(4, 4, 4)),
            zone: ZoneComponent(gravity: Vector3(0, 20, 0)),
          ),
          entityOf(
            'high',
            at: Vector3(0, 3, 0),
            body: place(Vector3(4, 4, 4)),
            zone: ZoneComponent(gravity: Vector3.zero(), priority: 1),
          ),
          crate(Vector3(0, 2, 0)),
        ]),
      );
      run(scene, 1);

      expect(middleOf(scene, 'crate').y, closeTo(2, 0.01));
    });
  });

  group('a belt in a document', () {
    SceneDocument belt(Vector3 surface) => sceneOf([
      entityOf(
        'belt',
        at: Vector3(0, -0.5, 0),
        body: place(Vector3(20, 1, 3)).copyWith(surface: surface),
      ),
      crate(Vector3(0, 0.6, 0)),
    ]);

    test('carries what stands on it', () {
      scene = ScenePhysics(belt(Vector3(2, 0, 0)));
      run(scene, 2);

      expect(middleOf(scene, 'crate').x, greaterThan(1.5));
    });

    test('carries it the other way when the surface goes the other way', () {
      scene = ScenePhysics(belt(Vector3(-2, 0, 0)));
      run(scene, 2);

      expect(middleOf(scene, 'crate').x, lessThan(-1.5));
    });

    test('leaves it where it is when the surface is still', () {
      scene = ScenePhysics(belt(Vector3.zero()));
      run(scene, 2);

      expect(middleOf(scene, 'crate').x, closeTo(0, 0.01));
    });
  });
}
