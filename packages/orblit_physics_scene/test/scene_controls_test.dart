import 'dart:math' as math;

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

/// Ground through the origin, facing up.
SceneEntity floor() => entityOf(
  'floor',
  at: Vector3.zero(),
  body: BodyComponent(shape: BodyShape.plane, motion: BodyMotion.fixed),
);

/// A one-metre crate [height] up.
SceneEntity crateAt(String id, double height) =>
    entityOf(id, at: Vector3(0, height, 0), body: BodyComponent());

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

Vector3 middleOf(ScenePhysics scene, String entity) {
  final pose = scene.physics.transformOf(scene.bodyOf(entity)!)!;
  return Vector3(pose[0], pose[1], pose[2]);
}

double speedOf(ScenePhysics scene, String entity) {
  final v = scene.physics.velocityOf(scene.bodyOf(entity)!)!;
  return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/// Runs [seconds] a sixtieth at a time, as a frame loop would.
void run(ScenePhysics scene, double seconds) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
  }
}

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  group('a body that says how it moves', () {
    test('locked to a plane, at half gravity, under a cap, behaves so', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'crate',
            at: Vector3(0, 20, 0),
            body: BodyComponent(
              locks: {BodyLock.moveZ},
              gravityScale: 0.5,
              maxSpeed: 3,
            ),
          ),
        ]),
      );
      scene.physics.push(scene.bodyOf('crate')!, impulse: const [2, 0, 2]);
      run(scene, 1);

      final at = middleOf(scene, 'crate');
      expect(at.z, closeTo(0, 1e-4));
      expect(at.x, greaterThan(0.5));
      expect(speedOf(scene, 'crate'), lessThanOrEqualTo(3.001));
      // Half gravity alone would fall 2.45 m in a second. The cap holds it
      // back further.
      expect(20 - at.y, allOf(greaterThan(0.8), lessThan(2.4)));
    });

    test('half gravity falls half as far', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf('plain', at: Vector3(0, 50, 0), body: BodyComponent()),
          entityOf(
            'light',
            at: Vector3(5, 50, 0),
            body: BodyComponent(gravityScale: 0.5),
          ),
        ]),
      );
      run(scene, 0.5);

      final plain = 50 - middleOf(scene, 'plain').y;
      final light = 50 - middleOf(scene, 'light').y;
      expect(light / plain, closeTo(0.5, 0.03));
    });

    test('a locked turn holds the body upright', () {
      scene = ScenePhysics(
        sceneOf([
          floor(),
          entityOf(
            'post',
            at: Vector3(0, 0.5, 0),
            body: BodyComponent(
              locks: {BodyLock.turnX, BodyLock.turnY, BodyLock.turnZ},
            ),
          ),
        ]),
      );
      scene.physics.push(
        scene.bodyOf('post')!,
        impulse: const [1, 0, 0],
        at: const [0, 1, 0],
      );
      run(scene, 1);

      final v = scene.physics.velocityOf(scene.bodyOf('post')!)!;
      expect(v[3].abs() + v[4].abs() + v[5].abs(), lessThan(1e-4));
    });

    test('added asleep, is asleep still with its controls', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'crate',
            at: Vector3(0, 5, 0),
            body: BodyComponent(startsAsleep: true, locks: {BodyLock.moveX}),
          ),
        ]),
      );
      run(scene, 0.5);

      expect(scene.physics.asleep(scene.bodyOf('crate')!), isTrue);
      expect(middleOf(scene, 'crate').y, closeTo(5, 1e-4));
    });

    test('a cap or inertia the world would refuse does not cost the locks', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'crate',
            at: Vector3(0, 20, 0),
            body: BodyComponent(
              locks: {BodyLock.moveZ},
              maxSpeed: -5,
              maxSpin: -1,
              inertia: Vector3(-1, 2, 3),
            ),
          ),
        ]),
      );
      scene.physics.push(scene.bodyOf('crate')!, impulse: const [0, 0, 4]);
      run(scene, 0.5);

      expect(middleOf(scene, 'crate').z, closeTo(0, 1e-4));
    });

    test('an edit sends the new controls with the rebuilt body', () {
      scene = ScenePhysics(sceneOf([crateAt('crate', 20)]));
      final locked = sceneOf([
        entityOf(
          'crate',
          at: Vector3(0, 20, 0),
          body: BodyComponent(locks: {BodyLock.moveX}),
        ),
      ]);
      scene.apply(SceneDiff.between(scene.document, locked));
      scene.physics.push(scene.bodyOf('crate')!, impulse: const [4, 0, 0]);
      run(scene, 0.5);

      expect(middleOf(scene, 'crate').x, closeTo(0, 1e-4));
    });
  });

  group('a body with its weight somewhere else', () {
    /// The spin about z one step after a shove along x through the middle of
    /// the shape, for a crate on an entity [size] across. Its weight sits a
    /// quarter of the crate's own height below the middle.
    double spinAfterShove(double size, {Vector3? inertia}) {
      final made = ScenePhysics(
        sceneOf([
          entityOf(
            'crate',
            at: Vector3(0, 10, 0),
            scale: Vector3.all(size),
            body: BodyComponent(
              centreOfMass: Vector3(0, -0.25, 0),
              inertia: inertia,
            ),
          ),
        ]),
      );
      try {
        made.physics.push(
          made.bodyOf('crate')!,
          impulse: const [1, 0, 0],
          at: const [0, 10, 0],
        );
        made.advance(made.step);
        return made.physics.velocityOf(made.bodyOf('crate')!)![5].abs();
      } finally {
        made.dispose();
      }
    }

    test('is where the document says, however its weight sits', () {
      scene = ScenePhysics(
        sceneOf([
          entityOf(
            'crate',
            at: Vector3(1, 5, 2),
            body: BodyComponent(centreOfMass: Vector3(0, -0.3, 0)),
          ),
        ]),
      );

      expect(middleOf(scene, 'crate'), Vector3(1, 5, 2));
    });

    test('grows its centre of mass with the entity', () {
      // A shove at the middle lands 0.25 m above the weight on a one-metre
      // crate: 0.25 / (1 / 6 + 0.25 ^ 2). On a two-metre one it lands 0.5 m
      // above: 0.5 / (4 / 6 + 0.5 ^ 2). Left unscaled, the second would be
      // 0.25 / (4 / 6 + 0.25 ^ 2) = 0.343.
      expect(spinAfterShove(1), closeTo(1.0909, 0.02));
      expect(spinAfterShove(2), closeTo(0.5455, 0.02));
    });

    test('turns by the inertia the document gives, all three parts', () {
      expect(
        spinAfterShove(1, inertia: Vector3(100, 100, 100)),
        closeTo(0.0025, 0.0005),
      );
    });

    test('keeps the shape\'s inertia when a part of the given one is zero', () {
      expect(
        spinAfterShove(1, inertia: Vector3(100, 0, 100)),
        closeTo(1.0909, 0.02),
      );
    });
  });

  group('two entities told to pass through each other', () {
    ScenePhysics stackOfTwo({double high = 3}) => scene = ScenePhysics(
      sceneOf([floor(), crateAt('low', 0.5), crateAt('high', high)]),
    );

    test('do', () {
      stackOfTwo();
      expect(scene.ignore('low', 'high'), isTrue);
      run(scene, 2);

      expect(middleOf(scene, 'high').y, closeTo(0.5, 0.05));
    });

    test('stack once they are let go', () {
      stackOfTwo();
      scene.ignore('low', 'high');

      expect(scene.unignore('high', 'low'), isTrue);
      run(scene, 2);

      expect(middleOf(scene, 'high').y, closeTo(1.5, 0.05));
    });

    test('are one pair whichever way round they are named', () {
      stackOfTwo();
      scene.ignore('high', 'low');

      expect(scene.unignore('low', 'high'), isTrue);
      expect(scene.unignore('low', 'high'), isFalse);
    });

    test('are refused for a body that is not there or one against itself', () {
      stackOfTwo();

      expect(scene.ignore('low', 'nobody'), isFalse);
      expect(scene.ignore('low', 'low'), isFalse);
      expect(scene.unignore('low', 'high'), isFalse);
    });

    test('still do after an edit rebuilds one of them', () {
      stackOfTwo();
      scene.ignore('low', 'high');
      final raised = sceneOf([
        floor(),
        crateAt('low', 0.5),
        crateAt('high', 6),
      ]);
      scene.apply(SceneDiff.between(scene.document, raised));
      run(scene, 2);

      expect(middleOf(scene, 'high').y, closeTo(0.5, 0.05));
    });

    test('are forgotten with an entity that goes', () {
      stackOfTwo();
      scene.ignore('low', 'high');
      final alone = sceneOf([floor(), crateAt('low', 0.5)]);
      scene.apply(SceneDiff.between(scene.document, alone));
      final again = sceneOf([floor(), crateAt('low', 0.5), crateAt('high', 3)]);
      scene.apply(SceneDiff.between(scene.document, again));

      expect(scene.unignore('low', 'high'), isFalse);
      run(scene, 2);
      expect(middleOf(scene, 'high').y, closeTo(1.5, 0.05));
    });
  });
}
