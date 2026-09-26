import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

/// A body, free unless told otherwise.
SceneEntity body(
  String id, {
  String? parent,
  required Vector3 at,
  Vector3? rotation,
  BodyComponent? shape,
  JointComponent? joint,
}) => SceneEntity(
  id: id,
  name: id,
  parent: parent,
  components: {
    SceneComponents.transform: TransformComponent(
      position: at,
      rotation: rotation,
    ),
    SceneComponents.body: shape ?? BodyComponent(),
    SceneComponents.joint: ?joint,
  },
);

/// An entity that is only a joint, placed [at] under [parent].
SceneEntity joint(
  String id,
  JointComponent joint, {
  String? parent,
  Vector3? at,
  Vector3? rotation,
}) => SceneEntity(
  id: id,
  name: id,
  parent: parent,
  components: {
    SceneComponents.transform: TransformComponent(
      position: at,
      rotation: rotation,
    ),
    SceneComponents.joint: joint,
  },
);

/// A bar a metre long along x, light across.
BodyComponent bar() => BodyComponent(size: Vector3(1, 0.1, 0.1));

SceneDocument sceneOf(List<SceneEntity> entities) =>
    SceneDocument(name: 'Scene', entities: entities);

/// Where the world has [entity]'s body's middle.
Vector3 middleOf(ScenePhysics scene, String entity) {
  final pose = scene.physics.transformOf(scene.bodyOf(entity)!)!;
  return Vector3(pose[0], pose[1], pose[2]);
}

JointState? stateOf(ScenePhysics scene, String entity) =>
    scene.physics.jointStateOf(scene.jointOf(entity)!);

void run(ScenePhysics scene, double seconds) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
  }
}

double toRadians(double degrees) => degrees * math.pi / 180;

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  group('a joint in a document', () {
    test('on a body with nothing above it welds that body to the world', () {
      scene = ScenePhysics(
        sceneOf([
          body(
            'crate',
            at: Vector3(0, 3, 0),
            joint: JointComponent(kind: JointKind.fixed),
          ),
        ]),
      );
      run(scene, 2);

      expect(middleOf(scene, 'crate').y, closeTo(3, 0.01));
      expect(stateOf(scene, 'crate'), isNotNull);
    });

    test('is numbered apart from the bodies, one to one with its entity', () {
      scene = ScenePhysics(
        sceneOf([
          body('ball', at: Vector3(1, 3, 0)),
          joint(
            'pivot',
            JointComponent(kind: JointKind.point),
            parent: 'ball',
            at: Vector3(-1, 0, 0),
          ),
        ]),
      );

      final number = scene.jointOf('pivot')!;
      expect(scene.entityOfJoint(number), 'pivot');
      expect(scene.jointOf('ball'), isNull);
      expect(scene.broken, isEmpty);
    });

    test('placed away from its body is where the body swings about', () {
      scene = ScenePhysics(
        sceneOf([
          body('ball', at: Vector3(1, 3, 0)),
          joint(
            'pivot',
            JointComponent(kind: JointKind.point),
            parent: 'ball',
            at: Vector3(-1, 0, 0),
          ),
        ]),
      );
      run(scene, 0.4);

      final ball = middleOf(scene, 'ball');
      expect(ball.y, lessThan(2.5), reason: 'it swings down');
      expect(ball.distanceTo(Vector3(0, 3, 0)), closeTo(1, 0.01));
    });

    test('holds a body to the body above it, and that one to the world', () {
      // An arm hanging from a shoulder, and a forearm from its elbow, each a
      // joint entity at the top of the body it holds.
      final point = JointComponent(kind: JointKind.point);
      scene = ScenePhysics(
        sceneOf([
          body('arm', at: Vector3(0, 5, 0), shape: _upright()),
          joint('shoulder', point, parent: 'arm', at: Vector3(0, 0.5, 0)),
          body(
            'forearm',
            parent: 'arm',
            at: Vector3(0, -1, 0),
            shape: _upright(),
          ),
          joint('elbow', point, parent: 'forearm', at: Vector3(0, 0.5, 0)),
        ]),
      );
      run(scene, 1);
      expect(middleOf(scene, 'forearm').y, closeTo(4, 0.02));

      // Without the shoulder the two fall together, still an arm.
      final shoulder = scene.document['shoulder']!;
      scene.apply(SceneDiff([RemoveEntity(shoulder.toJson())]));
      expect(scene.jointOf('shoulder'), isNull);
      run(scene, 0.5);

      final arm = middleOf(scene, 'arm');
      expect(arm.y, lessThan(4.5));
      expect(arm.distanceTo(middleOf(scene, 'forearm')), closeTo(1, 0.02));
    });

    test('with nothing at or above it to hold holds nothing', () {
      scene = ScenePhysics(
        sceneOf([joint('loose', JointComponent(kind: JointKind.fixed))]),
      );
      expect(scene.jointOf('loose'), isNotNull);
      expect(stateOf(scene, 'loose'), isNull);
    });
  });

  group('what a document says in degrees', () {
    test('a hinge stops at its limit', () {
      // A bar hinged at one end about the world's z axis, which is the
      // hinge entity's x turned a quarter about y, falls to its limit.
      scene = ScenePhysics(
        sceneOf([
          body('bar', at: Vector3(0.5, 3, 0), shape: bar()),
          joint(
            'hinge',
            JointComponent(
              limits: const {JointAxis.aboutX: JointRange(-30, 30)},
            ),
            parent: 'bar',
            at: Vector3(-0.5, 0, 0),
            rotation: Vector3(0, -90, 0),
          ),
        ]),
      );
      run(scene, 2);

      final twist = stateOf(scene, 'hinge')!.angles[0];
      expect(twist.abs(), closeTo(toRadians(30), 0.02));
      final turned = scene.document['bar']![SceneComponents.transform]!;
      expect((turned as TransformComponent).rotation.z, closeTo(-30, 1.5));
    });

    test('a motor drives at its speed', () {
      // A plate turning about the upright, which is the joint entity's x
      // turned a quarter about z.
      scene = ScenePhysics(
        sceneOf([
          body(
            'plate',
            at: Vector3(0, 3, 0),
            shape: BodyComponent(size: Vector3(1, 0.1, 1)),
          ),
          joint(
            'spindle',
            JointComponent(speed: 90, strength: 1000),
            parent: 'plate',
            rotation: Vector3(0, 0, 90),
          ),
        ]),
      );
      run(scene, 0.5);

      expect(
        stateOf(scene, 'spindle')!.angles[0],
        closeTo(toRadians(45), 0.05),
      );
      expect(middleOf(scene, 'plate').y, closeTo(3, 0.01));
    });

    test('a cone lets its body swing no further than its swing', () {
      scene = ScenePhysics(
        sceneOf([
          body('bar', at: Vector3(0.5, 3, 0), shape: bar()),
          joint(
            'socket',
            JointComponent(kind: JointKind.cone, swing: 30),
            parent: 'bar',
            at: Vector3(-0.5, 0, 0),
          ),
        ]),
      );
      run(scene, 2);

      final tip = middleOf(scene, 'bar');
      final below = math.atan2(3 - tip.y, tip.xz.length);
      expect(below, closeTo(toRadians(30), 0.03));
    });
  });

  group('lengths', () {
    test('a distance joint with a range is a rope', () {
      // The hook is a metre above the ball, on a rope of two.
      scene = ScenePhysics(
        sceneOf([
          body('ball', at: Vector3(0, 3, 0)),
          joint(
            'hook',
            JointComponent(
              kind: JointKind.distance,
              limits: const {JointAxis.alongX: JointRange(0, 2)},
            ),
            parent: 'ball',
            at: Vector3(0, 1, 0),
          ),
        ]),
      );
      run(scene, 2);

      expect(middleOf(scene, 'ball').y, closeTo(2, 0.02));
    });

    test('a six-axis joint frees only what it leaves free', () {
      // A lift: locked every way but down, and down a metre at most.
      const locked = JointRange.at(0);
      scene = ScenePhysics(
        sceneOf([
          body(
            'lift',
            at: Vector3(0, 3, 0),
            joint: JointComponent(
              kind: JointKind.sixAxis,
              limits: const {
                JointAxis.alongX: locked,
                JointAxis.alongY: JointRange(-1, 0),
                JointAxis.alongZ: locked,
                JointAxis.aboutX: locked,
                JointAxis.aboutY: locked,
                JointAxis.aboutZ: locked,
              },
            ),
          ),
        ]),
      );
      run(scene, 2);

      final lift = middleOf(scene, 'lift');
      expect(lift.y, closeTo(2, 0.02));
      expect(lift.x.abs() + lift.z.abs(), lessThan(0.01));
    });
  });

  group('edits', () {
    SceneDocument pendulum() => sceneOf([
      body('ball', at: Vector3(1, 3, 0)),
      joint(
        'pivot',
        JointComponent(kind: JointKind.point),
        parent: 'ball',
        at: Vector3(-1, 0, 0),
      ),
    ]);

    test('rebuilding a body makes its joint again, with the same number', () {
      scene = ScenePhysics(pendulum());
      final number = scene.jointOf('pivot');
      run(scene, 0.5);

      // Dragged a metre further out, it swings about a pivot a metre further
      // out too: the pivot is under it.
      scene.apply(
        SceneDiff([
          SetComponent(
            'ball',
            SceneComponents.transform,
            from: scene.document['ball']![SceneComponents.transform]!.toJson(),
            to: TransformComponent(position: Vector3(2, 3, 0)).toJson(),
          ),
        ]),
      );
      expect(scene.jointOf('pivot'), number);
      expect(stateOf(scene, 'pivot'), isNotNull);

      run(scene, 1);
      expect(
        middleOf(scene, 'ball').distanceTo(Vector3(1, 3, 0)),
        closeTo(1, 0.01),
      );
    });

    test('taking the component away lets go, and gives the number up', () {
      scene = ScenePhysics(pendulum());
      final number = scene.jointOf('pivot')!;
      final had = scene.document['pivot']![SceneComponents.joint]!;

      scene.apply(
        SceneDiff([
          SetComponent(
            'pivot',
            SceneComponents.joint,
            from: had.toJson(),
            to: null,
          ),
        ]),
      );
      expect(scene.jointOf('pivot'), isNull);
      expect(scene.entityOfJoint(number), isNull);
      expect(scene.physics.jointStateOf(number), isNull);

      run(scene, 1);
      expect(middleOf(scene, 'ball').x, closeTo(1, 0.01), reason: 'it fell');
    });

    test('an added joint holds', () {
      scene = ScenePhysics(sceneOf([body('crate', at: Vector3(0, 3, 0))]));
      scene.apply(
        SceneDiff([
          AddEntity(
            joint(
              'weld',
              JointComponent(kind: JointKind.fixed),
              parent: 'crate',
            ).toJson(),
          ),
        ]),
      );
      run(scene, 1);
      expect(middleOf(scene, 'crate').y, closeTo(3, 0.01));
    });
  });

  group('breaking', () {
    test('a joint that breaks lets go, says so, and stays broken until its '
        'own entity is edited', () {
      // A one-kilogram crate weighs ten newtons, more than the weld takes.
      final weak = JointComponent(kind: JointKind.fixed, breakingForce: 5);
      scene = ScenePhysics(
        sceneOf([
          body('crate', at: Vector3(0, 3, 0)),
          joint('weld', weak, parent: 'crate'),
        ]),
      );
      final number = scene.jointOf('weld')!;

      scene.advance(1 / 60);
      final broke = scene.events.where(
        (event) => event.kind == PhysicsEventKind.broke,
      );
      expect(broke.map((event) => event.a), [number]);
      expect(scene.entityOfJoint(number), 'weld');
      expect(scene.broken, {'weld'});
      expect(scene.physics.jointStateOf(number), isNull);

      // Dragging the crate is not mending the weld.
      scene.apply(
        SceneDiff([
          SetComponent(
            'crate',
            SceneComponents.transform,
            from: scene.document['crate']![SceneComponents.transform]!.toJson(),
            to: TransformComponent(position: Vector3(0, 6, 0)).toJson(),
          ),
        ]),
      );
      expect(scene.broken, {'weld'});
      run(scene, 0.5);
      expect(middleOf(scene, 'crate').y, lessThan(5.5));

      // Editing the weld is, and it holds where the crate now is.
      scene.apply(
        SceneDiff([
          SetField(
            'weld',
            SceneComponents.joint,
            'breakingForce',
            from: 5,
            to: 0,
          ),
        ]),
      );
      expect(scene.broken, isEmpty);
      expect(scene.jointOf('weld'), number);
      final mended = middleOf(scene, 'crate').y;
      run(scene, 1);
      expect(middleOf(scene, 'crate').y, closeTo(mended, 0.01));
    });
  });
}

/// A body a metre tall and thin across.
BodyComponent _upright() => BodyComponent(size: Vector3(0.1, 1, 0.1));
