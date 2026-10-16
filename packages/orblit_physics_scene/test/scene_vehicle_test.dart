import 'dart:math' as math;

import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _travel = 0.3;
const _sag = 9.81 / (2 * math.pi * 1.5 * 2 * math.pi * 1.5);

SceneEntity _wheel(String id, double x, double z, {bool front = false}) =>
    SceneEntity(
      id: id,
      name: id,
      parent: 'car',
      components: {
        SceneComponents.transform: TransformComponent(
          position: Vector3(x, _sag - _travel, z),
        ),
        SceneComponents.wheel: WheelComponent(
          radius: 0.35,
          steers: front,
          drives: !front,
          brake: 3000,
        ),
      },
    );

/// A floor and a 1200 kilogram car above it, rear wheel drive, with its
/// wheels placed where they sit at rest.
SceneDocument _world({
  VehicleComponent? car,
  Vector3? scale,
  BodyMotion motion = BodyMotion.free,
  bool wheels = true,
}) => SceneDocument(
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
    SceneEntity(
      id: 'car',
      name: 'Car',
      components: {
        SceneComponents.transform: TransformComponent(
          position: Vector3(0, 0.7, 0),
          scale: scale,
        ),
        SceneComponents.body: BodyComponent(
          size: Vector3(1.8, 0.6, 4),
          mass: 1200,
          motion: motion,
        ),
        SceneComponents.vehicle: car ?? VehicleComponent(),
      },
    ),
    if (wheels) ...[
      _wheel('fl', -0.8, -1.2, front: true),
      _wheel('fr', 0.8, -1.2, front: true),
      _wheel('rl', -0.8, 1.2),
      _wheel('rr', 0.8, 1.2),
    ],
  ],
);

Vector3 _at(ScenePhysics scene, String id) =>
    (scene.document[id]![SceneComponents.transform]! as TransformComponent)
        .position;

Vector3 _turned(ScenePhysics scene, String id) =>
    (scene.document[id]![SceneComponents.transform]! as TransformComponent)
        .rotation;

void _run(ScenePhysics scene, double seconds) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
  }
}

ScenePhysics _scene([SceneDocument? document]) {
  final scene = ScenePhysics(document ?? _world());
  addTearDown(scene.dispose);
  return scene;
}

void main() {
  group('a vehicle component', () {
    test('makes a car of the wheels under it', () {
      final scene = _scene();

      expect(scene.vehicleOf('car'), isNotNull);
      expect(scene.vehicleOf('car')!.wheels, hasLength(4));
      expect(scene.vehicleOf('floor'), isNull);
    });

    test('makes no car of a body that is scaled, driven or wheel-less', () {
      expect(_scene(_world(scale: Vector3.all(2))).vehicleOf('car'), isNull);
      expect(
        _scene(_world(motion: BodyMotion.driven)).vehicleOf('car'),
        isNull,
      );
      expect(_scene(_world(wheels: false)).vehicleOf('car'), isNull);
    });

    test('makes no car when the vehicle refuses what the file says', () {
      final scene = _scene(_world(car: VehicleComponent(ratios: [3.0, 1.0])));

      expect(scene.vehicleOf('car'), isNull);
      expect(scene.document['car']![SceneComponents.vehicle], isNotNull);
    });

    test('settles on its springs with the wheels where they were placed', () {
      final scene = _scene();
      _run(scene, 4);

      expect(_at(scene, 'car').y, closeTo(0.65 - _sag, 0.03));
      for (final (id, x, z) in const [
        ('fl', -0.8, -1.2),
        ('fr', 0.8, -1.2),
        ('rl', -0.8, 1.2),
        ('rr', 0.8, 1.2),
      ]) {
        final at = _at(scene, id);
        expect(at.x, closeTo(x, 1e-9), reason: id);
        expect(at.y, closeTo(_sag - _travel, 0.03), reason: '$id hangs');
        expect(at.z, closeTo(z, 1e-9), reason: id);
      }
    });

    test('keeps the scale of a wheel and turns only what it owns', () {
      final document = _world();
      final tall =
          (document['fl']![SceneComponents.transform]! as TransformComponent)
              .scale
            ..y = 2;
      expect(tall.y, 2);
      final scene = _scene(document);
      _run(scene, 1);

      final wheel =
          scene.document['fl']![SceneComponents.transform]!
              as TransformComponent;
      expect(wheel.scale.y, 2);
      expect(wheel.rotation.z, 0);
    });
  });

  group('a piloted car', () {
    test('drives off, rolls its wheels forward and shifts', () {
      final scene = _scene();
      _run(scene, 2);
      scene.pilot('car', VehicleInput(throttle: 1));
      _run(scene, 6);

      expect(_at(scene, 'car').z, lessThan(-20), reason: 'front is minus z');
      expect(scene.vehicleOf('car')!.gear, greaterThan(1));
      expect(
        _turned(scene, 'rl').x,
        isNot(0),
        reason: 'the roll is written to the wheel',
      );
      expect(scene.vehicleOf('car')!.wheels[2].roll, isNonNegative);
    });

    test('steers the front wheels and only those', () {
      final scene = _scene();
      _run(scene, 2);
      scene.pilot('car', VehicleInput(throttle: 0.3, steer: 1));
      _run(scene, 1);

      expect(_turned(scene, 'fl').y, greaterThan(15));
      expect(_turned(scene, 'fr').y, greaterThan(15));
      expect(_turned(scene, 'rl').y, 0);
      expect(
        _at(scene, 'car').x,
        lessThan(0),
        reason: 'positive steer is left',
      );
    });

    test('lets go of the controls with a null input', () {
      final scene = _scene();
      scene.pilot('car', VehicleInput(throttle: 1));
      expect(scene.vehicleOf('car')!.input.throttle, 1);

      scene.pilot('car', null);
      expect(scene.vehicleOf('car')!.input.throttle, 0);
    });

    test('refuses a gear the car does not have', () {
      final scene = _scene();

      expect(
        () => scene.pilot('car', VehicleInput(gear: 6)),
        throwsArgumentError,
      );
    });

    test('keeps its driver through an edit that makes the car again', () {
      final scene = _scene();
      scene.pilot('car', VehicleInput(throttle: 1));
      final before = scene.vehicleOf('car');

      final wheel =
          scene.document['rr']![SceneComponents.wheel]! as WheelComponent;
      scene.apply(
        SceneDiff([
          SetComponent(
            'rr',
            SceneComponents.wheel,
            from: wheel.toJson(),
            to: wheel.copyWith(radius: 0.4).toJson(),
          ),
        ]),
      );

      expect(scene.vehicleOf('car'), isNot(same(before)));
      expect(scene.vehicleOf('car')!.input.throttle, 1);
    });
  });

  group('editing a car', () {
    test('takes it away with the component, or with the last wheel', () {
      final scene = _scene();
      final car =
          scene.document['car']![SceneComponents.vehicle]! as VehicleComponent;
      scene.apply(
        SceneDiff([
          SetComponent(
            'car',
            SceneComponents.vehicle,
            from: car.toJson(),
            to: null,
          ),
        ]),
      );
      expect(scene.vehicleOf('car'), isNull);

      final other = _scene();
      for (final id in const ['fl', 'fr', 'rl', 'rr']) {
        other.apply(SceneDiff([RemoveEntity(other.document[id]!.toJson())]));
      }
      expect(other.vehicleOf('car'), isNull);
    });

    test('goes back to the start with a snapshot, and drives the same', () {
      final scene = _scene();
      final start = scene.snapshot();
      addTearDown(start.dispose);

      scene.pilot('car', VehicleInput(throttle: 1, steer: 0.2));
      _run(scene, 3);
      final first = scene.document.encode();

      scene.restore(start);
      expect(scene.vehicleOf('car'), isNotNull);
      _run(scene, 3);
      expect(scene.document.encode(), first);
    });
  });
}
