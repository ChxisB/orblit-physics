import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _travel = 0.3;
const _sag = 9.81 / (2 * math.pi * 1.5 * 2 * math.pi * 1.5);

SceneEntity _wheel(String car, String id, double x, double z, bool front) =>
    SceneEntity(
      id: id,
      name: id,
      parent: car,
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

/// A rear-drive car or truck of `mass` kilograms and a box `size`, with its
/// wheels placed where they sit at rest.
List<SceneEntity> _vehicle(
  String id, {
  required Vector3 at,
  double mass = 1200,
  Vector3? size,
}) => [
  SceneEntity(
    id: id,
    name: id,
    components: {
      SceneComponents.transform: TransformComponent(position: at),
      SceneComponents.body: BodyComponent(
        size: size ?? Vector3(1.8, 0.6, 4),
        mass: mass,
      ),
      SceneComponents.vehicle: VehicleComponent(),
    },
  ),
  for (final (name, x, z, front) in const [
    ('fl', -0.8, -1.2, true),
    ('fr', 0.8, -1.2, true),
    ('rl', -0.8, 1.2, false),
    ('rr', 0.8, 1.2, false),
  ])
    _wheel(id, '$id-$name', x, z, front),
];

SceneEntity _floor() => SceneEntity(
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

ScenePhysics _scene(List<SceneEntity> entities) {
  final scene = ScenePhysics(SceneDocument(name: 'Scene', entities: entities));
  addTearDown(scene.dispose);
  return scene;
}

/// Metres a second along the ground.
double _speed(ScenePhysics scene, String id) {
  final v = scene.physics.velocityOf(scene.bodyOf(id)!)!;
  return math.sqrt(v[0] * v[0] + v[2] * v[2]);
}

Vector3 _at(ScenePhysics scene, String id) {
  final t = scene.physics.transformOf(scene.bodyOf(id)!)!;
  return Vector3(t[0], t[1], t[2]);
}

Matrix3 _frame(ScenePhysics scene, String id) {
  final t = scene.physics.transformOf(scene.bodyOf(id)!)!;
  // asRotationMatrix, not Quaternion.rotated, which turns the other way.
  return Quaternion(t[3], t[4], t[5], t[6]).asRotationMatrix();
}

/// Radians the car faces, from straight along minus z. Left is positive.
double _heading(ScenePhysics scene, String id) {
  final ahead = _frame(scene, id).transformed(Vector3(0, 0, -1));
  return math.atan2(-ahead.x, -ahead.z);
}

bool _upright(ScenePhysics scene, String id) =>
    _frame(scene, id).transformed(Vector3(0, 1, 0)).y > 0.7;

/// Runs `seconds` of frames. `each` is called after every one.
void _run(ScenePhysics scene, double seconds, [void Function()? each]) {
  for (var i = 0; i < (seconds * 60).round(); i++) {
    scene.advance(1 / 60);
    each?.call();
  }
}

/// Holds `pace` metres a second with the pedal, as a driver does.
VehicleInput _drive(ScenePhysics scene, String id, double pace, double steer) =>
    VehicleInput(
      throttle: _speed(scene, id) < pace ? 0.7 : 0.0,
      brake: _speed(scene, id) > pace * 1.3 ? 0.5 : 0.0,
      steer: steer,
    );

/// Walks along minus z at `pace`, with gravity in the request.
final class _Stroll implements CharacterWalk {
  double pace = 0;

  @override
  List<double> ask(double seconds, PhysicsFooting? footing) {
    final fall = footing?.velocity[1] ?? 0.0;
    return [0.0, fall - 9.81 * seconds, -pace];
  }
}

void main() {
  test('drives a figure of eight on flat ground', () {
    final scene = _scene([
      _floor(),
      ..._vehicle('car', at: Vector3(0, 0.7, 0)),
    ]);
    _run(scene, 2);

    var last = _heading(scene, 'car');
    var turned = 0.0;
    var low = double.infinity;
    var far = 0.0;
    void follow(double steer) {
      scene.pilot('car', _drive(scene, 'car', 8, steer));
      final now = _heading(scene, 'car');
      turned += (now - last + math.pi) % (2 * math.pi) - math.pi;
      last = now;
      low = math.min(low, _at(scene, 'car').y);
      far = math.max(far, _at(scene, 'car').length);
      expect(_upright(scene, 'car'), isTrue, reason: 'at ${scene.document}');
    }

    _run(scene, 4, () => follow(0));
    expect(turned.abs(), lessThan(0.05), reason: 'straight');

    _run(scene, 8, () => follow(0.5));
    final left = turned;
    expect(left, greaterThan(2 * math.pi * 0.8), reason: 'a full loop left');

    _run(scene, 8, () => follow(-0.5));
    expect(left - turned, greaterThan(2 * math.pi * 0.8), reason: 'then right');

    expect(low, greaterThan(0.3), reason: 'it never sank into the floor');
    expect(far, lessThan(80), reason: 'it stayed on the course');
  });

  test('takes a ramp, flies and lands the right way up', () {
    // A thin plank 12 degrees up toward minus z, its low end on the floor.
    final ramp = SceneEntity(
      id: 'ramp',
      name: 'Ramp',
      components: {
        SceneComponents.transform: TransformComponent(
          position: Vector3(0, 0.74, -30),
          rotation: Vector3(12, 0, 0),
        ),
        SceneComponents.body: BodyComponent(
          size: Vector3(5, 0.2, 8),
          motion: BodyMotion.fixed,
        ),
      },
    );
    final scene = _scene([
      _floor(),
      ramp,
      ..._vehicle('car', at: Vector3(0, 0.7, 20)),
    ]);
    _run(scene, 2);

    var high = 0.0;
    var low = double.infinity;
    var airborne = 0;
    _run(scene, 10, () {
      scene.pilot('car', _drive(scene, 'car', 14, 0));
      final y = _at(scene, 'car').y;
      high = math.max(high, y);
      if (_at(scene, 'car').z < -36) low = math.min(low, y);
      final wheels = scene.vehicleOf('car')!.wheels;
      if (wheels.every((wheel) => !wheel.grounded)) airborne++;
    });
    scene.pilot('car', VehicleInput(brake: 0.6));
    _run(scene, 6);

    expect(high, greaterThan(1.5), reason: 'it climbed the plank');
    expect(airborne, greaterThan(5), reason: 'it left the end of it');
    expect(low, greaterThan(0.3), reason: 'it came down on its wheels');
    expect(_upright(scene, 'car'), isTrue);
    expect(_at(scene, 'car').z, lessThan(-36), reason: 'it cleared the ramp');
    expect(_at(scene, 'car').y, closeTo(0.54, 0.15), reason: 'and rests');
  });

  test('crosses a height field and stays on it', () {
    // Rolling ground, 2 m between samples, 1.2 m between crest and trough.
    const columns = 21;
    const rows = 61;
    final heights = [
      for (var r = 0; r < rows; r++)
        for (var c = 0; c < columns; c++)
          0.6 * math.sin(r * 0.3) * math.cos(c * 0.2),
    ];
    final scene = _scene(_vehicle('car', at: Vector3(0, 1.5, 0)));
    // A body made in the world itself wants a negative number.
    expect(
      scene.physics.layGround(
        -1,
        heights: heights,
        columns: columns,
        rows: rows,
        spacing: 2,
        at: const [-20, 0, -100],
      ),
      isTrue,
    );
    _run(scene, 2);

    var low = double.infinity;
    _run(scene, 9, () {
      scene.pilot('car', _drive(scene, 'car', 9, 0));
      low = math.min(low, _at(scene, 'car').y);
      expect(_upright(scene, 'car'), isTrue);
    });

    expect(_at(scene, 'car').z, lessThan(-50), reason: 'it crossed the hills');
    expect(low, greaterThan(-1.0), reason: 'it never went through the field');
    expect(_at(scene, 'car').y, lessThan(5));
  });

  test('carries a character who walked onto the bed of a truck', () {
    final bed = Vector3(2.2, 0.4, 6);
    final scene = _scene([
      _floor(),
      ..._vehicle('truck', at: Vector3(0, 0.7, 0), mass: 4000, size: bed),
      SceneEntity(
        id: 'hero',
        name: 'Hero',
        components: {
          SceneComponents.transform: TransformComponent(
            position: Vector3(0, 0.01, 6),
          ),
          SceneComponents.body: BodyComponent(
            shape: BodyShape.capsule,
            motion: BodyMotion.character,
            radius: 0.3,
            height: 1.8,
            centre: Vector3(0, 0.9, 0),
            stepHeight: 1.0,
          ),
        },
      ),
    ]);
    final stroll = _Stroll();
    scene.walk('hero', stroll);
    _run(scene, 2);
    final truck = scene.bodyOf('truck')!;
    final hero = scene.bodyOf('hero')!;

    stroll.pace = 1.5;
    _run(scene, 3);
    stroll.pace = 0;
    _run(scene, 1);

    var footing = scene.physics.footingOf(hero)!;
    expect(footing.ground, truck, reason: 'it is standing on the bed');
    // The body origin is the capsule's middle, 0.9 m over its feet.
    expect(_at(scene, 'hero').y, greaterThan(1.4), reason: 'feet on the bed');
    expect(_at(scene, 'hero').z, closeTo(_at(scene, 'truck').z, 3));

    final started = _at(scene, 'hero').z;
    var moved = 0.0;
    _run(scene, 6, () {
      scene.pilot('truck', _drive(scene, 'truck', 6, 0));
      moved = math.max(moved, -scene.physics.footingOf(hero)!.carried[2]);
    });

    footing = scene.physics.footingOf(hero)!;
    expect(footing.ground, truck, reason: 'still on the bed');
    expect(moved, greaterThan(2), reason: 'carried at a walking pace or more');
    expect(_at(scene, 'hero').z, lessThan(started - 8));
    expect(
      (_at(scene, 'hero').z - _at(scene, 'truck').z).abs(),
      lessThan(3),
      reason: 'it rode with the truck',
    );
  });
}
