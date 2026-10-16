import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_vehicle/orblit_physics_vehicle.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _dt = 1 / 60;
const _gravity = 9.81;
const _id = 1;
const _radius = 0.35;

/// The solver sheds a few percent of a body's speed a second, which stops a car
/// a little sooner than its grip alone would.
const _damped = 0.1;
const _wheelbase = 2.4;

/// A four-wheel car whose front wheels steer. Its front is along negative z.
VehicleSpec _car({
  TyreSpec? tyre,
  double centreZ = 0.0,
  bool handbrakeRear = false,
  double brakeTorque = 3000,
}) => VehicleSpec(
  chassis: const Shape.box(0.9, 0.3, 2.0),
  mass: 1200,
  centre: [0, 0, centreZ],
  wheels: [
    for (final x in const [-0.8, 0.8])
      for (final z in const [-1.2, 1.2])
        WheelSpec(
          mount: Vector3(x, 0, z),
          radius: _radius,
          steers: z < 0,
          brakeTorque: brakeTorque,
          handbrake: handbrakeRear && z > 0,
          tyre: tyre,
        ),
  ],
);

/// A car on a plane, run as a game does: update, then step.
final class _Road {
  _Road(VehicleSpec spec, {double slope = 0.0, SurfaceGrip? surface})
    : up = Vector3(0, math.cos(slope), math.sin(slope)) {
    physics.add(
      900,
      shape: Shape.plane(up.x, up.y, up.z),
      motion: PhysicsMotion.fixed,
    );
    final half = slope / 2;
    final at = up * 0.6;
    car = Vehicle(
      physics: physics,
      id: _id,
      spec: spec,
      at: [at.x, at.y, at.z],
      rotation: [math.sin(half), 0, 0, math.cos(half)],
      surface: surface ?? (_) => 1.0,
    );
  }

  final Vector3 up;
  final physics = Physics();
  late final Vehicle car;

  void run(double seconds, {void Function()? each}) {
    for (var i = 0; i < (seconds / _dt).round(); i++) {
      each?.call();
      car.update(_dt);
      physics.step(_dt);
    }
  }

  /// Sends it along its own front at `speed` and lets the wheels catch up.
  void launch(double speed) {
    physics.drive(_id, velocity: [0, 0, -speed]);
    run(1);
  }

  Vector3 get at {
    final t = physics.transformOf(_id)!;
    return Vector3(t[0], t[1], t[2]);
  }

  /// Metres a second along the ground, and radians a second about up.
  double get speed {
    final v = physics.velocityOf(_id)!;
    return math.sqrt(v[0] * v[0] + v[2] * v[2]);
  }

  double get yaw => physics.velocityOf(_id)![4];
}

void main() {
  group('steering', () {
    test('turns a car in the circle its geometry draws', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.launch(3);
      road.car.setInput(VehicleInput(steer: 0.6));
      road.run(1);

      var speed = 0.0, yaw = 0.0;
      road.run(
        3,
        each: () {
          speed += road.speed;
          yaw += road.yaw;
        },
      );

      // The rear axle goes round at wheelbase over tan(lock), and the middle of
      // the car, a half wheelbase ahead of it, goes round a little wider.
      final rear = _wheelbase / math.tan(0.6 * 0.5);
      final geometry = math.sqrt(rear * rear + 1.2 * 1.2);
      expect(speed / yaw, closeTo(geometry, geometry * 0.05));
    });

    test('turns right for a negative input and straight for none', () {
      final left = _Road(_car());
      final right = _Road(_car());
      final straight = _Road(_car());
      for (final (road, steer) in [
        (left, 1.0),
        (right, -1.0),
        (straight, 0.0),
      ]) {
        addTearDown(road.physics.dispose);
        road.run(2);
        road.launch(3);
        road.car.setInput(VehicleInput(steer: steer));
        road.run(1);
      }

      expect(left.yaw, greaterThan(0.1));
      expect(right.yaw, lessThan(-0.1));
      expect(straight.yaw.abs(), lessThan(0.01));
    });

    test('turns the inner wheel more than the outer one', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(1);
      road.car.setInput(VehicleInput(steer: 1));
      road.run(0.1);

      // Wheels are listed x -0.8 (the left of a car whose front is -z) first.
      final (inner, outer) = (
        road.car.wheels[0].steer,
        road.car.wheels[2].steer,
      );
      expect(inner, greaterThan(outer), reason: 'the left wheel is inside');
      expect(outer, greaterThan(0));
      expect(road.car.wheels[1].steer, 0);

      // Both wheels point at the same place on the line through the rear axle.
      final centre = [
        for (final (x, angle) in [(-0.8, inner), (0.8, outer)])
          -x + _wheelbase / math.tan(angle),
      ];
      expect(centre[0], closeTo(centre[1], 0.01));
    });
  });

  group('braking', () {
    /// Launches, brakes, and gives the metres it took against the metres the
    /// grip `slowing` allows from the speed it was really going at. The launch
    /// leaves it a little under `speed`, so the speed is read off its motion.
    (double, double) stop(_Road road, double slowing) {
      road.launch(8);
      final start = road.at;
      road.run(0.1);
      final speed = (road.at - start).length / 0.1;

      final from = road.at;
      road.car.setInput(VehicleInput(brake: 1));
      road.run(6);
      expect(road.speed, lessThan(0.05), reason: 'it did stop');
      return (
        (road.at - from).length,
        speed * speed / (2 * slowing * _gravity),
      );
    }

    test('stops a car in the distance its grip allows', () {
      final road = _Road(_car(tyre: TyreSpec(sliding: 1)));
      addTearDown(road.physics.dispose);
      road.run(2);

      final (metres, expected) = stop(road, 1.0);
      expect(metres, closeTo(expected, expected * _damped));
    });

    test('stops a car on locked wheels in the distance sliding allows', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);

      final (metres, expected) = stop(road, 0.8);
      expect(metres, closeTo(expected, expected * _damped));
    });

    test('stops in twice the distance on ground that grips half as well', () {
      final road = _Road(_car(tyre: TyreSpec(sliding: 1)), surface: (_) => 0.5);
      addTearDown(road.physics.dispose);
      road.run(2);

      final (metres, expected) = stop(road, 0.5);
      expect(metres, closeTo(expected, expected * _damped));
    });

    test('locks a wheel the brake can hold and lets a free one roll', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.launch(8);

      final start = road.at;
      road.run(0.1);
      final speed = (road.at - start).length / 0.1;
      final rolling = road.car.wheels.first.spin;
      expect(rolling, closeTo(speed / _radius, speed / _radius * 0.03));

      road.car.setInput(VehicleInput(brake: 1));
      road.run(0.5);
      expect(road.car.wheels.first.spin.abs(), lessThan(1));
    });
  });

  group('a parked car', () {
    const slope = 20 * math.pi / 180;

    /// Metres it moves in five seconds once it has settled.
    double creep(_Road road) {
      road.run(2);
      final from = road.at;
      road.run(5);
      return (road.at - from).length;
    }

    test('does not creep on a 20 degree slope with the brake on', () {
      final road = _Road(_car(), slope: slope);
      addTearDown(road.physics.dispose);
      road.car.setInput(VehicleInput(brake: 1));

      expect(creep(road), lessThan(0.01));
    });

    test('is held by the handbrake on its rear wheels', () {
      final road = _Road(_car(handbrakeRear: true), slope: slope);
      addTearDown(road.physics.dispose);
      road.car.setInput(VehicleInput(handbrake: true));

      expect(creep(road), lessThan(0.01));
    });

    test('rolls away when nothing holds it', () {
      final road = _Road(_car(), slope: slope);
      addTearDown(road.physics.dispose);

      expect(creep(road), greaterThan(1));
    });

    test('is not held by a slope its tyres cannot grip', () {
      final road = _Road(_car(), slope: slope, surface: (_) => 0.2);
      addTearDown(road.physics.dispose);
      road.car.setInput(VehicleInput(brake: 1));

      expect(creep(road), greaterThan(1));
    });
  });

  group('weight', () {
    test('is shared so that the moments balance about the centre of mass', () {
      final spec = _car(centreZ: -0.4);
      // Front wheels are at z -1.2: a third of the car each way round.
      expect(spec.sprung[0], closeTo(1200 / 3, 1e-6));
      expect(spec.sprung[1], closeTo(1200 / 6, 1e-6));
      expect(spec.sprung.reduce((a, b) => a + b), closeTo(1200, 1e-6));
    });

    test('rests a front-heavy car level, with the front springs loaded', () {
      final road = _Road(_car(centreZ: -0.4));
      addTearDown(road.physics.dispose);
      road.run(5);

      final wheels = road.car.wheels;
      expect(wheels[0].force, closeTo(400 * _gravity, 400 * _gravity * 0.03));
      expect(wheels[1].force, closeTo(200 * _gravity, 200 * _gravity * 0.03));
      expect(wheels[0].compression, closeTo(wheels[1].compression, 0.005));
    });
  });

  group('a spec', () {
    final wheel = WheelSpec(mount: Vector3.zero(), radius: 0.3);

    test('refuses a tyre that grips nothing or slips backwards', () {
      expect(() => TyreSpec(grip: 0), throwsArgumentError);
      expect(() => TyreSpec(peakSlip: 0), throwsArgumentError);
      expect(() => TyreSpec(sliding: 1.5), throwsArgumentError);
      expect(() => TyreSpec(relaxation: 0), throwsArgumentError);
    });

    test('refuses a wheel with a bad brake or no spin to hold', () {
      final mount = Vector3.zero();
      expect(
        () => WheelSpec(mount: mount, radius: 0.3, brakeTorque: -1),
        throwsArgumentError,
      );
      expect(
        () => WheelSpec(mount: mount, radius: 0.3, handbrake: true),
        throwsArgumentError,
      );
      expect(
        () => WheelSpec(mount: mount, radius: 0.3, inertia: 0),
        throwsArgumentError,
      );
    });

    test('refuses a centre of mass that is outside the wheels', () {
      const box = Shape.box(1, 1, 1);
      final wheels = _car().wheels;
      expect(
        () => VehicleSpec(
          chassis: box,
          mass: 100,
          wheels: wheels,
          centre: const [0, 0, 3],
        ),
        throwsArgumentError,
      );
      expect(
        () =>
            VehicleSpec(chassis: box, mass: 100, wheels: [wheel], maxSteer: 2),
        throwsArgumentError,
      );
    });

    test('refuses a steer or a brake that is out of range', () {
      expect(() => VehicleInput(steer: 1.1), throwsArgumentError);
      expect(() => VehicleInput(brake: -0.1), throwsArgumentError);
      expect(() => VehicleInput(brake: 2), throwsArgumentError);
    });
  });
}
