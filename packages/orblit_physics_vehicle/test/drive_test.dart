import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_vehicle/orblit_physics_vehicle.dart';
import 'package:orblit_physics_vehicle/src/driveline.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _dt = 1 / 60;
const _radius = 0.35;

DriveSpec _drive({double lock = 0.0, double finalDrive = 4.5}) => DriveSpec(
  engine: EngineSpec(
    torque: [(1000, 160), (3000, 220), (5000, 240), (6500, 200)],
  ),
  ratios: [3.6, 2.2, 1.5, 1.1, 0.9],
  finalDrive: finalDrive,
  lock: lock,
);

/// A rear-drive car. Its front is along negative z.
VehicleSpec _car({DriveSpec? drive, bool driven = true}) => VehicleSpec(
  chassis: const Shape.box(0.9, 0.3, 2.0),
  mass: 1200,
  drive: drive ?? _drive(),
  wheels: [
    for (final x in const [-0.8, 0.8])
      for (final z in const [-1.2, 1.2])
        WheelSpec(
          mount: Vector3(x, 0, z),
          radius: _radius,
          steers: z < 0,
          drives: driven && z > 0,
          brakeTorque: 3000,
        ),
  ],
);

/// A car above a plane, run as a game does: update, then step.
final class _Road {
  _Road(VehicleSpec spec, {double height = 0.6, SurfaceGrip? surface}) {
    physics.add(
      900,
      shape: const Shape.plane(0, 1, 0),
      motion: PhysicsMotion.fixed,
    );
    car = Vehicle(
      physics: physics,
      id: 1,
      spec: spec,
      at: [0, height, 0],
      surface: surface ?? (_) => 1.0,
    );
  }

  final physics = Physics();
  late final Vehicle car;

  void run(double seconds, {void Function()? each}) {
    for (var i = 0; i < (seconds / _dt).round(); i++) {
      car.update(_dt);
      physics.step(_dt);
      each?.call();
    }
  }

  /// Metres a second along the ground.
  double get speed {
    final v = physics.velocityOf(1)!;
    return math.sqrt(v[0] * v[0] + v[2] * v[2]);
  }
}

void main() {
  group('a car with an engine', () {
    test('pulls away and shifts up through its gears', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.car.setInput(VehicleInput(throttle: 1));

      final gears = <int>[];
      road.run(14, each: () => gears.add(road.car.gear));

      expect(road.speed, greaterThan(25));
      expect(gears.last, greaterThanOrEqualTo(3));
      for (var i = 1; i < gears.length; i++) {
        expect(gears[i], greaterThanOrEqualTo(gears[i - 1]), reason: 'step $i');
      }
      expect(
        road.car.rpm,
        inInclusiveRange(900, 6500 * 1.06),
        reason: 'the engine stayed between idle and the redline',
      );
    });

    test('turns its driven wheels at the road speed once it is cruising', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.car.setInput(VehicleInput(throttle: 0.2));
      road.run(8);

      final from = road.physics.transformOf(1)!;
      road.run(0.2);
      final to = road.physics.transformOf(1)!;
      final speed =
          math.sqrt(
            math.pow(to[0] - from[0], 2) + math.pow(to[2] - from[2], 2),
          ) /
          0.2;

      expect(speed, greaterThan(3));
      for (final i in [1, 3]) {
        final rolling = road.car.wheels[i].spin * _radius;
        expect(rolling, closeTo(speed, speed * 0.05), reason: 'wheel $i');
      }
    });

    test('spins up a driven wheel that has no road under it', () {
      final road = _Road(_car(), height: 20);
      addTearDown(road.physics.dispose);
      road.car.setInput(VehicleInput(throttle: 1));
      road.run(1);

      expect(road.car.wheels.every((wheel) => !wheel.grounded), isTrue);
      expect(road.car.wheels[1].spin, greaterThan(20));
      expect(road.car.wheels[3].spin, greaterThan(20));
      expect(road.car.wheels[0].spin, 0);
      expect(road.car.rpm, greaterThan(1500));
    });

    test('raises its wheelspin smoothly on slippery ground', () {
      final road = _Road(_car(), surface: (_) => 0.2);
      addTearDown(road.physics.dispose);
      road.run(2);
      road.car.setInput(VehicleInput(throttle: 1, gear: 1));

      final spins = <double>[];
      road.run(0.5, each: () => spins.add(road.car.wheels[1].spin));

      // The first steps take the clutch up. After them, a wheel that is
      // clear of the road gains a little a step and never gives it back.
      for (var i = 6; i < spins.length; i++) {
        final gain = spins[i] - spins[i - 1];
        expect(gain, inInclusiveRange(0, 3), reason: 'step $i');
      }
      expect(spins.last, greaterThan(20), reason: 'the wheels do spin');
    });

    test('does not creep with the pedal up', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(4);

      expect(road.speed, lessThan(0.05));
    });

    test('backs up in reverse', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.car.setInput(VehicleInput(throttle: 0.5, gear: -1));
      road.run(3);

      final v = road.physics.velocityOf(1)!;
      expect(road.car.gear, -1);
      expect(v[2], greaterThan(1), reason: 'it moves toward its rear');
    });

    test('holds the gear the driver picks', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.car.setInput(VehicleInput(throttle: 1, gear: 2));
      road.run(6);

      expect(road.car.gear, 2);
    });

    test('is slowed by the engine when the pedal is up', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      road.run(2);
      road.car.setInput(VehicleInput(throttle: 1));
      road.run(5);
      final fast = road.car.rpm;

      road.car.setInput(VehicleInput());
      road.run(1);
      expect(road.car.rpm, lessThan(fast));
    });
  });

  group('a differential', () {
    /// Two driven wheels on one axle, the second turning ten radians a second
    /// faster, with a gearbox that has had time to take hold.
    (Driveline, List<double>) run(double lock) {
      final wheels = [
        for (final x in const [-0.8, 0.8])
          WheelSpec(mount: Vector3(x, 0, 1.2), radius: _radius, drives: true),
      ];
      final line = Driveline(
        spec: _drive(lock: lock),
        wheels: wheels,
      );
      final spins = [0.0, 10.0];
      final answers = [
        for (var i = 0; i < 2; i++)
          (double torque) => spins[i] + torque * _dt / wheels[i].inertia,
      ];
      for (var i = 0; i < 120; i++) {
        line.step(spins, answers, VehicleInput(throttle: 1, gear: 1), _dt);
      }
      return (line, [wheels[0].inertia, wheels[1].inertia]);
    }

    test('gives both wheels the same torque when it is open', () {
      final (line, _) = run(0);
      expect(line.torque(0), greaterThan(0));
      expect(line.torque(0), closeTo(line.torque(1), 1e-9));
    });

    test('moves torque to the slower wheel up to what it can hold', () {
      final (limited, _) = run(50);
      expect(limited.torque(0) - limited.torque(1), closeTo(100, 1e-6));

      final (locked, inertia) = run(double.infinity);
      expect(
        locked.torque(0) - locked.torque(1),
        closeTo(inertia[0] * 10 / _dt, 1e-6),
      );
    });
  });

  group('a spec', () {
    final torque = [(1000.0, 160.0), (6500.0, 200.0)];

    test('refuses an engine that has no curve or that idles past its top', () {
      expect(() => EngineSpec(torque: [(1000, 100)]), throwsArgumentError);
      expect(
        () => EngineSpec(torque: [(2000, 100), (1000, 100)]),
        throwsArgumentError,
      );
      expect(() => EngineSpec(torque: torque, idle: 7000), throwsArgumentError);
      expect(() => EngineSpec(torque: torque, inertia: 0), throwsArgumentError);
      expect(() => EngineSpec(torque: torque, drag: -1), throwsArgumentError);
    });

    test('refuses a box with no gears or a downshift above the upshift', () {
      final engine = EngineSpec(torque: torque);
      expect(() => DriveSpec(engine: engine, ratios: []), throwsArgumentError);
      expect(
        () => DriveSpec(engine: engine, ratios: [3.0, 1.0]),
        throwsArgumentError,
        reason: 'a downshift from second would land above the upshift',
      );
      expect(
        () => DriveSpec(engine: engine, ratios: [3.0], reverse: 0),
        throwsArgumentError,
      );
    });

    test(
      'refuses a drive with no driven wheel and a driven wheel with no drive',
      () {
        expect(() => _car(driven: false), throwsArgumentError);
        expect(
          () => VehicleSpec(
            chassis: const Shape.box(1, 1, 1),
            mass: 100,
            wheels: [
              WheelSpec(mount: Vector3.zero(), radius: 0.3, drives: true),
            ],
          ),
          throwsArgumentError,
        );
      },
    );

    test('refuses a gear the car does not have', () {
      final road = _Road(_car());
      addTearDown(road.physics.dispose);
      expect(
        () => road.car.setInput(VehicleInput(gear: 6)),
        throwsArgumentError,
      );
      expect(() => VehicleInput(gear: -2), throwsArgumentError);
    });
  });
}
