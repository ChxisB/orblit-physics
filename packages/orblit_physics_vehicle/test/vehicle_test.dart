import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_vehicle/orblit_physics_vehicle.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _dt = 1 / 60;
const _gravity = 9.81;
const _carId = 1;
const _radius = 0.35;
const _travel = 0.3;

VehicleSpec _spec({
  double frequency = 1.5,
  double damping = 0.5,
  double antiRoll = 0.0,
}) => VehicleSpec(
  chassis: const Shape.box(0.9, 0.3, 2.0),
  mass: 1200,
  antiRoll: antiRoll,
  wheels: [
    for (final x in const [-0.8, 0.8])
      for (final z in const [-1.2, 1.2])
        WheelSpec(
          mount: Vector3(x, 0, z),
          radius: _radius,
          suspension: SuspensionSpec(
            travel: _travel,
            frequency: frequency,
            damping: damping,
          ),
        ),
  ],
);

/// A car above a plane, and the means to run it as a game does: update, then
/// step.
final class _World {
  _World(
    VehicleSpec spec, {
    double height = 0.6,
    Layers floor = Layers.everything,
  }) {
    physics.add(
      900,
      shape: const Shape.plane(0, 1, 0),
      motion: PhysicsMotion.fixed,
      layers: floor,
    );
    car = Vehicle(physics: physics, id: _carId, spec: spec, at: [0, height, 0]);
  }

  final physics = Physics();
  late final Vehicle car;

  void run(double seconds, {void Function()? each}) {
    for (var i = 0; i < (seconds / _dt).round(); i++) {
      each?.call();
      final before = physics.transformOf(_carId)!;
      car.update(_dt);
      physics.step(_dt);
      final after = physics.transformOf(_carId)!;
      rise = (after[1] - before[1]) / _dt;
      speed =
          math.sqrt(
            math.pow(after[0] - before[0], 2) +
                math.pow(after[1] - before[1], 2) +
                math.pow(after[2] - before[2], 2),
          ) /
          _dt;
    }
  }

  double get height => physics.transformOf(_carId)![1];

  /// How fast the chassis moved over the last step, from where it was and
  /// where it is. The solver's own velocity is read at the end of its
  /// sub-steps, where a body held up by one impulse a step is 7 cm/s either
  /// side of rest.
  double rise = 0.0;
  double speed = 0.0;
}

/// How high the chassis' middle stands once the springs hold its weight and
/// nothing else: the reach of the cast, less the squeeze that weight makes.
double _rideHeight(double frequency) {
  final omega = 2 * math.pi * frequency;
  return _travel + _radius - _gravity / (omega * omega);
}

void main() {
  group('settling', () {
    test('stops at its ride height with no speed left', () {
      final w = _World(_spec());
      addTearDown(w.physics.dispose);

      w.run(6);

      expect(w.height, closeTo(_rideHeight(1.5), _rideHeight(1.5) * 0.02));
      expect(w.speed, lessThan(0.01));
      expect(w.car.wheels.every((wheel) => wheel.grounded), isTrue);
    });

    test('does not buzz with a stiff spring and a hard damper', () {
      final w = _World(_spec(frequency: 8, damping: 1), height: 0.7);
      addTearDown(w.physics.dispose);
      w.run(3);

      var turns = 0;
      var last = 0.0;
      w.run(
        2,
        each: () {
          final now = w.rise;
          if (now * last < 0 && now.abs() > 1e-4) turns++;
          if (now.abs() > 1e-4) last = now;
        },
      );

      expect(turns, 0);
      expect(w.rise.abs(), lessThan(0.001));
    });
  });

  group('a hard landing', () {
    test('bottoms out on its chassis and never passes the floor', () {
      final w = _World(_spec(), height: 3);
      addTearDown(w.physics.dispose);

      var lowest = double.infinity;
      var deepest = 0.0;
      w.run(
        4,
        each: () {
          lowest = math.min(lowest, w.height);
          for (final wheel in w.car.wheels) {
            deepest = math.max(deepest, wheel.compression);
          }
        },
      );

      expect(deepest, greaterThan(_travel), reason: 'it did bottom out');
      expect(lowest, greaterThan(0.3 - 0.03), reason: 'box half height');
      expect(w.height, closeTo(_rideHeight(1.5), _rideHeight(1.5) * 0.02));
    });
  });

  group('wheels', () {
    test('give no force in the air and read as not grounded', () {
      final w = _World(_spec(), height: 5);
      addTearDown(w.physics.dispose);

      w.run(0.2);

      for (final wheel in w.car.wheels) {
        expect(wheel.grounded, isFalse);
        expect(wheel.force, 0);
      }
    });

    test('do not see a body their layers leave out', () {
      final spec = VehicleSpec(
        chassis: const Shape.box(0.9, 0.3, 2.0),
        mass: 1200,
        ground: const Layers(is_: 1, cares: 2),
        wheels: [WheelSpec(mount: Vector3(0, 0, 0), radius: _radius)],
      );
      final w = _World(
        spec,
        height: 0.5,
        floor: const Layers(is_: 4, cares: 0),
      );
      addTearDown(w.physics.dispose);

      w.run(0.05);

      expect(w.car.wheels.single.grounded, isFalse);
    });
  });

  group('an anti-roll bar', () {
    double rollWith(double antiRoll) {
      final w = _World(_spec(antiRoll: antiRoll));
      addTearDown(w.physics.dispose);
      // A passenger's weight pressing on the left side, as a force each step.
      w.run(
        4,
        each: () {
          final t = w.physics.transformOf(_carId)!;
          w.physics.push(
            _carId,
            impulse: [0, -4000 * _dt, 0],
            at: [t[0] - 0.8, t[1], t[2]],
          );
        },
      );
      final t = w.physics.transformOf(_carId)!;
      final up = Quaternion(
        t[3],
        t[4],
        t[5],
        t[6],
      ).asRotationMatrix().transformed(Vector3(0, 1, 0));
      return up.x.abs();
    }

    test('leans the car less under a one-sided load', () {
      final loose = rollWith(0);
      final barred = rollWith(60000);

      expect(loose, greaterThan(0.005), reason: 'the load does lean it');
      expect(barred, lessThan(loose * 0.8));
    });
  });

  group('a spec', () {
    test('refuses a spring a step cannot hold', () {
      expect(
        () => SuspensionSpec(frequency: SuspensionSpec.maxFrequency + 1),
        throwsArgumentError,
      );
      expect(() => SuspensionSpec(travel: 0), throwsArgumentError);
      expect(() => SuspensionSpec(damping: 3), throwsArgumentError);
    });

    test(
      'refuses a car with no wheels, no weight or a lone wheel on a bar',
      () {
        const box = Shape.box(1, 1, 1);
        final wheel = WheelSpec(mount: Vector3.zero(), radius: 0.3);
        expect(
          () => VehicleSpec(chassis: box, mass: 100, wheels: []),
          throwsArgumentError,
        );
        expect(
          () => VehicleSpec(chassis: box, mass: 0, wheels: [wheel]),
          throwsArgumentError,
        );
        expect(
          () => VehicleSpec(
            chassis: box,
            mass: 100,
            wheels: [wheel],
            antiRoll: 1,
          ),
          throwsArgumentError,
        );
      },
    );

    test('is not moved by changing the vector it was made from', () {
      final mount = Vector3(1, 2, 3);
      final wheel = WheelSpec(mount: mount, radius: 0.3);
      mount.x = 9;

      expect(wheel.mount.x, 1);
    });

    test('is not updated by a step that is too long', () {
      final w = _World(_spec());
      addTearDown(w.physics.dispose);

      expect(() => w.car.update(0.1), throwsArgumentError);
      expect(() => w.car.update(0), throwsArgumentError);
    });
  });
}
