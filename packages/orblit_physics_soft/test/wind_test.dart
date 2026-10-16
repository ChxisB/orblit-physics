import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_soft/orblit_physics_soft.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _columns = 20;
const _spacing = 0.05;
const _dt = 1 / 60;

/// A wind the test can switch.
final class _Gust implements SoftAir {
  Vector3 wind = Vector3.zero();

  @override
  Vector3 windAt(Vector3 point) => wind;
}

/// A flag on a pole: the left column pinned, the sheet hanging along x.
SoftBody _flag({Vector3? at, SoftAir? air, SoftSpec? spec}) {
  final flag = SoftBody.cloth(
    columns: _columns,
    rows: _columns,
    origin: at ?? Vector3(0, 2, 0),
    across: Vector3(_spacing, 0, 0),
    down: Vector3(0, -_spacing, 0),
    spec: spec,
    air: air,
  );
  for (var j = 0; j < _columns; j++) {
    flag.pin(j * _columns);
  }
  return flag;
}

void _run(SoftBody cloth, double seconds) {
  for (var n = 0; n < (seconds / _dt).round(); n++) {
    cloth.step(_dt);
  }
}

double _meanZ(SoftBody cloth) {
  var total = 0.0;
  for (var i = 0; i < cloth.count; i++) {
    total += cloth.positionOf(i).z;
  }
  return total / cloth.count;
}

/// A world with one trigger box blowing `wind`, centred at `at`. Its top is
/// below the flag's pole and above the flag's middle, so only a body that asks
/// the wind at its middle is blown.
(Physics, PhysicsWind) _zone(Vector3 at, Vector3 wind) {
  final physics = Physics();
  physics.add(
    7,
    shape: const Shape.box(2, 0.65, 3),
    motion: PhysicsMotion.fixed,
    at: at.storage,
    trigger: true,
  );
  physics.step(_dt);
  return (physics, PhysicsWind(physics, {7: wind}));
}

void main() {
  group('a flag', () {
    test('moves downwind in a zone and not outside it', () {
      final (physics, air) = _zone(Vector3(0.5, 1.15, 0), Vector3(0, 0, 10));
      final inside = _flag(air: air);
      final outside = _flag(at: Vector3(20, 2, 0), air: air);
      _run(inside, 3);
      _run(outside, 3);
      expect(_meanZ(inside), greaterThan(0.2));
      expect(_meanZ(outside).abs(), lessThan(1e-9));
      physics.dispose();
    });

    test('a free sheet comes up to the speed of the wind and no more', () {
      final sheet = SoftBody.cloth(
        columns: _columns,
        rows: _columns,
        origin: Vector3(0, 2, 0),
        across: Vector3(_spacing, 0, 0),
        down: Vector3(0, -_spacing, 0),
        spec: SoftSpec(gravity: Vector3.zero(), damping: 0, windDrag: 2),
        air: _Gust()..wind = Vector3(0, 0, 5),
      );
      _run(sheet, 5);
      for (var i = 0; i < sheet.count; i++) {
        expect(sheet.velocityOf(i).z, closeTo(5, 0.05));
      }
    });

    test('hangs as it did without any air when none blows', () {
      final still = _flag();
      final calm = _flag(air: _Gust());
      _run(still, 3);
      _run(calm, 3);
      // Air drag on a body at rest in still air is only a little extra damping.
      expect(
        calm
            .positionOf(_columns * _columns - 1)
            .distanceTo(still.positionOf(_columns * _columns - 1)),
        lessThan(0.02),
      );
    });

    test('is blown harder by a stronger wind and a larger drag', () {
      double blown(double wind, double drag) {
        final gust = _Gust()..wind = Vector3(0, 0, wind);
        final flag = _flag(
          air: gust,
          spec: SoftSpec(windDrag: drag),
        );
        _run(flag, 2);
        return _meanZ(flag);
      }

      expect(blown(10, 1), greaterThan(blown(5, 1)));
      expect(blown(10, 2), greaterThan(blown(10, 0.5)));
    });

    test('a wind that starts wakes a body that had settled', () {
      final gust = _Gust();
      final flag = _flag(air: gust);
      _run(flag, 12);
      expect(flag.asleep, isTrue);
      gust.wind = Vector3(0, 0, 10);
      flag.step(_dt);
      expect(flag.asleep, isFalse);
      _run(flag, 2);
      expect(_meanZ(flag), greaterThan(0.2));
    });

    test('a steady wind lets it settle and sleep', () {
      final flag = _flag(air: _Gust()..wind = Vector3(0, 0, 3));
      _run(flag, 20);
      expect(flag.asleep, isTrue);
    });
  });

  group('wind zones', () {
    test(
      'overlapping zones add, and a trigger that is not wind blows nothing',
      () {
        final physics = Physics();
        for (final id in [1, 2, 3]) {
          physics.add(
            id,
            shape: const Shape.box(1, 1, 1),
            motion: PhysicsMotion.fixed,
            trigger: true,
          );
        }
        physics.step(_dt);
        final air = PhysicsWind(physics, {
          1: Vector3(1, 0, 0),
          2: Vector3(0, 2, 0),
        });
        final wind = air.windAt(Vector3.zero());
        expect([wind.x, wind.y, wind.z], [1, 2, 0]);
        expect(air.windAt(Vector3(9, 9, 9)), Vector3.zero());
        physics.dispose();
      },
    );

    test('a drag below nothing is refused', () {
      expect(() => SoftSpec(windDrag: -1), throwsArgumentError);
      expect(() => SoftSpec(windDrag: double.infinity), throwsArgumentError);
    });
  });
}
