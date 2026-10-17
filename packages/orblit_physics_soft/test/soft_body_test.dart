import 'dart:math' as math;

import 'package:orblit_physics_soft/orblit_physics_soft.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _columns = 20;
const _spacing = 0.05;
const _mass = 0.01;

/// A 20 by 20 sheet standing in the x-y plane, its top row at y = 1.
SoftBody _curtain({SoftSpec? spec}) => SoftBody.cloth(
  columns: _columns,
  rows: _columns,
  origin: Vector3(0, 1, 0),
  across: Vector3(_spacing, 0, 0),
  down: Vector3(0, -_spacing, 0),
  particleMass: _mass,
  spec: spec,
);

/// The worst way to hang a sheet: all of it from the two top corners. A
/// finer sub-step than the default keeps the links near the pins from giving.
SoftSpec _corners({double compliance = 1e-6}) =>
    SoftSpec(substepLength: 1 / 960, stretchCompliance: compliance);

SoftBody _hung({SoftSpec? spec}) {
  final cloth = _curtain(spec: spec ?? _corners());
  cloth.pin(0);
  cloth.pin(_columns - 1);
  return cloth;
}

void _run(SoftBody cloth, double seconds, double dt) {
  for (var n = 0; n < (seconds / dt).round(); n++) {
    cloth.step(dt);
  }
}

double _energy(SoftBody cloth, SoftSpec spec) {
  var total = 0.0;
  for (var i = 0; i < cloth.count; i++) {
    if (cloth.isPinned(i)) continue;
    final p = cloth.positionOf(i);
    total +=
        0.5 * _mass * cloth.velocityOf(i).length2 - _mass * spec.gravity.dot(p);
  }
  return total;
}

void main() {
  group('hanging', () {
    test('a curtain pinned at two corners settles with little stretch', () {
      final cloth = _hung();
      _run(cloth, 5, 1 / 60);
      expect(cloth.maxStretch, lessThan(0.05));
      expect(cloth.maxSpeed, lessThan(0.01));
    });

    test(
      'a curtain pinned along its top edge stretches less at the default',
      () {
        final cloth = _curtain();
        for (var i = 0; i < _columns; i++) {
          cloth.pin(i);
        }
        _run(cloth, 5, 1 / 60);
        expect(cloth.maxStretch, lessThan(0.05));
        expect(cloth.maxSpeed, lessThan(0.01));
      },
    );

    test('it hangs the same at 30 and 60 frames a second', () {
      final fast = _hung();
      final slow = _hung();
      _run(fast, 5, 1 / 60);
      _run(slow, 5, 1 / 30);
      expect((fast.maxStretch - slow.maxStretch).abs(), lessThan(0.01));
      final low = _columns * _columns - _columns ~/ 2;
      final gap = fast.positionOf(low).distanceTo(slow.positionOf(low));
      expect(gap, lessThan(0.02));
    });

    test('a stiffer link stretches less', () {
      final soft = _hung(spec: _corners(compliance: 1e-3));
      final stiff = _hung(spec: _corners(compliance: 1e-7));
      _run(soft, 3, 1 / 60);
      _run(stiff, 3, 1 / 60);
      expect(soft.maxStretch, greaterThan(stiff.maxStretch));
    });

    test('tighter bend links let a clamped sheet droop less', () {
      double droop(double compliance) {
        final cloth = SoftBody.cloth(
          columns: _columns,
          rows: _columns,
          origin: Vector3(0, 1, 0),
          across: Vector3(_spacing, 0, 0),
          down: Vector3(0, 0, _spacing),
          particleMass: _mass,
          spec: SoftSpec(bendCompliance: compliance),
        );
        // Two columns clamp it. One only lets the sheet swing about its edge.
        for (var j = 0; j < _columns; j++) {
          cloth.pin(j * _columns);
          cloth.pin(j * _columns + 1);
        }
        _run(cloth, 3, 1 / 60);
        // A fold squeezes bend links a long way. That is not stretch.
        expect(cloth.maxStretch, lessThan(0.05));
        return 1 - cloth.positionOf(_columns - 1).y;
      }

      expect(droop(1e-5), lessThan(droop(10) - 0.02));
    });
  });

  group('energy', () {
    test('a pinned curtain never gains energy', () {
      final spec = SoftSpec();
      final cloth = _hung(spec: spec);
      final start = _energy(cloth, spec);
      var worst = double.negativeInfinity;
      for (var n = 0; n < 300; n++) {
        cloth.step(1 / 60);
        worst = math.max(worst, _energy(cloth, spec) - start);
      }
      expect(worst, lessThan(0.01 * start.abs()));
    });

    test('a free sheet falls as a body does', () {
      final cloth = _curtain(spec: SoftSpec(damping: 0));
      final top = cloth.positionOf(0).y;
      _run(cloth, 0.5, 1 / 60);
      final dropped = top - cloth.positionOf(0).y;
      expect(dropped, closeTo(0.5 * 9.81 * 0.25, 0.02));
    });
  });

  group('rest', () {
    test('a body that has come to rest stops being stepped', () {
      final cloth = _hung();
      _run(cloth, 10, 1 / 60);
      expect(cloth.asleep, isTrue);
    });

    test('moving a pin wakes it', () {
      final cloth = _hung();
      _run(cloth, 10, 1 / 60);
      cloth.place(0, cloth.positionOf(0) + Vector3(0.1, 0, 0));
      expect(cloth.asleep, isFalse);
      final low = _columns * _columns - 1;
      final before = cloth.positionOf(low);
      _run(cloth, 1, 1 / 60);
      expect(cloth.positionOf(low), isNot(before));
    });
  });

  group('pins', () {
    test('a pinned particle stays and a moved one is followed', () {
      final cloth = _hung();
      final held = cloth.positionOf(0);
      _run(cloth, 1, 1 / 60);
      expect(cloth.positionOf(0), held);
      cloth.place(0, held + Vector3(0, 0, 0.2));
      expect(cloth.positionOf(0).z, closeTo(0.2, 1e-12));
    });

    test('a released particle is free again', () {
      final cloth = _curtain();
      cloth.pin(0);
      cloth.release(0);
      expect(cloth.isPinned(0), isFalse);
    });

    test('placing a free particle is refused', () {
      expect(() => _curtain().place(3, Vector3.zero()), throwsStateError);
    });
  });

  group('rope', () {
    test('a rope hung from one end swings down and hangs straight', () {
      final rope = SoftBody.rope(
        points: [for (var i = 0; i < 20; i++) Vector3(i * 0.05, 1, 0)],
      );
      rope.pin(0);
      _run(rope, 8, 1 / 60);
      final tip = rope.positionOf(19);
      expect(tip.x, closeTo(0, 0.05));
      expect(tip.y, closeTo(1 - 19 * 0.05, 0.05));
    });
  });

  group('refusals', () {
    test('a spec outside its range', () {
      expect(() => SoftSpec(damping: -1), throwsArgumentError);
      expect(() => SoftSpec(stretchCompliance: -1), throwsArgumentError);
      expect(() => SoftSpec(bendCompliance: -1), throwsArgumentError);
      expect(() => SoftSpec(substepLength: 0), throwsArgumentError);
      expect(() => SoftSpec(substepLength: 0.1), throwsArgumentError);
      expect(() => SoftSpec(iterations: 0), throwsArgumentError);
    });

    test('a cloth or rope too small to hold together', () {
      Vector3 v() => Vector3(1, 0, 0);
      expect(
        () => SoftBody.cloth(
          columns: 1,
          rows: 4,
          origin: Vector3.zero(),
          across: v(),
          down: v(),
        ),
        throwsArgumentError,
      );
      expect(
        () => SoftBody.rope(points: [Vector3.zero()]),
        throwsArgumentError,
      );
      expect(
        () => SoftBody.rope(points: [Vector3.zero(), v()], particleMass: 0),
        throwsArgumentError,
      );
    });

    test('a step that goes nowhere', () {
      expect(() => _curtain().step(0), throwsArgumentError);
      expect(() => _curtain().step(double.nan), throwsArgumentError);
    });
  });
}
