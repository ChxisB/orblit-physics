import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_soft/orblit_physics_soft.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _columns = 20;
const _spacing = 0.05;
const _mass = 0.01;
const _dt = 1 / 60;
const _weight = _columns * _columns * _mass * 9.81;

/// A curtain in the x-y plane with its top row at y = 2, on the finer
/// sub-step a two-corner hang needs.
SoftBody _curtain() => SoftBody.cloth(
  columns: _columns,
  rows: _columns,
  origin: Vector3(0, 2, 0),
  across: Vector3(_spacing, 0, 0),
  down: Vector3(0, -_spacing, 0),
  particleMass: _mass,
  spec: SoftSpec(substepLength: 1 / 960),
);

/// A box in the air, held in place or driven, to hang a cloth from.
Physics _rig({PhysicsMotion motion = PhysicsMotion.driven}) {
  final physics = Physics();
  physics.add(
    1,
    shape: const Shape.box(0.5, 0.1, 0.5),
    motion: motion,
    at: [0.5, 2, 0],
  );
  physics.step(_dt);
  return physics;
}

void _run(SoftBody cloth, double seconds) {
  for (var n = 0; n < (seconds / _dt).round(); n++) {
    cloth.step(_dt);
  }
}

/// Both top corners of `cloth` attached to points on body 1, one `_columns`
/// spacings apart, with the same `limit`.
void _hang(SoftBody cloth, Physics physics, {double? limit}) {
  cloth.attach(0, bodyPoint(physics, 1, Vector3(-0.5, 0, 0)), limit: limit);
  cloth.attach(
    _columns - 1,
    bodyPoint(physics, 1, Vector3(-0.5 + (_columns - 1) * _spacing, 0, 0)),
    limit: limit,
  );
}

void main() {
  group('pull', () {
    test('two corners carry the weight between them', () {
      final physics = _rig(motion: PhysicsMotion.fixed);
      final cloth = _curtain();
      _hang(cloth, physics);
      _run(cloth, 5);
      final total = cloth.pullOf(0) + cloth.pullOf(_columns - 1);
      expect(total.y, closeTo(-_weight, 0.05 * _weight));
      expect(total.x.abs(), lessThan(0.05 * _weight));
      expect(cloth.pullOf(0).x, greaterThan(0));
      expect(cloth.pullOf(_columns - 1).x, lessThan(0));
    });

    test('a body at rest holds its load while it sleeps', () {
      final physics = _rig(motion: PhysicsMotion.fixed);
      final cloth = _curtain();
      _hang(cloth, physics);
      _run(cloth, 12);
      expect(cloth.asleep, isTrue);
      expect(cloth.pullOf(0).y + cloth.pullOf(_columns - 1).y, lessThan(-30));
    });

    test('a limit above the pull holds', () {
      final physics = _rig(motion: PhysicsMotion.fixed);
      final cloth = _curtain();
      // The first swing puts about 41 N on a corner. It settles at 22 N.
      _hang(cloth, physics, limit: 50);
      _run(cloth, 3);
      expect(cloth.isAttached(0), isTrue);
      expect(cloth.isAttached(_columns - 1), isTrue);
    });

    test('a limit below the pull lets go, and the rest then fall to it', () {
      final physics = _rig(motion: PhysicsMotion.fixed);
      final cloth = _curtain();
      cloth.attach(0, bodyPoint(physics, 1, Vector3(-0.5, 0, 0)), limit: 5);
      cloth.attach(_columns - 1, bodyPoint(physics, 1, Vector3(0.45, 0, 0)));
      _run(cloth, 3);
      expect(cloth.isAttached(0), isFalse);
      expect(cloth.isPinned(0), isFalse);
      expect(cloth.isAttached(_columns - 1), isTrue);
      expect(cloth.positionOf(0).y, lessThan(1.5));
    });
  });

  group('following', () {
    test('a cloth on a moving body goes where it goes', () {
      final physics = _rig();
      final cloth = _curtain();
      _hang(cloth, physics);
      final start = _centreX(cloth);
      for (var n = 0; n < 120; n++) {
        physics.drive(1, velocity: [1, 0, 0]);
        physics.step(_dt);
        cloth.step(_dt);
      }
      final pose = physics.transformOf(1)!;
      expect(pose[0], closeTo(2.5, 0.05));
      final corner = cloth.positionOf(0);
      expect(corner.x, closeTo(pose[0] - 0.5, 1e-9));
      expect(_centreX(cloth) - start, closeTo(pose[0] - 0.5, 0.15));
    });

    test('a body that moves wakes a sleeping cloth', () {
      final physics = _rig();
      final cloth = _curtain();
      _hang(cloth, physics);
      _run(cloth, 12);
      expect(cloth.asleep, isTrue);
      physics.place(1, at: [0.5, 2, 0.5]);
      physics.step(_dt);
      cloth.step(_dt);
      expect(cloth.asleep, isFalse);
      expect(cloth.positionOf(0).z, closeTo(0.5, 1e-9));
    });

    test('a point on a turned body turns with it', () {
      final physics = Physics();
      final half = 15 * math.pi / 180;
      physics.add(
        1,
        shape: const Shape.box(0.5, 0.5, 0.5),
        motion: PhysicsMotion.fixed,
        at: [1, 2, 3],
        rotation: [0, math.sin(half), 0, math.cos(half)],
      );
      physics.step(_dt);
      final point = bodyPoint(physics, 1, Vector3(0.5, 0, 0))();
      final turn = 30 * math.pi / 180;
      expect(point.x, closeTo(1 + 0.5 * math.cos(turn), 1e-5));
      expect(point.z, closeTo(3 - 0.5 * math.sin(turn), 1e-5));
      expect(point.y, closeTo(2, 1e-5));
    });

    test('a point on a body that is gone is refused', () {
      final physics = _rig();
      final where = bodyPoint(physics, 1, Vector3.zero());
      physics.remove(1);
      physics.step(_dt);
      expect(where, throwsStateError);
    });
  });

  group('refusals', () {
    test('an attach outside the body, or with a limit of nothing', () {
      final cloth = _curtain();
      Vector3 here() => Vector3.zero();
      expect(() => cloth.attach(-1, here), throwsRangeError);
      expect(() => cloth.attach(cloth.count, here), throwsRangeError);
      expect(() => cloth.attach(0, here, limit: 0), throwsArgumentError);
      expect(
        () => cloth.attach(0, here, limit: double.nan),
        throwsArgumentError,
      );
    });

    test('the pull on a particle that is not attached', () {
      expect(() => _curtain().pullOf(0), throwsStateError);
    });

    test('detach frees an attached particle and leaves a pinned one', () {
      final cloth = _curtain();
      cloth.attach(0, () => Vector3(0, 2, 0));
      cloth.pin(1);
      cloth.detach(0);
      cloth.detach(1);
      expect(cloth.isAttached(0), isFalse);
      expect(cloth.isPinned(0), isFalse);
      expect(cloth.isPinned(1), isTrue);
    });
  });
}

double _centreX(SoftBody cloth) {
  var total = 0.0;
  for (var i = 0; i < cloth.count; i++) {
    total += cloth.positionOf(i).x;
  }
  return total / cloth.count;
}
