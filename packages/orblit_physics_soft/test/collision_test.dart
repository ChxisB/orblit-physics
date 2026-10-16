import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_soft/orblit_physics_soft.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _columns = 20;
const _spacing = 0.05;
const _dt = 1 / 60;
const _skin = 0.01;

/// A world with a floor whose top is at y = 0, and whatever `fill` adds.
Physics _world({void Function(Physics)? fill, double floor = 1}) {
  final physics = Physics();
  physics.add(
    1,
    shape: Shape.box(5, floor, 5),
    motion: PhysicsMotion.fixed,
    at: [0, -floor, 0],
  );
  fill?.call(physics);
  physics.step(_dt);
  return physics;
}

/// A flat 20 by 20 sheet with its corner at `origin`, its sides along x and z.
SoftBody _sheet(
  Physics physics,
  Vector3 origin, {
  SoftSpec? spec,
  Vector3? across,
  Vector3? down,
}) => SoftBody.cloth(
  columns: _columns,
  rows: _columns,
  origin: origin,
  across: across ?? Vector3(_spacing, 0, 0),
  down: down ?? Vector3(0, 0, _spacing),
  spec: spec,
  surface: PhysicsSurface(physics),
);

/// Runs the world and then the cloth, as a game does, and calls `each` after.
void _run(SoftBody cloth, double seconds, {void Function()? each}) {
  for (var n = 0; n < (seconds / _dt).round(); n++) {
    cloth.step(_dt);
    each?.call();
  }
}

Iterable<Vector3> _points(SoftBody cloth) sync* {
  for (var i = 0; i < cloth.count; i++) {
    yield cloth.positionOf(i);
  }
}

Vector3 _centre(SoftBody cloth) {
  final total = Vector3.zero();
  for (final p in _points(cloth)) {
    total.add(p);
  }
  return total / cloth.count.toDouble();
}

void main() {
  group('a sphere', () {
    final middle = Vector3(0.475, 0.25, 0.475);

    SoftBody drop() {
      final physics = _world(
        fill: (w) => w.add(
          2,
          shape: const Shape.sphere(0.25),
          motion: PhysicsMotion.fixed,
          at: middle.storage,
        ),
      );
      return _sheet(physics, Vector3(0, 0.8, 0));
    }

    test('catches a cloth, which drapes over it and never ends inside', () {
      final cloth = drop();
      _run(cloth, 5);
      for (final p in _points(cloth)) {
        expect(p.distanceTo(middle), greaterThan(0.25 + _skin - 0.002));
        expect(p.y, greaterThan(_skin - 0.002));
      }
      final top = cloth.positionOf(_columns ~/ 2 * _columns + _columns ~/ 2);
      expect(top.y, closeTo(0.5 + _skin, 0.01));
      expect(cloth.maxSpeed, lessThan(0.05));
    });

    test('is not entered between the particles that touch it', () {
      final cloth = drop();
      _run(cloth, 5);
      for (var j = 0; j < _columns; j++) {
        for (var i = 0; i + 1 < _columns; i++) {
          final a = cloth.positionOf(j * _columns + i);
          final b = cloth.positionOf(j * _columns + i + 1);
          final mid = (a + b) * 0.5;
          expect(mid.distanceTo(middle), greaterThan(0.25 - 0.003));
        }
      }
    });
  });

  test('a box catches a cloth, which hangs over its edges', () {
    const half = 0.2;
    final centre = Vector3(0.475, half, 0.475);
    final physics = _world(
      fill: (w) => w.add(
        2,
        shape: const Shape.box(half, half, half),
        motion: PhysicsMotion.fixed,
        at: centre.storage,
      ),
    );
    final cloth = _sheet(physics, Vector3(0, 0.8, 0));
    _run(cloth, 5);
    for (final p in _points(cloth)) {
      final inside = [
        for (var c = 0; c < 3; c++)
          (p[c] - centre[c]).abs() < half + _skin - 0.002,
      ];
      expect(inside.every((v) => v), isFalse, reason: '$p is inside the box');
      expect(p.y, greaterThan(_skin - 0.002));
    }
    final middle = cloth.positionOf(_columns ~/ 2 * _columns + _columns ~/ 2);
    expect(middle.y, closeTo(2 * half + _skin, 0.01));
    final corner = cloth.positionOf(0);
    expect(corner.y, lessThan(2 * half));
  });

  test('a cloth dropped from a height does not go through a thin floor', () {
    final cloth = _sheet(
      _world(floor: 0.05),
      Vector3(0, 20, 0),
      spec: SoftSpec(damping: 0),
    );
    var fastest = 0.0;
    _run(
      cloth,
      4,
      each: () {
        fastest = math.max(fastest, cloth.maxSpeed);
        for (final p in _points(cloth)) {
          expect(p.y, greaterThan(_skin - 0.002));
        }
      },
    );
    // It was going fast enough to cross a skin many times over in one step.
    expect(fastest, greaterThan(10));
    expect(cloth.maxSpeed, lessThan(0.05));
  });

  test('a cloth landing flat stops dead and does not bounce', () {
    final cloth = _sheet(
      _world(),
      Vector3(0, 5, 0),
      spec: SoftSpec(damping: 0),
    );
    // It lands at 1.01 s, at 9.9 m/s.
    _run(cloth, 1.2);
    expect(cloth.maxSpeed, lessThan(1));
    expect(cloth.positionOf(0).y, lessThan(0.03));
  });

  test('a body that begins inside is pushed out without being thrown', () {
    final physics = _world(
      fill: (w) => w.add(
        2,
        shape: const Shape.box(2, 0.3, 2),
        motion: PhysicsMotion.fixed,
        at: [0, 0.8, 0],
      ),
    );
    final cloth = _sheet(physics, Vector3(0, 1.0, 0));
    // It is 10 cm deep. A skin a sub-step is all it may be moved, so it
    // cannot have come out in the first step, which has 8 sub-steps.
    cloth.step(_dt);
    final middle = _columns ~/ 2 * _columns + _columns ~/ 2;
    expect(cloth.positionOf(middle).y, lessThan(1.0 + 8 * _skin + 0.001));
    var fastest = 0.0;
    _run(cloth, 1, each: () => fastest = math.max(fastest, cloth.maxSpeed));
    expect(fastest, lessThan(8));
    for (final p in _points(cloth)) {
      expect(p.y, greaterThan(1.1 + _skin - 0.002));
    }
  });

  test('a pinned particle is not moved by a surface', () {
    final physics = _world();
    final cloth = _sheet(physics, Vector3(0, 0.005, 0));
    cloth.pin(0);
    final held = cloth.positionOf(0);
    _run(cloth, 1);
    expect(cloth.positionOf(0), held);
  });

  group('friction', () {
    const angle = 20 * math.pi / 180;
    final along = Vector3(math.cos(angle), math.sin(angle), 0);
    final normal = Vector3(-math.sin(angle), math.cos(angle), 0);

    /// A sheet lying on a ramp that rises towards +x, and how far down the
    /// ramp its middle goes in `seconds`.
    double slide(double friction, double seconds) {
      final physics = Physics();
      physics.add(
        1,
        shape: const Shape.box(3, 0.1, 3),
        motion: PhysicsMotion.fixed,
        rotation: [0, 0, math.sin(angle / 2), math.cos(angle / 2)],
      );
      physics.step(_dt);
      final cloth = _sheet(
        physics,
        normal * (0.1 + _skin + 0.001) - along * 0.475 - Vector3(0, 0, 0.475),
        spec: SoftSpec(friction: friction),
        across: along * _spacing,
      );
      final start = _centre(cloth);
      _run(cloth, seconds);
      return (start - _centre(cloth)).dot(along);
    }

    test('holds a cloth on a slope and lets go of it without any', () {
      expect(slide(2.0, 2), lessThan(0.01));
      expect(slide(0.05, 1), greaterThan(0.5));
    });
  });

  test('400 particles with a surface cost under a few milliseconds a step', () {
    final physics = _world(
      fill: (w) => w.add(
        2,
        shape: const Shape.sphere(0.25),
        motion: PhysicsMotion.fixed,
        at: [0.475, 0.25, 0.475],
      ),
    );
    final cloth = _sheet(physics, Vector3(0, 0.55, 0));
    _run(cloth, 0.3);
    final clock = Stopwatch()..start();
    _run(cloth, 1);
    final each = clock.elapsedMicroseconds / 60 / 1000;
    // The aim is 1 ms. The bound only catches a step that has got far worse.
    // ignore: avoid_print
    print('400 particles: $each ms a step');
    expect(each, lessThan(5));
  });
}
