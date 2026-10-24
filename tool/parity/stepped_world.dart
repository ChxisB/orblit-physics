// A wall of boxes hit by a sphere, a character that climbs a ledge and
// pushes a crate, and a chain of joints. Written as the raw bits of every
// body at a few steps. The C# run is `SteppedScenes` in
// `dotnet/Orblit.Physics.Tests`, and the pinned lines are
// `Reference/stepped_world.txt` there. Copy this file to
// `packages/orblit_physics/test/` and run it with `dart test`, then delete
// the copy. See README.md.

import 'dart:typed_data';

import 'package:orblit_physics/orblit_physics.dart';
import 'package:test/test.dart';

const _dt = 1 / 60;

String _f32(Float32List l) => [
  for (final w in Uint32List.view(l.buffer, l.offsetInBytes, l.length))
    w.toRadixString(16).padLeft(8, '0'),
].join(' ');

String _f64(List<double> l) {
  final d = ByteData(8 * l.length);
  for (var i = 0; i < l.length; i++) {
    d.setFloat64(8 * i, l[i], Endian.little);
  }
  return [
    for (var i = 0; i < l.length; i++)
      d.getUint32(8 * i + 4, Endian.little).toRadixString(16).padLeft(8, '0') +
          d.getUint32(8 * i, Endian.little).toRadixString(16).padLeft(8, '0'),
  ].join(' ');
}

void _dump(String scene, int step, Physics p, List<int> ids) {
  for (final id in ids) {
    print(
      '$scene $step $id T ${_f32(p.transformOf(id)!)} V ${_f32(p.velocityOf(id)!)}',
    );
  }
}

void _run(
  String scene,
  Physics p,
  List<int> ids,
  int steps,
  List<int> marks, {
  void Function()? before,
}) {
  for (var s = 1; s <= steps; s++) {
    before?.call();
    p.step(_dt);
    if (marks.contains(s)) _dump(scene, s, p, ids);
  }
}

void main() {
  test('wall', () {
    final p = Physics();
    p.add(
      1,
      shape: const Shape.plane(0.0, 1.0, 0.0),
      motion: PhysicsMotion.fixed,
    );
    final ids = <int>[];
    for (var r = 0; r < 5; r++) {
      for (var c = 0; c < 4; c++) {
        final id = 10 + r * 4 + c;
        ids.add(id);
        p.add(
          id,
          shape: const Shape.box(0.3, 0.2, 0.2),
          at: [(c - 1.5) * 0.62 + (r.isOdd ? 0.31 : 0.0), 0.2 + r * 0.4, 0.0],
          mass: 2.0,
          friction: 0.6,
        );
      }
    }
    ids.add(100);
    p.add(
      100,
      shape: const Shape.sphere(0.35),
      at: const [0.2, 0.9, 6.0],
      mass: 8.0,
    );
    p.push(100, impulse: const [0.0, 0.0, -80.0]);
    _run('wall', p, ids, 240, [30, 60, 120, 240]);
    p.dispose();
  });

  test('character', () {
    final p = Physics();
    p.add(
      1,
      shape: const Shape.plane(0.0, 1.0, 0.0),
      motion: PhysicsMotion.fixed,
    );
    p.add(
      2,
      shape: const Shape.box(1.0, 0.1, 1.0),
      motion: PhysicsMotion.fixed,
      at: const [0.0, 0.1, -3.0],
    );
    p.add(
      3,
      shape: const Shape.box(0.25, 0.25, 0.25),
      at: const [0.25, 0.25, -6.5],
      mass: 1.5,
    );
    p.addCharacter(10, at: const [0.0, 0.91, 0.0]);
    _run(
      'character',
      p,
      [10, 3],
      300,
      [30, 60, 90, 150, 210, 300],
      before: () {
        final left = p.footingOf(10)!.velocity;
        p.drive(10, velocity: [0.1, left[1] - 9.81 / 60, -2.2]);
      },
    );
    final f = p.footingOf(10)!;
    print(
      'character footing ${f.ground} ${f.grounded} ${f.wall} ${f.ceiling} ${_f64(f.velocity)} ${_f64(f.normal)}',
    );
    p.dispose();
  });

  test('chain', () {
    final p = Physics();
    p.add(
      20,
      shape: const Shape.box(0.2, 0.1, 0.2),
      motion: PhysicsMotion.fixed,
      at: const [0.0, 4.0, 0.0],
    );
    final ids = <int>[];
    for (var i = 0; i < 5; i++) {
      final id = 21 + i;
      ids.add(id);
      p.add(
        id,
        shape: const Shape.box(0.05, 0.25, 0.05),
        at: [0.0, 3.75 - i * 0.5, 0.0],
        mass: 1.0,
      );
    }
    p.add(
      40,
      shape: const Shape.box(0.3, 0.05, 0.3),
      at: const [0.0, 1.45, 0.0],
      mass: 2.0,
    );
    ids.add(40);
    for (var i = 0; i < 4; i++) {
      p.join(
        30 + i,
        const Joint.point(),
        a: i == 0 ? 20 : 20 + i,
        b: 21 + i,
        at: [0.0, 4.0 - i * 0.5, 0.0],
      );
    }
    p.join(
      34,
      const Joint.hinge(limit: JointLimit(-0.8, 0.8)),
      a: 24,
      b: 25,
      at: const [0.0, 2.0, 0.0],
    );
    p.join(35, const Joint.fixed(), a: 25, b: 40, at: const [0.0, 1.5, 0.0]);
    p.push(25, impulse: const [3.0, 0.0, 2.0]);
    _run('chain', p, ids, 360, [30, 60, 180, 360]);
    p.dispose();
  });
}
