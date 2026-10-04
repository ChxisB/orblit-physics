import 'package:orblit_physics/orblit_physics.dart';
import 'package:test/test.dart';

const floor = [
  -10.0,
  0.0,
  -10.0,
  10.0,
  0.0,
  -10.0,
  10.0,
  0.0,
  10.0,
  -10.0,
  0.0,
  10.0,
];
const triangles = [0, 2, 1, 0, 3, 2];

void main() {
  late Physics physics;
  setUp(() => physics = Physics());
  tearDown(() => physics.dispose());

  void lay() {
    expect(physics.layMesh(7, vertices: floor, indices: triangles), isTrue);
    physics.add(1, shape: const Shape.mesh(7), motion: PhysicsMotion.fixed);
  }

  test('mesh rays meet both sides and preserve the finite footprint', () {
    lay();
    for (final side in [-1.0, 1.0]) {
      final hit = physics.cast(
        from: [0, side * 3, 0],
        direction: [0, -side, 0],
        distance: 6,
      )!;
      expect(hit.distance, closeTo(3, 0.002));
      expect(hit.normal[1], closeTo(side, 0.002));
    }
    expect(
      physics.cast(from: [11, 3, 0], direction: [0, -1, 0], distance: 6),
      isNull,
    );
  });

  test('box, cylinder and hull settle across triangle seams', () {
    lay();
    physics.layHull(
      3,
      points: [
        for (final x in [-0.5, 0.5])
          for (final y in [-0.5, 0.5])
            for (final z in [-0.5, 0.5]) ...[x, y, z],
      ],
    );
    final shapes = [
      const Shape.box(0.5, 0.5, 0.5),
      const Shape.cylinder(0.5, 0.5),
      const Shape.hull(3),
    ];
    for (var i = 0; i < shapes.length; i++) {
      physics.add(i + 2, shape: shapes[i], at: [i * 2.0, 3, i * 2.0]);
    }
    for (var step = 0; step < 360; step++) {
      physics.step(1 / 60);
    }
    for (var i = 0; i < shapes.length; i++) {
      expect(physics.transformOf(i + 2)![1], closeTo(0.5, 0.025));
      expect(physics.asleep(i + 2), isTrue);
    }
  });

  test('a ball crossing a welded seam gets no sideways impulse or bounce', () {
    expect(
      physics.layMesh(
        7,
        vertices: [
          -10,
          0,
          -10,
          10,
          0,
          -10,
          10,
          0,
          10,
          -10,
          0,
          -10,
          -10,
          0,
          10,
          10,
          0,
          10,
        ],
        indices: [0, 2, 1, 3, 4, 5],
      ),
      isTrue,
    );
    physics.add(
      1,
      shape: const Shape.mesh(7),
      motion: PhysicsMotion.fixed,
      friction: 0,
    );
    physics.add(
      2,
      shape: const Shape.sphere(0.5),
      at: [-3, 0.5, 0],
      friction: 0,
      linearDamping: 0,
    );
    physics.drive(2, velocity: [2, 0, 0]);
    for (var step = 0; step < 180; step++) {
      physics.step(1 / 60);
      expect(physics.transformOf(2)![1], closeTo(0.5, 0.015));
      expect(physics.velocityOf(2)![0], closeTo(2, 0.01));
    }
  });

  test('convex casts and overlaps meet a rotated sheet from either side', () {
    expect(physics.layMesh(7, vertices: floor, indices: triangles), isTrue);
    physics.add(
      1,
      shape: const Shape.mesh(7),
      motion: PhysicsMotion.fixed,
      at: [2, 0, 0],
      rotation: [0, 0, -0.7071067812, 0.7071067812],
    );
    for (final side in [-1.0, 1.0]) {
      final hit = physics.cast(
        from: [2 + side * 3, 0, 0],
        direction: [-side, 0, 0],
        distance: 6,
        shape: const Shape.sphere(0.5),
      )!;
      expect(hit.distance, closeTo(2.5, 0.003));
      expect(hit.normal[0], closeTo(side, 0.003));
    }
    expect(
      physics.overlap(at: [2.2, 0, 0], shape: const Shape.box(0.5, 0.5, 0.5)),
      [1],
    );
    final inside = physics.cast(
      from: [2.2, 0, 0],
      direction: [1, 0, 0],
      distance: 1,
      shape: const Shape.sphere(0.5),
    )!;
    expect(inside.started, isTrue);
    expect(inside.distance, 0);
  });

  test('the gap between separated triangle patches stays empty', () {
    expect(
      physics.layMesh(
        7,
        vertices: [-3, 0, -1, -1, 0, -1, -2, 0, 1, 1, 0, -1, 3, 0, -1, 2, 0, 1],
        indices: [0, 2, 1, 3, 5, 4],
      ),
      isTrue,
    );
    physics.add(1, shape: const Shape.mesh(7), motion: PhysicsMotion.fixed);
    expect(
      physics.cast(from: [0, 2, 0], direction: [0, -1, 0], distance: 4),
      isNull,
    );
    expect(
      physics.overlap(at: [0, 0, 0], shape: const Shape.sphere(0.5)),
      isEmpty,
    );
    expect(
      physics.cast(from: [-2, 2, 0], direction: [0, -1, 0], distance: 4),
      isNotNull,
    );
  });

  test('characters and scaled compounds use the mesh collision path', () {
    lay();
    physics.addCharacter(2, at: [-3, 0.91, 0]);
    for (var step = 0; step < 180; step++) {
      final footing = physics.footingOf(2)!;
      physics.drive(2, velocity: [2, footing.velocity[1] - 9.81 / 60, 0]);
      physics.step(1 / 60);
      expect(physics.footingOf(2)!.grounded, isTrue);
      expect(physics.transformOf(2)![1], closeTo(0.91, 0.02));
    }
    expect(physics.transformOf(2)![0], closeTo(3, 0.02));
    expect(
      physics.layCompound(
        8,
        parts: [
          ShapePart(shape: const Shape.sphere(0.5), scale: [2, 1, 1]),
        ],
      ),
      isTrue,
    );
    physics.add(3, shape: const Shape.compound(8), at: [0, 3, 3]);
    for (var step = 0; step < 360; step++) {
      physics.step(1 / 60);
    }
    expect(physics.transformOf(3)![1], closeTo(0.5, 0.025));
    expect(physics.asleep(3), isTrue);
  });

  test('mesh ownership survives restoration into another world', () {
    lay();
    expect(physics.dropMesh(7), isFalse);
    final saved = physics.snapshot();
    physics.remove(1);
    expect(physics.dropMesh(7), isTrue);
    final other = Physics();
    try {
      other.restore(saved);
      saved.dispose();
      expect(
        other.cast(from: [0, 2, 0], direction: [0, -1, 0], distance: 3)!.body,
        1,
      );
    } finally {
      other.dispose();
      if (!saved.disposed) saved.dispose();
    }
  });

  test('invalid and empty meshes are refused and mesh bodies stay fixed', () {
    expect(physics.layMesh(7, vertices: floor, indices: [0, 1, 9]), isFalse);
    expect(physics.layMesh(7, vertices: floor, indices: [0, 0, 0]), isFalse);
    lay();
    physics.add(2, shape: const Shape.mesh(7));
    expect(physics.transformOf(2), isNull);
    physics.setMotion(1, PhysicsMotion.free);
    physics.step(1 / 60);
    expect(physics.transformOf(1)![1], 0);
  });
}
