import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:test/test.dart';

void main() {
  late Physics physics;
  setUp(() => physics = Physics());
  tearDown(() => physics.dispose());

  List<ShapePart> pillars() => [
    ShapePart(shape: const Shape.box(0.5, 1, 0.5), at: [-2, 0, 0]),
    ShapePart(shape: const Shape.box(0.5, 1, 0.5), at: [2, 0, 0]),
  ];

  void run() {
    for (var i = 0; i < 240; i++) {
      physics.step(1 / 60);
    }
  }

  test('rays and overlaps see parts and the empty space between them', () {
    expect(physics.layCompound(7, parts: pillars()), isTrue);
    physics.add(1, shape: const Shape.compound(7), motion: PhysicsMotion.fixed);
    expect(
      physics.cast(from: [0, 0, 3], direction: [0, 0, -1], distance: 6),
      isNull,
    );
    final hit = physics.cast(
      from: [2, 0, 3],
      direction: [0, 0, -1],
      distance: 6,
    )!;
    expect(hit.body, 1);
    expect(hit.distance, closeTo(2.5, 0.001));
    expect(hit.normal[2], closeTo(1, 0.001));
    expect(physics.overlap(at: [0, 0, 0]), isEmpty);
    expect(physics.overlap(at: [2, 0, 0]), [1]);
    expect(
      physics
          .castAll(from: [-4, 0, 0], direction: [1, 0, 0], distance: 8)
          .length,
      1,
    );
  });

  test(
    'a compound cast finds the earliest child, including an initial overlap',
    () {
      physics.layCompound(7, parts: pillars());
      physics.add(
        1,
        shape: const Shape.sphere(0.25),
        motion: PhysicsMotion.fixed,
      );
      final hit = physics.cast(
        from: [-4, 0, 0],
        direction: [1, 0, 0],
        distance: 8,
        shape: const Shape.compound(7),
      )!;
      expect(hit.distance, closeTo(1.25, 0.001));
      expect(
        physics
            .cast(
              from: [-2, 0, 0],
              direction: [1, 0, 0],
              distance: 1,
              shape: const Shape.compound(7),
            )!
            .started,
        isTrue,
      );
    },
  );

  test(
    'a ball falls through a compound gap while the compound rests on both parts',
    () {
      physics.add(
        1,
        shape: const Shape.plane(0, 1, 0),
        motion: PhysicsMotion.fixed,
      );
      physics.layCompound(7, parts: pillars());
      physics.add(2, shape: const Shape.compound(7), at: [0, 4, 0]);
      physics.add(3, shape: const Shape.sphere(0.3), at: [0, 3, 0]);
      run();
      expect(physics.transformOf(2)![1], closeTo(1, 0.03));
      expect(physics.transformOf(3)![1], closeTo(0.3, 0.03));
      expect(physics.asleep(2), isTrue);
    },
  );

  test(
    'nonuniform scaling makes an ellipsoid with exact bounds and ray distances',
    () {
      physics.layCompound(
        7,
        parts: [
          ShapePart(shape: const Shape.sphere(1), scale: [2, 1, 0.5]),
        ],
      );
      physics.add(
        1,
        shape: const Shape.compound(7),
        motion: PhysicsMotion.fixed,
      );
      final x = physics.cast(
        from: [4, 0, 0],
        direction: [-1, 0, 0],
        distance: 5,
      )!;
      final z = physics.cast(
        from: [0, 0, 4],
        direction: [0, 0, -1],
        distance: 5,
      )!;
      expect(x.distance, closeTo(2, 0.002));
      expect(z.distance, closeTo(3.5, 0.002));
      expect(physics.overlap(at: [0, 0, 1]), isEmpty);
      physics.add(
        2,
        shape: const Shape.sphere(0.2),
        at: [0, 0, 0.6],
        motion: PhysicsMotion.fixed,
      );
      expect(
        physics.overlap(at: [0, 0, 0], shape: const Shape.compound(7)),
        contains(2),
      );
    },
  );

  test('outer scaling preserves a rotated child and its contact normal', () {
    final angle = math.pi / 4;
    physics.layCompound(
      7,
      parts: [
        ShapePart(
          shape: const Shape.box(1, 0.1, 0.2),
          rotation: [0, 0, math.sin(angle / 2), math.cos(angle / 2)],
        ),
      ],
      scale: [2, 1, 1],
    );
    physics.add(1, shape: const Shape.compound(7), motion: PhysicsMotion.fixed);
    final hit = physics.cast(
      from: [0, 3, 0],
      direction: [0, -1, 0],
      distance: 6,
    )!;
    expect(hit.normal[0], closeTo(-1 / math.sqrt(5), 0.01));
    expect(hit.normal[1], closeTo(2 / math.sqrt(5), 0.01));
  });

  test(
    'a push through a translated shape centre causes no spin or origin jump',
    () {
      physics.layCompound(
        7,
        parts: [
          ShapePart(shape: const Shape.box(0.5, 0.5, 0.5), at: [2, 0, 0]),
        ],
      );
      physics.add(1, shape: const Shape.compound(7), at: [0, 4, 0]);
      physics.push(1, impulse: [0, 0, 1], at: [2, 4, 0]);
      physics.step(1 / 60);
      expect(
        physics.velocityOf(1)!.skip(3).every((v) => v.abs() < 1e-6),
        isTrue,
      );
      expect(physics.transformOf(1)![0], closeTo(0, 1e-6));
    },
  );

  test('asset ownership and snapshots retain compound hulls', () {
    final corners = [
      for (final x in [-0.5, 0.5])
        for (final y in [-0.5, 0.5])
          for (final z in [-0.5, 0.5]) ...[x, y, z],
    ];
    physics.layHull(3, points: corners);
    expect(
      physics.layCompound(7, parts: [ShapePart(shape: const Shape.hull(3))]),
      isTrue,
    );
    physics.add(1, shape: const Shape.compound(7), motion: PhysicsMotion.fixed);
    expect(physics.dropHull(3), isFalse);
    expect(physics.dropCompound(7), isFalse);
    final snapshot = physics.snapshot();
    physics.remove(1);
    expect(physics.dropCompound(7), isTrue);
    expect(physics.dropHull(3), isTrue);
    final other = Physics();
    try {
      other.restore(snapshot);
      snapshot.dispose();
      expect(other.overlap(at: [0, 0, 0]), [1]);
      other.add(
        2,
        shape: const Shape.compound(7),
        at: [3, 0, 0],
        motion: PhysicsMotion.fixed,
      );
      expect(other.overlap(at: [3, 0, 0]), [2]);
    } finally {
      other.dispose();
      if (!snapshot.disposed) snapshot.dispose();
    }
  });

  test('restoring a moving compound reproduces its poses and velocities', () {
    physics.add(
      1,
      shape: const Shape.plane(0, 1, 0),
      motion: PhysicsMotion.fixed,
    );
    physics.layCompound(7, parts: pillars());
    physics.add(2, shape: const Shape.compound(7), at: [0, 4, 0]);
    for (var i = 0; i < 50; i++) {
      physics.step(1 / 60);
    }
    final saved = physics.snapshot();
    try {
      for (var i = 0; i < 50; i++) {
        physics.step(1 / 60);
      }
      final pose = physics.transformOf(2)!.toList();
      final velocity = physics.velocityOf(2)!.toList();
      physics.restore(saved);
      for (var i = 0; i < 50; i++) {
        physics.step(1 / 60);
      }
      expect(physics.transformOf(2), pose);
      expect(physics.velocityOf(2), velocity);
    } finally {
      saved.dispose();
    }
  });

  test(
    'scaled capsules and cylinders keep their curved shape and flat ends',
    () {
      physics.layCompound(
        7,
        parts: [
          ShapePart(shape: const Shape.capsule(0.25, 0.5), scale: [2, 1, 0.5]),
        ],
      );
      physics.layCompound(
        8,
        parts: [
          ShapePart(shape: const Shape.cylinder(0.5, 1), scale: [2, 1, 0.5]),
        ],
      );
      physics.add(
        1,
        shape: const Shape.compound(7),
        motion: PhysicsMotion.fixed,
      );
      physics.add(
        2,
        shape: const Shape.compound(8),
        at: [4, 0, 0],
        motion: PhysicsMotion.fixed,
      );
      expect(
        physics
            .cast(from: [0, 3, 0], direction: [0, -1, 0], distance: 5)!
            .distance,
        closeTo(2.25, 0.002),
      );
      final cap = physics.cast(
        from: [4, 3, 0],
        direction: [0, -1, 0],
        distance: 5,
      )!;
      expect(cap.distance, closeTo(2, 0.002));
      expect(cap.normal[1], closeTo(1, 0.002));
    },
  );

  test('scaled compound geometry rests on a height field', () {
    physics.layGround(
      1,
      heights: [0, 0, 0, 0],
      columns: 2,
      rows: 2,
      spacing: 20,
      at: [-10, 0, -10],
    );
    physics.layCompound(
      7,
      parts: [
        ShapePart(shape: const Shape.sphere(1), scale: [2, 1, 0.5]),
      ],
    );
    physics.add(2, shape: const Shape.compound(7), at: [0, 3, 0]);
    run();
    expect(physics.transformOf(2)![1], closeTo(1, 0.03));
    expect(physics.velocityOf(2)![1].abs(), lessThan(0.01));
  });

  test('compound triggers report their parts while gaps stay outside', () {
    physics.layCompound(7, parts: pillars());
    physics.add(
      1,
      shape: const Shape.compound(7),
      motion: PhysicsMotion.fixed,
      trigger: true,
    );
    physics.add(2, shape: const Shape.sphere(0.2), at: [0, 0, 0]);
    physics.step(1 / 60);
    expect(
      physics.events.where((e) => e.kind == PhysicsEventKind.entered),
      isEmpty,
    );
    physics.place(2, at: [2, 0, 0]);
    physics.step(1 / 60);
    final entered = physics.events
        .where((e) => e.kind == PhysicsEventKind.entered)
        .single;
    expect((entered.a, entered.b), (1, 2));
    expect(physics.overlap(at: [2, 0, 0], triggers: true), [1]);
    expect(physics.contacts, isEmpty);
  });

  test('invalid parts are refused before registration', () {
    expect(physics.layCompound(0, parts: pillars()), isFalse);
    expect(physics.layCompound(7, parts: []), isFalse);
    expect(
      physics.layCompound(
        7,
        parts: [ShapePart(shape: const Shape.box(0, 1, 1))],
      ),
      isFalse,
    );
    expect(
      physics.layCompound(7, parts: [ShapePart(shape: const Shape.hull(99))]),
      isFalse,
    );
    expect(
      () => ShapePart(shape: const Shape.plane(0, 1, 0)),
      throwsArgumentError,
    );
    expect(
      () => ShapePart(shape: const Shape.sphere(1), scale: [1, 0, 1]),
      throwsArgumentError,
    );
    expect(physics.layCompound(7, parts: pillars()), isTrue);
    expect(physics.layCompound(7, parts: pillars()), isFalse);
  });
}
