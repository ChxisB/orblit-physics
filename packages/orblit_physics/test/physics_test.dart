import 'dart:ffi' show sizeOf;
import 'dart:math' show max, min, sqrt;
import 'dart:typed_data';

import 'package:orblit_physics/native.dart' as native;
import 'package:orblit_physics/orblit_physics.dart';
import 'package:test/test.dart';

/// Ground at y = 0, facing up, never moving.
const ground = 1;

void main() {
  late Physics physics;

  setUp(() => physics = Physics());
  tearDown(() => physics.dispose());

  /// Lays the floor every scenario here stands on.
  void floor({double friction = 0.5}) => physics.add(
    ground,
    shape: const Shape.plane(0.0, 1.0, 0.0),
    motion: PhysicsMotion.fixed,
    friction: friction,
  );

  /// Runs `seconds` of simulation at a fixed sixtieth, which is what the
  /// solver is tuned for.
  void run(double seconds) {
    for (var i = 0; i < (seconds * 60).round(); i++) {
      physics.step(1 / 60);
    }
  }

  /// Walks character `id` at `vx, vz` for `seconds`, asking each step for
  /// what it was left with plus a step of gravity: the loop every game with a
  /// character runs. Answers how many steps it spent off the ground.
  int walk(int id, double vx, double vz, double seconds) {
    var airborne = 0;
    for (var i = 0; i < (seconds * 60).round(); i++) {
      final left = physics.footingOf(id)!.velocity;
      physics.drive(id, velocity: [vx, left[1] - 9.81 / 60, vz]);
      physics.step(1 / 60);
      if (!physics.footingOf(id)!.grounded) airborne++;
    }
    return airborne;
  }

  double heightOf(int id) => physics.transformOf(id)![1];

  double fallSpeedOf(int id) => physics.velocityOf(id)![1];

  test('a dropped sphere comes to rest on the ground', () {
    floor();
    physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 3.0, 0.0]);

    run(3);

    // Resting means both things: sitting a radius above the plane, and no
    // longer moving. Either alone can be true of a sphere still falling.
    expect(heightOf(2), closeTo(0.5, 0.03));
    expect(fallSpeedOf(2).abs(), lessThan(0.01));
  });

  test('a resting body goes to sleep and can be woken', () {
    floor();
    physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 1.0, 0.0]);

    run(3);
    expect(physics.asleep(2), isTrue);

    physics.wake(2);
    physics.step(1 / 60);
    expect(physics.asleep(2), isFalse);
  });

  test('a landing reports a touch, and removal ends it', () {
    floor();
    physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 1.0, 0.0]);

    var began = <PhysicsEvent>[];
    for (var i = 0; i < 120 && began.isEmpty; i++) {
      physics.step(1 / 60);
      began = physics.events
          .where((it) => it.kind == PhysicsEventKind.touchBegan)
          .toList();
    }

    expect(began, hasLength(1));
    expect({began.single.a, began.single.b}, {ground, 2});
    expect(began.single.force, greaterThan(0.0));
    expect(began.single.at[1], closeTo(0.0, 0.05));

    physics.remove(2);
    physics.step(1 / 60);
    expect(
      physics.events.map((it) => it.kind),
      contains(PhysicsEventKind.touchEnded),
    );
  });

  test('restitution decides whether a sphere bounces', () {
    floor();
    physics.add(
      2,
      shape: const Shape.sphere(0.5),
      at: const [0.0, 3.0, 0.0],
      restitution: 0.8,
    );
    physics.add(
      3,
      shape: const Shape.sphere(0.5),
      at: const [2.0, 3.0, 0.0],
      restitution: 0.0,
    );

    var highestBouncy = 0.0;
    var highestDead = 0.0;
    for (var i = 0; i < 180; i++) {
      physics.step(1 / 60);
      if (i > 60) {
        highestBouncy = [
          highestBouncy,
          heightOf(2),
        ].reduce((a, b) => a > b ? a : b);
        highestDead = [
          highestDead,
          heightOf(3),
        ].reduce((a, b) => a > b ? a : b);
      }
    }

    expect(highestBouncy, greaterThan(1.0));
    expect(highestDead, lessThan(0.6));
  });

  test('friction stops a sliding box', () {
    floor();
    physics.add(
      2,
      shape: const Shape.box(0.5, 0.5, 0.5),
      at: const [0.0, 0.5, 0.0],
      friction: 0.8,
    );
    physics.push(2, impulse: const [5.0, 0.0, 0.0]);

    run(3);
    expect(physics.velocityOf(2)![0].abs(), lessThan(0.05));
  });

  test('layers decide what a body notices', () {
    // A pair interacts when either one cares about the other, so ignoring
    // something takes both of them: this ground watches only bit one, and the
    // body on bit two watches only bit two.
    physics.add(
      ground,
      shape: const Shape.plane(0.0, 1.0, 0.0),
      motion: PhysicsMotion.fixed,
      layers: const Layers(is_: 1, cares: 1),
    );
    physics.add(
      2,
      shape: const Shape.sphere(0.5),
      at: const [0.0, 2.0, 0.0],
      layers: const Layers(is_: 2, cares: 2),
    );
    physics.add(3, shape: const Shape.sphere(0.5), at: const [2.0, 2.0, 0.0]);

    run(2);
    expect(heightOf(2), lessThan(-0.5), reason: 'should have fallen through');
    expect(heightOf(3), closeTo(0.5, 0.05));
  });

  test('a driven body carries what stands on it', () {
    floor();
    physics.add(
      2,
      shape: const Shape.box(2.0, 0.25, 2.0),
      motion: PhysicsMotion.driven,
      at: const [0.0, 0.25, 0.0],
    );
    physics.add(3, shape: const Shape.sphere(0.5), at: const [0.0, 1.05, 0.0]);

    physics.drive(2, velocity: const [0.0, 0.5, 0.0]);
    run(2);

    expect(heightOf(2), closeTo(1.25, 0.02));
    expect(heightOf(3), greaterThan(1.5));
  });

  test('an off-centre push imparts spin', () {
    floor();
    physics.add(
      2,
      shape: const Shape.box(0.5, 0.5, 0.5),
      at: const [0.0, 5.0, 0.0],
    );

    physics.push(2, impulse: const [0.0, 0.0, 2.0], at: const [0.0, 5.5, 0.0]);
    physics.step(1 / 60);

    // Six floats: linear xyz, then angular xyz. A shove along Z landing above
    // the centre sends the top of the box that way and the bottom the other,
    // which is a turn about X.
    final velocity = physics.velocityOf(2)!;
    expect(velocity[2], closeTo(2.0, 0.01));
    expect(velocity[3], closeTo(6.0, 0.05));
    expect(velocity[4].abs(), lessThan(0.001));
    expect(velocity[5].abs(), lessThan(0.001));
  });

  test('a capsule crosses as a capsule, not as the sphere it resembles', () {
    floor();
    // Standing up, 0.3 of radius with 0.5 of straight either side, so it comes
    // to rest with its middle 0.8 above the floor. A sphere of the same radius
    // would rest at 0.3 — which is exactly what a half height arriving in the
    // wrong size slot would look like.
    physics.add(
      2,
      shape: const Shape.capsule(0.3, 0.5),
      at: const [0.0, 3.0, 0.0],
    );

    run(3);
    expect(heightOf(2), closeTo(0.8, 0.05));
  });

  test('a cast reports what it met, and its structs survive the crossing', () {
    floor();
    physics.add(
      2,
      shape: const Shape.box(0.5, 0.5, 0.5),
      motion: PhysicsMotion.fixed,
      at: const [0.0, 5.0, 0.0],
    );
    // A crate off to the side that watches nothing, for the layer cases.
    physics.add(
      3,
      shape: const Shape.box(0.5, 0.5, 0.5),
      motion: PhysicsMotion.fixed,
      at: const [3.0, 5.0, 0.0],
      layers: const Layers(is_: 2, cares: 0),
    );

    // Not stepped first on purpose: a cast flushes what is waiting, so both
    // crates are already there to be hit.
    final down = physics.cast(
      from: const [0.0, 10.0, 0.0],
      direction: const [0.0, -1.0, 0.0],
      distance: 100.0,
    )!;
    expect(down.body, 2, reason: 'the crate stands between it and the floor');
    expect(down.distance, closeTo(4.5, 0.01));
    expect(down.at[1], closeTo(5.5, 0.01));
    expect(down.normal[1], closeTo(1.0, 0.01));
    expect(down.started, isFalse);

    // Each case below reaches a different field of the query struct. Taken
    // together they are the only way to know every one of them landed where
    // the C is looking for it, rather than a neighbour's.
    PhysicsHit? fire({
      List<double> from = const [0.0, 10.0, 0.0],
      List<double> direction = const [0.0, -1.0, 0.0],
      double distance = 100.0,
      Shape? shape,
      List<double> rotation = const [0.0, 0.0, 0.0, 1.0],
      Layers layers = Layers.everything,
      int ignore = 0,
    }) => physics.cast(
      from: from,
      direction: direction,
      distance: distance,
      shape: shape,
      rotation: rotation,
      layers: layers,
      ignore: ignore,
    );

    expect(fire(direction: const [0.0, 1.0, 0.0]), isNull, reason: 'fired up');
    expect(fire(distance: 1.0), isNull, reason: 'stopped short of the crate');
    expect(
      fire(ignore: 2)?.body,
      ground,
      reason: 'past the crate to the floor',
    );

    // A body that cares about nothing is reached only by a cast that cares
    // about it, which is also what tells the two layer words apart: swap them
    // and the first of these misses.
    const beside = [3.0, 10.0, 0.0];
    final cares = fire(from: beside, layers: const Layers(is_: 4, cares: 2));
    final passesBy = fire(from: beside, layers: const Layers(is_: 4, cares: 8));
    expect(cares?.body, 3);
    expect(passesBy?.body, ground, reason: 'through it, to a floor that cares');

    expect(
      fire(shape: const Shape.sphere(0.25))!.distance,
      closeTo(4.25, 0.02),
      reason: 'a sphere stops its own radius earlier than a point',
    );

    // Laid along x by a quarter turn about z, so it is 0.2 thick underneath
    // rather than 0.7. Upright it would stop at 3.8.
    expect(
      fire(
        shape: const Shape.capsule(0.2, 0.5),
        rotation: const [0.0, 0.0, 0.70710678, 0.70710678],
      )!.distance,
      closeTo(4.3, 0.02),
      reason: 'the rotation reached the solver',
    );

    final inside = fire(
      from: const [0.0, 5.0, 0.0],
      direction: const [1.0, 0.0, 0.0],
      distance: 10.0,
    )!;
    expect(inside.started, isTrue);
    expect(inside.distance, 0.0);

    expect(
      fire(shape: const Shape.plane(0.0, 1.0, 0.0)),
      isNull,
      reason: 'a half-space reaches everywhere along a line, so it is refused',
    );
  });

  test('a character walks up a step and reads back what it stands on', () {
    // Ids past 32 bits on both sides, so a footing whose ground came back
    // through the wrong width of field fails here rather than in a game.
    const walker = 0x100000002;
    const step = 0x700000005;
    floor();
    physics.add(
      step,
      shape: const Shape.box(2.0, 0.1, 1.0),
      motion: PhysicsMotion.fixed,
      at: const [3.0, 0.1, 0.0],
    );
    physics.addCharacter(walker, at: const [0.0, 0.91, 0.0]);

    expect(walk(walker, 1.5, 0.0, 2.0), 0, reason: 'never left the ground');

    expect(physics.transformOf(walker)![0], closeTo(3.0, 0.05));
    expect(heightOf(walker), closeTo(1.11, 0.005), reason: 'up the step');
    final footing = physics.footingOf(walker)!;
    expect(footing.grounded, isTrue);
    expect(footing.ground, step);
    expect(footing.normal, [closeTo(0.0, 1e-4), closeTo(1.0, 1e-4), 0.0]);
    expect(footing.velocity, [closeTo(1.5, 1e-4), 0.0, 0.0]);
    expect(footing.carried, [0.0, 0.0, 0.0]);
    expect(footing.turning, 0.0);
  });

  test('a character stops at a wall, and so does what it asked for', () {
    floor();
    physics.add(
      3,
      shape: const Shape.box(0.25, 1.0, 2.0),
      motion: PhysicsMotion.fixed,
      at: const [2.0, 1.0, 0.0],
    );
    physics.addCharacter(2, at: const [0.0, 0.91, 0.0]);

    walk(2, 1.0, 0.0, 3.0);

    // The face is at 1.75; a radius and a skin short of it is 1.44.
    expect(physics.transformOf(2)![0], closeTo(1.44, 0.005));
    expect(physics.footingOf(2)!.velocity[0], closeTo(0.0, 1e-4));
  });

  test('a character rides a platform and is told how far it was carried', () {
    physics.add(
      3,
      shape: const Shape.box(2.0, 0.1, 2.0),
      motion: PhysicsMotion.driven,
      at: const [0.0, 1.0, 0.0],
    );
    physics.addCharacter(2, at: const [0.0, 2.01, 0.0]);
    physics.drive(3, velocity: const [1.0, 0.0, 0.0]);

    expect(walk(2, 0.0, 0.0, 1.0), 0);

    expect(physics.transformOf(2)![0], closeTo(1.0, 0.02));
    expect(heightOf(2), closeTo(2.01, 0.005));
    final footing = physics.footingOf(2)!;
    expect(footing.ground, 3);
    expect(footing.carried[0], closeTo(1.0, 0.01));
    expect(
      footing.velocity[0],
      closeTo(0.0, 1e-4),
      reason: 'the ride is not a walk',
    );
  });

  test('footings are read together, and only characters have one', () {
    floor();
    physics.addCharacter(2, at: const [0.0, 0.91, 0.0]);
    physics.addCharacter(3, at: const [2.0, 5.0, 0.0]);
    physics.add(4, shape: const Shape.sphere(0.5), at: const [4.0, 0.5, 0.0]);
    physics.step(1 / 60);

    final footings = physics.footingsOf([4, 2, 99, 3]);
    expect(footings.length, 4);
    expect(footings[0], isNull, reason: 'a ball is not a character');
    expect(footings[2], isNull, reason: 'nor is nothing');
    expect(footings[1]!.grounded, isTrue);
    expect(footings[1]!.ground, ground);
    expect(footings[3]!.grounded, isFalse, reason: 'in the air');
    expect(footings[3]!.ground, 0);
    expect(physics.footingsOf(const []), isEmpty);

    // A half-space has no underneath to stand on anything with.
    physics.addCharacter(5, shape: const Shape.plane(1.0, 0.0, 0.0));
    expect(physics.footingOf(5), isNull);

    physics.remove(2);
    expect(physics.footingOf(2), isNull, reason: 'gone with its body');
  });

  /// A square of ground `samples` wide, at `height` everywhere unless
  /// `heightAt` says otherwise.
  Float32List heightsOf(
    int samples, {
    double height = 0.0,
    double Function(int c, int r)? heightAt,
  }) {
    final heights = Float32List(samples * samples);
    for (var r = 0; r < samples; r++) {
      for (var c = 0; c < samples; c++) {
        heights[r * samples + c] = heightAt?.call(c, r) ?? height;
      }
    }
    return heights;
  }

  test(
    'ground holds up what lands on it, and its struct survives the crossing',
    () {
      // Each body below lands somewhere only one field of the struct puts
      // ground: take any field from its neighbour's slot and one of them falls.
      const field = 0x300000007;
      expect(
        physics.layGround(
          field,
          heights: heightsOf(5, height: 1.0),
          columns: 5,
          rows: 5,
          spacing: 2.0,
          at: const [-4.0, 0.5, -4.0],
          friction: 0.0,
          restitution: 0.8,
          layers: const Layers(is_: 2, cares: 2),
        ),
        isTrue,
      );
      expect(physics.alive(field), isTrue);

      // Past x = 2 only because the spacing is two, and at 1.5 only because
      // the field is raised half a metre.
      physics.add(2, shape: const Shape.sphere(0.5), at: const [3.0, 4.0, 3.5]);
      // Off the edge of it, where there is nothing.
      physics.add(3, shape: const Shape.sphere(0.5), at: const [4.5, 4.0, 0.0]);
      // Watching nothing the ground is in, and nothing watching it.
      physics.add(
        4,
        shape: const Shape.sphere(0.5),
        at: const [0.0, 4.0, 0.0],
        layers: const Layers(is_: 4, cares: 4),
      );
      physics.add(
        5,
        shape: const Shape.sphere(0.5),
        at: const [-2.0, 4.0, 0.0],
        restitution: 0.0,
      );

      var highest = 0.0;
      for (var i = 0; i < 180; i++) {
        physics.step(1 / 60);
        if (i > 60 && heightOf(5) > highest) highest = heightOf(5);
      }
      // Bouncing takes the ground's restitution, which is only there if it
      // landed in its own slot and not in friction's.
      expect(highest, greaterThan(2.0), reason: 'the ground is springy');
      expect(heightOf(3), lessThan(0.0), reason: 'past the edge');
      expect(heightOf(4), lessThan(0.0), reason: 'on a layer it ignores');

      physics.remove(5);
      run(3);
      expect(heightOf(2), closeTo(2.0, 0.05));

      final down = physics.cast(
        from: const [1.0, 9.0, 1.0],
        direction: const [0.0, -1.0, 0.0],
        distance: 20.0,
      )!;
      expect(down.body, field);
      expect(down.distance, closeTo(7.5, 0.01));
      expect(down.normal[1], closeTo(1.0, 0.001));
    },
  );

  test(
    'ground with a hole lets things through, and relaying it moves them',
    () {
      const field = 7;
      final holed = heightsOf(
        9,
        heightAt: (c, r) => c == 4 && r == 4 ? double.nan : 0.0,
      );
      expect(
        physics.layGround(
          field,
          heights: holed,
          columns: 9,
          rows: 9,
          at: const [-4.0, 0.0, -4.0],
        ),
        isTrue,
      );
      physics.add(2, shape: const Shape.sphere(0.3), at: const [0.0, 2.0, 0.0]);
      physics.add(
        3,
        shape: const Shape.box(0.3, 0.3, 0.3),
        at: const [2.5, 1.0, 2.5],
      );
      run(3);
      expect(heightOf(2), lessThan(-1.0), reason: 'down the hole');
      expect(heightOf(3), closeTo(0.3, 0.03));
      expect(physics.asleep(3), isTrue);

      // Laid again half a metre down, whole: the sleeping crate is woken and
      // follows it rather than hanging where the ground was.
      physics.layGround(
        field,
        heights: heightsOf(9, height: -0.5),
        columns: 9,
        rows: 9,
        at: const [-4.0, 0.0, -4.0],
      );
      expect(physics.count, 3, reason: 'replaced, not added');
      run(2);
      expect(heightOf(3), closeTo(-0.2, 0.03));

      // Taken away, it wakes nothing: ground streamed out from under a
      // sleeping crate far off leaves it where it was for when it comes back.
      run(2);
      expect(physics.asleep(3), isTrue);
      physics.remove(field);
      run(1);
      expect(heightOf(3), closeTo(-0.2, 0.03), reason: 'asleep, so left be');
      physics.wake(3);
      run(1);
      expect(heightOf(3), lessThan(-1.0), reason: 'the ground is gone');
    },
  );

  test('a margin is the neighbours\' ground, never stood on', () {
    // Six by six with a margin is four by four of its own, from one to four.
    // The margin is a wall of heights that would stop anything if it were
    // ground; as a margin it only says which way the ground carries on.
    final walled = heightsOf(
      6,
      heightAt: (c, r) => c == 0 || r == 0 || c == 5 || r == 5 ? 3.0 : 0.0,
    );
    expect(
      physics.layGround(10, heights: walled, columns: 6, rows: 6, margin: true),
      isTrue,
    );
    physics.add(2, shape: const Shape.sphere(0.2), at: const [2.5, 1.0, 2.5]);
    physics.add(3, shape: const Shape.sphere(0.2), at: const [0.5, 4.0, 2.5]);
    run(2);
    expect(heightOf(2), closeTo(0.2, 0.02));
    expect(heightOf(3), lessThan(0.0), reason: 'over the margin, so nothing');
  });

  test('ground refuses a grid it cannot lay', () {
    final four = heightsOf(4);
    bool lay({
      int id = 9,
      int columns = 4,
      int rows = 4,
      double spacing = 1.0,
      bool margin = false,
      List<double>? heights,
    }) => physics.layGround(
      id,
      heights: heights ?? four,
      columns: columns,
      rows: rows,
      spacing: spacing,
      margin: margin,
    );

    expect(lay(id: 0), isFalse, reason: 'nameless');
    expect(lay(columns: 1), isFalse, reason: 'no square to stand on');
    expect(lay(spacing: 0.0), isFalse);
    expect(lay(spacing: double.nan), isFalse);
    expect(lay(heights: heightsOf(3)), isFalse, reason: 'too few heights');
    expect(
      lay(columns: 3, rows: 3, margin: true),
      isFalse,
      reason: 'all margin',
    );
    expect(physics.count, 0);
    expect(lay(margin: true), isTrue, reason: 'four is enough with a margin');
    expect(lay(columns: 2, rows: 2, id: 10), isTrue, reason: 'two without');
    expect(physics.count, 2);
  });

  group('joints', () {
    /// A post that never moves, for a joint's first body, so what is measured
    /// is the second body as the post sees it.
    void post(int id, List<double> at) => physics.add(
      id,
      shape: const Shape.box(0.1, 0.1, 0.1),
      motion: PhysicsMotion.fixed,
      at: at,
    );

    /// A quarter turn about z, which takes a joint's x axis to straight up.
    const upright = [0.0, 0.0, 0.7071068, 0.7071068];

    double distanceFrom(int id, List<double> point) {
      final at = physics.transformOf(id)!;
      final dx = at[0] - point[0];
      final dy = at[1] - point[1];
      final dz = at[2] - point[2];
      return sqrt(dx * dx + dy * dy + dz * dz);
    }

    test('each kind says which of the six ways it holds', () {
      const cone = Joint.cone(swing: 0.5, twist: JointLimit(-0.2, 0.2));
      expect(cone.kind, PhysicsJointKind.cone);
      expect(cone.swing, 0.5);
      expect(cone.limits.map((limit) => limit?.high), [
        null,
        null,
        null,
        0.2,
        null,
        null,
      ]);

      const drawer = Joint.slider(limit: JointLimit(0.0, 0.4));
      expect(drawer.limits.indexWhere((limit) => limit != null), 0);

      const rail = Joint.sixAxis(
        alongY: JointLimit(-1.0, 1.0),
        aboutZ: JointLimit.locked(),
      );
      expect(rail.limits.map((limit) => limit == null), [
        true,
        false,
        true,
        true,
        true,
        false,
      ]);
      expect(rail.aboutZ!.low, rail.aboutZ!.high);
      expect(PhysicsJointKind.values.map((kind) => kind.code), [
        1,
        2,
        3,
        4,
        5,
        6,
        7,
      ]);
    });

    test('its structs are the size the engine reads', () {
      expect(sizeOf<native.OrblitPhysicsJoint>(), 144);
      expect(sizeOf<native.OrblitPhysicsJointState>(), 32);
    });

    test('a pendulum swings and keeps its length', () {
      physics.add(2, shape: const Shape.sphere(0.1), at: const [1.0, 3.0, 0.0]);
      expect(
        physics.join(1, const Joint.point(), a: 2, at: const [0.0, 3.0, 0.0]),
        isTrue,
      );

      var lowest = 3.0;
      var longest = 0.0;
      for (var i = 0; i < 120; i++) {
        physics.step(1 / 60);
        lowest = min(lowest, heightOf(2));
        longest = max(longest, distanceFrom(2, const [0.0, 3.0, 0.0]));
      }
      expect(lowest, lessThan(2.1), reason: 'it swung down');
      expect(longest, closeTo(1.0, 0.02), reason: 'and never stretched');
    });

    test('a hinge turns under its motor and stops at its limit', () {
      post(1, const [0.0, 1.0, 0.0]);
      physics.add(
        2,
        shape: const Shape.box(0.5, 1.0, 0.05),
        at: const [0.5, 1.0, 0.0],
      );
      physics.join(
        1,
        const Joint.hinge(
          limit: JointLimit(0.0, 1.2),
          speed: 1.0,
          strength: 50.0,
        ),
        a: 1,
        b: 2,
        at: const [0.0, 1.0, 0.0],
        rotation: upright,
      );

      run(0.5);
      expect(physics.jointStateOf(1)!.angles[0], closeTo(0.5, 0.05));
      expect(physics.velocityOf(2)![4], closeTo(1.0, 0.05));

      run(1.5);
      final state = physics.jointStateOf(1)!;
      expect(state.angles[0], closeTo(1.2, 0.03));
      expect(physics.velocityOf(2)![4].abs(), lessThan(0.05));
      expect(heightOf(2), closeTo(1.0, 0.01), reason: 'held up by its hinge');
      expect(
        state.force,
        closeTo(9.81, 0.5),
        reason: 'holding up its own weight',
      );
    });

    test('the world may be either end, and which decides the sense', () {
      double turned(int a, int b) {
        physics.dispose();
        physics = Physics();
        physics.add(
          2,
          shape: const Shape.box(0.5, 1.0, 0.05),
          at: const [0.5, 1.0, 0.0],
        );
        physics.join(
          1,
          const Joint.hinge(speed: 1.0, strength: 50.0),
          a: a,
          b: b,
          at: const [0.0, 1.0, 0.0],
          rotation: upright,
        );
        run(1.0);
        expect(physics.jointStateOf(1)!.angles[0], closeTo(1.0, 0.05));
        return physics.transformOf(2)![2];
      }

      expect(turned(0, 2), lessThan(-0.3), reason: "the door's own angle");
      expect(turned(2, 0), greaterThan(0.3), reason: "the world's, reversed");
    });

    test('a slider keeps to its rail, as far as its limit', () {
      post(1, const [0.0, 3.0, 0.0]);
      physics.add(
        2,
        shape: const Shape.box(0.2, 0.2, 0.2),
        at: const [0.0, 3.0, 0.0],
      );
      physics.join(
        1,
        const Joint.slider(
          limit: JointLimit(0.0, 0.5),
          speed: 1.0,
          strength: 100.0,
        ),
        a: 1,
        b: 2,
        at: const [0.0, 3.0, 0.0],
      );

      run(0.25);
      expect(physics.jointStateOf(1)!.offset[0], closeTo(0.25, 0.03));

      run(1.0);
      final at = physics.transformOf(2)!;
      expect(at[0], closeTo(0.5, 0.01));
      expect(at[1], closeTo(3.0, 0.01), reason: 'the rail holds it up');
      expect(at[2], closeTo(0.0, 0.01));
    });

    test('a six-axis joint limits each way in the order it names them', () {
      post(1, const [0.0, 3.0, 0.0]);
      physics.add(
        2,
        shape: const Shape.box(0.2, 0.2, 0.2),
        at: const [0.0, 3.0, 0.0],
      );
      physics.join(
        1,
        const Joint.sixAxis(
          alongX: JointLimit.locked(),
          alongY: JointLimit(-0.5, 0.0),
          alongZ: JointLimit.locked(),
          aboutX: JointLimit.locked(),
          aboutY: JointLimit.locked(),
          aboutZ: JointLimit.locked(),
        ),
        a: 1,
        b: 2,
        at: const [0.0, 3.0, 0.0],
      );
      physics.drive(2, velocity: const [1.0, 0.0, 1.0]);

      run(1.0);
      final at = physics.transformOf(2)!;
      expect(at[1], closeTo(2.5, 0.02), reason: 'it fell as far as y lets it');
      expect(at[0], closeTo(0.0, 0.01));
      expect(at[2], closeTo(0.0, 0.01));
      expect(physics.jointStateOf(1)!.offset[1], closeTo(-0.5, 0.02));
    });

    test('a rope is slack until it is taut', () {
      physics.add(2, shape: const Shape.sphere(0.1), at: const [0.0, 3.0, 0.0]);
      physics.join(
        1,
        const Joint.distance(limit: JointLimit(0.0, 3.0)),
        a: 2,
        at: const [0.0, 3.0, 0.0],
        to: const [0.0, 5.0, 0.0],
      );

      run(0.3);
      expect(
        fallSpeedOf(2),
        closeTo(-9.81 * 0.3, 0.1),
        reason: 'two metres of a three metre rope does not hold anything',
      );

      run(3.0);
      expect(heightOf(2), closeTo(2.0, 0.03));
      expect(physics.jointStateOf(1)!.offset[0], closeTo(3.0, 0.03));
    });

    test('a joint breaks past its strength, and says so', () {
      physics.add(
        2,
        shape: const Shape.box(0.2, 0.2, 0.2),
        at: const [0.0, 3.0, 0.0],
      );
      physics.join(
        7,
        const Joint.fixed(),
        a: 2,
        at: const [0.0, 3.0, 0.0],
        breakingForce: 5.0,
      );

      physics.step(1 / 60);
      final broke = physics.events
          .where((event) => event.kind == PhysicsEventKind.broke)
          .toList();
      expect(broke, hasLength(1));
      expect(broke.single.a, 7, reason: 'the joint, not a body');
      expect(broke.single.b, 0);
      expect(broke.single.force, greaterThan(5.0));
      expect(broke.single.at[1], closeTo(3.0, 0.01));
      expect(physics.jointStateOf(7), isNull);

      run(0.5);
      expect(heightOf(2), lessThan(2.0), reason: 'nothing holds it now');
    });

    test('a chain sleeps as one, and loses its joints with a body', () {
      physics.add(2, shape: const Shape.sphere(0.1), at: const [0.0, 2.0, 0.0]);
      physics.add(3, shape: const Shape.sphere(0.1), at: const [0.0, 1.0, 0.0]);
      physics.join(1, const Joint.point(), a: 2, at: const [0.0, 3.0, 0.0]);
      physics.join(
        2,
        const Joint.point(),
        a: 3,
        b: 2,
        at: const [0.0, 1.5, 0.0],
      );

      run(1.5);
      expect(physics.asleep(2), isTrue);
      expect(physics.asleep(3), isTrue);

      physics.remove(2);
      expect(physics.jointStateOf(1), isNull);
      expect(physics.jointStateOf(2), isNull);
      expect(physics.asleep(3), isFalse, reason: 'what it held is woken');
      expect(
        physics.events.where((event) => event.kind == PhysicsEventKind.broke),
        isEmpty,
        reason: 'nothing broke',
      );

      run(0.5);
      expect(heightOf(3), lessThan(0.5));
    });

    test('unjoining lets go', () {
      physics.add(2, shape: const Shape.sphere(0.1), at: const [0.0, 2.0, 0.0]);
      physics.join(1, const Joint.fixed(), a: 2, at: const [0.0, 2.0, 0.0]);
      run(1.0);
      expect(heightOf(2), closeTo(2.0, 0.01));
      expect(physics.asleep(2), isTrue);

      expect(physics.unjoin(1), isTrue);
      expect(physics.asleep(2), isFalse);
      expect(physics.unjoin(1), isFalse);
      run(0.5);
      expect(heightOf(2), lessThan(1.0));
    });

    test('the world refuses a joint it cannot make', () {
      physics.add(2, shape: const Shape.sphere(0.1), at: const [0.0, 2.0, 0.0]);
      physics.add(3, shape: const Shape.sphere(0.1), at: const [1.0, 2.0, 0.0]);
      bool join(int id, {int a = 2, int b = 0, List<double>? at}) =>
          physics.join(
            id,
            const Joint.point(),
            a: a,
            b: b,
            at: at ?? const [0.0, 2.0, 0.0],
          );

      expect(join(0), isFalse, reason: 'zero is never a joint');
      expect(join(1, a: 9), isFalse, reason: 'no such body');
      expect(join(1, b: 9), isFalse, reason: 'no such body');
      expect(join(1, b: 2), isFalse, reason: 'a body joined to itself');
      expect(join(1, a: 0), isFalse, reason: 'the world joined to itself');
      expect(join(1, at: const [double.nan, 2.0, 0.0]), isFalse);
      expect(join(1), isTrue);
      expect(join(1, b: 3), isFalse, reason: 'already a joint');
      expect(join(3, b: 3), isTrue, reason: 'joints are named apart');
      expect(physics.jointStateOf(99), isNull);
      expect(physics.unjoin(99), isFalse);
    });
  });

  test('readInto fills a strided buffer and skips what it does not know', () {
    floor();
    physics.add(2, shape: const Shape.sphere(0.5), at: const [1.0, 2.0, 3.0]);
    physics.add(4, shape: const Shape.sphere(0.5), at: const [4.0, 5.0, 6.0]);

    // Ten floats a row, starting one in: the shape of a transform column that
    // holds translation, rotation and scale.
    final out = Float32List(30)..fillRange(0, 30, -1.0);
    final written = physics.readInto([2, 3, 4], out, stride: 10, offset: 1);

    expect(written, 2);
    expect(out.sublist(1, 4), [1.0, 2.0, 3.0]);
    expect(out.sublist(21, 24), [4.0, 5.0, 6.0]);
    // The unknown id's row, and every float outside a row, is untouched.
    expect(out.sublist(11, 18), everyElement(-1.0));
    expect(out[0], -1.0);
    expect(out[8], -1.0);

    expect(physics.readInto([2], out, stride: 6), 0);
  });

  test('the world refuses a nameless or repeated body', () {
    physics.add(0, shape: const Shape.sphere(0.5));
    physics.add(2, shape: const Shape.sphere(0.5));
    physics.add(2, shape: const Shape.box(1.0, 1.0, 1.0));
    expect(physics.count, 1);

    expect(physics.alive(2), isTrue);
    expect(physics.alive(3), isFalse);
    expect(physics.transformOf(3), isNull);
    expect(physics.velocityOf(3), isNull);

    // A command naming a body that is no longer there is simply dropped.
    physics.remove(2);
    physics.place(2, at: const [0.0, 9.0, 0.0]);
    expect(physics.count, 0);
  });

  test('settings left null keep the engine defaults', () {
    final light = Physics(
      settings: const PhysicsSettings(
        gravity: [0.0, -1.0, 0.0],
        sleeping: false,
      ),
    );
    addTearDown(light.dispose);

    light.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 10.0, 0.0]);
    for (var i = 0; i < 60; i++) {
      light.step(1 / 60);
    }

    // A second under Earth's gravity costs about 4.9 metres; under this one,
    // about half of one. Damping trims both a little.
    expect(10.0 - light.transformOf(2)![1], lessThan(1.0));
    expect(light.asleep(2), isFalse);
  });

  test('a disposed world says so rather than crashing', () {
    final short = Physics();
    short.dispose();
    short.dispose();

    expect(() => short.step(1 / 60), throwsStateError);
    expect(
      () => short.add(2, shape: const Shape.sphere(0.5)),
      throwsStateError,
    );
    expect(() => short.count, throwsStateError);
  });
}
