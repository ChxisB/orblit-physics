import 'dart:ffi' show sizeOf;
import 'dart:math' show cos, max, min, sin, sqrt;
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

  group('asking the world', () {
    /// A cube trigger of half-size `half`, kept still.
    void trigger(
      int id, {
      required List<double> at,
      double half = 1.0,
      bool stay = false,
    }) => physics.add(
      id,
      shape: Shape.box(half, half, half),
      motion: PhysicsMotion.fixed,
      at: at,
      trigger: true,
      stay: stay,
    );

    /// Steps `seconds` and keeps every event, since a step drops the last
    /// one's.
    List<PhysicsEvent> watch(double seconds) {
      final seen = <PhysicsEvent>[];
      for (var i = 0; i < (seconds * 60).round(); i++) {
        physics.step(1 / 60);
        seen.addAll(physics.events);
      }
      return seen;
    }

    int count(List<PhysicsEvent> seen, PhysicsEventKind kind, int a, int b) =>
        seen.where((it) => it.kind == kind && it.a == a && it.b == b).length;

    test('its structs are the size the engine reads', () {
      expect(sizeOf<native.OrblitPhysicsZone>(), 40);
      expect(sizeOf<native.OrblitPhysicsRule>(), 40);
    });

    test('a body falling through a trigger enters and leaves it', () {
      physics.add(
        9,
        shape: const Shape.plane(0.0, 1.0, 0.0, offset: -6.0),
        motion: PhysicsMotion.fixed,
      );
      trigger(1, at: const [0.0, 0.0, 0.0]);
      physics.add(
        2,
        shape: const Shape.sphere(0.25),
        at: const [0.0, 3.0, 0.0],
      );

      final seen = watch(2.0);

      expect(count(seen, PhysicsEventKind.entered, 1, 2), 1);
      expect(count(seen, PhysicsEventKind.exited, 1, 2), 1);
      expect(count(seen, PhysicsEventKind.touchBegan, 1, 2), 0);
      expect(heightOf(2), closeTo(-5.75, 0.1), reason: 'nothing was pushed');
    });

    test('inside and touchStay are heard only by a body that asks', () {
      floor();
      trigger(5, at: const [0.0, 0.0, 0.0], stay: true);
      physics.add(
        2,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [5.0, 0.49, 0.0],
        stay: true,
      );
      physics.add(
        3,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [8.0, 0.49, 0.0],
      );
      physics.add(
        4,
        shape: const Shape.sphere(0.25),
        motion: PhysicsMotion.driven,
        at: const [0.0, 0.0, 0.0],
      );

      final seen = watch(0.3);

      expect(count(seen, PhysicsEventKind.inside, 5, 4), greaterThan(10));
      expect(
        count(seen, PhysicsEventKind.touchStay, ground, 2),
        greaterThan(10),
      );
      expect(count(seen, PhysicsEventKind.touchBegan, ground, 3), 1);
      expect(count(seen, PhysicsEventKind.touchStay, ground, 3), 0);
    });

    test('entering names the trigger first and gives a unit normal', () {
      trigger(5, at: const [0.0, 0.0, 0.0]);
      physics.add(
        2,
        shape: const Shape.sphere(0.25),
        motion: PhysicsMotion.driven,
        at: const [0.9, 0.0, 0.0],
      );

      final entered = watch(0.1)
          .singleWhere((it) => it.kind == PhysicsEventKind.entered);

      expect([entered.a, entered.b], [5, 2], reason: 'not smaller id first');
      final n = entered.normal;
      expect(n[0] * n[0] + n[1] * n[1] + n[2] * n[2], closeTo(1.0, 0.01));
    });

    test('a zone with no gravity holds a body, and removing it lets go', () {
      trigger(1, at: const [0.0, 10.0, 0.0], half: 20.0);
      physics.add(
        2,
        shape: const Shape.sphere(0.25),
        at: const [0.0, 10.0, 0.0],
      );

      expect(
        physics.setZone(1, const PhysicsZone(gravity: [0.0, 0.0, 0.0])),
        isTrue,
      );
      run(1.0);
      expect(heightOf(2), closeTo(10.0, 0.05));

      expect(physics.removeZone(1), isTrue);
      run(1.0);
      expect(heightOf(2), lessThan(6.0));
      expect(physics.removeZone(1), isFalse, reason: 'there was none');
    });

    test('zones agree by priority, then by lowest id', () {
      // Each zone is (id, priority, upward gravity), over the same ball.
      double heightUnder(List<(int, int, double)> zones) {
        final world = Physics();
        addTearDown(world.dispose);
        world.add(
          2,
          shape: const Shape.sphere(0.25),
          at: const [0.0, 10.0, 0.0],
        );
        for (final (id, priority, gravity) in zones) {
          world.add(
            id,
            shape: const Shape.box(20.0, 20.0, 20.0),
            motion: PhysicsMotion.fixed,
            at: const [0.0, 10.0, 0.0],
            trigger: true,
          );
          world.setZone(
            id,
            PhysicsZone(gravity: [0.0, gravity, 0.0], priority: priority),
          );
        }
        for (var i = 0; i < 60; i++) {
          world.step(1 / 60);
        }
        return world.transformOf(2)![1];
      }

      expect(heightUnder([(1, 0, 0.0), (3, 5, 9.81)]), greaterThan(12.0));
      expect(heightUnder([(1, 5, 9.81), (3, 0, 0.0)]), greaterThan(12.0));
      expect(heightUnder([(3, 3, -9.81), (1, 3, 9.81)]), greaterThan(12.0));
    });

    test('a zone can hold damping and leave gravity alone', () {
      trigger(1, at: const [0.0, 10.0, 0.0], half: 20.0);
      physics.add(
        2,
        shape: const Shape.sphere(0.25),
        at: const [0.0, 10.0, 0.0],
        linearDamping: 0.0,
      );
      physics.setZone(1, const PhysicsZone(linearDamping: 8.0));
      physics.drive(2, velocity: const [5.0, 0.0, 0.0]);

      run(1.0);

      expect(physics.velocityOf(2)![0], lessThan(0.1));
      expect(heightOf(2), lessThan(10.0), reason: 'gravity was not changed');
    });

    test('a zone is refused on what is not a trigger, and for NaN', () {
      physics.add(1, shape: const Shape.sphere(0.5), at: const [0.0, 2.0, 0.0]);
      trigger(2, at: const [0.0, 0.0, 0.0]);
      const weightless = PhysicsZone(gravity: [0.0, 0.0, 0.0]);

      expect(physics.setZone(1, weightless), isFalse);
      expect(physics.setZone(99, weightless), isFalse);
      expect(
        physics.setZone(2, const PhysicsZone(gravity: [0.0, double.nan, 0.0])),
        isFalse,
      );
    });

    test(
      'a belt carries a box at its own speed, and reversing it turns it',
      () {
        physics.add(
          1,
          shape: const Shape.box(8.0, 0.25, 1.0),
          motion: PhysicsMotion.fixed,
          at: const [0.0, 0.25, 0.0],
        );
        physics.add(
          2,
          shape: const Shape.box(0.5, 0.5, 0.5),
          at: const [0.0, 1.0, 0.0],
        );

        physics.setSurface(1, velocity: const [2.0, 0.0, 0.0]);
        run(2.0);
        expect(physics.velocityOf(2)![0], closeTo(2.0, 0.2));
        expect(heightOf(2), closeTo(1.0, 0.02));

        physics.setSurface(1, velocity: const [-2.0, 0.0, 0.0]);
        run(2.0);
        expect(physics.velocityOf(2)![0], closeTo(-2.0, 0.2));

        physics.setSurface(1, velocity: const [0.0, 0.0, 0.0]);
        run(2.0);
        expect(physics.velocityOf(2)![0].abs(), lessThan(0.1));
      },
    );

    test('a rule with no friction lets one box slide on grippy ground', () {
      floor(friction: 1.0);
      for (final (id, z) in [(2, 0.0), (3, 5.0)]) {
        physics.add(
          id,
          shape: const Shape.box(0.5, 0.5, 0.5),
          at: [0.0, 0.5, z],
          friction: 1.0,
        );
        physics.drive(id, velocity: const [4.0, 0.0, 0.0]);
      }
      physics.setRule(2, ground, const PhysicsRule(friction: 0.0));

      run(2.0);

      expect(physics.transformOf(3)![0], lessThan(1.0));
      expect(physics.transformOf(2)![0], greaterThan(2.5));
    });

    test('a rule is removed by naming it, and ends with either body', () {
      floor();
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 0.5, 0.0]);
      const bouncy = PhysicsRule(restitution: 0.9);

      expect(physics.setRule(2, ground, bouncy), isTrue);
      expect(physics.removeRule(ground, 2), isTrue);
      expect(physics.removeRule(ground, 2), isFalse, reason: 'there was none');

      physics.setRule(2, ground, bouncy);
      physics.remove(2);
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 0.5, 0.0]);
      expect(physics.removeRule(2, ground), isFalse, reason: 'it ended with 2');
    });

    test('a rule is refused for a body against itself and for nonsense', () {
      floor();
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 0.5, 0.0]);

      bool set(int a, int b, PhysicsRule rule) => physics.setRule(a, b, rule);
      expect(set(2, 2, const PhysicsRule(friction: 0.0)), isFalse);
      expect(set(2, 55, const PhysicsRule(friction: 0.0)), isFalse);
      expect(set(2, ground, const PhysicsRule(friction: double.nan)), isFalse);
      expect(set(2, ground, const PhysicsRule(moveScaleA: -1.0)), isFalse);
    });

    group('casting', () {
      /// Three boxes along +x, made in the reverse of the order a ray meets
      /// them, so that meeting them in order is not an accident of storage.
      void row() {
        for (var i = 3; i >= 1; i--) {
          physics.add(
            i,
            shape: const Shape.box(0.5, 0.5, 0.5),
            motion: PhysicsMotion.fixed,
            at: [3.0 * i, 0.0, 0.0],
          );
        }
      }

      List<PhysicsHit> alongRow({
        int ignore = 0,
        int limit = 32,
        double x = 0,
      }) => physics.castAll(
        from: [x, 0.0, 0.0],
        direction: const [1.0, 0.0, 0.0],
        distance: 20.0,
        ignore: ignore,
        limit: limit,
      );

      test('castAll meets every body along the way, nearest first', () {
        row();

        final all = alongRow();

        expect(all.map((it) => it.body), [1, 2, 3]);
        expect(all.first.distance, closeTo(2.5, 0.01));
        expect(all[1].normal[0], closeTo(-1.0, 0.01));
      });

      test('castAll keeps the nearest when the limit is short', () {
        row();

        expect(alongRow(limit: 2).map((it) => it.body), [1, 2]);
        expect(alongRow(limit: 0), isEmpty);
      });

      test('castAll skips what it is told to ignore and starts inside', () {
        row();

        final inside = alongRow(x: 3.0);

        expect(alongRow(ignore: 2).map((it) => it.body), [1, 3]);
        expect(inside.first.started, isTrue);
        expect(inside.first.distance, 0.0);
        expect(inside[1].started, isFalse);
      });

      test('castAny answers whether there is anything in the way', () {
        row();
        trigger(4, at: const [30.0, 0.0, 0.0], half: 0.5);

        bool any({
          double distance = 20.0,
          double x = 0.0,
          double direction = 1.0,
          bool triggers = false,
        }) => physics.castAny(
          from: [x, 0.0, 0.0],
          direction: [direction, 0.0, 0.0],
          distance: distance,
          triggers: triggers,
        );

        expect(any(), isTrue);
        expect(any(direction: -1.0), isFalse);
        expect(any(distance: 2.0), isFalse);
        expect(any(x: 25.0), isFalse, reason: 'a trigger is not a hit');
        expect(any(x: 25.0, triggers: true), isTrue);
      });

      test('cast sees a trigger only when asked for triggers', () {
        trigger(1, at: const [3.0, 0.0, 0.0], half: 0.5);

        PhysicsHit? shoot({required bool triggers}) => physics.cast(
          from: const [0.0, 0.0, 0.0],
          direction: const [1.0, 0.0, 0.0],
          distance: 20.0,
          triggers: triggers,
        );

        expect(shoot(triggers: false), isNull);
        expect(shoot(triggers: true)?.body, 1);
      });

      test('overlap with no shape asks about a point', () {
        physics.add(
          1,
          shape: const Shape.box(1.0, 1.0, 1.0),
          motion: PhysicsMotion.fixed,
        );
        trigger(2, at: const [10.0, 0.0, 0.0]);

        expect(physics.overlap(at: const [0.0, 0.0, 0.0]), [1]);
        expect(physics.overlap(at: const [0.0, 1.2, 0.0]), isEmpty);
        expect(physics.overlap(at: const [10.0, 0.0, 0.0]), isEmpty);
        expect(physics.overlap(at: const [10.0, 0.0, 0.0], triggers: true), [
          2,
        ]);
      });

      test('overlap by a shape reports what it touches, up to a limit', () {
        physics.add(
          1,
          shape: const Shape.box(1.0, 1.0, 1.0),
          motion: PhysicsMotion.fixed,
        );
        physics.add(
          2,
          shape: const Shape.sphere(0.5),
          motion: PhysicsMotion.fixed,
          at: const [1.4, 0.0, 0.0],
        );

        List<int> around({int ignore = 0, int limit = 32}) => physics.overlap(
          at: const [1.2, 0.0, 0.0],
          shape: const Shape.sphere(0.3),
          ignore: ignore,
          limit: limit,
        );

        expect(around().toSet(), {1, 2});
        expect(around(limit: 1), hasLength(1));
        expect(around(ignore: 1), [2]);
        expect(around(limit: 0), isEmpty);
      });
    });

    test('a ramp, a trigger, a zone and a belt answer all five questions', () {
      const angle = 0.5;
      floor();
      physics.add(
        2,
        shape: Shape.plane(
          -sin(angle),
          cos(angle),
          0.0,
          offset: -10.0 * sin(angle),
        ),
        motion: PhysicsMotion.fixed,
      );
      trigger(3, at: const [4.0, 1.0, 0.0]);
      trigger(4, at: const [-20.0, 2.0, 0.0], half: 2.0);
      physics.setZone(4, const PhysicsZone(gravity: [0.0, 0.0, 0.0]));
      physics.add(
        5,
        shape: const Shape.box(4.0, 0.25, 1.0),
        motion: PhysicsMotion.fixed,
        at: const [-5.0, 0.25, 0.0],
      );
      physics.add(
        6,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [-5.0, 1.0, 0.0],
      );
      physics.add(
        7,
        shape: const Shape.sphere(0.25),
        at: const [-20.0, 2.0, 0.0],
      );
      physics.setSurface(5, velocity: const [2.0, 0.0, 0.0]);

      // Point and overlap.
      expect(physics.overlap(at: const [4.0, 1.0, 0.0]), isEmpty);
      expect(physics.overlap(at: const [4.0, 1.0, 0.0], triggers: true), [3]);
      expect(physics.overlap(at: const [-20.0, 2.0, 0.0], triggers: true), [4]);
      expect(
        physics.overlap(
          at: const [-5.0, 1.0, 0.0],
          shape: const Shape.sphere(0.1),
        ),
        [6],
      );

      // Closest, all and any, down onto the ramp, reporting its normal.
      const down = [0.0, -1.0, 0.0];
      const from = [13.0, 10.0, 0.0];
      final hit = physics.cast(from: from, direction: down, distance: 100.0)!;
      expect(hit.body, 2);
      expect(hit.normal[0], closeTo(-sin(angle), 0.01));
      expect(hit.normal[1], closeTo(cos(angle), 0.01));
      final both = physics.castAll(
        from: from,
        direction: down,
        distance: 100.0,
      );
      expect(both.map((it) => it.body), [2, 1]);
      expect(both[1].distance, closeTo(10.0, 0.01));
      expect(
        physics.castAny(from: from, direction: down, distance: 5.0),
        isFalse,
      );
      expect(
        physics.castAny(from: from, direction: down, distance: 100.0),
        isTrue,
      );

      // The world, run: the belt carries its box and the zone holds its ball.
      run(2.0);
      expect(physics.velocityOf(6)![0], closeTo(2.0, 0.2));
      expect(heightOf(7), closeTo(2.0, 0.05));
    });

    test('the new event kinds follow the engine\'s numbers', () {
      expect(PhysicsEventKind.values[5], PhysicsEventKind.entered);
      expect(PhysicsEventKind.values[6], PhysicsEventKind.exited);
      expect(PhysicsEventKind.values[7], PhysicsEventKind.touchStay);
      expect(PhysicsEventKind.values[8], PhysicsEventKind.inside);
    });
  });

  group('body controls', () {
    const box = Shape.box(0.5, 0.5, 0.5);

    double speedOf(int id) {
      final v = physics.velocityOf(id)!;
      return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    test('its struct is the size the engine reads', () {
      expect(sizeOf<native.OrblitPhysicsControls>(), 56);
    });

    test('locks are the bits the engine numbers them by', () {
      expect(PhysicsLock.values.map((it) => it.bit), [1, 2, 4, 8, 16, 32]);
      expect(
        const PhysicsControls(
          locks: {PhysicsLock.moveY, PhysicsLock.turnZ},
        ).bits,
        2 | 32,
      );
    });

    test(
      'a body locked to a plane, at half gravity, under a cap, behaves so',
      () {
        physics.add(2, shape: box, at: const [0.0, 20.0, 0.0]);
        physics.add(3, shape: box, at: const [4.0, 20.0, 0.0]);
        const plane = PhysicsControls(
          locks: {PhysicsLock.moveZ, PhysicsLock.turnX, PhysicsLock.turnY},
          gravityScale: 0.5,
          maxSpeed: 3.0,
        );
        expect(physics.setControls(2, plane), isTrue);

        // Body 3 is the same box with nothing set, so what differs is the
        // controls.
        for (final id in [2, 3]) {
          physics.drive(
            id,
            velocity: const [2.0, 0.0, 2.0],
            spin: const [4.0, 4.0, 4.0],
          );
        }
        run(1.0);

        final held = physics.velocityOf(2)!;
        expect(physics.transformOf(2)![2], closeTo(0.0, 1e-4));
        expect(held[2], 0.0);
        expect(held[3], 0.0);
        expect(held[4], 0.0);
        expect(
          held[5],
          greaterThan(1.0),
          reason: 'the turn left is still free',
        );
        expect(speedOf(2), lessThanOrEqualTo(3.001));

        expect(physics.transformOf(3)![2], greaterThan(1.0));
        expect(20.0 - heightOf(2), lessThan(20.0 - heightOf(3)));
      },
    );

    test('half gravity falls half as far', () {
      physics.add(
        2,
        shape: const Shape.sphere(0.5),
        at: const [0.0, 50.0, 0.0],
      );
      physics.add(
        3,
        shape: const Shape.sphere(0.5),
        at: const [4.0, 50.0, 0.0],
      );
      physics.setControls(2, const PhysicsControls(gravityScale: 0.5));

      run(0.5);

      // Damping trims both falls a little, so the ratio is not exactly half.
      expect((50.0 - heightOf(2)) / (50.0 - heightOf(3)), closeTo(0.5, 0.03));
    });

    test('a speed cap holds a fall, and a spin cap holds a spin', () {
      physics.add(
        2,
        shape: const Shape.sphere(0.5),
        at: const [0.0, 500.0, 0.0],
      );
      physics.setControls(
        2,
        const PhysicsControls(maxSpeed: 3.0, maxSpin: 2.0),
      );
      physics.drive(2, spin: const [10.0, 0.0, 0.0]);

      // One step, since angular damping would trim a spin already at its cap.
      physics.step(1 / 60);
      expect(physics.velocityOf(2)![3], closeTo(2.0, 0.01));

      run(1.0);
      expect(speedOf(2), closeTo(3.0, 0.01), reason: 'held, not just slowed');
    });

    test('a push goes through the centre of mass unless told otherwise', () {
      const weighted = PhysicsControls(centre: [0.4, 0.0, 0.0]);
      physics.add(2, shape: box, at: const [10.0, 5.0, 0.0]);
      physics.add(3, shape: box, at: const [15.0, 5.0, 0.0]);
      physics.setControls(2, weighted);
      physics.setControls(3, weighted);

      physics.push(2, impulse: const [0.0, 0.0, 1.0]);
      physics.push(
        3,
        impulse: const [0.0, 0.0, 1.0],
        at: const [15.0, 5.0, 0.0],
      );
      physics.step(1 / 60);

      final through = physics.velocityOf(2)!;
      expect(through[2], closeTo(1.0, 0.01));
      expect(through[4].abs(), lessThan(0.001));
      expect(
        physics.velocityOf(3)![4],
        greaterThan(1.0),
        reason: 'through the origin, which is off to one side of the weight',
      );
    });

    test('a given inertia is the one the body turns by', () {
      physics.add(2, shape: box, at: const [0.0, 5.0, 0.0]);
      physics.add(3, shape: box, at: const [5.0, 5.0, 0.0]);
      physics.setControls(
        3,
        const PhysicsControls(inertia: [100.0, 100.0, 100.0]),
      );

      physics.push(
        2,
        impulse: const [0.0, 0.0, 2.0],
        at: const [0.0, 5.5, 0.0],
      );
      physics.push(
        3,
        impulse: const [0.0, 0.0, 2.0],
        at: const [5.0, 5.5, 0.0],
      );
      physics.step(1 / 60);

      expect(physics.velocityOf(2)![3], closeTo(6.0, 0.05));
      expect(physics.velocityOf(3)![3], closeTo(0.01, 0.002));
    });

    test('controls that cannot be used are refused and change nothing', () {
      floor();
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 3.0, 0.0]);

      const half = PhysicsControls(gravityScale: 0.5);
      expect(
        physics.setControls(2, const PhysicsControls(gravityScale: double.nan)),
        isFalse,
      );
      expect(
        physics.setControls(2, const PhysicsControls(maxSpeed: -1.0)),
        isFalse,
      );
      expect(
        physics.setControls(
          2,
          const PhysicsControls(inertia: [-1.0, 1.0, 1.0]),
        ),
        isFalse,
      );
      expect(physics.setControls(9, half), isFalse, reason: 'no such body');
      expect(physics.setControls(ground, half), isFalse, reason: 'not a body');

      run(0.5);
      expect(3.0 - heightOf(2), greaterThan(1.0), reason: 'still full gravity');
      expect(physics.setControls(2, half), isTrue);
    });

    test('the world\'s gravity is set, and wakes what was asleep', () {
      floor();
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 1.0, 0.0]);
      run(3.0);
      expect(physics.asleep(2), isTrue);

      expect(physics.setGravity(const [0.0, 9.81, 0.0]), isTrue);
      run(1.0);

      expect(physics.asleep(2), isFalse);
      expect(heightOf(2), greaterThan(2.0));
      expect(physics.setGravity(const [0.0, double.nan, 0.0]), isFalse);
    });

    test('a body is made fixed or free, and a fixed one stops', () {
      physics.add(
        2,
        shape: box,
        motion: PhysicsMotion.fixed,
        at: const [0.0, 10.0, 0.0],
      );
      run(0.5);
      expect(heightOf(2), 10.0, reason: 'a fixed body does not fall');

      physics.setMotion(2, PhysicsMotion.free);
      run(0.5);
      expect(heightOf(2), lessThan(9.0));

      physics.setMotion(2, PhysicsMotion.fixed);
      physics.step(1 / 60);
      final stopped = heightOf(2);
      run(0.5);
      expect(heightOf(2), stopped);
      expect(physics.velocityOf(2)!.every((it) => it == 0.0), isTrue);
    });

    test('a trigger is never made free', () {
      physics.add(
        2,
        shape: const Shape.box(1.0, 1.0, 1.0),
        motion: PhysicsMotion.fixed,
        at: const [0.0, 10.0, 0.0],
        trigger: true,
      );
      physics.setMotion(2, PhysicsMotion.free);
      run(0.5);
      expect(heightOf(2), 10.0);
    });

    test('controls given quietly to a body added asleep leave it asleep', () {
      physics.add(2, shape: box, at: const [0.0, 10.0, 0.0], asleep: true);
      final took = physics.setControls(
        2,
        const PhysicsControls(gravityScale: 0.5),
        quiet: true,
      );
      run(0.5);

      expect(took, isTrue);
      expect(physics.asleep(2), isTrue);
      expect(heightOf(2), 10.0);
    });

    test('controls given to a sleeper wake it, unless asked not to', () {
      physics.add(2, shape: box, at: const [0.0, 10.0, 0.0], asleep: true);
      physics.setControls(2, const PhysicsControls(gravityScale: 0.5));

      expect(physics.asleep(2), isFalse);
    });

    test('a quiet body keeps its controls for when it wakes', () {
      physics.add(2, shape: box, at: const [0.0, 10.0, 0.0], asleep: true);
      physics.setControls(
        2,
        const PhysicsControls(gravityScale: 0.5),
        quiet: true,
      );
      physics.wake(2);
      run(0.5);

      // Half gravity for half a second falls about 0.61 m, not 1.2.
      expect(10.0 - heightOf(2), closeTo(0.61, 0.08));
    });

    test('two bodies that ignore each other pass through', () {
      floor();
      // Two columns: a box on the ground with another dropped on it. Only the
      // first column is told to ignore.
      void column(int low, int high, double x) {
        physics.add(low, shape: box, at: [x, 0.5, 0.0]);
        physics.add(high, shape: box, at: [x, 3.0, 0.0]);
      }

      column(2, 3, 0.0);
      column(4, 5, 5.0);
      expect(physics.setRule(2, 3, const PhysicsRule(ignore: true)), isTrue);

      run(3.0);

      expect(heightOf(3), closeTo(0.5, 0.05), reason: 'fell through body 2');
      expect(heightOf(5), closeTo(1.5, 0.05), reason: 'stacked as usual');
    });
  });

  group('seeing what happened', () {
    /// Every body the scene below moves, or removes and adds, by name.
    const watched = [2, 3, 4, 5, 7, 10, 21, 30, 50];

    /// Most of what a snapshot has to hold: a stack, a body that sleeps, a
    /// joint, a zone over a body, and a character that is driven every step.
    void richWorld(Physics world) {
      world.add(
        ground,
        shape: const Shape.plane(0.0, 1.0, 0.0),
        motion: PhysicsMotion.fixed,
      );
      world.add(
        2,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [0.0, 0.5, 0.0],
        stay: true,
      );
      world.add(
        3,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [0.0, 1.6, 0.0],
      );
      world.add(
        4,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [0.05, 2.7, 0.0],
      );
      world.add(5, shape: const Shape.sphere(0.5), at: const [3.0, 2.0, 0.0]);
      world.add(
        7,
        shape: const Shape.sphere(0.5),
        at: const [-3.0, 0.5, 0.0],
        asleep: true,
      );
      world.add(10, shape: const Shape.sphere(0.2), at: const [1.0, 6.0, 4.0]);
      world.join(100, const Joint.point(), a: 10, at: const [0.0, 6.0, 4.0]);
      world.add(
        20,
        shape: const Shape.box(2.0, 2.0, 2.0),
        motion: PhysicsMotion.fixed,
        at: const [6.0, 1.0, 0.0],
        trigger: true,
      );
      world.setZone(20, const PhysicsZone(gravity: [0.0, 2.0, 0.0]));
      world.add(21, shape: const Shape.sphere(0.25), at: const [6.0, 1.0, 0.0]);
      world.addCharacter(30, at: const [-6.0, 1.0, 0.0]);
    }

    /// What the game does on step `at`, then the step. Scripted by number, so
    /// a replay does the same thing at the same point of the run.
    void play(Physics world, int at) {
      if (at == 55) {
        world.add(
          50,
          shape: const Shape.sphere(0.3),
          at: const [1.0, 4.0, 1.0],
        );
      }
      if (at == 60) world.remove(3);
      if (at == 70) world.push(7, impulse: const [0.0, 3.0, 0.0]);
      if (at == 75) world.remove(2);

      final left = world.footingOf(30)?.velocity ?? const [0.0, 0.0, 0.0];
      world.drive(30, velocity: [1.0, left[1] - 9.81 / 60, 0.0]);
      world.step(1 / 60);
    }

    /// Everything a caller can read about the world after a step, as text.
    /// Doubles print as the shortest string that reads back as the same
    /// number, so two records are equal only if every bit is.
    String record(Physics world) {
      final out = StringBuffer();
      for (final id in watched) {
        out.write('$id ${world.transformOf(id)} ${world.velocityOf(id)} ');
        out.writeln(world.alive(id) && world.asleep(id));
      }
      for (final it in world.events) {
        out.writeln(
          '${it.kind.name} ${it.a} ${it.b} ${it.at} ${it.normal} ${it.force}',
        );
      }
      final stats = world.stats;
      // Step time is the machine's, and differs between two identical runs.
      out.writeln([
        stats.bodies,
        stats.staticBodies,
        stats.kinematicBodies,
        stats.dynamicBodies,
        stats.asleep,
        stats.triggers,
        stats.characters,
        stats.joints,
        stats.zones,
        stats.rules,
        stats.pairs,
        stats.touching,
        stats.points,
      ]);
      for (final it in world.contacts) {
        out.writeln(
          '${it.a} ${it.b} ${it.at} ${it.normal} ${it.depth} ${it.impulse}',
        );
      }
      return out.toString();
    }

    /// Plays steps `from` up to `to` and keeps a record of each.
    List<String> replay(Physics world, int from, int to) => [
      for (var i = from; i < to; i++)
        () {
          play(world, i);
          return record(world);
        }(),
    ];

    test('its structs are the size the engine reads', () {
      expect(sizeOf<native.OrblitPhysicsContact>(), 48);
      expect(sizeOf<native.OrblitPhysicsStats>(), 56);
    });

    test('a world restored from step 50 reaches the same step 100', () {
      richWorld(physics);
      for (var i = 0; i < 50; i++) {
        play(physics, i);
      }
      final snapshot = physics.snapshot();
      addTearDown(snapshot.dispose);
      final atFifty = record(physics);
      expect(physics.asleep(7), isTrue);

      final first = replay(physics, 50, 100);
      // The run has to do something, or repeating it proves nothing.
      expect(physics.asleep(7), isFalse, reason: 'woken at step 70');
      expect(physics.alive(3), isFalse, reason: 'removed at step 60');
      expect(physics.alive(50), isTrue, reason: 'added at step 55');

      physics.restore(snapshot);

      expect(record(physics), atFifty, reason: 'not the world that was saved');
      expect(physics.asleep(7), isTrue);
      expect(physics.alive(3), isTrue);
      expect(physics.alive(50), isFalse);
      expect(replay(physics, 50, 100), first);

      // The snapshot is left as it was, so it goes back again.
      physics.restore(snapshot);
      expect(replay(physics, 50, 100), first);
    });

    test('a snapshot outlives its world and restores into another', () {
      richWorld(physics);
      for (var i = 0; i < 50; i++) {
        play(physics, i);
      }
      final snapshot = physics.snapshot();
      final expected = replay(physics, 50, 100);
      physics.dispose();

      // Settings that are nothing like the first world's: a restore brings
      // its own, since a replay under different ones would not be a replay.
      final other = Physics(
        settings: const PhysicsSettings(
          gravity: [0.0, -1.0, 0.0],
          velocitySteps: 2,
          positionSteps: 1,
          sleeping: false,
        ),
      );
      addTearDown(other.dispose);
      other.restore(snapshot);
      snapshot.dispose();

      expect(replay(other, 50, 100), expected);
    });

    test('what was queued is in a snapshot and is undone by a restore', () {
      floor();
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 3.0, 0.0]);
      run(0.5);

      physics.add(3, shape: const Shape.sphere(0.5), at: const [4.0, 3.0, 0.0]);
      final snapshot = physics.snapshot();
      addTearDown(snapshot.dispose);
      expect(physics.alive(3), isTrue, reason: 'queued before the copy');

      physics.add(4, shape: const Shape.sphere(0.5), at: const [8.0, 3.0, 0.0]);
      physics.remove(2);
      physics.restore(snapshot);
      physics.step(1 / 60);

      expect(physics.count, 3, reason: 'the ground, 2 and 3');
      expect(physics.alive(2), isTrue);
      expect(physics.alive(3), isTrue);
      expect(physics.alive(4), isFalse, reason: 'queued after the copy');
    });

    test('a snapshot or a world that is gone says so', () {
      floor();
      final snapshot = physics.snapshot();
      snapshot.dispose();
      snapshot.dispose();

      expect(snapshot.disposed, isTrue);
      expect(() => physics.restore(snapshot), throwsStateError);

      final live = physics.snapshot();
      addTearDown(live.dispose);
      final short = Physics()..dispose();
      expect(() => short.snapshot(), throwsStateError);
      expect(() => short.restore(live), throwsStateError);
      expect(() => short.contacts, throwsStateError);
      expect(() => short.stats, throwsStateError);
    });

    test('a resting box lists its contacts, facing out of the second body', () {
      floor();
      physics.add(
        2,
        shape: const Shape.box(0.5, 0.5, 0.5),
        at: const [0, 1, 0],
      );
      // It lands at about a third of a second and sleeps half a second after.
      run(0.5);

      final listed = physics.contacts;

      expect(listed, isNotEmpty);
      for (final it in listed) {
        expect([it.a, it.b], [ground, 2]);
        // Out of the box, towards the ground: down.
        expect(it.normal[1], lessThan(-0.9));
      }
      // Holding a one kilogram box still takes its weight over the step.
      final held = listed.fold<double>(0.0, (sum, it) => sum + it.impulse);
      expect(held, closeTo(9.81 / 60, 0.05));

      final stats = physics.stats;
      expect(stats.points, listed.length);
      expect(stats.touching, 1);

      run(3.0);
      expect(physics.asleep(2), isTrue);
      expect(physics.contacts, isEmpty, reason: 'nothing looked at a sleeper');
    });

    test('the counts add up and survive a restore', () {
      floor();
      physics.add(2, shape: const Shape.sphere(0.5), at: const [0.0, 1.0, 0.0]);
      physics.add(3, shape: const Shape.sphere(0.5), at: const [2.0, 1.0, 0.0]);
      physics.add(
        4,
        shape: const Shape.box(1.0, 1.0, 1.0),
        motion: PhysicsMotion.driven,
        at: const [6.0, 1.0, 0.0],
      );
      physics.add(
        5,
        shape: const Shape.box(1.0, 1.0, 1.0),
        motion: PhysicsMotion.fixed,
        at: const [9.0, 1.0, 0.0],
        trigger: true,
      );
      physics.addCharacter(6, at: const [12.0, 1.0, 0.0]);
      run(0.1);

      final stats = physics.stats;
      expect(stats.bodies, 6);
      expect(stats.staticBodies, 2);
      expect(stats.kinematicBodies, 2, reason: 'a driven body and a character');
      expect(stats.dynamicBodies, 2);
      expect(stats.triggers, 1);
      expect(stats.characters, 1);
      expect(stats.stepMicroseconds, greaterThan(0));

      final snapshot = physics.snapshot();
      addTearDown(snapshot.dispose);
      physics.remove(2);
      physics.remove(3);
      physics.step(1 / 60);
      expect(physics.stats.dynamicBodies, 0);

      physics.restore(snapshot);
      final restored = physics.stats;
      expect(restored.bodies, 6);
      expect(restored.dynamicBodies, 2);
      // The clock belongs to the handle, not to the world that was saved.
      expect(restored.stepMicroseconds, greaterThan(0));
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
