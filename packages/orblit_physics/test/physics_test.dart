import 'dart:typed_data';

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
