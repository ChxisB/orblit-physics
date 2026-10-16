import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_rig/src/turn_angles.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

void main() {
  late Physics physics;

  setUp(() {
    physics = Physics();
    physics.add(
      1,
      shape: const Shape.sphere(0.1),
      motion: PhysicsMotion.driven,
    );
    physics.add(
      2,
      shape: const Shape.sphere(0.1),
      motion: PhysicsMotion.driven,
    );
    physics.join(
      1,
      const Joint.sixAxis(),
      a: 1,
      b: 2,
      at: const [0.0, 0.0, 0.0],
    );
  });
  tearDown(() => physics.dispose());

  /// What the world reads for a second body turned to [turn] against a first
  /// that has not moved.
  List<double> read(Quaternion turn) {
    physics.place(2, at: const [0.0, 0.0, 0.0], rotation: turn.storage);
    return physics.jointStateOf(1)!.angles;
  }

  Quaternion about(double x, double y, double z, double angle) =>
      Quaternion.axisAngle(Vector3(x, y, z), angle);

  test('a twist is read about x', () {
    expect(turnAngles(about(1, 0, 0, 0.4)), [
      closeTo(0.4, 1e-9),
      closeTo(0, 1e-9),
      closeTo(0, 1e-9),
    ]);
  });

  test('a swing is read about y and z', () {
    final y = turnAngles(about(0, 1, 0, 0.7));
    expect(y[1], closeTo(0.7, 1e-9));
    final z = turnAngles(about(0, 0, 1, -0.5));
    expect(z[2], closeTo(-0.5, 1e-9));
  });

  test('agrees with the solver for a spread of turns', () {
    final turns = <Quaternion>[
      about(1, 0, 0, 0.4),
      about(0, 1, 0, 0.7),
      about(0, 0, 1, -0.5),
      about(0, 1, 0, 0.5) * about(1, 0, 0, 0.3),
      about(0, 0, 1, 0.9) * about(1, 0, 0, -0.6),
      about(1, 1, 0, 1.1),
      about(0, 1, 1, -1.4),
      about(1, 2, 3, 0.8),
      about(0, 1, 0, 2.2) * about(1, 0, 0, 0.4),
    ];
    for (final turn in turns) {
      final expected = read(turn);
      final got = turnAngles(turn);
      for (var i = 0; i < 3; i++) {
        expect(got[i], closeTo(expected[i], 2e-3), reason: '$turn axis $i');
      }
    }
  });

  test('takes the shorter way when the quaternion is the long one', () {
    final turn = about(1, 0, 0, 0.4);
    final long = Quaternion(-turn.x, -turn.y, -turn.z, -turn.w);
    expect(turnAngles(long)[0], closeTo(0.4, 1e-9));
  });
}
