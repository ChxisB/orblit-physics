import 'dart:math';
import 'dart:typed_data';

import 'package:orblit_physics_scene/src/pose.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

Matrix3 turnOf(Vector3 degrees) => rotationOf(degrees).getRotation();

/// Whether two rotations are the same turn, whatever angles wrote them.
Matcher sameTurnAs(Matrix3 expected, {double within = 1e-9}) =>
    predicate<Matrix3>((actual) {
      for (var i = 0; i < 9; i++) {
        final apart = (actual.storage[i] - expected.storage[i]).abs();
        if (apart > within) return false;
      }
      return true;
    }, 'the same rotation as $expected');

/// A pose as the world reports it: translation xyz then rotation xyzw.
Float32List poseOf(Quaternion turn, [Vector3? at]) {
  final p = at ?? Vector3.zero();
  return Float32List.fromList([p.x, p.y, p.z, turn.x, turn.y, turn.z, turn.w]);
}

Quaternion aboutY(double angle) =>
    Quaternion.axisAngle(Vector3(0, 1, 0), radians(angle));

/// Degrees between the rotations of two poses, whichever sign each is written
/// with.
double apart(Float32List a, Float32List b) {
  var dot = 0.0;
  for (var i = 3; i < 7; i++) {
    dot += a[i] * b[i];
  }
  return degrees(2 * acos(dot.abs().clamp(0.0, 1.0)));
}

Float32List blended(Float32List from, Float32List to, double alpha) {
  final out = Float32List(7);
  blendPose(from, to, alpha, out);
  return out;
}

void main() {
  group('angles from a rotation', () {
    test('are angles for the same rotation, whatever it was', () {
      final random = Random(7);
      for (var i = 0; i < 500; i++) {
        final angles = Vector3(
          random.nextDouble() * 720 - 360,
          random.nextDouble() * 720 - 360,
          random.nextDouble() * 720 - 360,
        );
        final turn = turnOf(angles);

        expect(turnOf(eulerOf(turn)), sameTurnAs(turn), reason: '$angles');
        expect(
          turnOf(eulerOf(turn, near: angles)),
          sameTurnAs(turn),
          reason: '$angles near itself',
        );
      }
    });

    test('are the angles it was given when those are nearby', () {
      final random = Random(11);
      for (var i = 0; i < 500; i++) {
        final angles = Vector3(
          random.nextDouble() * 720 - 360,
          random.nextDouble() * 170 - 85,
          random.nextDouble() * 720 - 360,
        );
        final found = eulerOf(turnOf(angles), near: angles);

        expect(found.x, closeTo(angles.x, 1e-6), reason: '$angles');
        expect(found.y, closeTo(angles.y, 1e-6), reason: '$angles');
        expect(found.z, closeTo(angles.z, 1e-6), reason: '$angles');
      }
    });

    test('take the other writing when that is the one nearby', () {
      // (180, 150, 180) is (0, 30, 0) written the other way round.
      final near = Vector3(179, 151, 181);
      final found = eulerOf(turnOf(Vector3(0, 30, 0)), near: near);

      expect(found.x, closeTo(180, 1e-6));
      expect(found.y, closeTo(150, 1e-6));
      expect(found.z, closeTo(180, 1e-6));
    });

    test('keep counting past a whole turn', () {
      // A wheel on its fourth revolution says so, rather than starting again.
      final found = eulerOf(
        turnOf(Vector3(0, 0, 1090)),
        near: Vector3(0, 0, 1085),
      );

      expect(found.z, closeTo(1090, 1e-6));
    });

    test('turned a quarter about Y, X keeps the angle it had', () {
      // Here X and Z turn about the same axis, and a crate turned to face
      // sideways would otherwise show two angles that are only rounding.
      for (final angles in [
        Vector3(0, 90, 0),
        Vector3(30, 90, 10),
        Vector3(0, -90, 45),
        Vector3(-20, -90, 70),
      ]) {
        final turn = turnOf(angles);

        final alone = eulerOf(turn);
        expect(turnOf(alone), sameTurnAs(turn), reason: '$angles');
        expect(alone.x, 0, reason: '$angles');

        final kept = eulerOf(turn, near: angles);
        expect(turnOf(kept), sameTurnAs(turn), reason: '$angles near itself');
        expect(kept.x, closeTo(angles.x, 1e-9), reason: '$angles');
        expect(kept.z, closeTo(angles.z, 1e-6), reason: '$angles');
      }
    });

    test('just short of a quarter turn about Y are still the rotation', () {
      for (final off in [1e-3, 1e-5, 1e-7, 1e-9]) {
        final angles = Vector3(40, 90 - off, -25);
        final turn = turnOf(angles);

        // Without the angles it had, X is let go a millionth short of the
        // quarter turn, and the rotation is right to within that.
        expect(
          turnOf(eulerOf(turn)),
          sameTurnAs(turn, within: 1e-6),
          reason: '$off off',
        );
        expect(
          turnOf(eulerOf(turn, near: angles)),
          sameTurnAs(turn),
          reason: '$off off, near itself',
        );
      }
    });
  });

  group('a blend of two poses', () {
    final from = poseOf(
      Quaternion.fromRotation(turnOf(Vector3(30, 40, 50))),
      Vector3(1, 2, 3),
    );
    final to = poseOf(
      Quaternion.fromRotation(turnOf(Vector3(35, 60, 20))),
      Vector3(11, -18, 33),
    );

    test('is the first pose at 0 and the second at 1', () {
      final start = blended(from, to, 0);
      final end = blended(from, to, 1);

      for (var i = 0; i < 7; i++) {
        expect(start[i], closeTo(from[i], 1e-6), reason: 'start $i');
        expect(end[i], closeTo(to[i], 1e-6), reason: 'end $i');
      }
    });

    test('moves in a straight line', () {
      final out = blended(from, to, 0.25);

      expect(out[0], closeTo(3.5, 1e-6));
      expect(out[1], closeTo(-3, 1e-6));
      expect(out[2], closeTo(10.5, 1e-6));
    });

    test('turns halfway through half the turn', () {
      final out = blended(poseOf(aboutY(0)), poseOf(aboutY(90)), 0.5);

      final expected = poseOf(aboutY(45));
      for (var i = 3; i < 7; i++) {
        expect(out[i], closeTo(expected[i], 1e-6), reason: 'part $i');
      }
    });

    test('turns at an even rate', () {
      final start = poseOf(aboutY(0));
      final end = poseOf(aboutY(90));

      for (final alpha in [0.1, 0.25, 0.5, 0.75, 0.9]) {
        final out = blended(start, end, alpha);

        expect(apart(out, start), closeTo(90 * alpha, 0.01), reason: '$alpha');
      }
    });

    test('is a unit rotation all the way through', () {
      for (var i = 0; i <= 20; i++) {
        final out = blended(from, to, i / 20);

        final length = sqrt(
          out[3] * out[3] + out[4] * out[4] + out[5] * out[5] + out[6] * out[6],
        );
        expect(length, closeTo(1, 1e-6), reason: '$i of 20');
      }
    });

    test('takes the short way when one rotation is written negated', () {
      // Q and -Q are the same turn. Written the far way, the blend between
      // them would go nearly all the way round the other side.
      final near = poseOf(aboutY(10));
      for (var i = 3; i < 7; i++) {
        near[i] = -near[i];
      }

      final out = blended(poseOf(aboutY(0)), near, 0.5);

      final expected = poseOf(aboutY(5));
      for (var i = 3; i < 7; i++) {
        expect(out[i], closeTo(expected[i], 1e-6), reason: 'part $i');
      }
    });

    test('copies a rotation that has not changed, as it is', () {
      // Not a unit rotation, so any arithmetic on it shows. A body at rest
      // has the same turn in both poses and must not read as having moved.
      final still = Float32List.fromList([0, 0, 0, 0.5, 0.5, 0.5, 0.6]);
      final moved = Float32List.fromList([4, 0, 0, 0.5, 0.5, 0.5, 0.6]);

      final out = blended(still, moved, 0.3);

      expect(out.sublist(3), still.sublist(3));
      expect(out[0], closeTo(1.2, 1e-6));
    });

    test('goes straight across between rotations too close for an arc', () {
      final start = poseOf(aboutY(0));
      final end = poseOf(aboutY(0.05));

      final out = blended(start, end, 0.5);

      expect(out.every((value) => value.isFinite), isTrue);
      expect(out[4], closeTo((start[4] + end[4]) / 2, 1e-6));
      expect(out[6], closeTo(1, 1e-6));
    });

    test('writes into the seven floats it is given and no others', () {
      final all = Float32List(21)..fillRange(0, 21, 9);
      final window = Float32List.sublistView(all, 7, 14);

      blendPose(from, to, 0.5, window);

      expect(all.sublist(0, 7), everyElement(9));
      expect(all.sublist(14), everyElement(9));
      expect(window, blended(from, to, 0.5));
    });
  });

  group('where an entity is', () {
    test('is under every one of its parents', () {
      final document = SceneDocument(
        name: 'Scene',
        entities: [
          SceneEntity(
            id: 'cart',
            name: 'Cart',
            components: {
              SceneComponents.transform: TransformComponent(
                position: Vector3(10, 0, 0),
                rotation: Vector3(0, 90, 0),
                scale: Vector3.all(2),
              ),
            },
          ),
          SceneEntity(
            id: 'wheel',
            name: 'Wheel',
            parent: 'cart',
            components: {
              SceneComponents.transform: TransformComponent(
                position: Vector3(1, 0, 0),
              ),
            },
          ),
        ],
      );

      // A quarter turn about Y takes +X to -Z, and the cart's scale doubles
      // the distance.
      final at = worldOf(document, 'wheel').getTranslation();
      expect(at.x, closeTo(10, 1e-9));
      expect(at.z, closeTo(-2, 1e-9));
    });

    test('an entity it does not know, or nothing, is where the world is', () {
      final document = SceneDocument(name: 'Scene', entities: const []);

      expect(worldOf(document, 'nobody'), Matrix4.identity());
      expect(worldOf(document, null), Matrix4.identity());
    });

    test('a folder with no transform moves nothing in it', () {
      final document = SceneDocument(
        name: 'Scene',
        entities: [
          const SceneEntity(id: 'folder', name: 'Folder'),
          SceneEntity(
            id: 'crate',
            name: 'Crate',
            parent: 'folder',
            components: {
              SceneComponents.transform: TransformComponent(
                position: Vector3(1, 2, 3),
              ),
            },
          ),
        ],
      );

      expect(worldOf(document, 'crate').getTranslation(), Vector3(1, 2, 3));
    });
  });
}
