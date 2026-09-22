import 'dart:math';

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
          SceneEntity(id: 'folder', name: 'Folder'),
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
