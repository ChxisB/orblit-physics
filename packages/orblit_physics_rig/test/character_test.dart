import 'dart:typed_data';

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_rig/orblit_physics_rig.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'harness.dart';

const _capsuleId = 800;

void main() {
  late Harness h;
  late RagdollCharacter character;

  setUp(() {
    h = Harness();
    character = RagdollCharacter(
      ragdoll: h.ragdoll,
      id: _capsuleId,
      at: Vector3.zero(),
      pose: h.modifier,
      layers: const Layers(is_: characterLayer),
    );
  });
  tearDown(() => h.dispose());

  Float32List? capsule() => h.physics.transformOf(_capsuleId);

  group('standing', () {
    test('keeps its capsule while the ragdoll follows the pose', () {
      h.run(1);
      final t = capsule()!;
      expect(t[0], closeTo(0, 0.02));
      expect(t[2], closeTo(0, 0.02));
      // Half height, radius and the skin above the ground.
      expect(t[1], closeTo(0.6 + 0.3 + 0.01, 0.05));
      expect(character.isDown, isFalse);
    });

    test('refuses a layer the ragdoll still cares about', () {
      expect(
        () => RagdollCharacter(
          ragdoll: h.ragdoll,
          id: 801,
          at: Vector3.zero(),
          layers: const Layers(is_: 4),
        ),
        throwsA(isA<ArgumentError>()),
      );
    });

    test('has nothing to stand up from', () {
      expect(character.standUp(), isNull);
      expect(capsule(), isNotNull);
    });
  });

  group('hit, limp, and up again', () {
    void knockOnForearm() =>
        character.knock('forearm.L', impulse: Vector3(0, 0, 6), spread: 0.05);

    test('takes the capsule out and goes limp from the hit bone', () {
      knockOnForearm();
      expect(character.isDown, isTrue);
      expect(capsule(), isNull);
      expect(h.ragdoll.modeOf('forearm.L'), RagdollMode.limp);
      expect(h.ragdoll.modeOf('head'), RagdollMode.animated);

      h.run(0.6);
      for (final bone in h.ragdoll.bones) {
        expect(h.ragdoll.modeOf(bone), RagdollMode.limp, reason: bone);
      }
    });

    test('lies on the ground and then stands up where it lies', () {
      knockOnForearm();
      h.run(4);
      expect(h.ragdoll.isAtRest(), isTrue);

      final feet = character.standUp();
      expect(feet, isNotNull);
      expect(feet!.y, closeTo(0, 1e-3));
      expect(capsule(), isNotNull);
      expect(character.isDown, isFalse);
      for (final bone in h.ragdoll.bones) {
        expect(h.ragdoll.modeOf(bone), RagdollMode.animated, reason: bone);
      }
    });

    test('stays down when the ground is out of reach', () {
      knockOnForearm();
      h.run(2);
      expect(character.standUp(reach: 0.01), isNull);
      expect(character.isDown, isTrue);
      expect(capsule(), isNull);
    });

    test('a second blow on a character that is down hits the bodies', () {
      knockOnForearm();
      h.run(2);
      character.knock('head', impulse: Vector3(0, 0, -4), spread: 0);
      expect(character.isDown, isTrue);
      h.run(0.5);
      expect(h.ragdoll.modeOf('head'), RagdollMode.limp);
    });

    test('the pose is mixed from the fall to the get-up', () {
      knockOnForearm();
      h.run(3);
      final fallen = h.headOf('head');

      final feet = character.standUp(blend: 0.4)!;
      h.model = Matrix4.translation(feet);
      expect(h.modifier.recovering, isTrue);

      h.frame();
      expect((h.headOf('head') - fallen).length, lessThan(0.05));

      h.run(0.6);
      expect(h.modifier.recovering, isFalse);
      final up = h.headOf('head');
      expect(up.x, closeTo(feet.x, 1e-3));
      expect(up.y, closeTo(feet.y + 1.45, 1e-3));
      expect(up.z, closeTo(feet.z, 1e-3));
    });
  });
}
