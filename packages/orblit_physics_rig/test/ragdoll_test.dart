import 'package:orblit_physics_rig/orblit_physics_rig.dart';
import 'package:orblit_rig/orblit_rig.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

import 'harness.dart';

void main() {
  late Harness h;

  setUp(() => h = Harness());
  tearDown(() => h.dispose());

  group('building', () {
    test('stands a body for each spec entry where the rig rests', () {
      expect(h.ragdoll.bones, hasLength(humanSpec().bodies.length));
      final torso = h.bodyOf('spine');
      // The torso runs from the hips' head to the spine's tail.
      expect(torso.y, closeTo((1.0 + 1.45) / 2, 1e-4));
      final forearm = h.bodyOf('forearm.L');
      expect(forearm.x, closeTo(0.6, 1e-4));
      expect(forearm.y, closeTo(1.4, 1e-4));
    });

    test('stands where the model is', () {
      final moved = Harness();
      addTearDown(moved.dispose);
      final model = Matrix4.translation(Vector3(2, 0, -3));
      moved.model = model;
      moved.ragdoll.snap(moved.pose, ModifierFrame(modelToWorld: model));
      expect(moved.bodyOf('spine').x, closeTo(2, 1e-4));
      expect(moved.bodyOf('spine').z, closeTo(-3, 1e-4));
    });

    test('has every body animated and out of the character layer', () {
      for (final bone in h.ragdoll.bones) {
        expect(h.ragdoll.modeOf(bone), RagdollMode.animated);
      }
      expect(h.ragdoll.layers.is_, 0x100);
      expect(h.ragdoll.layers.cares & characterLayer, 0);
      expect(h.ragdoll.layers.cares & 0x100, 0);
    });

    test('names bodies from firstId and resolves an uncovered bone', () {
      expect(h.ragdoll.idOf('hips'), h.ragdoll.idOf('spine'));
      expect(h.ragdoll.idOf('spine'), 1);
      expect(() => h.ragdoll.idOf('nowhere'), throwsA(isA<ArgumentError>()));
    });

    test('refuses a bone the armature lacks', () {
      final spec = RagdollSpec([RagdollBody(bone: 'tail', radius: 0.1)]);
      expect(
        () => Ragdoll.build(
          physics: h.physics,
          pose: h.pose,
          spec: spec,
          modelToWorld: Matrix4.identity(),
          firstId: 500,
        ),
        throwsA(isA<ArmatureError>()),
      );
    });

    test('refuses a layer that is not one bit', () {
      expect(
        () => Ragdoll.build(
          physics: h.physics,
          pose: h.pose,
          spec: humanSpec(),
          modelToWorld: Matrix4.identity(),
          firstId: 500,
          layer: 0x300,
        ),
        throwsA(isA<ArgumentError>()),
      );
    });
  });

  group('the spec', () {
    test('needs parents before children and one root', () {
      expect(
        () => RagdollSpec([
          RagdollBody(
            bone: 'head',
            parent: 'spine',
            joint: RagdollBall(),
            radius: 0.1,
          ),
          RagdollBody(bone: 'spine', radius: 0.1),
        ]),
        throwsArgumentError,
      );
      expect(
        () => RagdollSpec([
          RagdollBody(bone: 'spine', radius: 0.1),
          RagdollBody(bone: 'hips', radius: 0.1),
        ]),
        throwsArgumentError,
      );
    });

    test('needs a parent and a joint together', () {
      expect(
        () => RagdollBody(bone: 'head', parent: 'spine', radius: 0.1),
        throwsArgumentError,
      );
    });

    test('refuses a body whose first bone is not above its last', () {
      final spec = RagdollSpec([
        RagdollBody(bone: 'hips', from: 'head', radius: 0.1),
      ]);
      expect(() => spec.check(h.armature), throwsA(isA<ArmatureError>()));
    });

    test('refuses two bodies over one bone', () {
      final spec = RagdollSpec([
        RagdollBody(bone: 'spine', from: 'hips', radius: 0.1),
        RagdollBody(
          bone: 'head',
          from: 'spine',
          parent: 'spine',
          joint: RagdollBall(),
          radius: 0.1,
        ),
      ]);
      expect(() => spec.check(h.armature), throwsA(isA<ArmatureError>()));
    });

    test('an animated body below a free one holds its end in place', () {
      h.ragdoll.setMode('upper_arm.L', RagdollMode.limp);
      h.run(1);
      expect(h.bodyOf('upper_arm.L').y, closeTo(1.4, 0.02));
    });
  });

  group('following the pose', () {
    test('carries animated bodies onto the pose in one step', () {
      h.animate = (pose) {
        pose['upper_arm.L'].rotation = Quaternion.axisAngle(
          Vector3(0, 0, 1),
          0.6,
        );
      };
      h.frame();
      h.frame();
      final head = h.headOf('upper_arm.L');
      final tail = h.model.transformed3(h.pose.tailOf('upper_arm.L'));
      final centre = (head + tail) * 0.5;
      final body = h.bodyOf('upper_arm.L');
      expect((body - centre).length, lessThan(1e-3));
      // The arm really did move, or the test would pass by standing still.
      expect(tail.y, isNot(closeTo(1.4, 0.05)));
    });

    test('puts a body that is far off straight onto the pose', () {
      h.model = Matrix4.translation(Vector3(10, 0, 0));
      h.frame();
      expect(h.bodyOf('spine').x, closeTo(10, 1e-3));
    });

    test('leaves the pose alone while every body is animated', () {
      h.frame();
      expect(h.pose.headOf('spine').y, closeTo(1.15, 1e-6));
      expect(h.ragdoll.hasFree, isFalse);
    });
  });

  group('going limp', () {
    test('a hit spreads outward from the bone that was struck', () {
      h.ragdoll.hit('forearm.L', impulse: Vector3(0, 0, 5), spread: 0.1);
      expect(h.ragdoll.modeOf('forearm.L'), RagdollMode.limp);
      expect(h.ragdoll.modeOf('upper_arm.L'), RagdollMode.animated);

      h.run(0.12);
      expect(h.ragdoll.modeOf('upper_arm.L'), RagdollMode.limp);
      expect(h.ragdoll.modeOf('spine'), RagdollMode.animated);

      h.run(0.12);
      expect(h.ragdoll.modeOf('spine'), RagdollMode.limp);
      expect(h.ragdoll.modeOf('head'), RagdollMode.animated);

      h.run(0.4);
      for (final bone in h.ragdoll.bones) {
        expect(h.ragdoll.modeOf(bone), RagdollMode.limp, reason: bone);
      }
    });

    test('a spread of nought is limp everywhere at once', () {
      h.ragdoll.hit('head', impulse: Vector3(1, 0, 0), spread: 0);
      for (final bone in h.ragdoll.bones) {
        expect(h.ragdoll.modeOf(bone), RagdollMode.limp, reason: bone);
      }
    });

    test('a free arm falls and the other arm stays on the pose', () {
      h.ragdoll.setFrom('upper_arm.L', RagdollMode.limp);
      h.run(1);
      expect(h.pose.tailOf('upper_arm.L').y, lessThan(1.25));
      expect(h.pose.tailOf('upper_arm.R').y, closeTo(1.4, 1e-3));
      expect(h.pose.headOf('spine').y, closeTo(1.15, 1e-3));
    });

    test('the arm below a limp arm travels with it', () {
      h.ragdoll.setFrom('upper_arm.L', RagdollMode.limp);
      h.run(1);
      final elbow = h.pose.tailOf('upper_arm.L');
      final wrist = h.pose.tailOf('forearm.L');
      expect((wrist - h.pose.headOf('forearm.L')).length, closeTo(0.3, 1e-3));
      expect((h.pose.headOf('forearm.L') - elbow).length, lessThan(1e-3));
    });

    test('bones keep their lengths however far the body goes', () {
      h.ragdoll.hit('spine', impulse: Vector3(30, 20, 0), spread: 0);
      h.run(1.5);
      for (final bone in h.armature.bones) {
        final length =
            (h.pose.tailOf(bone.name) - h.pose.headOf(bone.name)).length;
        expect(length, closeTo(bone.length, 1e-3), reason: bone.name);
      }
    });

    test('a torso over two bones keeps the bone between its ends', () {
      h.ragdoll.setMode('spine', RagdollMode.limp);
      h.run(1);
      final hips = h.pose.tailOf('hips');
      expect((h.pose.headOf('spine') - hips).length, lessThan(1e-3));
    });

    test('a limp ragdoll comes to rest on the ground', () {
      h.ragdoll.hit('spine', impulse: Vector3(0, 0, 8), spread: 0.03);
      h.run(3);
      expect(h.ragdoll.isAtRest(), isTrue);
      for (final bone in h.ragdoll.bones) {
        final y = h.bodyOf(bone).y;
        expect(y.isFinite, isTrue, reason: bone);
        // A capsule's centre cannot be lower than its radius.
        expect(y, greaterThan(0.03), reason: bone);
        expect(y, lessThan(0.6), reason: bone);
      }
    });
  });

  group('powered', () {
    test('a hinge holds the angle the pose bends it to', () {
      h.animate = (pose) {
        pose['forearm.L'].rotation = Quaternion.axisAngle(
          Vector3(1, 0, 0),
          0.9,
        );
      };
      h.ragdoll.setMode('forearm.L', RagdollMode.powered);
      h.run(1.5);
      final state = h.physics.jointStateOf(h.ragdoll.idOf('forearm.L'))!;
      expect(state.angles[0], closeTo(0.9, 0.1));
    });

    test('a ball holds the swing the pose gives it', () {
      h.animate = (pose) {
        pose['upper_arm.L'].rotation = Quaternion.axisAngle(
          Vector3(0, 0, 1),
          0.5,
        );
      };
      h.ragdoll.setFrom('upper_arm.L', RagdollMode.powered);
      h.run(1.5);
      final state = h.physics.jointStateOf(h.ragdoll.idOf('upper_arm.L'))!;
      expect(state.angles[2], closeTo(0.5, 0.1));
      expect(state.angles[0], closeTo(0, 0.1));
    });

    test('a hinge stays inside its limits whatever the pose asks', () {
      h.animate = (pose) {
        pose['forearm.L'].rotation = Quaternion.axisAngle(
          Vector3(1, 0, 0),
          -1.0,
        );
      };
      h.ragdoll.setMode('forearm.L', RagdollMode.powered);
      h.run(1);
      final state = h.physics.jointStateOf(h.ragdoll.idOf('forearm.L'))!;
      expect(state.angles[0], greaterThan(-0.05));
    });

    test('a powered body gives under a blow and comes back', () {
      h.ragdoll.setFrom('upper_arm.L', RagdollMode.powered);
      h.run(0.5);
      final held = h.bodyOf('forearm.L').y;
      h.physics.push(
        h.ragdoll.idOf('forearm.L'),
        impulse: const [0.0, -6.0, 0.0],
      );
      h.run(0.1);
      expect(h.bodyOf('forearm.L').y, lessThan(held - 0.02));
      h.run(2);
      expect(h.bodyOf('forearm.L').y, closeTo(held, 0.1));
    });

    test('a powered root is animated', () {
      h.ragdoll.setMode('spine', RagdollMode.powered);
      expect(h.ragdoll.modeOf('spine'), RagdollMode.animated);
    });
  });
}
