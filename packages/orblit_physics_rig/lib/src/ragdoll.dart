import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_rig/orblit_rig.dart';
import 'package:vector_math/vector_math_64.dart';

import 'frame.dart';
import 'part.dart';
import 'spec.dart';

export 'part.dart' show RagdollMode;

/// How hard a ragdoll holds and how it settles.
final class RagdollTuning {
  const RagdollTuning({
    this.powerHertz = 8,
    this.powerRatio = 1,
    this.linearDamping = 0.1,
    this.angularDamping = 0.5,
    this.snapDistance = 2,
  });

  /// How quickly a powered joint pulls to the pose, in cycles a second.
  final double powerHertz;

  /// Damping of that pull. One settles as fast as it can without overshoot.
  final double powerRatio;

  final double linearDamping;
  final double angularDamping;

  /// A driven body further than this from where the pose puts it, in the
  /// model's units, is put there rather than carried.
  final double snapDistance;
}

/// Where the pose puts one body this frame, in the model's space.
final class _Target {
  _Target(this.bone, this.body, this.scale);

  /// The frame of the body's first bone.
  final Frame bone;

  /// The frame of the body, which is the bone's offset to the capsule.
  final Frame body;

  final Vector3 scale;
}

/// A body for each bone a [RagdollSpec] names, joined as the rig is.
///
/// Each body is [RagdollMode.animated], [RagdollMode.powered] or
/// [RagdollMode.limp], and a bone changes between them whenever it is told
/// to. A character is hit with [hit]. A [RagdollPose] runs [followPose] and
/// [writePose] inside a modifier stack, and they can be called by hand when
/// the ragdoll is run without one.
///
/// Bodies are named by a run of ids from `firstId`, one for each body in the
/// spec's order. A joint has the same number as its body, which is allowed
/// because the two are numbered apart.
///
/// The bodies are one layer that does not collide with itself, so a forearm
/// passes through the thigh it hangs beside and only the world and other
/// things push on it.
final class Ragdoll {
  Ragdoll._({
    required this.physics,
    required this.armature,
    required this.spec,
    required this.layer,
    required this.layers,
    required this.tuning,
    required List<BodyPart> parts,
    required Map<String, int> covering,
  }) : _parts = parts,
       _covering = covering;

  /// Makes bodies and joints for [spec] in [physics], standing where [pose]
  /// puts them under [modelToWorld].
  ///
  /// [pose] has to have been evaluated. Every body starts animated.
  /// [layer] is the one bit the group is in, and [ignore] the layers it takes
  /// no notice of.
  ///
  /// Throws an [ArmatureError] for a bone the pose's armature lacks.
  factory Ragdoll.build({
    required Physics physics,
    required Pose pose,
    required RagdollSpec spec,
    required Matrix4 modelToWorld,
    int firstId = 1,
    int layer = 0x100,
    int ignore = 0,
    RagdollTuning tuning = const RagdollTuning(),
  }) {
    if (layer <= 0 || layer & (layer - 1) != 0) {
      throw ArgumentError.value(layer, 'layer', 'A layer is a single bit.');
    }
    final armature = pose.armature;
    spec.check(armature);
    final frame = ModifierFrame(modelToWorld: modelToWorld);
    final parts = <BodyPart>[];
    final covering = <String, int>{};
    for (final body in spec.bodies) {
      final parent = body.parent == null ? null : parts[covering[body.parent]!];
      final part = BodyPart.of(
        armature,
        body,
        index: parts.length,
        id: firstId + parts.length,
        parent: parent,
        scale: ModelSpace(frame).scale,
      );
      parent?.children.add(part.index);
      for (final name in RagdollSpec.chainOf(armature, body)) {
        covering[name] = part.index;
      }
      parts.add(part);
    }

    final ragdoll = Ragdoll._(
      physics: physics,
      armature: armature,
      spec: spec,
      layer: layer,
      layers: Layers(is_: layer, cares: 0xFFFFFFFF & ~layer & ~ignore),
      tuning: tuning,
      parts: parts,
      covering: covering,
    );
    ragdoll._lay(ModelSpace(frame));
    ragdoll.snap(pose, frame);
    return ragdoll;
  }

  final Physics physics;
  final Armature armature;
  final RagdollSpec spec;
  final RagdollTuning tuning;

  /// The one layer bit every body is in.
  final int layer;

  /// What the bodies are in and care about. Also the layers to cast with when
  /// the ragdoll should not see itself.
  final Layers layers;

  final List<BodyPart> _parts;
  final Map<String, int> _covering;

  List<_Target> _targets = const [];
  double _clock = 0;

  /// The bone each body ends at, root first.
  List<String> get bones => [for (final part in _parts) part.bone];

  /// The physics id of the body that covers [bone].
  int idOf(String bone) => _partFor(bone).id;

  RagdollMode modeOf(String bone) => _partFor(bone).mode;

  /// Whether any body is free, so the ragdoll is writing bones.
  bool get hasFree => _parts.any(_isFree);

  /// Where the root body is, in the world.
  Vector3 get rootPosition => _worldFrameOf(_parts.first).position;

  /// Holds the body that covers [bone] as [mode] says.
  ///
  /// A bone that is powered or limp has its body made free, and its joint
  /// made again with springs for powered and without for limp. The body stays
  /// where it is.
  ///
  /// A body that stays animated below a free one holds that one's far end
  /// where the pose has it, as a hand holds a rail. For a limb that goes
  /// loose from a bone down, use [setFrom].
  void setMode(String bone, RagdollMode mode) => _enter(_partFor(bone), mode);

  /// Holds the body that covers [bone] and every body that hangs below it as
  /// [mode] says.
  void setFrom(String bone, RagdollMode mode) {
    final queue = <int>[_partFor(bone).index];
    for (var i = 0; i < queue.length; i++) {
      final part = _parts[queue[i]];
      _enter(part, mode);
      queue.addAll(part.children);
    }
  }

  void setAll(RagdollMode mode) {
    for (final part in _parts) {
      _enter(part, mode);
    }
  }

  /// Hits the body that covers [bone] with [impulse], newton-seconds in the
  /// world, at the world point [at] or its centre.
  ///
  /// That body goes limp at once. Each body one joint away follows after
  /// [spread] seconds, and the next after the same again, so the blow runs
  /// out from where it landed. A spread of nought makes every body limp now.
  void hit(
    String bone, {
    required Vector3 impulse,
    Vector3? at,
    double spread = 0.04,
  }) {
    final origin = _partFor(bone);
    _enter(origin, RagdollMode.limp);
    physics.push(origin.id, impulse: impulse.storage, at: at?.storage);

    final hops = _hopsFrom(origin);
    for (final part in _parts) {
      if (part.mode == RagdollMode.limp) continue;
      final wait = hops[part.index] * spread;
      if (wait <= 0) {
        _enter(part, RagdollMode.limp);
        continue;
      }
      final soon = part.limpAt;
      if (soon == null || _clock + wait < soon) part.limpAt = _clock + wait;
    }
  }

  /// Whether every free body has all but stopped, which is when a fallen
  /// character is ready to be got up.
  bool isAtRest({double speed = 0.2}) {
    for (final part in _parts) {
      if (!_isFree(part)) continue;
      final v = physics.velocityOf(part.id);
      if (v == null) continue;
      final moving = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      final turning = math.sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
      if (moving > speed || turning > speed * 4) return false;
    }
    return true;
  }

  /// Puts every body where [pose] puts it, with no velocity: the teleport.
  /// Free bodies are put there too.
  void snap(Pose pose, ModifierFrame frame) {
    final space = ModelSpace(frame);
    _targets = _readTargets(pose);
    for (final part in _parts) {
      _place(part, space.toWorld(_targets[part.index].body));
    }
  }

  /// Sends what [pose] asks for to the bodies for the next step: animated
  /// bodies are given the velocity that brings them to it, powered joints are
  /// aimed at its angles, and bodies due to go limp from a hit do.
  ///
  /// [pose] has to be the animated pose, evaluated, and not yet written to by
  /// [writePose]. [ModifierFrame.dt] is the length of the step that follows.
  void followPose(Pose pose, ModifierFrame frame) {
    _targets = _readTargets(pose);
    if (frame.dt <= 0) return;
    _clock += frame.dt;
    for (final part in _parts) {
      final due = part.limpAt;
      if (due != null && due <= _clock) _enter(part, RagdollMode.limp);
    }

    final space = ModelSpace(frame);
    for (final part in _parts) {
      switch (part.mode) {
        case RagdollMode.animated:
          final goal = space.toWorld(_targets[part.index].body);
          _carry(part, goal, space, frame.dt);
        case RagdollMode.powered:
          _hold(part);
        case RagdollMode.limp:
          break;
      }
    }
  }

  /// Writes the bones whose bodies are free from where the bodies are.
  ///
  /// Only the root is moved. Below it a bone is turned and stays on its
  /// parent, so it keeps the length the rig gave it however far a joint has
  /// been pulled. Bones that have no body of their own follow the body above
  /// them with their animated pose.
  void writePose(Pose pose, ModifierFrame frame) {
    final space = ModelSpace(frame);
    for (final part in _parts) {
      if (!_isFree(part)) continue;
      _writeBone(pose, part, space.toModel(_worldFrameOf(part)));
    }
  }

  /// Takes every body and joint out of the world.
  void dispose() {
    for (final part in _parts) {
      physics.remove(part.id);
    }
  }

  void _lay(ModelSpace space) {
    for (final part in _parts) {
      final at = space.toWorld(part.restBody);
      physics.add(
        part.id,
        shape: part.shape,
        motion: PhysicsMotion.driven,
        at: at.position.storage,
        rotation: at.rotation.storage,
        mass: part.body.mass,
        friction: part.body.friction,
        linearDamping: tuning.linearDamping,
        angularDamping: tuning.angularDamping,
        layers: layers,
      );
    }
    for (final part in _parts) {
      if (part.parent != null) _join(part, sprung: false);
    }
  }

  BodyPart _partFor(String bone) {
    String? name = bone;
    while (name != null) {
      final found = _covering[name];
      if (found != null) return _parts[found];
      name = armature[name]?.parent;
    }
    throw ArgumentError.value(bone, 'bone', 'No body covers this bone.');
  }

  bool _isFree(BodyPart part) =>
      part.mode == RagdollMode.limp ||
      (part.mode == RagdollMode.powered && part.parent != null);

  void _enter(BodyPart part, RagdollMode mode) {
    // A root has no joint to spring, so powered is as good as animated.
    final wanted = part.parent == null && mode == RagdollMode.powered
        ? RagdollMode.animated
        : mode;
    part.limpAt = null;
    if (part.mode == wanted) return;
    part.mode = wanted;
    physics.setMotion(
      part.id,
      wanted == RagdollMode.animated
          ? PhysicsMotion.driven
          : PhysicsMotion.free,
    );
    final sprung = wanted == RagdollMode.powered;
    final held = part.parent != null && wanted != RagdollMode.animated;
    if (held && sprung != part.sprung) _join(part, sprung: sprung);
  }

  /// How many joints each body is from [origin], by way of parents and
  /// children alike.
  List<int> _hopsFrom(BodyPart origin) {
    final hops = List<int>.filled(_parts.length, -1);
    final queue = <int>[origin.index];
    hops[origin.index] = 0;
    for (var i = 0; i < queue.length; i++) {
      final part = _parts[queue[i]];
      final next = [if (part.parent != null) part.parent!, ...part.children];
      for (final other in next) {
        if (hops[other] >= 0) continue;
        hops[other] = hops[part.index] + 1;
        queue.add(other);
      }
    }
    return hops;
  }

  Joint _jointFor(BodyPart part, {required bool sprung}) {
    final spring = sprung
        ? JointSpring(hertz: tuning.powerHertz, ratio: tuning.powerRatio)
        : null;
    const locked = JointLimit.locked();
    switch (part.body.joint!) {
      case RagdollHinge(:final low, :final high):
        return Joint.sixAxis(
          alongX: locked,
          alongY: locked,
          alongZ: locked,
          aboutX: JointLimit(low, high),
          aboutY: locked,
          aboutZ: locked,
          springs: JointSprings(aboutX: spring),
        );
      case RagdollBall(:final swing, :final twistLow, :final twistHigh):
        return Joint.sixAxis(
          alongX: locked,
          alongY: locked,
          alongZ: locked,
          aboutX: JointLimit(twistLow, twistHigh),
          aboutY: JointLimit(-swing, swing),
          aboutZ: JointLimit(-swing, swing),
          springs: JointSprings(aboutX: spring, aboutY: spring, aboutZ: spring),
        );
    }
  }

  /// Makes the joint of [part] again, with or without springs.
  ///
  /// A joint holds the turn its bodies stood in when it was made as its
  /// nought, and its limits and springs are measured from there. So the body
  /// is stood at its rest against its parent for a moment, the joint is made,
  /// and the body goes back with the velocity it had. No step runs between,
  /// so nothing sees it move.
  void _join(BodyPart part, {required bool sprung}) {
    final parent = _parts[part.parent!];
    final outer = _worldFrameOf(parent);
    final here = _worldFrameOf(part);
    final moving = physics.velocityOf(part.id);

    _place(part, outer.then(part.restInParent));
    final joint = outer.then(part.jointInParent);
    physics.unjoin(part.id);
    physics.join(
      part.id,
      _jointFor(part, sprung: sprung),
      a: parent.id,
      b: part.id,
      at: joint.position.storage,
      rotation: joint.rotation.storage,
    );
    _place(part, here);
    if (moving != null) {
      physics.drive(
        part.id,
        velocity: [moving[0], moving[1], moving[2]],
        spin: [moving[3], moving[4], moving[5]],
      );
    }
    part.sprung = sprung;
  }

  void _place(BodyPart part, Frame at) {
    physics.place(
      part.id,
      at: at.position.storage,
      rotation: at.rotation.storage,
    );
  }

  Frame _worldFrameOf(BodyPart part) {
    final t = physics.transformOf(part.id);
    if (t == null) {
      throw StateError('Body ${part.id} of the ragdoll has been removed.');
    }
    return Frame(Vector3(t[0], t[1], t[2]), Quaternion(t[3], t[4], t[5], t[6]));
  }

  List<_Target> _readTargets(Pose pose) => [
    for (final part in _parts) _targetOf(part, pose),
  ];

  _Target _targetOf(BodyPart part, Pose pose) {
    final position = Vector3.zero();
    final rotation = Quaternion.identity();
    final scale = Vector3.zero();
    pose.worldOf(part.body.from).decompose(position, rotation, scale);
    final bone = Frame(position, rotation..normalize());
    return _Target(bone, bone.then(part.offset), scale);
  }

  /// Gives an animated body the velocity that brings it to [goal] in one
  /// step, or puts it there if it is further off than a step can mean.
  void _carry(BodyPart part, Frame goal, ModelSpace space, double dt) {
    final here = _worldFrameOf(part);
    final gap = goal.position - here.position;
    if (gap.length > tuning.snapDistance * space.scale) {
      _place(part, goal);
      return;
    }
    final turn = turnBetween(here.rotation, goal.rotation);
    physics.drive(
      part.id,
      velocity: (gap / dt).storage,
      spin: (turn / dt).storage,
    );
  }

  /// Aims a powered joint at the angle the pose has between its two bones.
  void _hold(BodyPart part) {
    final up = part.parent;
    if (up == null) return;
    final above = _departure(_parts[up]);
    final below = _departure(part);
    final joint = part.restFrame;
    final turn = joint.conjugated() * above.conjugated() * below * joint;
    physics.aim(part.id, rest: part.restsFor(turn));
  }

  /// How far the pose has turned the part's first bone from rest, in the
  /// model's space.
  Quaternion _departure(BodyPart part) =>
      _targets[part.index].bone.rotation * part.restBone.rotation.conjugated();

  void _writeBone(Pose pose, BodyPart part, Frame body) {
    final rotation = (body.rotation * part.offset.rotation.conjugated())
      ..normalize();
    final turned = Frame(Vector3.zero(), rotation);
    final head = part.parent == null
        ? body.position - turned.turn(part.offset.position)
        : pose.headOf(part.body.from);
    placeBone(
      pose,
      part.body.from,
      Matrix4.compose(head, rotation, _targets[part.index].scale),
    );
  }
}
