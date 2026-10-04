// Raw FFI declarations for the engine's C ABI. Nothing here interprets a
// result or owns a lifetime — see world.dart for that.

import 'dart:ffi';

const String kOrblitPhysicsAsset = 'package:orblit_physics/orblit_physics';

final class OrblitPhysicsStruct extends Opaque {}

/// `OrblitPhysicsCommand`, field for field.
final class OrblitPhysicsCommand extends Struct {
  @Uint32()
  external int kind;

  @Uint32()
  external int shape;

  @Uint64()
  external int id;

  @Array(4)
  external Array<Float> size;

  @Array(3)
  external Array<Float> at;

  @Array(4)
  external Array<Float> rotation;

  @Array(3)
  external Array<Float> vector;

  @Array(3)
  external Array<Float> spin;

  @Uint32()
  external int motion;

  @Float()
  external double mass;

  @Float()
  external double friction;

  @Float()
  external double restitution;

  @Array(2)
  external Array<Float> damping;

  @Uint32()
  external int layerIs;

  @Uint32()
  external int layerCares;

  @Bool()
  external bool asleep;

  @Bool()
  external bool sensor;

  @Bool()
  external bool stay;

  @Array(1)
  external Array<Uint8> reserved;

  @Uint64()
  external int hull;
}

/// `OrblitPhysicsPart`, field for field.
final class OrblitPhysicsPart extends Struct {
  @Uint32()
  external int shape;
  @Uint32()
  external int reserved;
  @Uint64()
  external int hull;
  @Array(4)
  external Array<Float> size;
  @Array(3)
  external Array<Float> at;
  @Array(9)
  external Array<Float> linear;
}

/// `OrblitPhysicsEvent`, field for field.
final class OrblitPhysicsEvent extends Struct {
  @Uint32()
  external int kind;

  @Uint32()
  external int pad;

  @Uint64()
  external int a;

  @Uint64()
  external int b;

  @Array(3)
  external Array<Float> at;

  @Array(3)
  external Array<Float> normal;

  @Float()
  external double force;
}

/// `OrblitPhysicsCast`, field for field.
final class OrblitPhysicsCast extends Struct {
  @Uint32()
  external int shape;

  @Uint32()
  external int layerIs;

  @Uint32()
  external int layerCares;

  @Bool()
  external bool triggers;

  @Array(3)
  external Array<Uint8> reserved;

  @Array(4)
  external Array<Float> size;

  @Array(3)
  external Array<Float> from;

  @Array(4)
  external Array<Float> rotation;

  @Array(3)
  external Array<Float> direction;

  @Float()
  external double distance;

  @Uint64()
  external int ignore;

  @Uint64()
  external int hull;
}

/// `OrblitPhysicsHit`, field for field.
final class OrblitPhysicsHit extends Struct {
  @Uint64()
  external int body;

  @Array(3)
  external Array<Float> at;

  @Array(3)
  external Array<Float> normal;

  @Float()
  external double distance;

  @Bool()
  external bool started;

  @Array(3)
  external Array<Uint8> reserved;
}

/// `OrblitPhysicsGround`, field for field.
final class OrblitPhysicsGround extends Struct {
  @Uint64()
  external int id;

  external Pointer<Float> heights;

  @Uint32()
  external int columns;

  @Uint32()
  external int rows;

  @Float()
  external double spacing;

  @Bool()
  external bool margin;

  @Array(3)
  external Array<Uint8> reserved;

  @Array(3)
  external Array<Float> at;

  @Float()
  external double friction;

  @Float()
  external double restitution;

  @Uint32()
  external int layerIs;

  @Uint32()
  external int layerCares;

  @Uint32()
  external int pad;
}

/// `OrblitPhysicsFooting`, field for field.
final class OrblitPhysicsFooting extends Struct {
  @Uint64()
  external int ground;

  @Array(3)
  external Array<Float> normal;

  @Array(3)
  external Array<Float> velocity;

  @Array(3)
  external Array<Float> carried;

  @Float()
  external double turning;

  @Bool()
  external bool grounded;

  @Array(7)
  external Array<Uint8> reserved;
}

/// `OrblitPhysicsJoint`, field for field.
final class OrblitPhysicsJoint extends Struct {
  @Uint64()
  external int id;

  @Uint64()
  external int a;

  @Uint64()
  external int b;

  @Uint32()
  external int kind;

  @Uint32()
  external int limited;

  @Array(3)
  external Array<Float> at;

  @Array(4)
  external Array<Float> rotation;

  @Array(3)
  external Array<Float> to;

  @Array(6)
  external Array<Float> low;

  @Array(6)
  external Array<Float> high;

  @Float()
  external double swing;

  @Float()
  external double speed;

  @Float()
  external double strength;

  @Float()
  external double breakingForce;

  @Float()
  external double breakingTorque;

  @Bool()
  external bool collide;

  @Array(3)
  external Array<Uint8> reserved;
}

/// `OrblitPhysicsJointState`, field for field.
final class OrblitPhysicsJointState extends Struct {
  @Array(3)
  external Array<Float> offset;

  @Array(3)
  external Array<Float> angles;

  @Float()
  external double force;

  @Float()
  external double torque;
}

/// `OrblitPhysicsZone`, field for field.
final class OrblitPhysicsZone extends Struct {
  @Uint64()
  external int body;

  @Uint32()
  external int overrides;

  @Int32()
  external int priority;

  @Array(3)
  external Array<Float> gravity;

  @Array(2)
  external Array<Float> damping;
}

/// `OrblitPhysicsRule`, field for field.
final class OrblitPhysicsRule extends Struct {
  @Uint64()
  external int a;

  @Uint64()
  external int b;

  @Uint32()
  external int overrides;

  @Float()
  external double friction;

  @Float()
  external double restitution;

  @Array(2)
  external Array<Float> moveScale;
}

/// `OrblitPhysicsControls`, field for field.
final class OrblitPhysicsControls extends Struct {
  @Uint64()
  external int body;

  @Uint32()
  external int locks;

  @Float()
  external double gravityScale;

  @Float()
  external double maxSpeed;

  @Float()
  external double maxSpin;

  @Array(3)
  external Array<Float> centre;

  @Array(3)
  external Array<Float> inertia;

  @Bool()
  external bool quiet;
}

/// `OrblitPhysicsSettings`, field for field.
final class OrblitPhysicsSettings extends Struct {
  @Array(3)
  external Array<Float> gravity;

  @Uint32()
  external int velocitySteps;

  @Uint32()
  external int positionSteps;

  @Float()
  external double slop;

  @Float()
  external double stiffness;

  @Float()
  external double bounceThreshold;

  @Float()
  external double sleepSpeed;

  @Float()
  external double sleepAfter;

  @Bool()
  external bool sleeping;

  @Array(3)
  external Array<Uint8> reserved;
}

/// `OrblitPhysicsSnapshot`, which Dart only ever holds a pointer to.
final class OrblitPhysicsSnapshotStruct extends Opaque {}

/// `OrblitPhysicsContact`, field for field.
final class OrblitPhysicsContact extends Struct {
  @Uint64()
  external int a;

  @Uint64()
  external int b;

  @Array(3)
  external Array<Float> at;

  @Array(3)
  external Array<Float> normal;

  @Float()
  external double depth;

  @Float()
  external double impulse;
}

/// `OrblitPhysicsStats`, field for field.
final class OrblitPhysicsStats extends Struct {
  @Uint32()
  external int bodies;

  @Uint32()
  external int staticBodies;

  @Uint32()
  external int kinematicBodies;

  @Uint32()
  external int dynamicBodies;

  @Uint32()
  external int asleep;

  @Uint32()
  external int triggers;

  @Uint32()
  external int characters;

  @Uint32()
  external int joints;

  @Uint32()
  external int zones;

  @Uint32()
  external int rules;

  @Uint32()
  external int pairs;

  @Uint32()
  external int touching;

  @Uint32()
  external int points;

  @Float()
  external double stepMicroseconds;
}

@Native<Void Function(Pointer<OrblitPhysicsSettings>)>(
  symbol: 'orblit_physics_defaults',
  assetId: kOrblitPhysicsAsset,
)
external void physicsDefaults(Pointer<OrblitPhysicsSettings> out);

@Native<Pointer<OrblitPhysicsStruct> Function(Pointer<OrblitPhysicsSettings>)>(
  symbol: 'orblit_physics_create',
  assetId: kOrblitPhysicsAsset,
)
external Pointer<OrblitPhysicsStruct> physicsCreate(
  Pointer<OrblitPhysicsSettings> settings,
);

@Native<Void Function(Pointer<OrblitPhysicsStruct>)>(
  symbol: 'orblit_physics_destroy',
  assetId: kOrblitPhysicsAsset,
)
external void physicsDestroy(Pointer<OrblitPhysicsStruct> physics);

@Native<
  Void Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<OrblitPhysicsCommand>,
    Uint32,
  )
>(symbol: 'orblit_physics_submit', assetId: kOrblitPhysicsAsset)
external void physicsSubmit(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsCommand> commands,
  int count,
);

@Native<Void Function(Pointer<OrblitPhysicsStruct>, Float)>(
  symbol: 'orblit_physics_step',
  assetId: kOrblitPhysicsAsset,
)
external void physicsStep(Pointer<OrblitPhysicsStruct> physics, double delta);

@Native<
  Uint32 Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<Uint64>,
    Uint32,
    Pointer<Float>,
    Uint32,
    Uint32,
  )
>(symbol: 'orblit_physics_read', assetId: kOrblitPhysicsAsset)
external int physicsRead(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<Uint64> ids,
  int count,
  Pointer<Float> out,
  int stride,
  int offset,
);

@Native<
  Pointer<OrblitPhysicsEvent> Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<Uint32>,
  )
>(symbol: 'orblit_physics_events', assetId: kOrblitPhysicsAsset)
external Pointer<OrblitPhysicsEvent> physicsEvents(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<Uint32> count,
);

@Native<Uint32 Function(Pointer<OrblitPhysicsStruct>)>(
  symbol: 'orblit_physics_count',
  assetId: kOrblitPhysicsAsset,
)
external int physicsCount(Pointer<OrblitPhysicsStruct> physics);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64)>(
  symbol: 'orblit_physics_alive',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsAlive(Pointer<OrblitPhysicsStruct> physics, int id);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64)>(
  symbol: 'orblit_physics_asleep',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsAsleep(Pointer<OrblitPhysicsStruct> physics, int id);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64, Pointer<Float>)>(
  symbol: 'orblit_physics_transform',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsTransform(
  Pointer<OrblitPhysicsStruct> physics,
  int id,
  Pointer<Float> out,
);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64, Pointer<Float>)>(
  symbol: 'orblit_physics_velocity',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsVelocity(
  Pointer<OrblitPhysicsStruct> physics,
  int id,
  Pointer<Float> out,
);

@Native<
  Bool Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<OrblitPhysicsCast>,
    Pointer<OrblitPhysicsHit>,
  )
>(symbol: 'orblit_physics_cast', assetId: kOrblitPhysicsAsset)
external bool physicsCast(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsCast> cast,
  Pointer<OrblitPhysicsHit> out,
);

@Native<
  Uint32 Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<OrblitPhysicsCast>,
    Pointer<OrblitPhysicsHit>,
    Uint32,
  )
>(symbol: 'orblit_physics_cast_all', assetId: kOrblitPhysicsAsset)
external int physicsCastAll(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsCast> cast,
  Pointer<OrblitPhysicsHit> out,
  int capacity,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsCast>)
>(symbol: 'orblit_physics_cast_any', assetId: kOrblitPhysicsAsset)
external bool physicsCastAny(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsCast> cast,
);

@Native<
  Uint32 Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<OrblitPhysicsCast>,
    Pointer<Uint64>,
    Uint32,
  )
>(symbol: 'orblit_physics_overlap', assetId: kOrblitPhysicsAsset)
external int physicsOverlap(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsCast> cast,
  Pointer<Uint64> out,
  int capacity,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsZone>)
>(symbol: 'orblit_physics_zone', assetId: kOrblitPhysicsAsset)
external bool physicsZone(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsZone> zone,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsRule>)
>(symbol: 'orblit_physics_rule', assetId: kOrblitPhysicsAsset)
external bool physicsRule(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsRule> rule,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsControls>)
>(symbol: 'orblit_physics_controls', assetId: kOrblitPhysicsAsset)
external bool physicsControls(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsControls> controls,
);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<Float>)>(
  symbol: 'orblit_physics_gravity',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsGravity(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<Float> gravity,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsGround>)
>(symbol: 'orblit_physics_ground', assetId: kOrblitPhysicsAsset)
external bool physicsGround(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsGround> ground,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Uint64, Pointer<Float>, Uint32)
>(symbol: 'orblit_physics_hull', assetId: kOrblitPhysicsAsset)
external bool physicsHull(
  Pointer<OrblitPhysicsStruct> physics,
  int id,
  Pointer<Float> xyz,
  int count,
);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64)>(
  symbol: 'orblit_physics_hull_drop',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsHullDrop(Pointer<OrblitPhysicsStruct> physics, int id);

@Native<
  Uint32 Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<Uint64>,
    Uint32,
    Pointer<OrblitPhysicsFooting>,
  )
>(symbol: 'orblit_physics_footing', assetId: kOrblitPhysicsAsset)
external int physicsFooting(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<Uint64> ids,
  int count,
  Pointer<OrblitPhysicsFooting> out,
);

@Native<
  Bool Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsJoint>)
>(symbol: 'orblit_physics_join', assetId: kOrblitPhysicsAsset)
external bool physicsJoin(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsJoint> joint,
);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64)>(
  symbol: 'orblit_physics_unjoin',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsUnjoin(Pointer<OrblitPhysicsStruct> physics, int joint);

@Native<
  Bool Function(
    Pointer<OrblitPhysicsStruct>,
    Uint64,
    Pointer<OrblitPhysicsJointState>,
  )
>(symbol: 'orblit_physics_joint', assetId: kOrblitPhysicsAsset)
external bool physicsJoint(
  Pointer<OrblitPhysicsStruct> physics,
  int joint,
  Pointer<OrblitPhysicsJointState> out,
);

@Native<
  Pointer<OrblitPhysicsSnapshotStruct> Function(Pointer<OrblitPhysicsStruct>)
>(symbol: 'orblit_physics_snapshot', assetId: kOrblitPhysicsAsset)
external Pointer<OrblitPhysicsSnapshotStruct> physicsSnapshot(
  Pointer<OrblitPhysicsStruct> physics,
);

@Native<
  Bool Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<OrblitPhysicsSnapshotStruct>,
  )
>(symbol: 'orblit_physics_restore', assetId: kOrblitPhysicsAsset)
external bool physicsRestore(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsSnapshotStruct> snapshot,
);

@Native<Void Function(Pointer<OrblitPhysicsSnapshotStruct>)>(
  symbol: 'orblit_physics_snapshot_destroy',
  assetId: kOrblitPhysicsAsset,
)
external void physicsSnapshotDestroy(
  Pointer<OrblitPhysicsSnapshotStruct> snapshot,
);

@Native<
  Uint32 Function(
    Pointer<OrblitPhysicsStruct>,
    Pointer<OrblitPhysicsContact>,
    Uint32,
  )
>(symbol: 'orblit_physics_contacts', assetId: kOrblitPhysicsAsset)
external int physicsContacts(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsContact> out,
  int capacity,
);

@Native<
  Void Function(Pointer<OrblitPhysicsStruct>, Pointer<OrblitPhysicsStats>)
>(symbol: 'orblit_physics_stats', assetId: kOrblitPhysicsAsset)
external void physicsStats(
  Pointer<OrblitPhysicsStruct> physics,
  Pointer<OrblitPhysicsStats> out,
);

@Native<
  Bool Function(
    Pointer<OrblitPhysicsStruct>,
    Uint64,
    Pointer<OrblitPhysicsPart>,
    Uint32,
  )
>(symbol: 'orblit_physics_compound', assetId: kOrblitPhysicsAsset)
external bool physicsCompound(
  Pointer<OrblitPhysicsStruct> physics,
  int id,
  Pointer<OrblitPhysicsPart> parts,
  int count,
);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64)>(
  symbol: 'orblit_physics_compound_drop',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsCompoundDrop(Pointer<OrblitPhysicsStruct> physics, int id);

@Native<
  Bool Function(
    Pointer<OrblitPhysicsStruct>,
    Uint64,
    Pointer<Float>,
    Uint32,
    Pointer<Uint32>,
    Uint32,
  )
>(symbol: 'orblit_physics_mesh', assetId: kOrblitPhysicsAsset)
external bool physicsMesh(
  Pointer<OrblitPhysicsStruct> physics,
  int id,
  Pointer<Float> xyz,
  int vertices,
  Pointer<Uint32> indices,
  int count,
);

@Native<Bool Function(Pointer<OrblitPhysicsStruct>, Uint64)>(
  symbol: 'orblit_physics_mesh_drop',
  assetId: kOrblitPhysicsAsset,
)
external bool physicsMeshDrop(Pointer<OrblitPhysicsStruct> physics, int id);
