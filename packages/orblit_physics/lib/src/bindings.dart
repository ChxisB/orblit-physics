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

  @Array(3)
  external Array<Uint8> reserved;
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

  @Uint32()
  external int pad;

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
