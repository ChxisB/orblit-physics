import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_vehicle/orblit_physics_vehicle.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:vector_math/vector_math_64.dart';

import 'pose.dart';

/// A [Vehicle] made from a vehicle component, with the wheel entities it
/// moves.
///
/// A wheel's position in the document is where it sits when the car stands at
/// rest, so the top of its spring is found a little above that, by the sag the
/// spring has under the car's weight.
final class Car {
  Car._(this.vehicle, this.wheels, this._tops, this._travel);

  /// Null when the entity cannot be a car: it has no free body, it is scaled,
  /// or its component, a wheel or the weight are ones the vehicle refuses. A
  /// document with such a car keeps it as data and the body is just a body.
  static Car? make(
    SceneDocument document,
    String id,
    Physics physics,
    int number,
    double gravity,
  ) {
    final entity = document[id];
    final component = entity?[SceneComponents.vehicle];
    final body = entity?[SceneComponents.body];
    if (component is! VehicleComponent || body is! BodyComponent) return null;
    if (body.motion != BodyMotion.free || !_unscaled(document, id)) return null;

    final children = [
      for (final child in document.childrenOf(id))
        if (child[SceneComponents.wheel] is WheelComponent) child,
    ];
    final tops = <Vector3>[];
    final specs = <WheelSpec>[];
    try {
      for (final child in children) {
        final wheel = child[SceneComponents.wheel]! as WheelComponent;
        final at = child[SceneComponents.transform];
        final suspension = SuspensionSpec(
          travel: wheel.travel,
          frequency: wheel.hertz,
          damping: wheel.damping,
        );
        final top =
            (at is TransformComponent ? at.position.clone() : Vector3.zero())
              ..y += wheel.travel - suspension.sag(gravity);
        tops.add(top);
        specs.add(
          WheelSpec(
            mount: _snapped(top - body.centre),
            radius: wheel.radius,
            steers: wheel.steers,
            drives: wheel.drives,
            brakeTorque: wheel.brake,
            handbrake: wheel.handbrake,
            suspension: suspension,
            tyre: TyreSpec(grip: wheel.grip),
          ),
        );
      }
      final centre = body.centreOfMass;
      final spec = VehicleSpec(
        mass: body.mass,
        wheels: specs,
        centre: [centre.x, centre.y, centre.z],
        antiRoll: component.antiRoll,
        maxSteer: radians(component.maxSteer),
        drive: specs.any((wheel) => wheel.drives) ? _driveOf(component) : null,
        ground: Layers(is_: 0, cares: component.ground),
      );
      return Car._(
        Vehicle.attach(physics: physics, id: number, spec: spec),
        [for (final child in children) child.id],
        tops,
        [for (final wheel in specs) wheel.suspension.travel],
      );
    } on ArgumentError {
      return null;
    }
  }

  final Vehicle vehicle;

  /// The wheel entities, in the order of the vehicle's wheels.
  final List<String> wheels;

  /// Where each wheel entity sits with its spring at full reach, in the
  /// chassis entity's own frame, and how far it can drop from there.
  final List<Vector3> _tops;
  final List<double> _travel;

  /// The transform of every wheel entity that is not already as the car has
  /// it: the hub under its spring, turned by the steering and the roll.
  ///
  /// The rotation is the car's to write. Steer turns about the chassis' up,
  /// and the roll about the wheel's own axle, which is a rotation about
  /// minus x for a wheel that rolls forward. The scale is the wheel's.
  Map<String, TransformComponent> poses(SceneDocument document) {
    final changed = <String, TransformComponent>{};
    final states = vehicle.wheels;
    for (var i = 0; i < wheels.length; i++) {
      final had = document[wheels[i]]?[SceneComponents.transform];
      final was = had is TransformComponent ? had : null;
      final squeeze = math.min(states[i].compression, _travel[i]);
      final next = TransformComponent(
        position: _tops[i] - Vector3(0, _travel[i] - squeeze, 0),
        rotation: Vector3(
          -degrees(states[i].roll),
          degrees(states[i].steer),
          0,
        ),
        scale: was?.scale.clone(),
      );
      if (was != null &&
          was.position == next.position &&
          was.rotation == next.rotation) {
        continue;
      }
      changed[wheels[i]] = next;
    }
    return changed;
  }

  static DriveSpec _driveOf(VehicleComponent car) {
    final curve = car.torque;
    return DriveSpec(
      engine: EngineSpec(
        torque: [
          for (var i = 0; i + 1 < curve.length; i += 2)
            (curve[i], curve[i + 1]),
        ],
        idle: car.idle,
      ),
      ratios: car.ratios,
      reverse: car.reverse,
      finalDrive: car.finalDrive,
      lock: car.lock,
    );
  }

  static bool _unscaled(SceneDocument document, String id) {
    final scale = Vector3.zero();
    worldOf(
      document,
      id,
    ).decompose(Vector3.zero(), Quaternion.identity(), scale);
    return (scale - Vector3.all(1)).length < 1e-6;
  }

  /// Wheels at the same z are an axle, and the vehicle compares them exactly.
  /// A tenth of a millimetre is nothing to a car and keeps a hand-placed pair
  /// from falling into two axles of one.
  static Vector3 _snapped(Vector3 mount) =>
      mount..z = (mount.z * 1e4).roundToDouble() / 1e4;
}
