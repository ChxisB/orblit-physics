// Three cars: settling, steering and driving. Written as the raw bits of the
// chassis and every wheel. The C# run is `SteppedCarScenes` in
// `dotnet/Orblit.Physics.Vehicle.Tests`, and the pinned lines are
// `Reference/stepped_cars.txt` there. Copy this file to
// `packages/orblit_physics_vehicle/test/` and run it with `dart test`, then
// delete the copy. See README.md.

import 'dart:typed_data';

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_vehicle/orblit_physics_vehicle.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _dt = 1 / 60;

DriveSpec _drive() => DriveSpec(
  engine: EngineSpec(
    torque: [(1000, 160), (3000, 220), (5000, 240), (6500, 200)],
  ),
  ratios: [3.6, 2.2, 1.5, 1.1, 0.9],
  finalDrive: 4.5,
  lock: 50,
);

VehicleSpec _car({DriveSpec? drive}) => VehicleSpec(
  mass: 1200,
  centre: [0, 0, -0.4],
  antiRoll: 20000,
  drive: drive,
  wheels: [
    for (final x in const [-0.8, 0.8])
      for (final z in const [-1.2, 1.2])
        WheelSpec(
          mount: Vector3(x, 0, z),
          radius: 0.35,
          steers: z < 0,
          drives: drive != null && z > 0,
          brakeTorque: 3000,
        ),
  ],
);

String _f32(Float32List l) => [
  for (final w in Uint32List.view(l.buffer, l.offsetInBytes, l.length))
    w.toRadixString(16).padLeft(8, '0'),
].join(' ');

String _f64(List<double> l) {
  final d = ByteData(8 * l.length);
  for (var i = 0; i < l.length; i++) {
    d.setFloat64(8 * i, l[i], Endian.little);
  }
  return [
    for (var i = 0; i < l.length; i++)
      d.getUint32(8 * i + 4, Endian.little).toRadixString(16).padLeft(8, '0') +
          d.getUint32(8 * i, Endian.little).toRadixString(16).padLeft(8, '0'),
  ].join(' ');
}

void _report(String name, Physics physics, Vehicle car) {
  print(
    'car $name chassis T ${_f32(physics.transformOf(1)!)} V ${_f32(physics.velocityOf(1)!)}',
  );
  for (var i = 0; i < 4; i++) {
    final w = car.wheels[i];
    print(
      'car $name wheel$i ${w.grounded} ${_f64([w.compression, w.force, w.spin, w.steer, w.roll])}',
    );
  }
  print('car $name rpm ${_f64([car.rpm])} gear ${car.gear}');
}

Vehicle _make(Physics physics, VehicleSpec spec) {
  physics.add(
    900,
    shape: const Shape.plane(0, 1, 0),
    motion: PhysicsMotion.fixed,
  );
  return Vehicle(
    physics: physics,
    id: 1,
    spec: spec,
    chassis: const Shape.box(0.9, 0.3, 2.0),
    at: [0, 0.6, 0],
  );
}

void _run(Physics physics, Vehicle car, double seconds) {
  for (var i = 0; i < (seconds / _dt).round(); i++) {
    car.update(_dt);
    physics.step(_dt);
  }
}

void main() {
  test('oracle', () {
    var physics = Physics();
    var car = _make(physics, _car());
    _run(physics, car, 3);
    _report('settle', physics, car);
    physics.dispose();

    physics = Physics();
    car = _make(physics, _car());
    _run(physics, car, 2);
    physics.drive(1, velocity: [0, 0, -3]);
    _run(physics, car, 1);
    car.setInput(VehicleInput(steer: 0.6));
    _run(physics, car, 2);
    _report('steer', physics, car);
    physics.dispose();

    physics = Physics();
    car = _make(physics, _car(drive: _drive()));
    _run(physics, car, 2);
    car.setInput(VehicleInput(throttle: 1));
    _run(physics, car, 4);
    _report('drive', physics, car);
    physics.dispose();
  });
}
