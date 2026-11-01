# Changelog

## 0.4.0

The version is the one the package had when the port began.

- `Vehicle.Attach` makes a car for a chassis body that is already in the
  world. `chassis` and `layers` belong to the `Vehicle` constructor, which
  makes the body, so a spec does not say what a body is.
- `WheelState.Roll` is how far the wheel has turned, from nought to a full
  turn, for a model of the tyre to follow.
- `SuspensionSpec.Sag` is how far the spring is squeezed once it holds its
  weight, so a wheel placed at rest can be turned into a mount.
- `DriveSpec` names its differential lock `diffLock`, since `lock` is a
  keyword. `DriveSpec.Lock` is the property.

## 0.3.0

- `EngineSpec` is a torque curve with an idle, an inertia and a drag. `DriveSpec`
  adds the gears, the final drive, a centrifugal clutch, a differential and an
  automatic box. `VehicleSpec.Drive` and `WheelSpec.Drives` wire them to the
  wheels. A drive with no driven wheel, or a driven wheel with no drive, is
  refused.
- `VehicleInput` takes `Throttle` from 0 to 1 and `Gear`. Null picks forward
  gears by itself. Minus one is reverse, and a gear the car lacks is refused.
- `Vehicle.Rpm` and `Vehicle.Gear`.
- The clutch is solved against how each tyre will answer a torque, by
  bisection. A wheel on the road, a wheel in the air and a wheel on ice all
  settle without a step to step flip.
- The box shifts on the speed the wheels give the engine, not the engine's
  own, and rev matches through a shift. The clutch is open just above idle, so
  a car with the pedal up stays put.

## 0.2.0

- `TyreSpec` and a tyre model. A tyre holds its patch to the ground with
  bristles that bend as the patch slips, so a parked car does not creep and a
  turning car gets a steady side force. The force is limited by grip times
  load, and falls to `Sliding` of that past the peak slip. The bend is solved
  with the wheel's spin in one implicit step, so a wheel of a few kilograms is
  stable at 60 steps a second.
- The bend follows the chassis' real motion, not only the tyre's own force.
  Without that a tyre on a slope settled into a slide at the speed one step
  of weight gives.
- `VehicleInput` takes `Steer` from -1 to 1 (left is positive), `Brake` from
  0 to 1 and `Handbrake`.
- Wheels that `Steers` turn by Ackermann geometry about the line through the
  wheels that do not. `VehicleSpec.MaxSteer` is the angle at full lock.
- `WheelSpec.BrakeTorque` and `Handbrake`. A brake that can stop the wheel in
  a step locks it, and a locked tyre uses its sliding grip.
- Weight on each wheel is a plane fit that balances moments about the centre
  of mass, so a front-heavy car rests level with its front springs loaded.
  A centre outside the wheels is refused.
- `Vehicle(surface:)` scales every tyre's grip by the body under it.
- The patch speed comes from where the chassis was at the last update, not
  the solver's velocity, which is a sawtooth for a body held by pushes.
- `WheelState` adds `Spin` and `Steer`.

## 0.1.0

- First release. `Vehicle` makes a chassis body in a `Physics` world and,
  once a step, casts from each wheel along the chassis' down and pushes the
  chassis with the spring's force. `Update(dt)` goes before `Physics.Step`.
- A `SuspensionSpec` is a travel, a frequency in hertz and a damping ratio.
  The stiffness comes from the mass each wheel carries, so a kart and a truck
  share units. The frequency is capped at 8 Hz, which a step of 1/30 second
  holds.
- The damper may take the compression rate to zero in one step and no further.
  The rate is the change in compression since the last update, because the
  solver's own velocity is a sawtooth for a body held by one impulse a step.
- `VehicleSpec.AntiRoll` moves load between the two wheels of an axle. Wheels
  with the same mount z are an axle.
- `WheelState` reads back whether a wheel found ground, how far its spring is
  squeezed and what force it gave.
- A parked car never sleeps, because its springs push it every step.
