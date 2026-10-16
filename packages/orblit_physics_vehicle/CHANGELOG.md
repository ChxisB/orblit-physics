## 0.1.0

- First release. `Vehicle` makes a chassis body in a `Physics` world and,
  once a step, casts from each wheel along the chassis' down and pushes the
  chassis with the spring's force. `update(dt)` goes before `Physics.step`.
- A `SuspensionSpec` is a travel, a frequency in hertz and a damping ratio.
  The stiffness comes from the mass each wheel carries, so a kart and a truck
  share units. The frequency is capped at 8 Hz, which a step of 1/30 second
  holds.
- The damper may take the compression rate to zero in one step and no further.
  The rate is the change in compression since the last update, because the
  solver's own velocity is a sawtooth for a body held by one impulse a step.
- `VehicleSpec.antiRoll` moves load between the two wheels of an axle. Wheels
  with the same mount z are an axle.
- `WheelState` reads back whether a wheel found ground, how far its spring is
  squeezed and what force it gave.
- A parked car never sleeps, because its springs push it every step. An update
  costs about 4 microseconds a car on a 4-wheel car.
