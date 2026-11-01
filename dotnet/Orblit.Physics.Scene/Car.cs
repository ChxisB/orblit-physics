using Orblit.Physics.Vehicle;
using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene;

// The namespace of the same name would win over an alias written outside this one.
using Vehicle = Orblit.Physics.Vehicle.Vehicle;

/// <summary>
/// A <see cref="Vehicle"/> made from a vehicle component, with the wheel entities it moves.
/// </summary>
/// <remarks>
/// A wheel's position in the document is where it sits when the car stands at rest, so the top of its
/// spring is found a little above that, by the sag the spring has under the car's weight.
/// </remarks>
internal sealed class Car
{
    /// <summary>
    /// Where each wheel entity sits with its spring at full reach, in the chassis entity's own frame, and
    /// how far it can drop from there.
    /// </summary>
    private readonly List<Vector3> _tops;
    private readonly List<double> _travel;

    private Car(Vehicle vehicle, List<string> wheels, List<Vector3> tops, List<double> travel)
    {
        Vehicle = vehicle;
        Wheels = wheels;
        _tops = tops;
        _travel = travel;
    }

    public Vehicle Vehicle { get; }

    /// <summary>The wheel entities, in the order of the vehicle's wheels.</summary>
    public IReadOnlyList<string> Wheels { get; }

    /// <summary>
    /// Null when the entity cannot be a car: it has no free body, it is scaled, or its component, a wheel
    /// or the weight are ones the vehicle refuses. A document with such a car keeps it as data and the
    /// body is just a body.
    /// </summary>
    public static Car? Make(SceneDocument document, string id, Physics physics, ulong number, double gravity)
    {
        var entity = document[id];
        if (entity?[SceneComponents.Vehicle] is not VehicleComponent component
            || entity[SceneComponents.Body] is not BodyComponent body)
        {
            return null;
        }

        if (body.Motion != BodyMotion.Free || !Unscaled(document, id))
        {
            return null;
        }

        var children = document.ChildrenOf(id).Where(child => child[SceneComponents.Wheel] is WheelComponent).ToList();
        try
        {
            return Build(children, component, body, physics, number, gravity);
        }
        catch (ArgumentException)
        {
            return null;
        }
    }

    /// <summary>
    /// The transform of every wheel entity that is not already as the car has it: the hub under its
    /// spring, turned by the steering and the roll.
    /// </summary>
    /// <remarks>
    /// The rotation is the car's to write. Steer turns about the chassis' up, and the roll about the
    /// wheel's own axle, which is a rotation about minus x for a wheel that rolls forward. The scale is
    /// the wheel's.
    /// </remarks>
    public Dictionary<string, TransformComponent> Poses(SceneDocument document)
    {
        var changed = new Dictionary<string, TransformComponent>();
        var states = Vehicle.Wheels;
        for (var i = 0; i < Wheels.Count; i++)
        {
            var was = document[Wheels[i]]?[SceneComponents.Transform] as TransformComponent;
            var squeeze = Math.Min(states[i].Compression, _travel[i]);
            var next = new TransformComponent(
                _tops[i] - new Vector3(0, _travel[i] - squeeze, 0),
                new Vector3(-Doubles.Degrees(states[i].Roll), Doubles.Degrees(states[i].Steer), 0),
                was?.Scale.Clone());
            if (was is not null && was.Position.Equals(next.Position) && was.Rotation.Equals(next.Rotation))
            {
                continue;
            }

            changed[Wheels[i]] = next;
        }

        return changed;
    }

    private static Car Build(
        List<SceneEntity> children,
        VehicleComponent component,
        BodyComponent body,
        Physics physics,
        ulong number,
        double gravity)
    {
        var tops = new List<Vector3>();
        var specs = new List<WheelSpec>();
        foreach (var child in children)
        {
            var wheel = (WheelComponent)child[SceneComponents.Wheel]!;
            var suspension = new SuspensionSpec(wheel.Travel, wheel.Hertz, wheel.Damping);
            var top = (child[SceneComponents.Transform] as TransformComponent)?.Position.Clone() ?? Vector3.Zero();
            top.Y += wheel.Travel - suspension.Sag(gravity);
            tops.Add(top);
            specs.Add(new WheelSpec(
                Snapped(top - body.Centre),
                wheel.Radius,
                steers: wheel.Steers,
                drives: wheel.Drives,
                brakeTorque: wheel.Brake,
                handbrake: wheel.Handbrake,
                suspension: suspension,
                tyre: new TyreSpec(grip: wheel.Grip)));
        }

        var centre = body.CentreOfMass;
        var spec = new VehicleSpec(
            body.Mass,
            specs,
            centre: [centre.X, centre.Y, centre.Z],
            antiRoll: component.AntiRoll,
            maxSteer: Doubles.Radians(component.MaxSteer),
            drive: specs.Any(wheel => wheel.Drives) ? DriveOf(component) : null,
            ground: new Layers(Is: 0, Cares: (uint)component.Ground));
        return new Car(
            Vehicle.Attach(physics, number, spec),
            [.. children.Select(child => child.Id)],
            tops,
            [.. specs.Select(wheel => wheel.Suspension.Travel)]);
    }

    private static DriveSpec DriveOf(VehicleComponent car)
    {
        var curve = car.Torque;
        var torque = new List<(double Rpm, double NewtonMetres)>();
        for (var i = 0; i + 1 < curve.Count; i += 2)
        {
            torque.Add((curve[i], curve[i + 1]));
        }

        return new DriveSpec(
            new EngineSpec(torque, idle: car.Idle),
            car.Ratios,
            reverse: car.Reverse,
            finalDrive: car.FinalDrive,
            diffLock: car.Lock);
    }

    private static bool Unscaled(SceneDocument document, string id)
    {
        var scale = Vector3.Zero();
        ScenePose.WorldOf(document, id).Decompose(Vector3.Zero(), Quaternion.Identity(), scale);
        return (scale - Vector3.All(1)).Length < 1e-6;
    }

    /// <summary>
    /// Wheels at the same z are an axle, and the vehicle compares them exactly. A tenth of a millimetre is
    /// nothing to a car and keeps a hand-placed pair from falling into two axles of one.
    /// </summary>
    private static Vector3 Snapped(Vector3 mount)
    {
        mount.Z = Math.Round(mount.Z * 1e4, MidpointRounding.AwayFromZero) / 1e4;
        return mount;
    }
}
