using Orblit.Physics.Tests;
using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle.Tests;

/// <summary>
/// Three scenes whose numbers come from an independent run of the original
/// solver, taken once on this machine and pinned here. They guard the whole
/// chain: the casts, the springs, the tyres and the drive line.
/// </summary>
public class VehicleReferenceAddedTests
{
    internal const double Dt = 1.0 / 60;

    // The numbers are pinned to nine decimals, so on a Mac they are held to every digit they have. Another
    // system rounds its float32 steps in other places and is held to a thousandth of the figure.
    private static void Near(double expected, double actual) =>
        Assert.Equal(expected, actual, SteppedDump.Allowed(expected));

    internal static DriveSpec Drive() =>
        new(
            new EngineSpec([(1000, 160), (3000, 220), (5000, 240), (6500, 200)]),
            [3.6, 2.2, 1.5, 1.1, 0.9],
            finalDrive: 4.5,
            diffLock: 50);

    internal static VehicleSpec Car(DriveSpec? drive = null) =>
        new(
            mass: 1200,
            centre: [0, 0, -0.4],
            antiRoll: 20000,
            drive: drive,
            wheels:
            [
                .. new[] { -0.8, 0.8 }.SelectMany(x =>
                    new[] { -1.2, 1.2 }.Select(z =>
                        new WheelSpec(
                            mount: new Vector3(x, 0, z),
                            radius: 0.35,
                            steers: z < 0,
                            drives: drive != null && z > 0,
                            brakeTorque: 3000))),
            ]);

    internal static (Physics Physics, Vehicle Car) Make(VehicleSpec spec)
    {
        var physics = new Physics();
        physics.Add(900, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
        return (physics, new Vehicle(physics, 1, Shape.Box(0.9, 0.3, 2.0), spec, at: [0, 0.6, 0]));
    }

    internal static void Run(Physics physics, Vehicle car, double seconds)
    {
        var steps = (int)Math.Round(seconds / Dt, MidpointRounding.AwayFromZero);
        for (var i = 0; i < steps; i++)
        {
            car.Update(Dt);
            physics.Step(Dt);
        }
    }

    [Fact]
    public void ACarSettlesToTheSameRestAsTheOriginalSolver()
    {
        var (physics, car) = Make(Car());
        using var _ = physics;
        Run(physics, car, 3);

        Near(0.539559901, physics.TransformOf(1)![1]);
        Near(-0.068118088, physics.VelocityOf(1)![1]);
        Assert.All(car.Wheels, wheel => Near(0.110440040, wheel.Compression));

        // The centre is a third of the way to the front, so the front pair carry twice the rear.
        Near(3923.998204447, car.Wheels[0].Force);
        Near(1961.999102223, car.Wheels[1].Force);
        Near(3923.998204447, car.Wheels[2].Force);
        Near(1961.999102223, car.Wheels[3].Force);
    }

    [Fact]
    public void ASteeredCarTurnsAsTheOriginalSolverDoes()
    {
        var (physics, car) = Make(Car());
        using var _ = physics;
        Run(physics, car, 2);
        physics.Drive(1, velocity: [0, 0, -3]);
        Run(physics, car, 1);
        car.SetInput(new VehicleInput(steer: 0.6));
        Run(physics, car, 2);

        var t = physics.TransformOf(1)!;
        Near(-2.068804264, t[0]);
        Near(0.539846003, t[1]);
        Near(-7.304778099, t[2]);
        Near(0.301405787, physics.VelocityOf(1)![4]);

        // The inner front wheel turns further than the outer, and the rear wheels stay straight.
        Near(0.332123793, car.Wheels[0].Steer);
        Near(0.273399437, car.Wheels[2].Steer);
        Assert.Equal(0, car.Wheels[1].Steer);
        Near(6.334612133, car.Wheels[0].Spin);
        Near(7.655829522, car.Wheels[2].Spin);
        Near(3775.487494442, car.Wheels[0].Force);
        Near(2.671517998, car.Wheels[0].Roll);
    }

    [Fact]
    public void ADrivenCarPullsAwayAsTheOriginalSolverDoes()
    {
        var (physics, car) = Make(Car(Drive()));
        using var _ = physics;
        Run(physics, car, 2);
        car.SetInput(new VehicleInput(throttle: 1));
        Run(physics, car, 4);

        Near(-22.509765625, physics.TransformOf(1)![2]);
        Near(3197.951377719, car.Rpm);
        Assert.Equal(3, car.Gear);

        // The rear wheels are driven and the front wheels turn with the road.
        Near(49.441541701, car.Wheels[1].Spin);
        Near(4.135505522, car.Wheels[1].Roll);
        Near(31.000929924, car.Wheels[0].Spin);
    }
}
