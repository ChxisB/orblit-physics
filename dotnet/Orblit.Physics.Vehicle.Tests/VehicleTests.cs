using Orblit.VectorMath;
using static Orblit.Physics.Vehicle.Tests.Bounds;

namespace Orblit.Physics.Vehicle.Tests;

public class VehicleTests
{
    private const double Dt = 1.0 / 60;
    private const double Gravity = 9.81;
    private const ulong CarId = 1;
    private const double Radius = 0.35;
    private const double Travel = 0.3;

    private static VehicleSpec Spec(double frequency = 1.5, double damping = 0.5, double antiRoll = 0.0) =>
        new(
            mass: 1200,
            antiRoll: antiRoll,
            wheels:
            [
                .. new[] { -0.8, 0.8 }.SelectMany(x =>
                    new[] { -1.2, 1.2 }.Select(z =>
                        new WheelSpec(
                            mount: new Vector3(x, 0, z),
                            radius: Radius,
                            suspension: new SuspensionSpec(Travel, frequency, damping)))),
            ]);

    /// <summary>
    /// How high the chassis' middle stands once the springs hold its weight and
    /// nothing else: the reach of the cast, less the squeeze that weight makes.
    /// </summary>
    private static double RideHeight(double frequency)
    {
        var omega = 2 * Math.PI * frequency;
        return Travel + Radius - (Gravity / (omega * omega));
    }

    [Fact]
    public void StopsAtItsRideHeightWithNoSpeedLeft()
    {
        using var w = new World(Spec());

        w.Run(6);

        Assert.Equal(RideHeight(1.5), w.Height, RideHeight(1.5) * 0.02);
        Less(w.Speed, 0.01);
        Assert.All(w.Car.Wheels, wheel => Assert.True(wheel.Grounded));
    }

    [Fact]
    public void DoesNotBuzzWithAStiffSpringAndAHardDamper()
    {
        using var w = new World(Spec(frequency: 8, damping: 1), height: 0.7);
        w.Run(3);

        var turns = 0;
        var last = 0.0;
        w.Run(
            2,
            () =>
            {
                var now = w.Rise;
                if ((now * last < 0) && (Math.Abs(now) > 1e-4))
                {
                    turns++;
                }

                if (Math.Abs(now) > 1e-4)
                {
                    last = now;
                }
            });

        Assert.Equal(0, turns);
        Less(Math.Abs(w.Rise), 0.001);
    }

    [Fact]
    public void BottomsOutOnItsChassisAndNeverPassesTheFloor()
    {
        using var w = new World(Spec(), height: 3);

        var lowest = double.PositiveInfinity;
        var deepest = 0.0;
        w.Run(
            4,
            () =>
            {
                lowest = Math.Min(lowest, w.Height);
                foreach (var wheel in w.Car.Wheels)
                {
                    deepest = Math.Max(deepest, wheel.Compression);
                }
            });

        Greater(deepest, Travel, "it did bottom out");
        Greater(lowest, 0.3 - 0.03, "box half height");
        Assert.Equal(RideHeight(1.5), w.Height, RideHeight(1.5) * 0.02);
    }

    [Fact]
    public void GiveNoForceInTheAirAndReadAsNotGrounded()
    {
        using var w = new World(Spec(), height: 5);

        w.Run(0.2);

        foreach (var wheel in w.Car.Wheels)
        {
            Assert.False(wheel.Grounded);
            Assert.Equal(0, wheel.Force);
        }
    }

    [Fact]
    public void DoNotSeeABodyTheirLayersLeaveOut()
    {
        var spec = new VehicleSpec(
            mass: 1200,
            ground: new Layers(Is: 1, Cares: 2),
            wheels: [new WheelSpec(new Vector3(0, 0, 0), Radius)]);
        using var w = new World(spec, height: 0.5, floor: new Layers(Is: 4, Cares: 0));

        w.Run(0.05);

        Assert.False(Assert.Single(w.Car.Wheels).Grounded);
    }

    private static double RollWith(double antiRoll)
    {
        using var w = new World(Spec(antiRoll: antiRoll));

        // A passenger's weight pressing on the left side, as a force each step.
        w.Run(
            4,
            () =>
            {
                var t = w.Physics.TransformOf(CarId)!;
                w.Physics.Push(CarId, [0, -4000 * Dt, 0], at: [t[0] - 0.8, t[1], t[2]]);
            });
        var after = w.Physics.TransformOf(CarId)!;
        var up = new Quaternion(after[3], after[4], after[5], after[6])
            .AsRotationMatrix()
            .Transformed(new Vector3(0, 1, 0));
        return Math.Abs(up.X);
    }

    [Fact]
    public void LeansTheCarLessUnderAOneSidedLoad()
    {
        var loose = RollWith(0);
        var barred = RollWith(60000);

        Greater(loose, 0.005, "the load does lean it");
        Less(barred, loose * 0.8);
    }

    [Fact]
    public void RefusesASpringAStepCannotHold()
    {
        Assert.ThrowsAny<ArgumentException>(() => new SuspensionSpec(frequency: SuspensionSpec.MaxFrequency + 1));
        Assert.ThrowsAny<ArgumentException>(() => new SuspensionSpec(travel: 0));
        Assert.ThrowsAny<ArgumentException>(() => new SuspensionSpec(damping: 3));
    }

    [Fact]
    public void HasASagThatFollowsTheSpringAndStopsAtTheTravel()
    {
        Assert.Equal(9.81 / Math.Pow(3 * Math.PI, 2), new SuspensionSpec().Sag(), 1e-9);
        Assert.Equal(0.3, new SuspensionSpec(frequency: 0.5).Sag());
        Less(new SuspensionSpec().Sag(1.62), new SuspensionSpec().Sag());
    }

    [Fact]
    public void RefusesACarWithNoWheelsNoWeightOrALoneWheelOnABar()
    {
        var wheel = new WheelSpec(Vector3.Zero(), 0.3);
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(100, []));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(0, [wheel]));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(100, [wheel], antiRoll: 1));
    }

    [Fact]
    public void IsNotMovedByChangingTheVectorItWasMadeFrom()
    {
        var mount = new Vector3(1, 2, 3);
        var wheel = new WheelSpec(mount, 0.3);
        mount.X = 9;

        Assert.Equal(1, wheel.Mount.X);
    }

    [Fact]
    public void IsNotUpdatedByAStepThatIsTooLong()
    {
        using var w = new World(Spec());

        Assert.ThrowsAny<ArgumentException>(() => w.Car.Update(0.1));
        Assert.ThrowsAny<ArgumentException>(() => w.Car.Update(0));
    }

    /// <summary>A car above a plane, and the means to run it as a game does: update, then step.</summary>
    private sealed class World : IDisposable
    {
        public World(VehicleSpec spec, double height = 0.6, Layers? floor = null)
        {
            Physics.Add(900, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed, layers: floor);
            Car = new Vehicle(Physics, CarId, Shape.Box(0.9, 0.3, 2.0), spec, at: [0, height, 0]);
        }

        public Physics Physics { get; } = new();

        public Vehicle Car { get; }

        public double Height => Physics.TransformOf(CarId)![1];

        /// <summary>
        /// How fast the chassis moved up over the last step, from where it was and
        /// where it is. The solver's own velocity is read at the end of its
        /// sub-steps, where a body held up by one impulse a step is 7 cm/s either
        /// side of rest.
        /// </summary>
        public double Rise { get; private set; }

        /// <summary>How fast it moved over the last step, by the same reckoning.</summary>
        public double Speed { get; private set; }

        public void Run(double seconds, Action? each = null)
        {
            var steps = (int)Math.Round(seconds / Dt, MidpointRounding.AwayFromZero);
            for (var i = 0; i < steps; i++)
            {
                each?.Invoke();
                var before = Physics.TransformOf(CarId)!;
                Car.Update(Dt);
                Physics.Step(Dt);
                var after = Physics.TransformOf(CarId)!;
                Rise = (after[1] - before[1]) / Dt;
                Speed =
                    Math.Sqrt(
                        Math.Pow(after[0] - before[0], 2)
                        + Math.Pow(after[1] - before[1], 2)
                        + Math.Pow(after[2] - before[2], 2))
                    / Dt;
            }
        }

        public void Dispose() => Physics.Dispose();
    }
}
