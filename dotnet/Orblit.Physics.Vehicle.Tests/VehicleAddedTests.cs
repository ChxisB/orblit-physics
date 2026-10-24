using Orblit.VectorMath;
using static Orblit.Physics.Vehicle.Tests.Bounds;

namespace Orblit.Physics.Vehicle.Tests;

/// <summary>
/// What a caller of <see cref="Vehicle"/> can ask of it that the ported cases do not.
/// </summary>
public class VehicleAddedTests
{
    private const double Dt = 1.0 / 60;
    private const ulong Floor = 900;
    private const ulong CarId = 1;

    private static EngineSpec Engine() => new([(1000, 160), (3000, 220), (5000, 240), (6500, 200)]);

    private static VehicleSpec Plain() =>
        new(1200, [.. Wheels(drives: false, handbrakeRear: false)]);

    private static VehicleSpec Driven() =>
        new(
            1200,
            [.. Wheels(drives: true, handbrakeRear: true)],
            drive: new DriveSpec(Engine(), [3.6, 2.2, 1.5, 1.1, 0.9], finalDrive: 4.5));

    private static IEnumerable<WheelSpec> Wheels(bool drives, bool handbrakeRear) =>
        new[] { -0.8, 0.8 }.SelectMany(x =>
            new[] { -1.2, 1.2 }.Select(z =>
                new WheelSpec(
                    new Vector3(x, 0, z),
                    0.35,
                    steers: z < 0,
                    drives: drives,
                    brakeTorque: 3000,
                    handbrake: handbrakeRear && z > 0)));

    [Theory]
    [InlineData(0.0)]
    [InlineData(-Dt)]
    [InlineData(1.0 / 30 + 1e-9)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void UpdateRefusesAStepOutsideItsRange(double dt)
    {
        using var world = new World(Plain());

        Assert.ThrowsAny<ArgumentException>(() => world.Car.Update(dt));
    }

    [Fact]
    public void UpdateAcceptsTheLongestStepItDocuments()
    {
        using var world = new World(Plain());

        world.Car.Update(Vehicle.MaxStep);
        world.Car.Update(1e-9);
    }

    [Theory]
    [InlineData(5, true)]
    [InlineData(6, false)]
    [InlineData(0, true)]
    [InlineData(-1, true)]
    public void SetInputRefusesAGearTheDriveLacks(int gear, bool accepted)
    {
        using var world = new World(Driven());
        var input = new VehicleInput(gear: gear);

        if (accepted)
        {
            world.Car.SetInput(input);
            Assert.Same(input, world.Car.Input);
        }
        else
        {
            Assert.ThrowsAny<ArgumentException>(() => world.Car.SetInput(input));
        }
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(0, true)]
    [InlineData(-1, true)]
    public void SetInputRefusesAnyForwardGearForACarWithNoDrive(int gear, bool accepted)
    {
        using var world = new World(Plain());

        if (accepted)
        {
            world.Car.SetInput(new VehicleInput(gear: gear));
        }
        else
        {
            Assert.ThrowsAny<ArgumentException>(() => world.Car.SetInput(new VehicleInput(gear: gear)));
        }
    }

    [Fact]
    public void ACarWithNoDriveHasNoRevsAndNoGear()
    {
        using var world = new World(Plain());
        world.Run(0.5);

        Assert.Equal(0.0, world.Car.Rpm);
        Assert.Equal(0, world.Car.Gear);
    }

    [Fact]
    public void ACarWithADriveStartsInFirstNearIdle()
    {
        using var world = new World(Driven());

        Assert.Equal(1, world.Car.Gear);
        Assert.Equal(900.0, world.Car.Rpm, 6);
    }

    [Fact]
    public void ACarStartsWithEveryWheelOffTheGround()
    {
        using var world = new World(Plain());

        Assert.Equal(4, world.Car.Wheels.Count);
        Assert.All(world.Car.Wheels, wheel => Assert.Equal(new WheelState(), wheel));
    }

    [Fact]
    public void ACarAttachedToABodyThatIsAlreadyThereRidesOnItsWheels()
    {
        using var physics = new Physics();
        physics.Add(Floor, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
        physics.Add(CarId, Shape.Box(0.9, 0.3, 2.0), at: [0, 0.6, 0], mass: 1200);
        var car = Vehicle.Attach(physics, CarId, Plain());

        for (var i = 0; i < 120; i++)
        {
            car.Update(Dt);
            physics.Step(Dt);
        }

        Assert.All(car.Wheels, wheel => Assert.True(wheel.Grounded));
        Assert.Same(physics, car.Physics);
        Assert.Equal(CarId, car.Id);
    }

    [Fact]
    public void ACarAttachedToABodyThatIsNotThereDoesNothing()
    {
        using var physics = new Physics();
        var car = Vehicle.Attach(physics, 77, Plain());

        car.Update(Dt);

        Assert.All(car.Wheels, wheel => Assert.Equal(new WheelState(), wheel));
    }

    [Fact]
    public void ACarAsksTheSurfaceAboutTheBodyUnderEachWheel()
    {
        var seen = new HashSet<ulong>();
        using var world = new World(Plain(), surface: body =>
        {
            seen.Add(body);
            return 1.0;
        });

        world.Run(1);

        Assert.Equal([Floor], seen);
    }

    [Fact]
    public void ALowGripOnTheSurfaceLeavesTheWheelsSpinningInsteadOfPullingAway()
    {
        using var dry = new World(Driven());
        using var icy = new World(Driven(), surface: _ => 0.1);

        foreach (var world in new[] { dry, icy })
        {
            world.Run(1);
            world.Car.SetInput(new VehicleInput(throttle: 1));
            world.Run(3);
        }

        Greater(dry.Speed, icy.Speed * 2);
    }

    [Fact]
    public void ACarWhoseWheelsStartInsideTheGroundHasNothingToMeasureFrom()
    {
        using var world = new World(Plain(), height: -1);

        world.Run(0.2);

        Assert.All(world.Car.Wheels, wheel => Assert.False(wheel.Grounded));
    }

    [Fact]
    public void AWheelThatRollsStaysWithinOneTurnWhicheverWayItGoes()
    {
        foreach (var gear in new[] { 1, -1 })
        {
            using var world = new World(Driven(), height: 200);
            world.Car.SetInput(new VehicleInput(throttle: 1, gear: gear));
            world.Run(3);

            Assert.All(world.Car.Wheels, wheel => Assert.InRange(wheel.Roll, 0.0, 2 * Math.PI));
        }
    }

    [Fact]
    public void ABrakeHoldsEveryWheelAndTheHandbrakeJustTheRearOnes()
    {
        using var world = new World(Driven(), height: 200);
        world.Car.SetInput(new VehicleInput(throttle: 1));
        world.Run(2);
        var spinning = world.Car.Wheels.Select(wheel => wheel.Spin).ToArray();
        Assert.All(spinning, spin => Greater(spin, 100));

        world.Car.SetInput(new VehicleInput(gear: 0, handbrake: true));
        world.Run(1.5);
        Assert.Equal(0.0, world.Car.Wheels[1].Spin);
        Assert.Equal(0.0, world.Car.Wheels[3].Spin);
        Greater(world.Car.Wheels[0].Spin, spinning[0] * 0.5);
        Greater(world.Car.Wheels[2].Spin, spinning[2] * 0.5);

        world.Car.SetInput(new VehicleInput(gear: 0, brake: 1));
        world.Run(1.5);
        Assert.All(world.Car.Wheels, wheel => Assert.Equal(0.0, wheel.Spin));
    }

    [Fact]
    public void AHalfBrakeSlowsAWheelByHalfAsMuchAsAFullOne()
    {
        using var full = new World(Driven(), height: 200);
        using var half = new World(Driven(), height: 200);
        foreach (var world in new[] { full, half })
        {
            world.Car.SetInput(new VehicleInput(throttle: 1));
            world.Run(1);
        }

        var before = full.Car.Wheels[0].Spin;
        full.Car.SetInput(new VehicleInput(gear: 0, brake: 1));
        half.Car.SetInput(new VehicleInput(gear: 0, brake: 0.5));
        full.Run(Dt);
        half.Run(Dt);

        var fullDrop = before - full.Car.Wheels[0].Spin;
        var halfDrop = before - half.Car.Wheels[0].Spin;
        Assert.Equal(3000 * 0.5 * Dt, fullDrop - halfDrop, 3);
    }

    [Fact]
    public void AnAntiRollBarNeverPullsAWheelDownIntoTheGround()
    {
        var wheels = Wheels(drives: false, handbrakeRear: false).ToArray();
        var banking = new VehicleSpec(1200, wheels, antiRoll: 1e6);
        const double Roll = 0.1;
        using var world = new World(banking, height: 0.5, rotation: [0, 0, Math.Sin(Roll / 2), Math.Cos(Roll / 2)]);

        world.Car.Update(Dt);

        Assert.All(world.Car.Wheels, wheel => Assert.True(wheel.Grounded));
        Assert.All(world.Car.Wheels, wheel => Assert.True(wheel.Force >= 0, $"{wheel.Force} pulls the wheel down."));
        Assert.Contains(world.Car.Wheels, wheel => wheel.Force == 0.0);
    }

    private sealed class World : IDisposable
    {
        public World(
            VehicleSpec spec,
            double height = 0.6,
            SurfaceGrip? surface = null,
            IReadOnlyList<double>? rotation = null)
        {
            Physics.Add(Floor, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
            Car = new Vehicle(
                Physics,
                CarId,
                Shape.Box(0.9, 0.3, 2.0),
                spec,
                at: [0, height, 0],
                rotation: rotation,
                surface: surface);
        }

        public Physics Physics { get; } = new();

        public Vehicle Car { get; }

        public double Speed
        {
            get
            {
                var v = Physics.VelocityOf(CarId)!;
                return Math.Sqrt((v[0] * v[0]) + (v[2] * v[2]));
            }
        }

        public void Run(double seconds)
        {
            var steps = (int)Math.Round(seconds / Dt, MidpointRounding.AwayFromZero);
            for (var i = 0; i < steps; i++)
            {
                Car.Update(Dt);
                Physics.Step(Dt);
            }
        }

        public void Dispose() => Physics.Dispose();
    }
}
