using Orblit.VectorMath;
using static Orblit.Physics.Vehicle.Tests.Bounds;

namespace Orblit.Physics.Vehicle.Tests;

public class DriveTests
{
    private const double Dt = 1.0 / 60;
    private const double Radius = 0.35;

    private static DriveSpec Drive(double diffLock = 0.0, double finalDrive = 4.5) =>
        new(
            new EngineSpec([(1000, 160), (3000, 220), (5000, 240), (6500, 200)]),
            [3.6, 2.2, 1.5, 1.1, 0.9],
            finalDrive: finalDrive,
            diffLock: diffLock);

    /// <summary>A rear-drive car. Its front is along negative z.</summary>
    private static VehicleSpec Car(DriveSpec? drive = null, bool driven = true) =>
        new(
            mass: 1200,
            drive: drive ?? Drive(),
            wheels:
            [
                .. new[] { -0.8, 0.8 }.SelectMany(x =>
                    new[] { -1.2, 1.2 }.Select(z =>
                        new WheelSpec(
                            mount: new Vector3(x, 0, z),
                            radius: Radius,
                            steers: z < 0,
                            drives: driven && z > 0,
                            brakeTorque: 3000))),
            ]);

    [Fact]
    public void PullsAwayAndShiftsUpThroughItsGears()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 1));

        var gears = new List<int>();
        road.Run(14, () => gears.Add(road.Car.Gear));

        Greater(road.Speed, 25);
        AtLeast(gears[^1], 3);
        for (var i = 1; i < gears.Count; i++)
        {
            AtLeast(gears[i], gears[i - 1], $"step {i}");
        }

        Assert.InRange(road.Car.Rpm, 900, 6500 * 1.06);
    }

    [Fact]
    public void TurnsItsDrivenWheelsAtTheRoadSpeedOnceItIsCruising()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 0.2));
        road.Run(8);

        var from = road.Physics.TransformOf(1)!;
        road.Run(0.2);
        var to = road.Physics.TransformOf(1)!;
        var speed = Math.Sqrt(Math.Pow(to[0] - from[0], 2) + Math.Pow(to[2] - from[2], 2)) / 0.2;

        Greater(speed, 3);
        foreach (var i in new[] { 1, 3 })
        {
            var rolling = road.Car.Wheels[i].Spin * Radius;
            Assert.Equal(speed, rolling, speed * 0.05);
        }
    }

    [Fact]
    public void SpinsUpADrivenWheelThatHasNoRoadUnderIt()
    {
        using var road = new Road(Car(), height: 20);
        road.Car.SetInput(new VehicleInput(throttle: 1));
        road.Run(1);

        Assert.All(road.Car.Wheels, wheel => Assert.False(wheel.Grounded));
        Greater(road.Car.Wheels[1].Spin, 20);
        Greater(road.Car.Wheels[3].Spin, 20);
        Assert.Equal(0, road.Car.Wheels[0].Spin);
        Greater(road.Car.Rpm, 1500);
    }

    [Fact]
    public void RaisesItsWheelspinSmoothlyOnSlipperyGround()
    {
        using var road = new Road(Car(), surface: _ => 0.2);
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 1, gear: 1));

        var spins = new List<double>();
        road.Run(0.5, () => spins.Add(road.Car.Wheels[1].Spin));

        // The first steps take the clutch up. After them, a wheel that is
        // clear of the road gains a little a step and never gives it back.
        for (var i = 6; i < spins.Count; i++)
        {
            Assert.InRange(spins[i] - spins[i - 1], 0, 3);
        }

        Greater(spins[^1], 20, "the wheels do spin");
    }

    [Fact]
    public void AddsEachWheelsSpinToItsRollAndWrapsAtAFullTurn()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 0.6));

        var last = road.Car.Wheels[1].Roll;
        var wrapped = false;
        road.Run(
            6,
            () =>
            {
                var wheel = road.Car.Wheels[1];
                Assert.InRange(wheel.Roll, 0, 2 * Math.PI);
                var turned = Doubles.Mod(wheel.Roll - last, 2 * Math.PI);
                Assert.Equal(Doubles.Mod(wheel.Spin * Dt, 2 * Math.PI), turned, 1e-9);
                wrapped = wrapped || wheel.Roll < last;
                last = wheel.Roll;
            });
        Assert.True(wrapped, "it turned through more than one turn");
    }

    [Fact]
    public void DoesNotCreepWithThePedalUp()
    {
        using var road = new Road(Car());
        road.Run(4);

        Less(road.Speed, 0.05);
    }

    [Fact]
    public void BacksUpInReverse()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 0.5, gear: -1));
        road.Run(3);

        var v = road.Physics.VelocityOf(1)!;
        Assert.Equal(-1, road.Car.Gear);
        Greater(v[2], 1, "it moves toward its rear");
    }

    [Fact]
    public void HoldsTheGearTheDriverPicks()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 1, gear: 2));
        road.Run(6);

        Assert.Equal(2, road.Car.Gear);
    }

    [Fact]
    public void IsSlowedByTheEngineWhenThePedalIsUp()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Car.SetInput(new VehicleInput(throttle: 1));
        road.Run(5);
        var fast = road.Car.Rpm;

        road.Car.SetInput(new VehicleInput());
        road.Run(1);
        Less(road.Car.Rpm, fast);
    }

    /// <summary>
    /// Two driven wheels on one axle, the second turning ten radians a second
    /// faster, with a gearbox that has had time to take hold.
    /// </summary>
    private static (Driveline Line, double[] Inertia) RunDifferential(double diffLock)
    {
        WheelSpec[] wheels = [.. new[] { -0.8, 0.8 }.Select(x => new WheelSpec(new Vector3(x, 0, 1.2), Radius, drives: true))];
        var line = new Driveline(Drive(diffLock), wheels);
        double[] spins = [0.0, 10.0];
        Func<double, double>[] answers =
        [
            .. Enumerable.Range(0, 2).Select<int, Func<double, double>>(i =>
                torque => spins[i] + (torque * Dt / wheels[i].Inertia)),
        ];
        for (var i = 0; i < 120; i++)
        {
            line.Step(spins, answers, new VehicleInput(throttle: 1, gear: 1), Dt);
        }

        return (line, [wheels[0].Inertia, wheels[1].Inertia]);
    }

    [Fact]
    public void GivesBothWheelsTheSameTorqueWhenItIsOpen()
    {
        var (line, _) = RunDifferential(0);
        Greater(line.Torque(0), 0);
        Assert.Equal(line.Torque(0), line.Torque(1), 1e-9);
    }

    [Fact]
    public void MovesTorqueToTheSlowerWheelUpToWhatItCanHold()
    {
        var (limited, _) = RunDifferential(50);
        Assert.Equal(100, limited.Torque(0) - limited.Torque(1), 1e-6);

        var (locked, inertia) = RunDifferential(double.PositiveInfinity);
        Assert.Equal(inertia[0] * 10 / Dt, locked.Torque(0) - locked.Torque(1), 1e-6);
    }

    private static readonly (double, double)[] Curve = [(1000.0, 160.0), (6500.0, 200.0)];

    [Fact]
    public void RefusesAnEngineThatHasNoCurveOrThatIdlesPastItsTop()
    {
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec([(1000, 100)]));
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec([(2000, 100), (1000, 100)]));
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec(Curve, idle: 7000));
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec(Curve, inertia: 0));
        Assert.ThrowsAny<ArgumentException>(() => new EngineSpec(Curve, drag: -1));
    }

    [Fact]
    public void RefusesABoxWithNoGearsOrADownshiftAboveTheUpshift()
    {
        var engine = new EngineSpec(Curve);
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(engine, []));

        // A downshift from second would land above the upshift.
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(engine, [3.0, 1.0]));
        Assert.ThrowsAny<ArgumentException>(() => new DriveSpec(engine, [3.0], reverse: 0));
    }

    [Fact]
    public void RefusesADriveWithNoDrivenWheelAndADrivenWheelWithNoDrive()
    {
        Assert.ThrowsAny<ArgumentException>(() => Car(driven: false));
        Assert.ThrowsAny<ArgumentException>(() =>
            new VehicleSpec(100, [new WheelSpec(Vector3.Zero(), 0.3, drives: true)]));
    }

    [Fact]
    public void RefusesAGearTheCarDoesNotHave()
    {
        using var road = new Road(Car());
        Assert.ThrowsAny<ArgumentException>(() => road.Car.SetInput(new VehicleInput(gear: 6)));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleInput(gear: -2));
    }

    /// <summary>A car above a plane, run as a game does: update, then step.</summary>
    private sealed class Road : IDisposable
    {
        public Road(VehicleSpec spec, double height = 0.6, SurfaceGrip? surface = null)
        {
            Physics.Add(900, Shape.Plane(0, 1, 0), PhysicsMotion.Fixed);
            Car = new Vehicle(Physics, 1, Shape.Box(0.9, 0.3, 2.0), spec, at: [0, height, 0], surface: surface ?? (_ => 1.0));
        }

        public Physics Physics { get; } = new();

        public Vehicle Car { get; }

        /// <summary>Metres a second along the ground.</summary>
        public double Speed
        {
            get
            {
                var v = Physics.VelocityOf(1)!;
                return Math.Sqrt((v[0] * v[0]) + (v[2] * v[2]));
            }
        }

        public void Run(double seconds, Action? each = null)
        {
            var steps = (int)Math.Round(seconds / Dt, MidpointRounding.AwayFromZero);
            for (var i = 0; i < steps; i++)
            {
                Car.Update(Dt);
                Physics.Step(Dt);
                each?.Invoke();
            }
        }

        public void Dispose() => Physics.Dispose();
    }
}
