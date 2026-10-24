using Orblit.VectorMath;
using static Orblit.Physics.Vehicle.Tests.Bounds;

namespace Orblit.Physics.Vehicle.Tests;

public class TyreTests
{
    private const double Dt = 1.0 / 60;
    private const double Gravity = 9.81;
    private const ulong Id = 1;
    private const double Radius = 0.35;

    /// <summary>
    /// The solver sheds a few percent of a body's speed a second, which stops a
    /// car a little sooner than its grip alone would.
    /// </summary>
    private const double Damped = 0.1;

    private const double Wheelbase = 2.4;

    /// <summary>A four-wheel car whose front wheels steer. Its front is along negative z.</summary>
    private static VehicleSpec Car(
        TyreSpec? tyre = null,
        double centreZ = 0.0,
        bool handbrakeRear = false,
        double brakeTorque = 3000) =>
        new(
            mass: 1200,
            centre: [0, 0, centreZ],
            wheels:
            [
                .. new[] { -0.8, 0.8 }.SelectMany(x =>
                    new[] { -1.2, 1.2 }.Select(z =>
                        new WheelSpec(
                            mount: new Vector3(x, 0, z),
                            radius: Radius,
                            steers: z < 0,
                            brakeTorque: brakeTorque,
                            handbrake: handbrakeRear && z > 0,
                            tyre: tyre))),
            ]);

    [Fact]
    public void TurnsACarInTheCircleItsGeometryDraws()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Launch(3);
        road.Car.SetInput(new VehicleInput(steer: 0.6));
        road.Run(1);

        var (speed, yaw) = (0.0, 0.0);
        road.Run(
            3,
            () =>
            {
                speed += road.Speed;
                yaw += road.Yaw;
            });

        // The rear axle goes round at wheelbase over tan(lock), and the middle of
        // the car, a half wheelbase ahead of it, goes round a little wider.
        var rear = Wheelbase / Math.Tan(0.6 * 0.5);
        var geometry = Math.Sqrt((rear * rear) + (1.2 * 1.2));
        Assert.Equal(geometry, speed / yaw, geometry * 0.05);
    }

    [Fact]
    public void TurnsRightForANegativeInputAndStraightForNone()
    {
        using var left = new Road(Car());
        using var right = new Road(Car());
        using var straight = new Road(Car());
        foreach (var (road, steer) in new[] { (left, 1.0), (right, -1.0), (straight, 0.0) })
        {
            road.Run(2);
            road.Launch(3);
            road.Car.SetInput(new VehicleInput(steer: steer));
            road.Run(1);
        }

        Greater(left.Yaw, 0.1);
        Less(right.Yaw, -0.1);
        Less(Math.Abs(straight.Yaw), 0.01);
    }

    [Fact]
    public void TurnsTheInnerWheelMoreThanTheOuterOne()
    {
        using var road = new Road(Car());
        road.Run(1);
        road.Car.SetInput(new VehicleInput(steer: 1));
        road.Run(0.1);

        // Wheels are listed x -0.8 (the left of a car whose front is -z) first.
        var (inner, outer) = (road.Car.Wheels[0].Steer, road.Car.Wheels[2].Steer);
        Greater(inner, outer, "the left wheel is inside");
        Greater(outer, 0);
        Assert.Equal(0, road.Car.Wheels[1].Steer);

        // Both wheels point at the same place on the line through the rear axle.
        var centre = new[] { (-0.8, inner), (0.8, outer) }
            .Select(wheel => -wheel.Item1 + (Wheelbase / Math.Tan(wheel.Item2)))
            .ToArray();
        Assert.Equal(centre[0], centre[1], 0.01);
    }

    /// <summary>
    /// Launches, brakes, and gives the metres it took against the metres the
    /// grip <paramref name="slowing"/> allows from the speed it was really going
    /// at. The launch leaves it a little under the speed asked for, so the speed
    /// is read off its motion.
    /// </summary>
    private static (double Metres, double Expected) Stop(Road road, double slowing)
    {
        road.Launch(8);
        var start = road.At;
        road.Run(0.1);
        var speed = (road.At - start).Length / 0.1;

        var from = road.At;
        road.Car.SetInput(new VehicleInput(brake: 1));
        road.Run(6);
        Less(road.Speed, 0.05, "it did stop");
        return ((road.At - from).Length, speed * speed / (2 * slowing * Gravity));
    }

    [Fact]
    public void StopsACarInTheDistanceItsGripAllows()
    {
        using var road = new Road(Car(tyre: new TyreSpec(sliding: 1)));
        road.Run(2);

        var (metres, expected) = Stop(road, 1.0);
        Assert.Equal(expected, metres, expected * Damped);
    }

    [Fact]
    public void StopsACarOnLockedWheelsInTheDistanceSlidingAllows()
    {
        using var road = new Road(Car());
        road.Run(2);

        var (metres, expected) = Stop(road, 0.8);
        Assert.Equal(expected, metres, expected * Damped);
    }

    [Fact]
    public void StopsInTwiceTheDistanceOnGroundThatGripsHalfAsWell()
    {
        using var road = new Road(Car(tyre: new TyreSpec(sliding: 1)), surface: _ => 0.5);
        road.Run(2);

        var (metres, expected) = Stop(road, 0.5);
        Assert.Equal(expected, metres, expected * Damped);
    }

    [Fact]
    public void LocksAWheelTheBrakeCanHoldAndLetsAFreeOneRoll()
    {
        using var road = new Road(Car());
        road.Run(2);
        road.Launch(8);

        var start = road.At;
        road.Run(0.1);
        var speed = (road.At - start).Length / 0.1;
        var rolling = road.Car.Wheels[0].Spin;
        Assert.Equal(speed / Radius, rolling, speed / Radius * 0.03);

        road.Car.SetInput(new VehicleInput(brake: 1));
        road.Run(0.5);
        Less(Math.Abs(road.Car.Wheels[0].Spin), 1);
    }

    private const double Slope = 20 * Math.PI / 180;

    /// <summary>Metres it moves in five seconds once it has settled.</summary>
    private static double Creep(Road road)
    {
        road.Run(2);
        var from = road.At;
        road.Run(5);
        return (road.At - from).Length;
    }

    [Fact]
    public void DoesNotCreepOnA20DegreeSlopeWithTheBrakeOn()
    {
        using var road = new Road(Car(), Slope);
        road.Car.SetInput(new VehicleInput(brake: 1));

        Less(Creep(road), 0.01);
    }

    [Fact]
    public void IsHeldByTheHandbrakeOnItsRearWheels()
    {
        using var road = new Road(Car(handbrakeRear: true), Slope);
        road.Car.SetInput(new VehicleInput(handbrake: true));

        Less(Creep(road), 0.01);
    }

    [Fact]
    public void RollsAwayWhenNothingHoldsIt()
    {
        using var road = new Road(Car(), Slope);

        Greater(Creep(road), 1);
    }

    [Fact]
    public void IsNotHeldByASlopeItsTyresCannotGrip()
    {
        using var road = new Road(Car(), Slope, _ => 0.2);
        road.Car.SetInput(new VehicleInput(brake: 1));

        Greater(Creep(road), 1);
    }

    [Fact]
    public void IsSharedSoThatTheMomentsBalanceAboutTheCentreOfMass()
    {
        var spec = Car(centreZ: -0.4);

        // Front wheels are at z -1.2: a third of the car each way round.
        Assert.Equal(1200.0 / 3, spec.Sprung[0], 1e-6);
        Assert.Equal(1200.0 / 6, spec.Sprung[1], 1e-6);
        Assert.Equal(1200, spec.Sprung.Sum(), 1e-6);
    }

    [Fact]
    public void RestsAFrontHeavyCarLevelWithTheFrontSpringsLoaded()
    {
        using var road = new Road(Car(centreZ: -0.4));
        road.Run(5);

        var wheels = road.Car.Wheels;
        Assert.Equal(400 * Gravity, wheels[0].Force, 400 * Gravity * 0.03);
        Assert.Equal(200 * Gravity, wheels[1].Force, 200 * Gravity * 0.03);
        Assert.Equal(wheels[0].Compression, wheels[1].Compression, 0.005);
    }

    [Fact]
    public void RefusesATyreThatGripsNothingOrSlipsBackwards()
    {
        Assert.ThrowsAny<ArgumentException>(() => new TyreSpec(grip: 0));
        Assert.ThrowsAny<ArgumentException>(() => new TyreSpec(peakSlip: 0));
        Assert.ThrowsAny<ArgumentException>(() => new TyreSpec(sliding: 1.5));
        Assert.ThrowsAny<ArgumentException>(() => new TyreSpec(relaxation: 0));
    }

    [Fact]
    public void RefusesAWheelWithABadBrakeOrNoSpinToHold()
    {
        var mount = Vector3.Zero();
        Assert.ThrowsAny<ArgumentException>(() => new WheelSpec(mount, 0.3, brakeTorque: -1));
        Assert.ThrowsAny<ArgumentException>(() => new WheelSpec(mount, 0.3, handbrake: true));
        Assert.ThrowsAny<ArgumentException>(() => new WheelSpec(mount, 0.3, inertia: 0));
    }

    [Fact]
    public void RefusesACentreOfMassThatIsOutsideTheWheels()
    {
        var wheel = new WheelSpec(Vector3.Zero(), 0.3);
        var wheels = Car().Wheels;
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(100, wheels, centre: [0, 0, 3]));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleSpec(100, [wheel], maxSteer: 2));
    }

    [Fact]
    public void RefusesASteerOrABrakeThatIsOutOfRange()
    {
        Assert.ThrowsAny<ArgumentException>(() => new VehicleInput(steer: 1.1));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleInput(brake: -0.1));
        Assert.ThrowsAny<ArgumentException>(() => new VehicleInput(brake: 2));
    }

    /// <summary>A car on a plane, run as a game does: update, then step.</summary>
    private sealed class Road : IDisposable
    {
        public Road(VehicleSpec spec, double slope = 0.0, SurfaceGrip? surface = null)
        {
            var up = new Vector3(0, Math.Cos(slope), Math.Sin(slope));
            Physics.Add(900, Shape.Plane(up.X, up.Y, up.Z), PhysicsMotion.Fixed);
            var half = slope / 2;
            var at = up * 0.6;
            Car = new Vehicle(
                Physics,
                Id,
                Shape.Box(0.9, 0.3, 2.0),
                spec,
                at: [at.X, at.Y, at.Z],
                rotation: [Math.Sin(half), 0, 0, Math.Cos(half)],
                surface: surface ?? (_ => 1.0));
        }

        public Physics Physics { get; } = new();

        public Vehicle Car { get; }

        public Vector3 At
        {
            get
            {
                var t = Physics.TransformOf(Id)!;
                return new Vector3(t[0], t[1], t[2]);
            }
        }

        /// <summary>Metres a second along the ground.</summary>
        public double Speed
        {
            get
            {
                var v = Physics.VelocityOf(Id)!;
                return Math.Sqrt((v[0] * v[0]) + (v[2] * v[2]));
            }
        }

        /// <summary>Radians a second about up.</summary>
        public double Yaw => Physics.VelocityOf(Id)![4];

        public void Run(double seconds, Action? each = null)
        {
            var steps = (int)Math.Round(seconds / Dt, MidpointRounding.AwayFromZero);
            for (var i = 0; i < steps; i++)
            {
                each?.Invoke();
                Car.Update(Dt);
                Physics.Step(Dt);
            }
        }

        /// <summary>Sends it along its own front at <paramref name="speed"/> and lets the wheels catch up.</summary>
        public void Launch(double speed)
        {
            Physics.Drive(Id, velocity: [0, 0, -speed]);
            Run(1);
        }

        public void Dispose() => Physics.Dispose();
    }
}
