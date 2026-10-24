using Orblit.Physics.Vehicle;
using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneVehicleCourseTests : SceneTestBase
{
    private const double Travel = 0.3;

    /// <summary>Metres a 1.5 hertz spring gives under the weight of the car.</summary>
    private static readonly double Sag = 9.81 / (2 * Math.PI * 1.5 * 2 * Math.PI * 1.5);

    /// <summary>A body made in the world itself takes a number the scene never hands out.</summary>
    private const ulong GroundId = ulong.MaxValue;

    /// <summary>Walks along minus z at <see cref="Pace"/>, with gravity in the request.</summary>
    private sealed class Stroll : ICharacterWalk
    {
        public double Pace { get; set; }

        public IReadOnlyList<double> Ask(double seconds, PhysicsFooting? footing)
        {
            var fall = footing?.Velocity[1] ?? 0.0;
            return [0.0, fall - (9.81 * seconds), -Pace];
        }
    }

    private static SceneEntity Wheel(string car, string id, double x, double z, bool front) => new(
        id,
        id,
        car,
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(x, Sag - Travel, z)),
            [SceneComponents.Wheel] = new WheelComponent(radius: 0.35, steers: front, drives: !front, brake: 3000),
        });

    /// <summary>
    /// A rear-drive car or truck of <paramref name="mass"/> kilograms and a box <paramref name="size"/>,
    /// with its wheels placed where they sit at rest.
    /// </summary>
    // F2: four fields of one vehicle, the last two optional.
    private static SceneEntity[] Car(string id, Vector3 at, double mass = 1200, Vector3? size = null) =>
    [
        new SceneEntity(
            id,
            id,
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(at),
                [SceneComponents.Body] = new BodyComponent(size: size ?? new Vector3(1.8, 0.6, 4), mass: mass),
                [SceneComponents.Vehicle] = new VehicleComponent(),
            }),
        Wheel(id, $"{id}-fl", -0.8, -1.2, true),
        Wheel(id, $"{id}-fr", 0.8, -1.2, true),
        Wheel(id, $"{id}-rl", -0.8, 1.2, false),
        Wheel(id, $"{id}-rr", 0.8, 1.2, false),
    ];

    private ScenePhysics Scene(params SceneEntity[] entities) => Make(SceneOf(entities));

    /// <summary>Metres a second along the ground.</summary>
    private static double Speed(ScenePhysics scene, string id)
    {
        var v = scene.Physics.VelocityOf(scene.BodyOf(id)!.Value)!;
        double x = v[0];
        double z = v[2];
        return Math.Sqrt((x * x) + (z * z));
    }

    private static Vector3 At(ScenePhysics scene, string id)
    {
        var t = scene.Physics.TransformOf(scene.BodyOf(id)!.Value)!;
        return new Vector3(t[0], t[1], t[2]);
    }

    private static Matrix3 Frame(ScenePhysics scene, string id)
    {
        var t = scene.Physics.TransformOf(scene.BodyOf(id)!.Value)!;

        // AsRotationMatrix, not Quaternion.Rotated, which turns the other way.
        return new Quaternion(t[3], t[4], t[5], t[6]).AsRotationMatrix();
    }

    /// <summary>Radians the car faces, from straight along minus z. Left is positive.</summary>
    private static double Heading(ScenePhysics scene, string id)
    {
        var ahead = Frame(scene, id).Transformed(new Vector3(0, 0, -1));
        return Math.Atan2(-ahead.X, -ahead.Z);
    }

    private static bool Upright(ScenePhysics scene, string id) => Frame(scene, id).Transformed(new Vector3(0, 1, 0)).Y > 0.7;

    /// <summary>An angle taken into -pi to pi, with a remainder that is never negative.</summary>
    private static double Wrapped(double angle)
    {
        var turn = 2 * Math.PI;
        var remainder = (angle + Math.PI) % turn;
        return (remainder < 0 ? remainder + turn : remainder) - Math.PI;
    }

    /// <summary>Holds <paramref name="pace"/> metres a second with the pedal, as a driver does.</summary>
    private static VehicleInput Drive(ScenePhysics scene, string id, double pace, double steer) => new(
        throttle: Speed(scene, id) < pace ? 0.7 : 0.0,
        brake: Speed(scene, id) > pace * 1.3 ? 0.5 : 0.0,
        steer: steer);

    [Fact]
    public void DrivesAFigureOfEightOnFlatGround()
    {
        var scene = Scene([Floor(), .. Car("car", new Vector3(0, 0.7, 0))]);
        Run(scene, 2);

        var last = Heading(scene, "car");
        var turned = 0.0;
        var low = double.PositiveInfinity;
        var far = 0.0;
        void Follow(double steer)
        {
            scene.Pilot("car", Drive(scene, "car", 8, steer));
            var now = Heading(scene, "car");
            turned += Wrapped(now - last);
            last = now;
            low = Math.Min(low, At(scene, "car").Y);
            far = Math.Max(far, At(scene, "car").Length);
            Assert.True(Upright(scene, "car"), "the car tipped over");
        }

        Run(scene, 4, () => Follow(0));
        Assert.True(Math.Abs(turned) < 0.05, $"straight: turned {turned}");

        Run(scene, 8, () => Follow(0.5));
        var left = turned;
        Assert.True(left > 2 * Math.PI * 0.8, $"a full loop left: turned {left}");

        Run(scene, 8, () => Follow(-0.5));
        Assert.True(left - turned > 2 * Math.PI * 0.8, $"then right: turned {left - turned}");

        Assert.True(low > 0.3, $"it never sank into the floor: {low}");
        Assert.True(far < 80, $"it stayed on the course: {far}");
    }

    [Fact]
    public void TakesARampFliesAndLandsTheRightWayUp()
    {
        // A thin plank 12 degrees up toward minus z, its low end on the floor.
        var ramp = new SceneEntity(
            "ramp",
            "Ramp",
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(new Vector3(0, 0.74, -30), new Vector3(12, 0, 0)),
                [SceneComponents.Body] = new BodyComponent(size: new Vector3(5, 0.2, 8), motion: BodyMotion.Fixed),
            });
        var scene = Scene([Floor(), ramp, .. Car("car", new Vector3(0, 0.7, 20))]);
        Run(scene, 2);

        var high = 0.0;
        var low = double.PositiveInfinity;
        var airborne = 0;
        Run(scene, 10, () =>
        {
            scene.Pilot("car", Drive(scene, "car", 14, 0));
            var y = At(scene, "car").Y;
            high = Math.Max(high, y);
            if (At(scene, "car").Z < -36)
            {
                low = Math.Min(low, y);
            }

            if (scene.VehicleOf("car")!.Wheels.All(wheel => !wheel.Grounded))
            {
                airborne++;
            }
        });
        scene.Pilot("car", new VehicleInput(brake: 0.6));
        Run(scene, 6);

        Assert.True(high > 1.5, $"it climbed the plank: {high}");
        Assert.True(airborne > 5, $"it left the end of it: {airborne}");
        Assert.True(low > 0.3, $"it came down on its wheels: {low}");
        Assert.True(Upright(scene, "car"));
        Assert.True(At(scene, "car").Z < -36, "it cleared the ramp");
        Near(0.54, At(scene, "car").Y, 0.15, "and rests");
    }

    [Fact]
    public void CrossesAHeightFieldAndStaysOnIt()
    {
        // Rolling ground, 2 m between samples, 1.2 m between crest and trough.
        const int columns = 21;
        const int rows = 61;
        var heights = new double[columns * rows];
        for (var r = 0; r < rows; r++)
        {
            for (var c = 0; c < columns; c++)
            {
                heights[(r * columns) + c] = 0.6 * Math.Sin(r * 0.3) * Math.Cos(c * 0.2);
            }
        }

        var scene = Scene(Car("car", new Vector3(0, 1.5, 0)));
        Assert.True(scene.Physics.LayGround(GroundId, heights, columns, rows, spacing: 2, at: [-20.0, 0.0, -100.0]));
        Run(scene, 2);

        var low = double.PositiveInfinity;
        Run(scene, 9, () =>
        {
            scene.Pilot("car", Drive(scene, "car", 9, 0));
            low = Math.Min(low, At(scene, "car").Y);
            Assert.True(Upright(scene, "car"), "the car tipped over");
        });

        Assert.True(At(scene, "car").Z < -50, "it crossed the hills");
        Assert.True(low > -1.0, $"it never went through the field: {low}");
        Assert.True(At(scene, "car").Y < 5);
    }

    [Fact]
    public void CarriesACharacterWhoWalkedOntoTheBedOfATruck()
    {
        var bed = new Vector3(2.2, 0.4, 6);
        var hero = new SceneEntity(
            "hero",
            "Hero",
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(new Vector3(0, 0.01, 6)),
                [SceneComponents.Body] = new BodyComponent(
                    BodyShape.Capsule,
                    motion: BodyMotion.Character,
                    radius: 0.3,
                    height: 1.8,
                    centre: new Vector3(0, 0.9, 0),
                    stepHeight: 1.0),
            });
        var scene = Scene([Floor(), .. Car("truck", new Vector3(0, 0.7, 0), mass: 4000, size: bed), hero]);
        var stroll = new Stroll();
        scene.Walk("hero", stroll);
        Run(scene, 2);
        var truck = scene.BodyOf("truck")!.Value;
        var body = scene.BodyOf("hero")!.Value;

        stroll.Pace = 1.5;
        Run(scene, 3);
        stroll.Pace = 0;
        Run(scene, 1);

        var footing = scene.Physics.FootingOf(body)!;
        Assert.Equal(truck, footing.Ground);

        // The body origin is the capsule's middle, 0.9 m over its feet.
        Assert.True(At(scene, "hero").Y > 1.4, "feet on the bed");
        Near(At(scene, "truck").Z, At(scene, "hero").Z, 3, "on the bed");

        var started = At(scene, "hero").Z;
        var moved = 0.0;
        Run(scene, 6, () =>
        {
            scene.Pilot("truck", Drive(scene, "truck", 6, 0));
            moved = Math.Max(moved, -scene.Physics.FootingOf(body)!.Carried[2]);
        });

        footing = scene.Physics.FootingOf(body)!;
        Assert.Equal(truck, footing.Ground);
        Assert.True(moved > 2, $"carried at a walking pace or more: {moved}");
        Assert.True(At(scene, "hero").Z < started - 8);
        Near(At(scene, "truck").Z, At(scene, "hero").Z, 3, "it rode with the truck");
    }
}
