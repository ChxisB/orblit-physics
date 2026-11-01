using Orblit.Physics.Vehicle;
using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneVehicleTests : SceneTestBase
{
    private const double Travel = 0.3;

    /// <summary>Metres a 1.5 hertz spring gives under the weight of the car.</summary>
    private static readonly double Sag = 9.81 / (2 * Math.PI * 1.5 * 2 * Math.PI * 1.5);

    private static SceneEntity Wheel(string id, double x, double z, bool front = false) => new(
        id,
        id,
        "car",
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(x, Sag - Travel, z)),
            [SceneComponents.Wheel] = new WheelComponent(radius: 0.35, steers: front, drives: !front, brake: 3000),
        });

    /// <summary>
    /// A floor and a 1200 kilogram car above it, rear wheel drive, with its wheels placed where they sit
    /// at rest.
    /// </summary>
    // F2: four options of one scene, called by name.
    private static SceneDocument World(
        VehicleComponent? car = null,
        Vector3? scale = null,
        BodyMotion motion = BodyMotion.Free,
        bool wheels = true)
    {
        var chassis = new SceneEntity(
            "car",
            "Car",
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(new Vector3(0, 0.7, 0), scale: scale),
                [SceneComponents.Body] = new BodyComponent(size: new Vector3(1.8, 0.6, 4), mass: 1200, motion: motion),
                [SceneComponents.Vehicle] = car ?? new VehicleComponent(),
            });
        return wheels
            ? SceneOf(
                Floor(),
                chassis,
                Wheel("fl", -0.8, -1.2, front: true),
                Wheel("fr", 0.8, -1.2, front: true),
                Wheel("rl", -0.8, 1.2),
                Wheel("rr", 0.8, 1.2))
            : SceneOf(Floor(), chassis);
    }

    private static Vector3 At(ScenePhysics scene, string id) => TransformOf(scene.Document, id).Position;

    private static Vector3 Turned(ScenePhysics scene, string id) => TransformOf(scene.Document, id).Rotation;

    [Fact]
    public void AVehicleComponentMakesACarOfTheWheelsUnderIt()
    {
        var scene = Make(World());

        Assert.NotNull(scene.VehicleOf("car"));
        Assert.Equal(4, scene.VehicleOf("car")!.Wheels.Count);
        Assert.Null(scene.VehicleOf("floor"));
    }

    [Fact]
    public void AVehicleComponentMakesNoCarOfABodyThatIsScaledDrivenOrWheelLess()
    {
        Assert.Null(Make(World(scale: Vector3.All(2))).VehicleOf("car"));
        Assert.Null(Make(World(motion: BodyMotion.Driven)).VehicleOf("car"));
        Assert.Null(Make(World(wheels: false)).VehicleOf("car"));
    }

    [Fact]
    public void AVehicleComponentMakesNoCarWhenTheVehicleRefusesWhatTheFileSays()
    {
        var scene = Make(World(car: new VehicleComponent(ratios: [3.0, 1.0])));

        Assert.Null(scene.VehicleOf("car"));
        Assert.NotNull(scene.Document["car"]![SceneComponents.Vehicle]);
    }

    [Fact]
    public void AVehicleComponentSettlesOnItsSpringsWithTheWheelsWhereTheyWerePlaced()
    {
        var scene = Make(World());
        Run(scene, 4);

        Near(0.65 - Sag, At(scene, "car").Y, 0.03, "car");
        foreach (var (id, x, z) in new[] { ("fl", -0.8, -1.2), ("fr", 0.8, -1.2), ("rl", -0.8, 1.2), ("rr", 0.8, 1.2) })
        {
            var at = At(scene, id);
            Near(x, at.X, 1e-9, id);
            Near(Sag - Travel, at.Y, 0.03, $"{id} hangs");
            Near(z, at.Z, 1e-9, id);
        }
    }

    [Fact]
    public void AVehicleComponentKeepsTheScaleOfAWheelAndTurnsOnlyWhatItOwns()
    {
        var document = World();
        var tall = TransformOf(document, "fl").Scale;
        tall.Y = 2;
        Assert.Equal(2, tall.Y);
        var scene = Make(document);
        Run(scene, 1);

        var wheel = TransformOf(scene.Document, "fl");
        Assert.Equal(2, wheel.Scale.Y);
        Assert.Equal(0, wheel.Rotation.Z);
    }

    [Fact]
    public void APilotedCarDrivesOffRollsItsWheelsForwardAndShifts()
    {
        var scene = Make(World());
        Run(scene, 2);
        scene.Pilot("car", new VehicleInput(throttle: 1));
        Run(scene, 6);

        Assert.True(At(scene, "car").Z < -20, "front is minus z");
        Assert.True(scene.VehicleOf("car")!.Gear > 1);
        Assert.NotEqual(0, Turned(scene, "rl").X);
        Assert.True(scene.VehicleOf("car")!.Wheels[2].Roll >= 0);
    }

    [Fact]
    public void APilotedCarSteersTheFrontWheelsAndOnlyThose()
    {
        var scene = Make(World());
        Run(scene, 2);
        scene.Pilot("car", new VehicleInput(throttle: 0.3, steer: 1));
        Run(scene, 1);

        Assert.True(Turned(scene, "fl").Y > 15);
        Assert.True(Turned(scene, "fr").Y > 15);
        Assert.Equal(0, Turned(scene, "rl").Y);
        Assert.True(At(scene, "car").X < 0, "positive steer is left");
    }

    [Fact]
    public void APilotedCarLetsGoOfTheControlsWithANullInput()
    {
        var scene = Make(World());
        scene.Pilot("car", new VehicleInput(throttle: 1));
        Assert.Equal(1, scene.VehicleOf("car")!.Input.Throttle);

        scene.Pilot("car", null);
        Assert.Equal(0, scene.VehicleOf("car")!.Input.Throttle);
    }

    [Fact]
    public void APilotedCarRefusesAGearTheCarDoesNotHave()
    {
        var scene = Make(World());

        Assert.ThrowsAny<ArgumentException>(() => scene.Pilot("car", new VehicleInput(gear: 6)));
    }

    [Fact]
    public void APilotedCarKeepsItsDriverThroughAnEditThatMakesTheCarAgain()
    {
        var scene = Make(World());
        scene.Pilot("car", new VehicleInput(throttle: 1));
        var before = scene.VehicleOf("car");

        var wheel = (WheelComponent)scene.Document["rr"]![SceneComponents.Wheel]!;
        scene.Apply(new SceneDiff([
            new SetComponent("rr", SceneComponents.Wheel, wheel.ToJson(), wheel.CopyWith(radius: 0.4).ToJson()),
        ]));

        Assert.NotSame(before, scene.VehicleOf("car"));
        Assert.Equal(1, scene.VehicleOf("car")!.Input.Throttle);
    }

    [Fact]
    public void EditingACarTakesItAwayWithTheComponentOrWithTheLastWheel()
    {
        var scene = Make(World());
        var car = (VehicleComponent)scene.Document["car"]![SceneComponents.Vehicle]!;
        scene.Apply(new SceneDiff([new SetComponent("car", SceneComponents.Vehicle, car.ToJson(), null)]));
        Assert.Null(scene.VehicleOf("car"));

        var other = Make(World());
        foreach (var id in new[] { "fl", "fr", "rl", "rr" })
        {
            other.Apply(new SceneDiff([new RemoveEntity(other.Document[id]!.ToJson())]));
        }

        Assert.Null(other.VehicleOf("car"));
    }

    [Fact]
    public void EditingACarGoesBackToTheStartWithASnapshotAndDrivesTheSame()
    {
        var scene = Make(World());
        using var start = scene.Snapshot();

        scene.Pilot("car", new VehicleInput(throttle: 1, steer: 0.2));
        Run(scene, 3);
        var first = scene.Document.Encode();

        scene.Restore(start);
        Assert.NotNull(scene.VehicleOf("car"));
        Run(scene, 3);
        Assert.Equal(first, scene.Document.Encode());
    }
}
