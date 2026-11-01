using Orblit.Physics.Vehicle;
using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class ScenePhysicsAddedTests : SceneTestBase
{
    private const double Travel = 0.3;

    private const double WheelHeight = 0.2;

    /// <summary>Asks for no walk but counts the asks and falls under gravity.</summary>
    private sealed class Counting : ICharacterWalk
    {
        public int Asked { get; private set; }

        public PhysicsFooting? Last { get; private set; }

        public IReadOnlyList<double> Ask(double seconds, PhysicsFooting? footing)
        {
            Asked++;
            Last = footing;
            return [0.0, (footing?.Velocity[1] ?? 0.0) - (9.81 * seconds), 0.0];
        }
    }

    private static double SagOf(double gravity) => gravity / Math.Pow(2 * Math.PI * 1.5, 2);

    private static SceneEntity Wheel(string id, double x, double z, bool front) => new(
        id,
        id,
        "car",
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(x, WheelHeight, z)),
            [SceneComponents.Wheel] = new WheelComponent(
                radius: 0.35,
                steers: front,
                drives: !front,
                brake: 3000,
                travel: Travel,
                hertz: 1.5),
        });

    private static SceneDocument CarWorld(VehicleComponent? car = null) => SceneOf(
        Floor(),
        new SceneEntity(
            "car",
            "Car",
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(new Vector3(0, 0.7, 0)),
                [SceneComponents.Body] = new BodyComponent(size: new Vector3(1.8, 0.6, 4), mass: 1200),
                [SceneComponents.Vehicle] = car ?? new VehicleComponent(),
            }),
        Wheel("fl", -0.8, -1.2, true),
        Wheel("fr", 0.8, -1.2, true),
        Wheel("rl", -0.8, 1.2, false),
        Wheel("rr", 0.8, 1.2, false));

    private static SceneDocument HeroWorld() => SceneOf(
        Floor(),
        Entity(
            "hero",
            new BodyComponent(
                BodyShape.Capsule,
                motion: BodyMotion.Character,
                radius: 0.3,
                height: 1.8,
                centre: new Vector3(0, 0.9, 0)),
            new Vector3(0, 0.01, 0)));

    private static SceneEntity Weld(string id, JointComponent joint, string parent) => new(
        id,
        id,
        parent,
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(),
            [SceneComponents.Joint] = joint,
        });

    private static void Edit(ScenePhysics scene, string id, string type, SceneComponent? to)
    {
        var from = scene.Document[id]![type];
        scene.Apply(new SceneDiff([new SetComponent(id, type, from?.ToJson(), to?.ToJson())]));
    }

    [Fact]
    public void TheBrokenJointsAreACopyThatLaterEditsDoNotEmpty()
    {
        var weak = new JointComponent(JointKind.Fixed, breakingForce: 5);
        var scene = Make(SceneOf(Crate("crate", at: new Vector3(0, 3, 0)), Weld("weld", weak, "crate")));
        scene.Advance(1.0 / 60);
        var seen = scene.Broken;
        Assert.Equal(["weld"], seen.ToArray());

        Edit(scene, "weld", SceneComponents.Joint, new JointComponent(JointKind.Fixed, breakingForce: 6));

        Assert.Empty(scene.Broken);
        Assert.Equal(["weld"], seen.ToArray());
    }

    [Fact]
    public void TwoEntitiesAreIgnoredOnlyWhenBothHaveBodiesAndTheyAreNotOneBody()
    {
        var empty = new SceneEntity(
            "empty",
            "Empty",
            components: new Dictionary<string, SceneComponent> { [SceneComponents.Transform] = new TransformComponent() });
        var scene = Make(SceneOf(Crate("a"), Crate("b", at: new Vector3(3, 3, 0)), empty));

        Assert.False(scene.Ignore("a", "nobody"));
        Assert.False(scene.Ignore("nobody", "a"));
        Assert.False(scene.Ignore("a", "empty"));
        Assert.False(scene.Ignore("a", "a"));
        Assert.False(scene.Unignore("a", "nobody"));
        Assert.False(scene.Unignore("nobody", "a"));
        Assert.True(scene.Ignore("a", "b"));
    }

    [Fact]
    public void ASmoothSceneThatTakesNoStepDoesNotSeatTheWheelsOfACar()
    {
        var scene = Make(CarWorld(), smooth: true);

        var diff = scene.Advance(0.001);

        Assert.DoesNotContain(diff.Operations.OfType<SetComponent>(), op => op.Id is "fl" or "fr" or "rl" or "rr");
    }

    [Fact]
    public void TheCarIsWrittenBackBeforeItsWheelsAreSeatedOnIt()
    {
        var scene = Make(CarWorld());

        var ids = scene.Advance(1.0 / 60).Operations.OfType<SetComponent>().Select(op => op.Id).ToList();

        Assert.Contains("car", ids);
        Assert.Contains("fl", ids);
        Assert.True(ids.IndexOf("car") < ids.IndexOf("fl"), string.Join(",", ids));
    }

    [Fact]
    public void ACarSagsOnTheGravityOfItsSettingsAndOtherwiseOnEarths()
    {
        var earth = Make(CarWorld());
        var moon = Make(CarWorld(), settings: new PhysicsSettings { Gravity = [0, -4, 0] });

        Assert.Equal(WheelHeight + Travel - SagOf(9.81), earth.VehicleOf("car")!.Spec.Wheels[0].Mount.Y, 1e-9);
        Assert.Equal(WheelHeight + Travel - SagOf(4), moon.VehicleOf("car")!.Spec.Wheels[0].Mount.Y, 1e-9);
    }

    [Fact]
    public void ALetGoWalkIsNotAskedAgain()
    {
        var scene = Make(HeroWorld());
        var walk = new Counting();
        scene.Walk("hero", walk);
        scene.Advance(1.0 / 60);
        var asked = walk.Asked;
        Assert.True(asked > 0);

        scene.Walk("hero", null);
        scene.Advance(1.0 / 60);

        Assert.Equal(asked, walk.Asked);
    }

    [Fact]
    public void AWalkIsToldWhatTheCharacterStandsOn()
    {
        var scene = Make(HeroWorld());
        var walk = new Counting();
        scene.Walk("hero", walk);

        Run(scene, 0.5);

        Assert.NotNull(walk.Last);
        Assert.Equal(scene.BodyOf("floor")!.Value, walk.Last.Ground);
    }

    [Fact]
    public void AWalkIsForgottenWhenItsBodyIsTakenOffAndPutBack()
    {
        var scene = Make(HeroWorld());
        var walk = new Counting();
        scene.Walk("hero", walk);
        scene.Advance(1.0 / 60);
        var asked = walk.Asked;
        var body = scene.Document["hero"]![SceneComponents.Body]!;

        Edit(scene, "hero", SceneComponents.Body, null);
        scene.Advance(1.0 / 60);
        Edit(scene, "hero", SceneComponents.Body, body);
        scene.Advance(1.0 / 60);

        Assert.Equal(asked, walk.Asked);
    }

    [Fact]
    public void ADriverWhoLetGoIsNotGivenTheWheelBackWhenTheCarIsMadeAgain()
    {
        var scene = Make(CarWorld());
        scene.Pilot("car", new VehicleInput(throttle: 1));
        scene.Pilot("car", null);

        var wheel = (WheelComponent)scene.Document["rr"]![SceneComponents.Wheel]!;
        Edit(scene, "rr", SceneComponents.Wheel, wheel.CopyWith(radius: 0.4));

        Assert.Equal(0, scene.VehicleOf("car")!.Input.Throttle);
    }

    [Fact]
    public void ADriverIsLostWhenTheVehicleComponentIsTakenOffAndPutBack()
    {
        var scene = Make(CarWorld());
        scene.Pilot("car", new VehicleInput(throttle: 1));
        var car = scene.Document["car"]![SceneComponents.Vehicle]!;

        Edit(scene, "car", SceneComponents.Vehicle, null);
        Edit(scene, "car", SceneComponents.Vehicle, car);

        Assert.NotNull(scene.VehicleOf("car"));
        Assert.Equal(0, scene.VehicleOf("car")!.Input.Throttle);
    }

    [Fact]
    public void ADriverInAGearTheEditedCarLacksIsLetGoWithoutAnErrorAndStaysGone()
    {
        var scene = Make(CarWorld());
        scene.Pilot("car", new VehicleInput(throttle: 1, gear: 5));
        var car = scene.Document["car"]![SceneComponents.Vehicle]!;

        Edit(scene, "car", SceneComponents.Vehicle, new VehicleComponent(ratios: [3.6, 2.2, 1.5]));
        Assert.NotNull(scene.VehicleOf("car"));
        Edit(scene, "car", SceneComponents.Vehicle, car);

        Assert.Null(scene.VehicleOf("car")!.Input.Gear);
        Assert.Equal(0, scene.VehicleOf("car")!.Input.Throttle);
    }

    [Fact]
    public void AnEntityMovedUnderAnotherIsRebuiltWhereItNowStands()
    {
        var holder = Entity("holder", new BodyComponent(motion: BodyMotion.Fixed), new Vector3(10, 0, 0));
        var scene = Make(SceneOf(Floor(), holder, Crate("crate", at: new Vector3(0, 3, 0))));

        scene.Apply(new SceneDiff([new Reparent("crate", null, "holder")]));

        var pose = scene.Physics.TransformOf(scene.BodyOf("crate")!.Value)!;
        Assert.Equal(10.0, pose[0], 1e-6);
    }

    [Fact]
    public void ASceneTakesNoMoreStepsThanItIsToldToAndDropsTheTimeItCouldNotSpend()
    {
        var scene = Make(SceneOf(Crate("crate", at: new Vector3(0, 100, 0))), maxSteps: 3);
        var body = scene.BodyOf("crate")!.Value;

        scene.Advance(10);
        Assert.Equal(-9.81 * 3 / 60, scene.Physics.VelocityOf(body)![1], 0.005);

        scene.Advance(1.0 / 60);
        Assert.Equal(-9.81 * 4 / 60, scene.Physics.VelocityOf(body)![1], 0.005);
    }

    [Fact]
    public void ARestoreForgetsACarThatWasMadeAfterTheSnapshot()
    {
        var scene = Make(CarWorld());
        var car = scene.Document["car"]![SceneComponents.Vehicle]!;
        Edit(scene, "car", SceneComponents.Vehicle, null);
        using var before = scene.Snapshot();
        Edit(scene, "car", SceneComponents.Vehicle, car);
        Assert.NotNull(scene.VehicleOf("car"));

        scene.Restore(before);

        Assert.Null(scene.VehicleOf("car"));
    }
}
