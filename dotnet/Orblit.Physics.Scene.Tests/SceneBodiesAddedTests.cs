using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneBodiesAddedTests : SceneTestBase
{
    private static readonly PhysicsSettings Space = new() { Gravity = [0, 0, 0] };

    /// <summary>Walks along minus z at <see cref="Pace"/>, with gravity in the request.</summary>
    private sealed class Stroll : ICharacterWalk
    {
        public double Pace { get; init; } = 2;

        public IReadOnlyList<double> Ask(double seconds, PhysicsFooting? footing) =>
            [0.0, (footing?.Velocity[1] ?? 0.0) - (9.81 * seconds), -Pace];
    }

    private static SceneEntity Ball(string id, double x, BodyComponent? body = null) => Entity(
        id,
        body ?? new BodyComponent(BodyShape.Sphere, radius: 0.5),
        new Vector3(x, 0, 0));

    private static SceneEntity Hero(string id, double x, BodyComponent? body = null) => Entity(
        id,
        body ?? Walker(),
        new Vector3(x, 0.01, 0));

    private static BodyComponent Walker(double steepest = 0.7853981633974483) => new(
        BodyShape.Capsule,
        motion: BodyMotion.Character,
        radius: 0.3,
        height: 1.8,
        centre: new Vector3(0, 0.9, 0),
        steepest: steepest);

    private static double Linear(ScenePhysics scene, string id)
    {
        var v = scene.Physics.VelocityOf(scene.BodyOf(id)!.Value)!;
        return Math.Sqrt((double)v[0] * v[0] + ((double)v[1] * v[1]) + ((double)v[2] * v[2]));
    }

    private static double Spin(ScenePhysics scene, string id)
    {
        var v = scene.Physics.VelocityOf(scene.BodyOf(id)!.Value)!;
        return Math.Sqrt((double)v[3] * v[3] + ((double)v[4] * v[4]) + ((double)v[5] * v[5]));
    }

    private static double MiddleY(ScenePhysics scene, string id) => scene.Physics.TransformOf(scene.BodyOf(id)!.Value)![1];

    private static void Shove(ScenePhysics scene, string id, double x) =>
        scene.Physics.Push(scene.BodyOf(id)!.Value, [5.0, 0.0, 0.0], [x, 0.0, 0.0]);

    /// <summary>A push off centre that sets a ball turning about z.</summary>
    private static void Twist(ScenePhysics scene, string id, double x) =>
        scene.Physics.Push(scene.BodyOf(id)!.Value, [0.0, 1.0, 0.0], [x + 0.5, 0.0, 0.0]);

    [Fact]
    public void ABallThatIsBouncyBouncesBackUpAndOneThatIsNotDoesNot()
    {
        var scene = Make(SceneOf(
            Floor(),
            Crate("lively", at: new Vector3(0, 2, 0), body: new BodyComponent(BodyShape.Sphere, radius: 0.5, restitution: 0.9))));
        var fastest = 0.0;

        Run(scene, 1.5, () => fastest = Math.Max(fastest, scene.Physics.VelocityOf(scene.BodyOf("lively")!.Value)![1]));

        Assert.True(fastest > 3, $"it came back up at {fastest}");
    }

    [Fact]
    public void LinearDampingSlowsABodyAndTheDefaultSlowsItLess()
    {
        var scene = Make(
            SceneOf(
                Ball("free", 0),
                Ball("held", 20, new BodyComponent(BodyShape.Sphere, radius: 0.5, linearDamping: 3))),
            settings: Space);
        Shove(scene, "free", 0);
        Shove(scene, "held", 20);

        Run(scene, 1);

        Assert.True(Linear(scene, "free") > 3, $"{Linear(scene, "free")}");
        Assert.True(Linear(scene, "held") < Linear(scene, "free") / 2, $"{Linear(scene, "held")}");
    }

    [Fact]
    public void AngularDampingSlowsATurnAndTheDefaultSlowsItLess()
    {
        var scene = Make(
            SceneOf(
                Ball("free", 0),
                Ball("held", 20, new BodyComponent(BodyShape.Sphere, radius: 0.5, angularDamping: 3))),
            settings: Space);
        Twist(scene, "free", 0);
        Twist(scene, "held", 20);

        Run(scene, 1);

        Assert.True(Spin(scene, "free") > 2, $"{Spin(scene, "free")}");
        Assert.True(Spin(scene, "held") < Spin(scene, "free") / 2, $"{Spin(scene, "held")}");
    }

    [Fact]
    public void AZoneSlowsTheTurnOfABodyInsideItAndNotOneOutside()
    {
        var zone = new SceneEntity(
            "zone",
            "Zone",
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(new Vector3(0, 0, 0)),
                [SceneComponents.Body] = new BodyComponent(size: new Vector3(10, 10, 10), motion: BodyMotion.Fixed, trigger: true),
                [SceneComponents.Zone] = new ZoneComponent(angularDamping: 3),
            });
        var scene = Make(SceneOf(zone, Ball("inside", 0), Ball("outside", 30)), settings: Space);
        Twist(scene, "inside", 0);
        Twist(scene, "outside", 30);

        Run(scene, 1);

        Assert.True(Spin(scene, "outside") > 2, $"{Spin(scene, "outside")}");
        Assert.True(Spin(scene, "inside") < Spin(scene, "outside") / 2, $"{Spin(scene, "inside")}");
    }

    [Fact]
    public void ABodyThatIsOnOneLayerAndCaresAboutAnotherLandsOnTheFloorItCaresAbout()
    {
        var scene = Make(SceneOf(Floor(), Crate("crate", body: new BodyComponent(layers: 2, cares: 1))));

        Run(scene, 2);

        Assert.True(MiddleY(scene, "crate") > 0.4, $"{MiddleY(scene, "crate")}");
    }

    [Fact]
    public void ABodyWithASpeedCapIsHeldToIt()
    {
        var scene = Make(SceneOf(Ball("capped", 0, new BodyComponent(BodyShape.Sphere, radius: 0.5, maxSpeed: 2))), settings: Space);
        Shove(scene, "capped", 0);

        Run(scene, 0.5);

        Assert.True(Linear(scene, "capped") < 2.2, $"{Linear(scene, "capped")}");
    }

    [Fact]
    public void ABodyWithASpinCapIsHeldToIt()
    {
        var scene = Make(SceneOf(Ball("capped", 0, new BodyComponent(BodyShape.Sphere, radius: 0.5, maxSpin: 1))), settings: Space);
        Twist(scene, "capped", 0);

        Run(scene, 0.5);

        Assert.True(Spin(scene, "capped") < 1.1, $"{Spin(scene, "capped")}");
    }

    [Fact]
    public void ABodyWithItsOwnInertiaTurnsAsThatInertiaSays()
    {
        var scene = Make(
            SceneOf(
                Ball("light", 0),
                Ball("heavy", 20, new BodyComponent(BodyShape.Sphere, radius: 0.5, inertia: new Vector3(50, 50, 50)))),
            settings: Space);
        Twist(scene, "light", 0);
        Twist(scene, "heavy", 20);

        Run(scene, 0.2);

        Assert.True(Spin(scene, "light") > 2, $"{Spin(scene, "light")}");
        Assert.True(Spin(scene, "heavy") < 0.5, $"{Spin(scene, "heavy")}");
    }

    [Theory]
    [InlineData(-2, 2, 2)]
    [InlineData(2, -2, 2)]
    [InlineData(2, 2, -2)]
    public void ABoxWithANegativeSizeIsTheSameBoxAsOneWithAPositiveSize(double x, double y, double z)
    {
        var scene = Make(SceneOf(Floor(), Crate("crate", body: new BodyComponent(size: new Vector3(x, y, z)))));

        Run(scene, 2);

        Near(1.0, MiddleY(scene, "crate"), Resting, "a two metre box rests with its middle a metre up");
    }

    [Fact]
    public void AMirroredEntityIsPlacedAndWrittenBackWithItsCentreMirroredToo()
    {
        var body = new BodyComponent(centre: new Vector3(0.4, 0, 0));
        var scene = Make(SceneOf(Floor(), Crate("crate", at: new Vector3(0, 3, 0), scale: new Vector3(-1, 1, 1), body: body)));

        Run(scene, 2);

        Near(0.0, TransformOf(scene.Document, "crate").Position.X, 0.01, "it fell straight down");
        Near(-0.4, scene.Physics.TransformOf(scene.BodyOf("crate")!.Value)![0], 0.01, "and its body is off to the other side of it");
        Near(0.5, MiddleY(scene, "crate"), Resting, "a metre box rests half a metre up");
    }

    [Fact]
    public void ACylinderLyingDownIsAsWideAsTheBroaderOfItsTwoScales()
    {
        var body = new BodyComponent(BodyShape.Cylinder, radius: 0.5, height: 2);
        var scene = Make(SceneOf(
            Floor(),
            Crate("cylinder", rotation: new Vector3(0, 0, 90), scale: new Vector3(1, 3, 2), body: body)));

        Run(scene, 3);

        Near(1.0, MiddleY(scene, "cylinder"), Resting, "it rests on a radius of 0.5 * 2");
    }

    [Fact]
    public void ACapsuleLyingDownIsAsWideAsTheBroaderOfItsTwoScales()
    {
        var body = new BodyComponent(BodyShape.Capsule, radius: 0.5, height: 4);
        var scene = Make(SceneOf(
            Floor(),
            Crate("capsule", rotation: new Vector3(0, 0, 90), scale: new Vector3(1, 1, 2), body: body)));

        Run(scene, 3);

        Near(1.0, MiddleY(scene, "capsule"), Resting, "it rests on a radius of 0.5 * 2");
    }

    [Theory]
    [InlineData(BodyShape.Sphere)]
    [InlineData(BodyShape.Box)]
    [InlineData(BodyShape.Cylinder)]
    [InlineData(BodyShape.Capsule)]
    public void EveryShapeThatCanWalkIsSweptAsACharacter(BodyShape shape)
    {
        var body = new BodyComponent(
            shape,
            motion: BodyMotion.Character,
            size: new Vector3(0.6, 1.8, 0.6),
            radius: 0.3,
            height: 1.8,
            centre: new Vector3(0, 0.9, 0));
        var scene = Make(SceneOf(Floor(), Hero("hero", 0, body)));

        Assert.NotNull(scene.Physics.FootingOf(scene.BodyOf("hero")!.Value));
    }

    [Fact]
    public void ACharacterClimbsASlopeItIsAllowedAndIsStoppedByOneItIsNot()
    {
        var ramp = Crate(
            "ramp",
            at: new Vector3(0, 1.913, -4.514),
            rotation: new Vector3(30, 0, 0),
            body: new BodyComponent(size: new Vector3(4, 0.2, 8), motion: BodyMotion.Fixed));
        var scene = Make(SceneOf(Floor(), ramp, Hero("climber", -1), Hero("held", 1, Walker(steepest: 0.3))));
        var stroll = new Stroll();
        scene.Walk("climber", stroll);
        scene.Walk("held", stroll);

        Run(scene, 3);

        var climbed = TransformOf(scene.Document, "climber").Position;
        var stopped = TransformOf(scene.Document, "held").Position;
        Assert.True(climbed.Z < -3, $"climber at {climbed.Z}");
        Assert.True(stopped.Z > -1.7, $"held at {stopped.Z}");
    }

    [Fact]
    public void ACharacterPushesACrateWithTheStrengthItIsGiven()
    {
        var scene = Make(SceneOf(Floor(), Hero("pusher", 0), Crate("crate", at: new Vector3(0, 0.5, -2))));
        scene.Walk("pusher", new Stroll());

        Run(scene, 3);

        Assert.True(TransformOf(scene.Document, "crate").Position.Z < -4, $"{TransformOf(scene.Document, "crate").Position.Z}");
    }

    [Fact]
    public void AMirroredCharacterCanCrouchAndStaysWhereItStood()
    {
        var body = Walker().CopyWith(crouchHeight: 1.0);
        var hero = Entity("hero", body, new Vector3(0, 1.81, 0), scale: new Vector3(-1, -1, -1));
        var scene = Make(SceneOf(Floor(), hero));

        Assert.True(scene.Crouch("hero", true));
        Assert.True(scene.IsCrouching("hero"));
    }

    [Fact]
    public void ACrouchIsForgottenWhenTheBodyIsBuiltAgain()
    {
        var body = Walker();
        var scene = Make(SceneOf(Floor(), Hero("hero", 0, body.CopyWith(crouchHeight: 1.0))));
        Assert.True(scene.Crouch("hero", true));
        Assert.True(scene.IsCrouching("hero"));

        scene.Apply(new SceneDiff([new SetField("hero", SceneComponents.Body, "radius", 0.3, 0.31)]));

        Assert.False(scene.IsCrouching("hero"));
    }

    [Fact]
    public void ABodyLandsOnlyOnAFloorThatIsOnALayerItCaresAboutAndCaresAboutItsOwn()
    {
        var floor = Entity("floor", new BodyComponent(BodyShape.Plane, motion: BodyMotion.Fixed, layers: 4, cares: 2));
        var scene = Make(SceneOf(
            floor,
            Crate("lands", at: new Vector3(-3, 3, 0), body: new BodyComponent(layers: 2, cares: 4)),
            Crate("falls", at: new Vector3(3, 3, 0), body: new BodyComponent(layers: 4, cares: 2))));

        Run(scene, 2);

        Assert.True(MiddleY(scene, "lands") > 0.4, $"{MiddleY(scene, "lands")}");
        Assert.True(MiddleY(scene, "falls") < 0, $"{MiddleY(scene, "falls")}");
    }

    [Fact]
    public void ACapsuleTurnedOverByANegativeScaleIsAsTallAsOneThatIsNot()
    {
        var capsule = new BodyComponent(BodyShape.Capsule, radius: 0.3, height: 1.8);
        var scene = Make(SceneOf(Floor(), Crate("capsule", scale: new Vector3(1, -1, 1), body: capsule)));

        Run(scene, 3);

        Near(0.9, MiddleY(scene, "capsule"), Resting, "a capsule 1.8 m tall rests with its middle 0.9 m up");
    }
}
