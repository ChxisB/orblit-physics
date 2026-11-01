using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneJointsAddedTests : SceneTestBase
{
    private static readonly JointAxis[] Axes =
    [
        JointAxis.AlongX,
        JointAxis.AlongY,
        JointAxis.AlongZ,
        JointAxis.AboutX,
        JointAxis.AboutY,
        JointAxis.AboutZ,
    ];

    /// <summary>The world with gravity switched off, so only what a test pushes moves.</summary>
    private static PhysicsSettings Space => new() { Gravity = [0, 0, 0] };

    /// <summary>A hinge entity's x turned a quarter about y, which is the world's z.</summary>
    private static Vector3 AboutZ => new(0, -90, 0);

    /// <summary>A joint entity's x turned a quarter about z, which is the world's up.</summary>
    private static Vector3 Upright => new(0, 0, 90);

    // F2: six fields of one entity, called by name.
    private static SceneEntity Body(
        string id,
        Vector3 at,
        string? parent = null,
        Vector3? rotation = null,
        BodyComponent? shape = null,
        JointComponent? joint = null)
    {
        var components = new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(at, rotation),
            [SceneComponents.Body] = shape ?? new BodyComponent(),
        };
        if (joint is not null)
        {
            components[SceneComponents.Joint] = joint;
        }

        return new SceneEntity(id, id, parent, components: components);
    }

    // F2: five fields of one entity, called by name.
    private static SceneEntity Joint(
        string id,
        JointComponent joint,
        string? parent = null,
        Vector3? at = null,
        Vector3? rotation = null) => new(
        id,
        id,
        parent,
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(at, rotation),
            [SceneComponents.Joint] = joint,
        });

    private static BodyComponent Bar() => new(size: new Vector3(1, 0.1, 0.1));

    private static JointState State(ScenePhysics scene, string entity) =>
        scene.Physics.JointStateOf(scene.JointOf(entity)!.Value)!;

    private static Vector3 MiddleOf(ScenePhysics scene, string entity)
    {
        var pose = scene.Physics.TransformOf(scene.BodyOf(entity)!.Value)!;
        return new Vector3(pose[0], pose[1], pose[2]);
    }

    private static double[] Values(JointState state) => [.. state.Offset.Take(3), .. state.Angles.Take(3)];

    private static void Edit(ScenePhysics scene, string id, string type, SceneComponent? to)
    {
        var from = scene.Document[id]![type];
        scene.Apply(new SceneDiff([new SetComponent(id, type, from?.ToJson(), to?.ToJson())]));
    }

    private static JointComponent Limited(JointKind kind, JointAxis axis, double low, double high) =>
        new(kind, new Dictionary<JointAxis, JointRange> { [axis] = new(low, high) });

    [Theory]
    [InlineData(1.0, true)]
    [InlineData(50.0, false)]
    public void AWeldBreaksOnTheTorqueOfABarHungOutFromItAlone(double limit, bool breaks)
    {
        var scene = Make(SceneOf(
            Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
            Joint("weld", new JointComponent(JointKind.Fixed, breakingTorque: limit), "bar", new Vector3(-0.5, 0, 0))));

        Run(scene, 0.5);

        Assert.Equal(breaks, scene.Broken.Contains("weld"));
    }

    [Fact]
    public void ABodyHungFromAnotherPassesThroughItOnlyWhenTheJointLetsThemCollide()
    {
        double Lowest(bool collide)
        {
            var scene = Make(SceneOf(
                Body("block", new Vector3(0, 2, 0), shape: new BodyComponent(motion: BodyMotion.Fixed)),
                Body("bob", new Vector3(1, 1, 0), "block"),
                Joint("pivot", new JointComponent(JointKind.Point, collide: collide), "bob", new Vector3(-1, 0, 0))));
            var low = double.PositiveInfinity;
            Run(scene, 1.5, () => low = Math.Min(low, MiddleOf(scene, "bob").Y));
            return low;
        }

        Assert.True(Lowest(false) < 2.2, "it swings down through the block");
        Assert.True(Lowest(true) > 2.8, "it is held off the block");
    }

    [Fact]
    public void AConeStopsTheTwistOfItsBodyAtItsOwnLimit()
    {
        var scene = Make(
            SceneOf(
                Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
                Joint("socket", Limited(JointKind.Cone, JointAxis.AboutX, -20, 20), "bar", new Vector3(-0.5, 0, 0))),
            settings: Space);
        var bar = scene.BodyOf("bar")!.Value;

        // Two pushes across the bar, one above its axis and one below, which is a pure spin about it.
        scene.Physics.Push(bar, [0.0, 0.0, 0.05], [0.5, 3.05, 0.0]);
        scene.Physics.Push(bar, [0.0, 0.0, -0.05], [0.5, 2.95, 0.0]);
        Run(scene, 1.5);

        Assert.Equal(20 * Math.PI / 180, Math.Abs(State(scene, "socket").Angles[0]), 0.01);
    }

    [Theory]
    [InlineData(JointAxis.AlongX)]
    [InlineData(JointAxis.AlongY)]
    [InlineData(JointAxis.AlongZ)]
    [InlineData(JointAxis.AboutX)]
    [InlineData(JointAxis.AboutY)]
    [InlineData(JointAxis.AboutZ)]
    public void ASixAxisJointMovesOnlyOnTheOneAxisItLeavesFree(JointAxis free)
    {
        var limits = Axes.ToDictionary(axis => axis, axis => axis == free ? new JointRange(-100, 100) : JointRange.At(0));
        var scene = Make(
            SceneOf(Body("body", new Vector3(0, 3, 0), joint: new JointComponent(JointKind.SixAxis, limits))),
            settings: Space);

        // Off the middle of the body, so the push moves it and turns it every way at once.
        scene.Physics.Push(scene.BodyOf("body")!.Value, [0.2, 0.2, 0.2], [0.3, 3.2, 0.1]);
        Run(scene, 0.5);

        var values = Values(State(scene, "body"));
        for (var i = 0; i < Axes.Length; i++)
        {
            if (Axes[i] == free)
            {
                Assert.True(Math.Abs(values[i]) > 0.04, $"{Axes[i]} moved {values[i]}");
            }
            else
            {
                Assert.True(Math.Abs(values[i]) < 0.005, $"{Axes[i]} moved {values[i]}");
            }
        }
    }

    [Fact]
    public void ASliderStopsAtItsLimitAlongTheAxisItSlidesOn()
    {
        var scene = Make(SceneOf(
            Body("slab", new Vector3(0, 3, 0)),
            Joint("rail", Limited(JointKind.Slider, JointAxis.AlongX, -2, 2), "slab", rotation: Upright)));

        Run(scene, 3);

        Assert.Equal(2, Math.Abs(State(scene, "rail").Offset[0]), 0.02);
    }

    [Fact]
    public void ASliderGoalIsInMetresAndNotDegrees()
    {
        var scene = Make(SceneOf(
            Body("slab", new Vector3(0, 3, 0)),
            Joint("rail", new JointComponent(JointKind.Slider, goal: 0.5, goalHertz: 6, strength: 1000), "slab", rotation: Upright)));

        Run(scene, 3);

        Assert.Equal(0.5, State(scene, "rail").Offset[0], 0.02);
    }

    [Theory]
    [InlineData(false, 0.0)]
    [InlineData(true, -26.565)]
    public void APathTurnsItsBodyToTheLineOnlyWhenToldToFollowIt(bool follow, double turned)
    {
        var scene = Make(SceneOf(
            Body("ball", new Vector3(0, 3, 0), shape: new BodyComponent(size: new Vector3(0.4, 0.1, 0.1))),
            Joint(
                "rail",
                new JointComponent(JointKind.Path, path: [new Vector3(0, 0, 0), new Vector3(3, -1.5, 0), new Vector3(6, -1.5, 0)], follow: follow),
                "ball")));

        Run(scene, 0.5);

        Assert.Equal(turned, TransformOf(scene.Document, "ball").Rotation.Z, 0.5);
    }

    [Fact]
    public void APulleyWithARatioOfTwoLetsTheHeavierWeightFallTwiceAsFarAsTheOtherRises()
    {
        var scene = Make(SceneOf(
            Body("heavy", new Vector3(-1, 1, 0), shape: new BodyComponent(size: new Vector3(0.2, 0.2, 0.2), mass: 2)),
            Body("light", new Vector3(2, 0, 0), "heavy", shape: new BodyComponent(size: new Vector3(0.2, 0.2, 0.2))),
            Joint(
                "rope",
                new JointComponent(JointKind.Pulley, over: [new Vector3(0, 2, 0), new Vector3(2, 2, 0)], ratio: 2),
                "light",
                new Vector3(-2, 0, 0))));

        Run(scene, 0.5);

        var fall = 1 - MiddleOf(scene, "heavy").Y;
        var rise = MiddleOf(scene, "light").Y - 1;
        Assert.True(rise > 0.3, $"it rose {rise}");
        Assert.Equal(2 * rise, fall, 0.02);
    }

    [Fact]
    public void ARackAtARatioOfTwoTurnsItsPinionHalfAsFarAsItSlides()
    {
        var scene = Make(
            SceneOf(
                Body("rack", new Vector3(0, 3, 0), shape: new BodyComponent(size: new Vector3(2, 0.2, 0.2))),
                Joint("slide", new JointComponent(JointKind.Slider, speed: 1, strength: 1000), "rack"),
                Body("pinion", new Vector3(0, 1, 0), "rack", shape: new BodyComponent(BodyShape.Cylinder, radius: 0.3, height: 0.2)),
                Joint("bearing", new JointComponent(), "pinion"),
                Joint("teeth", new JointComponent(JointKind.Rack, ratio: 2), "pinion")),
            settings: Space);

        Run(scene, 0.5);

        var slid = State(scene, "slide").Offset[0];
        Assert.Equal(0.5, slid, 0.02);
        Assert.Equal(slid / 2, State(scene, "bearing").Angles[0], 0.02);
    }

    [Fact]
    public void AGearTurnsABodyAboutTheAxisItIsToldItsOwnIsOn()
    {
        var scene = Make(
            SceneOf(
                Body("carrier", new Vector3(0, 3, 0), shape: new BodyComponent(size: new Vector3(0.4, 0.2, 0.4))),
                Joint("axle", new JointComponent(speed: 60, strength: 1000), "carrier", rotation: Upright),
                Body("wheel", new Vector3(1.5, 0, 0), "carrier", shape: new BodyComponent(size: new Vector3(0.4, 0.2, 0.4))),
                Joint("bearing", new JointComponent(), "wheel", rotation: AboutZ),
                Joint("mesh", new JointComponent(JointKind.Gear, ratio: -1, axisB: new Vector3(0, 0, 1)), "wheel", rotation: Upright)),
            settings: Space);

        Run(scene, 0.5);

        var carrier = State(scene, "axle").Angles[0];
        Assert.True(carrier > 0.3, $"the carrier turned {carrier}");
        Assert.Equal(-carrier, State(scene, "bearing").Angles[0], 0.05);
    }

    [Fact]
    public void ASoftLimitAtALowRatioGivesFurtherThanOneAtARatioOfOne()
    {
        double Lowest(double ratio)
        {
            var scene = Make(SceneOf(
                Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
                Joint(
                    "hinge",
                    new JointComponent(
                        limits: new Dictionary<JointAxis, JointRange> { [JointAxis.AboutX] = new(-30, 30) },
                        softHertz: 1,
                        softRatio: ratio),
                    "bar",
                    new Vector3(-0.5, 0, 0),
                    AboutZ)));
            var low = double.PositiveInfinity;
            Run(scene, 3, () => low = Math.Min(low, State(scene, "hinge").Angles[0]));
            return low;
        }

        Assert.True(Lowest(0.05) < Lowest(1) - 0.3);
    }

    [Fact]
    public void ASpringAtALowRatioSagsFurtherThanOneAtARatioOfOne()
    {
        double Lowest(double ratio)
        {
            var scene = Make(SceneOf(
                Body("ball", new Vector3(0, 2, 0)),
                Joint(
                    "hook",
                    new JointComponent(JointKind.Distance, springs: new Dictionary<JointAxis, JointGive> { [JointAxis.AlongX] = new(2, ratio) }),
                    "ball",
                    new Vector3(0, 1, 0))));
            var low = double.PositiveInfinity;
            Run(scene, 2, () => low = Math.Min(low, MiddleOf(scene, "ball").Y));
            return low;
        }

        Assert.True(Lowest(0.05) < Lowest(1) - 0.03);
    }

    [Fact]
    public void AGoalAtALowRatioOvershootsFurtherThanOneAtARatioOfOne()
    {
        double Lowest(double ratio)
        {
            var scene = Make(SceneOf(
                Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
                Joint(
                    "hinge",
                    new JointComponent(goal: -40, goalHertz: 6, goalRatio: ratio, strength: 1000),
                    "bar",
                    new Vector3(-0.5, 0, 0),
                    AboutZ)));
            var low = double.PositiveInfinity;
            Run(scene, 2, () => low = Math.Min(low, State(scene, "hinge").Angles[0]));
            return low;
        }

        Assert.True(Lowest(0.05) < Lowest(1) - 0.3);
    }

    [Fact]
    public void ASpringAboutZPullsToItsRest()
    {
        var scene = Make(
            SceneOf(
                Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
                Joint(
                    "pin",
                    new JointComponent(JointKind.Point, springs: new Dictionary<JointAxis, JointGive> { [JointAxis.AboutZ] = new(8, Rest: 30) }),
                    "bar",
                    new Vector3(-0.5, 0, 0))),
            settings: Space);

        Run(scene, 2);

        Assert.Equal(30 * Math.PI / 180, State(scene, "pin").Angles[2], 0.02);
    }

    [Fact]
    public void JointsAreNumberedNearestTheRootFirstWhateverTheOrderOfTheDocument()
    {
        var scene = Make(SceneOf(
            Body("deep", new Vector3(0, 0, 0), "mid"),
            Body("mid", new Vector3(0, 1, 0), "top"),
            Body("top", new Vector3(0, 10, 0)),
            Joint("low", new JointComponent(JointKind.Point), "deep"),
            Joint("high", new JointComponent(JointKind.Point), "top")));

        Assert.True(scene.JointOf("high") < scene.JointOf("low"));
    }

    [Fact]
    public void ABrokenJointIsForgottenWhenItsComponentIsTakenOff()
    {
        var scene = Make(SceneOf(
            Body("crate", new Vector3(0, 3, 0)),
            Joint("weld", new JointComponent(JointKind.Fixed, breakingForce: 5), "crate")));
        scene.Advance(1.0 / 60);
        Assert.Contains("weld", scene.Broken);

        Edit(scene, "weld", SceneComponents.Joint, null);

        Assert.DoesNotContain("weld", scene.Broken);
    }

    [Fact]
    public void ABrokenJointIsNotMadeAgainWhenTheBodyItHeldIsBuiltAgain()
    {
        var scene = Make(SceneOf(
            Body("crate", new Vector3(0, 3, 0)),
            Joint("weld", new JointComponent(JointKind.Fixed, breakingForce: 5), "crate")));
        scene.Advance(1.0 / 60);
        var body = (BodyComponent)scene.Document["crate"]![SceneComponents.Body]!;

        Edit(scene, "crate", SceneComponents.Body, body.CopyWith(mass: 2));

        Assert.Contains("weld", scene.Broken);
        Assert.Null(scene.Physics.JointStateOf(scene.JointOf("weld")!.Value));
    }

    [Fact]
    public void AChildBodyIsWrittenAgainstWhereItsParentWentEvenWhenTheDocumentListsItFirst()
    {
        var scene = Make(SceneOf(
            Body("inner", new Vector3(0, 2, 0), "outer"),
            Body("outer", new Vector3(0, 20, 0))));

        Run(scene, 1);

        Assert.Equal(2, TransformOf(scene.Document, "inner").Position.Y, 1e-3);
    }

    [Fact]
    public void ABodyTheWorldHasLostIsLeftWhereTheDocumentHasIt()
    {
        var scene = Make(SceneOf(Body("crate", new Vector3(0, 3, 0), rotation: new Vector3(10, 20, 30))));
        scene.Physics.Remove(scene.BodyOf("crate")!.Value);

        scene.Advance(1.0 / 60);

        var crate = TransformOf(scene.Document, "crate");
        Assert.Equal(3, crate.Position.Y, 1e-6);
        Assert.Equal(30, crate.Rotation.Z, 1e-3);
    }

    [Fact]
    public void ATurnPastAHalfTurnIsWrittenBackOnTheSideItWasOn()
    {
        var scene = Make(SceneOf(Body("hero", new Vector3(0, 20, 0), rotation: new Vector3(0, 0, 200))));

        Run(scene, 0.1);

        Assert.Equal(200, TransformOf(scene.Document, "hero").Rotation.Z, 0.01);
    }

    [Fact]
    public void AScaledEntityWithACentreIsWrittenBackByWhereItsOriginWent()
    {
        var fat = new SceneEntity(
            "fat",
            "Fat",
            components: new Dictionary<string, SceneComponent>
            {
                [SceneComponents.Transform] = new TransformComponent(new Vector3(0, 20, 0), scale: Vector3.All(2)),
                [SceneComponents.Body] = new BodyComponent(centre: new Vector3(0.4, 0, 0)),
            });
        var scene = Make(SceneOf(fat));

        Run(scene, 0.5);

        Assert.Equal(0.8, MiddleOf(scene, "fat").X, 1e-3);
        Assert.Equal(0, TransformOf(scene.Document, "fat").Position.X, 1e-4);
    }
}
