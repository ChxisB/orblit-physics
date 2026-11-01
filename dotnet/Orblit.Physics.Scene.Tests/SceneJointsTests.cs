using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneJointsTests : SceneTestBase
{
    /// <summary>A body, free unless told otherwise.</summary>
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

    /// <summary>An entity that is only a joint, placed <paramref name="at"/> under <paramref name="parent"/>.</summary>
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

    /// <summary>A bar a metre long along x, light across.</summary>
    private static BodyComponent Bar() => new(size: new Vector3(1, 0.1, 0.1));

    /// <summary>A body a metre tall and thin across.</summary>
    private static BodyComponent Upright() => new(size: new Vector3(0.1, 1, 0.1));

    /// <summary>Where the world has <paramref name="entity"/>'s body's middle.</summary>
    private static Vector3 MiddleOf(ScenePhysics scene, string entity)
    {
        var pose = scene.Physics.TransformOf(scene.BodyOf(entity)!.Value)!;
        return new Vector3(pose[0], pose[1], pose[2]);
    }

    private static JointState? StateOf(ScenePhysics scene, string entity) =>
        scene.Physics.JointStateOf(scene.JointOf(entity)!.Value);

    private static double ToRadians(double degrees) => degrees * Math.PI / 180;

    [Fact]
    public void OnABodyWithNothingAboveItWeldsThatBodyToTheWorld()
    {
        var scene = Make(SceneOf(Body("crate", new Vector3(0, 3, 0), joint: new JointComponent(JointKind.Fixed))));
        Run(scene, 2);

        Assert.Equal(3, MiddleOf(scene, "crate").Y, 0.01);
        Assert.NotNull(StateOf(scene, "crate"));
    }

    private static SceneDocument Pendulum() => SceneOf(
        Body("ball", new Vector3(1, 3, 0)),
        Joint("pivot", new JointComponent(JointKind.Point), "ball", new Vector3(-1, 0, 0)));

    [Fact]
    public void IsNumberedApartFromTheBodiesOneToOneWithItsEntity()
    {
        var scene = Make(Pendulum());

        var number = scene.JointOf("pivot")!.Value;
        Assert.Equal("pivot", scene.EntityOfJoint(number));
        Assert.Null(scene.JointOf("ball"));
        Assert.Empty(scene.Broken);
    }

    [Fact]
    public void PlacedAwayFromItsBodyIsWhereTheBodySwingsAbout()
    {
        var scene = Make(Pendulum());
        Run(scene, 0.4);

        var ball = MiddleOf(scene, "ball");
        Assert.True(ball.Y < 2.5, "it swings down");
        Assert.Equal(1, ball.DistanceTo(new Vector3(0, 3, 0)), 0.01);
    }

    [Fact]
    public void HoldsABodyToTheBodyAboveItAndThatOneToTheWorld()
    {
        // An arm hanging from a shoulder, and a forearm from its elbow, each a joint entity at the top of
        // the body it holds.
        var point = new JointComponent(JointKind.Point);
        var scene = Make(SceneOf(
            Body("arm", new Vector3(0, 5, 0), shape: Upright()),
            Joint("shoulder", point, "arm", new Vector3(0, 0.5, 0)),
            Body("forearm", new Vector3(0, -1, 0), "arm", shape: Upright()),
            Joint("elbow", point, "forearm", new Vector3(0, 0.5, 0))));
        Run(scene, 1);
        Assert.Equal(4, MiddleOf(scene, "forearm").Y, 0.02);

        // Without the shoulder the two fall together, still an arm.
        var shoulder = scene.Document["shoulder"]!;
        scene.Apply(new SceneDiff([new RemoveEntity(shoulder.ToJson())]));
        Assert.Null(scene.JointOf("shoulder"));
        Run(scene, 0.5);

        var arm = MiddleOf(scene, "arm");
        Assert.True(arm.Y < 4.5);
        Assert.Equal(1, arm.DistanceTo(MiddleOf(scene, "forearm")), 0.02);
    }

    [Fact]
    public void WithNothingAtOrAboveItToHoldHoldsNothing()
    {
        var scene = Make(SceneOf(Joint("loose", new JointComponent(JointKind.Fixed))));

        Assert.NotNull(scene.JointOf("loose"));
        Assert.Null(StateOf(scene, "loose"));
    }

    [Fact]
    public void AHingeStopsAtItsLimit()
    {
        // A bar hinged at one end about the world's z axis, which is the hinge entity's x turned a
        // quarter about y, falls to its limit.
        var scene = Make(SceneOf(
            Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
            Joint(
                "hinge",
                new JointComponent(limits: new Dictionary<JointAxis, JointRange> { [JointAxis.AboutX] = new(-30, 30) }),
                "bar",
                new Vector3(-0.5, 0, 0),
                new Vector3(0, -90, 0))));
        Run(scene, 2);

        var twist = StateOf(scene, "hinge")!.Angles[0];
        Assert.Equal(ToRadians(30), Math.Abs(twist), 0.02);
        var turned = (TransformComponent)scene.Document["bar"]![SceneComponents.Transform]!;
        Assert.Equal(-30, turned.Rotation.Z, 1.5);
    }

    [Fact]
    public void AMotorDrivesAtItsSpeed()
    {
        // A plate turning about the upright, which is the joint entity's x turned a quarter about z.
        var scene = Make(SceneOf(
            Body("plate", new Vector3(0, 3, 0), shape: new BodyComponent(size: new Vector3(1, 0.1, 1))),
            Joint("spindle", new JointComponent(speed: 90, strength: 1000), "plate", rotation: new Vector3(0, 0, 90))));
        Run(scene, 0.5);

        Assert.Equal(ToRadians(45), StateOf(scene, "spindle")!.Angles[0], 0.05);
        Assert.Equal(3, MiddleOf(scene, "plate").Y, 0.01);
    }

    [Fact]
    public void AConeLetsItsBodySwingNoFurtherThanItsSwing()
    {
        var scene = Make(SceneOf(
            Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
            Joint("socket", new JointComponent(JointKind.Cone, swing: 30), "bar", new Vector3(-0.5, 0, 0))));
        Run(scene, 2);

        var tip = MiddleOf(scene, "bar");
        var below = Math.Atan2(3 - tip.Y, Math.Sqrt((tip.X * tip.X) + (tip.Z * tip.Z)));
        Assert.Equal(ToRadians(30), below, 0.03);
    }

    [Fact]
    public void ADistanceJointWithARangeIsARope()
    {
        // The hook is a metre above the ball, on a rope of two.
        var scene = Make(SceneOf(
            Body("ball", new Vector3(0, 3, 0)),
            Joint(
                "hook",
                new JointComponent(
                    JointKind.Distance,
                    new Dictionary<JointAxis, JointRange> { [JointAxis.AlongX] = new(0, 2) }),
                "ball",
                new Vector3(0, 1, 0))));
        Run(scene, 2);

        Assert.Equal(2, MiddleOf(scene, "ball").Y, 0.02);
    }

    [Fact]
    public void ASixAxisJointFreesOnlyWhatItLeavesFree()
    {
        // A lift: locked every way but down, and down a metre at most.
        var locked = JointRange.At(0);
        var scene = Make(SceneOf(
            Body(
                "lift",
                new Vector3(0, 3, 0),
                joint: new JointComponent(
                    JointKind.SixAxis,
                    new Dictionary<JointAxis, JointRange>
                    {
                        [JointAxis.AlongX] = locked,
                        [JointAxis.AlongY] = new(-1, 0),
                        [JointAxis.AlongZ] = locked,
                        [JointAxis.AboutX] = locked,
                        [JointAxis.AboutY] = locked,
                        [JointAxis.AboutZ] = locked,
                    }))));
        Run(scene, 2);

        var lift = MiddleOf(scene, "lift");
        Assert.Equal(2, lift.Y, 0.02);
        Assert.True(Math.Abs(lift.X) + Math.Abs(lift.Z) < 0.01);
    }

    [Fact]
    public void ASpringOnADistanceJointRestsBelowItsHook()
    {
        // A ball a metre below a hook on a spring of two hertz. It settles a rest length plus gravity
        // over omega squared down.
        var scene = Make(SceneOf(
            Body("ball", new Vector3(0, 2, 0)),
            Joint(
                "hook",
                new JointComponent(
                    JointKind.Distance,
                    springs: new Dictionary<JointAxis, JointGive> { [JointAxis.AlongX] = new(2) }),
                "ball",
                new Vector3(0, 1, 0))));
        Run(scene, 4);

        const double omega = 4 * Math.PI;
        var length = 3 - MiddleOf(scene, "ball").Y;
        Assert.Equal(1 + (9.81 / (omega * omega)), length, 0.01);
    }

    [Fact]
    public void ASpringWithItsOwnRestPullsToItInDegrees()
    {
        // A bar hinged at one end, held up by a spring that rests at -40 degrees. It sags below that by
        // gravity over the spring.
        var scene = Make(SceneOf(
            Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
            Joint(
                "hinge",
                new JointComponent(springs: new Dictionary<JointAxis, JointGive> { [JointAxis.AboutX] = new(8, Rest: -40) }),
                "bar",
                new Vector3(-0.5, 0, 0),
                new Vector3(0, -90, 0))));
        Run(scene, 3);

        var settled = StateOf(scene, "hinge")!.Angles[0];
        Assert.True(settled < ToRadians(-40) + 0.01);
        Assert.True(settled > ToRadians(-40) - 0.4);
    }

    [Fact]
    public void AGoalHoldsAHingeAtAnAngleAgainstItsWeight()
    {
        var scene = Make(SceneOf(
            Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
            Joint(
                "hinge",
                new JointComponent(goal: -40, goalHertz: 6, strength: 1000),
                "bar",
                new Vector3(-0.5, 0, 0),
                new Vector3(0, -90, 0))));
        Run(scene, 2);

        Assert.Equal(ToRadians(-40), StateOf(scene, "hinge")!.Angles[0], 0.03);
    }

    [Fact]
    public void ASoftLimitGivesPastItsEndWhereAHardOneDoesNot()
    {
        double Lowest(double hertz)
        {
            var scene = Make(SceneOf(
                Body("bar", new Vector3(0.5, 3, 0), shape: Bar()),
                Joint(
                    "hinge",
                    new JointComponent(
                        limits: new Dictionary<JointAxis, JointRange> { [JointAxis.AboutX] = new(-30, 30) },
                        softHertz: hertz),
                    "bar",
                    new Vector3(-0.5, 0, 0),
                    new Vector3(0, -90, 0))));
            Run(scene, 3);
            return StateOf(scene, "hinge")!.Angles[0];
        }

        var hard = Lowest(0);
        var soft = Lowest(1);
        Assert.Equal(ToRadians(-30), hard, 0.02);
        Assert.True(soft < hard - 0.1);
    }

    [Fact]
    public void APathCarriesABallDownASlopeAndRoundACorner()
    {
        // The line is in the joint's own frame: a slope of one in two, then a flat. The ball starts at
        // the top of it.
        var scene = Make(SceneOf(
            Body("ball", new Vector3(0, 3, 0), shape: new BodyComponent(BodyShape.Sphere, radius: 0.1)),
            Joint(
                "rail",
                new JointComponent(JointKind.Path, path: [new Vector3(0, 0, 0), new Vector3(3, -1.5, 0), new Vector3(6, -1.5, 0)]),
                "ball")));
        Run(scene, 1.6);

        var ball = MiddleOf(scene, "ball");
        Assert.Equal(1.5, ball.Y, 0.03);
        Assert.True(ball.X > 4);
    }

    [Fact]
    public void APathWithALimitStopsTheBallAtALength()
    {
        var scene = Make(SceneOf(
            Body("ball", new Vector3(0, 3, 0), shape: new BodyComponent(BodyShape.Sphere, radius: 0.1)),
            Joint(
                "rail",
                new JointComponent(
                    JointKind.Path,
                    new Dictionary<JointAxis, JointRange> { [JointAxis.AlongX] = new(0, 2) },
                    path: [new Vector3(0, 0, 0), new Vector3(6, -3, 0)]),
                "ball")));
        Run(scene, 3);

        Assert.Equal(2, StateOf(scene, "rail")!.Offset[0], 0.05);
    }

    [Fact]
    public void APulleyWithItsPointsInTheJointFrameLiftsOneWeight()
    {
        // The light weight hangs under the heavy one in the tree, which is how a document says which two
        // a joint holds. Its joint entity sits at the heavy weight's middle, because that is where the
        // rope is tied.
        var scene = Make(SceneOf(
            Body("heavy", new Vector3(-1, 1, 0), shape: new BodyComponent(size: new Vector3(0.2, 0.2, 0.2), mass: 2)),
            Body("light", new Vector3(2, 0, 0), "heavy", shape: new BodyComponent(size: new Vector3(0.2, 0.2, 0.2))),
            Joint(
                "rope",
                new JointComponent(JointKind.Pulley, over: [new Vector3(0, 2, 0), new Vector3(2, 2, 0)]),
                "light",
                new Vector3(-2, 0, 0))));
        Run(scene, 0.5);

        var fall = 1 - MiddleOf(scene, "heavy").Y;
        Assert.Equal(0.41, fall, 0.06);
        Assert.Equal(fall, MiddleOf(scene, "light").Y - 1, 0.02);
    }

    [Fact]
    public void APulleyWithTheWrongNumberOfPointsIsNotMade()
    {
        var scene = Make(SceneOf(
            Body("heavy", new Vector3(-1, 1, 0)),
            Body("light", new Vector3(2, 0, 0), "heavy"),
            Joint("rope", new JointComponent(JointKind.Pulley, over: [new Vector3(0, 2, 0)]), "light")));

        var number = scene.JointOf("rope")!.Value;
        Assert.Null(scene.Physics.JointStateOf(number));
    }

    [Fact]
    public void AGearTiesTheTurnOfABodyToTheOneItHangsUnder()
    {
        // A carrier turns on the world and a wheel hangs under it, hinged to it at its own middle. The
        // gear keeps the wheel's own turn minus the carrier's at minus one times it, so the wheel turns
        // against the carrier twice as far as the carrier turns.
        var upright = new Vector3(0, 0, 90);
        var scene = Make(SceneOf(
            Body("carrier", new Vector3(0, 3, 0), shape: new BodyComponent(size: new Vector3(0.4, 0.2, 0.4))),
            Joint("axle", new JointComponent(speed: 60, strength: 1000), "carrier", rotation: upright),
            Body("wheel", new Vector3(1.5, 0, 0), "carrier", shape: new BodyComponent(size: new Vector3(0.4, 0.2, 0.4))),
            Joint("bearing", new JointComponent(), "wheel", rotation: upright),
            Joint("mesh", new JointComponent(JointKind.Gear, ratio: -1), "wheel", rotation: upright)));
        Run(scene, 0.5);

        var carrier = StateOf(scene, "axle")!.Angles[0];
        var against = StateOf(scene, "bearing")!.Angles[0];
        Assert.True(carrier > 0.3);
        Assert.Equal(-2 * carrier, against, 0.05);
    }

    [Fact]
    public void RebuildingABodyMakesItsJointAgainWithTheSameNumber()
    {
        var scene = Make(Pendulum());
        var number = scene.JointOf("pivot");
        Run(scene, 0.5);

        // Dragged a metre further out, it swings about a pivot a metre further out too: the pivot is
        // under it.
        scene.Apply(new SceneDiff([
            new SetComponent(
                "ball",
                SceneComponents.Transform,
                scene.Document["ball"]![SceneComponents.Transform]!.ToJson(),
                new TransformComponent(new Vector3(2, 3, 0)).ToJson()),
        ]));
        Assert.Equal(number, scene.JointOf("pivot"));
        Assert.NotNull(StateOf(scene, "pivot"));

        Run(scene, 1);
        Assert.Equal(1, MiddleOf(scene, "ball").DistanceTo(new Vector3(1, 3, 0)), 0.01);
    }

    [Fact]
    public void TakingTheComponentAwayLetsGoAndGivesTheNumberUp()
    {
        var scene = Make(Pendulum());
        var number = scene.JointOf("pivot")!.Value;
        var had = scene.Document["pivot"]![SceneComponents.Joint]!;

        scene.Apply(new SceneDiff([new SetComponent("pivot", SceneComponents.Joint, had.ToJson(), null)]));
        Assert.Null(scene.JointOf("pivot"));
        Assert.Null(scene.EntityOfJoint(number));
        Assert.Null(scene.Physics.JointStateOf(number));

        Run(scene, 1);
        Assert.Equal(1, MiddleOf(scene, "ball").X, 0.01);
    }

    [Fact]
    public void AnAddedJointHolds()
    {
        var scene = Make(SceneOf(Body("crate", new Vector3(0, 3, 0))));
        scene.Apply(new SceneDiff([
            new AddEntity(Joint("weld", new JointComponent(JointKind.Fixed), "crate").ToJson()),
        ]));
        Run(scene, 1);

        Assert.Equal(3, MiddleOf(scene, "crate").Y, 0.01);
    }

    [Fact]
    public void AJointThatBreaksLetsGoSaysSoAndStaysBrokenUntilItsOwnEntityIsEdited()
    {
        // A one-kilogram crate weighs ten newtons, more than the weld takes.
        var weak = new JointComponent(JointKind.Fixed, breakingForce: 5);
        var scene = Make(SceneOf(Body("crate", new Vector3(0, 3, 0)), Joint("weld", weak, "crate")));
        var number = scene.JointOf("weld")!.Value;

        scene.Advance(1.0 / 60);
        var broke = scene.Events.Where(e => e.Kind == PhysicsEventKind.Broke);
        Assert.Equal([number], broke.Select(e => e.A));
        Assert.Equal("weld", scene.EntityOfJoint(number));
        Assert.Equal(["weld"], scene.Broken);
        Assert.Null(scene.Physics.JointStateOf(number));

        // Dragging the crate is not mending the weld.
        scene.Apply(new SceneDiff([
            new SetComponent(
                "crate",
                SceneComponents.Transform,
                scene.Document["crate"]![SceneComponents.Transform]!.ToJson(),
                new TransformComponent(new Vector3(0, 6, 0)).ToJson()),
        ]));
        Assert.Equal(["weld"], scene.Broken);
        Run(scene, 0.5);
        Assert.True(MiddleOf(scene, "crate").Y < 5.5);

        // Editing the weld is, and it holds where the crate now is.
        scene.Apply(new SceneDiff([new SetField("weld", SceneComponents.Joint, "breakingForce", 5, 0)]));
        Assert.Empty(scene.Broken);
        Assert.Equal(number, scene.JointOf("weld"));
        var mended = MiddleOf(scene, "crate").Y;
        Run(scene, 1);
        Assert.Equal(mended, MiddleOf(scene, "crate").Y, 0.01);
    }
}
