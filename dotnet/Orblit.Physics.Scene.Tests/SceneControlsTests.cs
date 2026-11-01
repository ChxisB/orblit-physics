using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneControlsTests : SceneTestBase
{
    private static Vector3 MiddleOf(ScenePhysics scene, string entity)
    {
        var pose = scene.Physics.TransformOf(scene.BodyOf(entity)!.Value)!;
        return new Vector3(pose[0], pose[1], pose[2]);
    }

    private static double SpeedOf(ScenePhysics scene, string entity)
    {
        var v = scene.Physics.VelocityOf(scene.BodyOf(entity)!.Value)!;
        return Math.Sqrt(((double)v[0] * v[0]) + ((double)v[1] * v[1]) + ((double)v[2] * v[2]));
    }

    private static SceneEntity CrateAt(string id, double height) => Entity(id, new BodyComponent(), new Vector3(0, height, 0));

    [Fact]
    public void LockedToAPlaneAtHalfGravityUnderACapBehavesSo()
    {
        var scene = Make(SceneOf(Entity(
            "crate",
            new BodyComponent(locks: new HashSet<BodyLock> { BodyLock.MoveZ }, gravityScale: 0.5, maxSpeed: 3),
            new Vector3(0, 20, 0))));
        scene.Physics.Push(scene.BodyOf("crate")!.Value, [2, 0, 2]);
        Run(scene, 1);

        var at = MiddleOf(scene, "crate");
        Assert.Equal(0, at.Z, 1e-4);
        Assert.True(at.X > 0.5);
        Assert.True(SpeedOf(scene, "crate") <= 3.001);

        // Half gravity alone would fall 2.45 m in a second. The cap holds it back further.
        Assert.True(20 - at.Y is > 0.8 and < 2.4, $"fell {20 - at.Y}");
    }

    [Fact]
    public void HalfGravityFallsHalfAsFar()
    {
        var scene = Make(SceneOf(
            Entity("plain", new BodyComponent(), new Vector3(0, 50, 0)),
            Entity("light", new BodyComponent(gravityScale: 0.5), new Vector3(5, 50, 0))));
        Run(scene, 0.5);

        var plain = 50 - MiddleOf(scene, "plain").Y;
        var light = 50 - MiddleOf(scene, "light").Y;
        Assert.Equal(0.5, light / plain, 0.03);
    }

    [Fact]
    public void ALockedTurnHoldsTheBodyUpright()
    {
        var scene = Make(SceneOf(
            Floor(),
            Entity(
                "post",
                new BodyComponent(locks: new HashSet<BodyLock> { BodyLock.TurnX, BodyLock.TurnY, BodyLock.TurnZ }),
                new Vector3(0, 0.5, 0))));
        scene.Physics.Push(scene.BodyOf("post")!.Value, [1, 0, 0], [0, 1, 0]);
        Run(scene, 1);

        var v = scene.Physics.VelocityOf(scene.BodyOf("post")!.Value)!;
        Assert.True(Math.Abs(v[3]) + Math.Abs(v[4]) + Math.Abs(v[5]) < 1e-4);
    }

    [Fact]
    public void AddedAsleepIsAsleepStillWithItsControls()
    {
        var scene = Make(SceneOf(Entity(
            "crate",
            new BodyComponent(startsAsleep: true, locks: new HashSet<BodyLock> { BodyLock.MoveX }),
            new Vector3(0, 5, 0))));
        Run(scene, 0.5);

        Assert.True(scene.Physics.Asleep(scene.BodyOf("crate")!.Value));
        Assert.Equal(5, MiddleOf(scene, "crate").Y, 1e-4);
    }

    [Fact]
    public void ACapOrInertiaTheWorldWouldRefuseDoesNotCostTheLocks()
    {
        var scene = Make(SceneOf(Entity(
            "crate",
            new BodyComponent(
                locks: new HashSet<BodyLock> { BodyLock.MoveZ },
                maxSpeed: -5,
                maxSpin: -1,
                inertia: new Vector3(-1, 2, 3)),
            new Vector3(0, 20, 0))));
        scene.Physics.Push(scene.BodyOf("crate")!.Value, [0, 0, 4]);
        Run(scene, 0.5);

        Assert.Equal(0, MiddleOf(scene, "crate").Z, 1e-4);
    }

    [Fact]
    public void AnEditSendsTheNewControlsWithTheRebuiltBody()
    {
        var scene = Make(SceneOf(CrateAt("crate", 20)));
        var locked = SceneOf(Entity(
            "crate",
            new BodyComponent(locks: new HashSet<BodyLock> { BodyLock.MoveX }),
            new Vector3(0, 20, 0)));
        scene.Apply(SceneDiff.Between(scene.Document, locked));
        scene.Physics.Push(scene.BodyOf("crate")!.Value, [4, 0, 0]);
        Run(scene, 0.5);

        Assert.Equal(0, MiddleOf(scene, "crate").X, 1e-4);
    }

    /// <summary>
    /// Twelve throws at 2.9 m/s, each from a different part of a step away, at a wall 1 cm thick with a
    /// ball 2 cm across. Returns how many got through.
    /// </summary>
    private int Crossings(bool continuous)
    {
        var crossed = 0;
        for (var i = 0; i < 12; i++)
        {
            var scene = Make(
                SceneOf(
                    Entity("wall", new BodyComponent(motion: BodyMotion.Fixed, size: new Vector3(0.01, 4, 4))),
                    Entity(
                        "ball",
                        new BodyComponent(BodyShape.Sphere, radius: 0.01, continuous: continuous),
                        new Vector3(0.3 + (0.004 * i), 0, 0))),
                settings: new PhysicsSettings { Gravity = [0, 0, 0] });
            scene.Physics.Drive(scene.BodyOf("ball")!.Value, [-2.9, 0, 0]);
            Run(scene, 40.0 / 60);
            if (MiddleOf(scene, "ball").X < -0.02)
            {
                crossed++;
            }
        }

        return crossed;
    }

    [Fact]
    public void ASmallBodyThrownAtAThinWallGoesThroughFromSomePartOfAStep() => Assert.True(Crossings(false) > 0);

    [Fact]
    public void ASmallBodyThrownAtAThinWallIsStoppedByItFromEveryPartOfAStepWhenContinuous() =>
        Assert.Equal(0, Crossings(true));

    /// <summary>
    /// The spin about z one step after a shove along x through the middle of the shape, for a crate on an
    /// entity <paramref name="size"/> across. Its weight sits a quarter of the crate's own height below
    /// the middle.
    /// </summary>
    private double SpinAfterShove(double size, Vector3? inertia = null)
    {
        var made = Make(SceneOf(Entity(
            "crate",
            new BodyComponent(centreOfMass: new Vector3(0, -0.25, 0), inertia: inertia),
            new Vector3(0, 10, 0),
            Vector3.All(size))));
        made.Physics.Push(made.BodyOf("crate")!.Value, [1, 0, 0], [0, 10, 0]);
        made.Advance(made.Step);
        return Math.Abs(made.Physics.VelocityOf(made.BodyOf("crate")!.Value)![5]);
    }

    [Fact]
    public void ABodyWithItsWeightSomewhereElseIsWhereTheDocumentSaysHoweverItsWeightSits()
    {
        var scene = Make(SceneOf(Entity(
            "crate",
            new BodyComponent(centreOfMass: new Vector3(0, -0.3, 0)),
            new Vector3(1, 5, 2))));

        Assert.Equal(new Vector3(1, 5, 2), MiddleOf(scene, "crate"));
    }

    [Fact]
    public void GrowsItsCentreOfMassWithTheEntity()
    {
        // A shove at the middle lands 0.25 m above the weight on a one-metre crate:
        // 0.25 / (1 / 6 + 0.25 ^ 2). On a two-metre one it lands 0.5 m above:
        // 0.5 / (4 / 6 + 0.5 ^ 2). Left unscaled, the second would be 0.25 / (4 / 6 + 0.25 ^ 2) = 0.343.
        Assert.Equal(1.0909, SpinAfterShove(1), 0.02);
        Assert.Equal(0.5455, SpinAfterShove(2), 0.02);
    }

    [Fact]
    public void TurnsByTheInertiaTheDocumentGivesAllThreeParts() =>
        Assert.Equal(0.0025, SpinAfterShove(1, new Vector3(100, 100, 100)), 0.0005);

    [Fact]
    public void KeepsTheShapesInertiaWhenAPartOfTheGivenOneIsZero() =>
        Assert.Equal(1.0909, SpinAfterShove(1, new Vector3(100, 0, 100)), 0.02);

    private ScenePhysics StackOfTwo(double high = 3) =>
        Make(SceneOf(Floor(), CrateAt("low", 0.5), CrateAt("high", high)));

    [Fact]
    public void TwoEntitiesToldToPassThroughEachOtherDo()
    {
        var scene = StackOfTwo();
        Assert.True(scene.Ignore("low", "high"));
        Run(scene, 2);

        Assert.Equal(0.5, MiddleOf(scene, "high").Y, 0.05);
    }

    [Fact]
    public void TwoEntitiesToldToPassThroughEachOtherStackOnceTheyAreLetGo()
    {
        var scene = StackOfTwo();
        scene.Ignore("low", "high");

        Assert.True(scene.Unignore("high", "low"));
        Run(scene, 2);

        Assert.Equal(1.5, MiddleOf(scene, "high").Y, 0.05);
    }

    [Fact]
    public void TwoEntitiesToldToPassThroughEachOtherAreOnePairWhicheverWayRoundTheyAreNamed()
    {
        var scene = StackOfTwo();
        scene.Ignore("high", "low");

        Assert.True(scene.Unignore("low", "high"));
        Assert.False(scene.Unignore("low", "high"));
    }

    [Fact]
    public void AreRefusedForABodyThatIsNotThereOrOneAgainstItself()
    {
        var scene = StackOfTwo();

        Assert.False(scene.Ignore("low", "nobody"));
        Assert.False(scene.Ignore("low", "low"));
        Assert.False(scene.Unignore("low", "high"));
    }

    [Fact]
    public void TwoEntitiesToldToPassThroughEachOtherStillDoAfterAnEditRebuildsOneOfThem()
    {
        var scene = StackOfTwo();
        scene.Ignore("low", "high");
        var raised = SceneOf(Floor(), CrateAt("low", 0.5), CrateAt("high", 6));
        scene.Apply(SceneDiff.Between(scene.Document, raised));
        Run(scene, 2);

        Assert.Equal(0.5, MiddleOf(scene, "high").Y, 0.05);
    }

    [Fact]
    public void TwoEntitiesToldToPassThroughEachOtherAreForgottenWithAnEntityThatGoes()
    {
        var scene = StackOfTwo();
        scene.Ignore("low", "high");
        var alone = SceneOf(Floor(), CrateAt("low", 0.5));
        scene.Apply(SceneDiff.Between(scene.Document, alone));
        var again = SceneOf(Floor(), CrateAt("low", 0.5), CrateAt("high", 3));
        scene.Apply(SceneDiff.Between(scene.Document, again));

        Assert.False(scene.Unignore("low", "high"));
        Run(scene, 2);
        Assert.Equal(1.5, MiddleOf(scene, "high").Y, 0.05);
    }
}
