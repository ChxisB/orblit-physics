using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneZonesTests : SceneTestBase
{
    private static SceneEntity Placed(string id, Vector3 at, BodyComponent body, ZoneComponent? zone = null)
    {
        var components = new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(at),
            [SceneComponents.Body] = body,
        };
        if (zone is not null)
        {
            components[SceneComponents.Zone] = zone;
        }

        return new SceneEntity(id, id, components: components);
    }

    private static SceneEntity Crate(Vector3 at, BodyComponent? body = null) => Placed("crate", at, body ?? new BodyComponent());

    /// <summary>A fixed box <paramref name="size"/> across: a place, or a wall when it is not a trigger.</summary>
    private static BodyComponent Place(Vector3 size, bool trigger = false, bool stay = false) =>
        new(size: size, motion: BodyMotion.Fixed, trigger: trigger, stay: stay);

    private static Vector3 MiddleOf(ScenePhysics scene, string entity)
    {
        var pose = scene.Physics.TransformOf(scene.BodyOf(entity)!.Value)!;
        return new Vector3(pose[0], pose[1], pose[2]);
    }

    /// <summary>Runs <paramref name="seconds"/> and answers with everything that was heard.</summary>
    private static List<PhysicsEvent> Listen(ScenePhysics scene, double seconds)
    {
        var seen = new List<PhysicsEvent>();
        for (var i = 0; i < (int)Math.Round(seconds * 60, MidpointRounding.AwayFromZero); i++)
        {
            scene.Advance(1.0 / 60);
            seen.AddRange(scene.Events);
        }

        return seen;
    }

    private static List<PhysicsEvent> Only(List<PhysicsEvent> seen, PhysicsEventKind kind) =>
        [.. seen.Where(e => e.Kind == kind)];

    [Fact]
    public void ATriggerInADocumentLetsABodyFallThroughAndSaysWhenItCameAndWent()
    {
        var scene = Make(SceneOf(
            Floor(),
            Placed("gate", new Vector3(0, 2, 0), Place(new Vector3(3, 1, 3), trigger: true)),
            Crate(new Vector3(0, 5, 0))));
        var seen = Listen(scene, 2);

        var entered = Assert.Single(Only(seen, PhysicsEventKind.Entered));
        Assert.Equal("gate", scene.EntityOf(entered.A));
        Assert.Equal("crate", scene.EntityOf(entered.B));
        Assert.Single(Only(seen, PhysicsEventKind.Exited));
        Assert.Single(Only(seen, PhysicsEventKind.TouchBegan));
        Assert.Equal(0.5, MiddleOf(scene, "crate").Y, 0.05);
    }

    [Fact]
    public void ATriggerInADocumentIsAWallWhenItIsNotOne()
    {
        var scene = Make(SceneOf(
            Floor(),
            Placed("shelf", new Vector3(0, 2, 0), Place(new Vector3(3, 1, 3))),
            Crate(new Vector3(0, 5, 0))));
        Run(scene, 2);

        Assert.Equal(3, MiddleOf(scene, "crate").Y, 0.05);
    }

    [Fact]
    public void ATriggerInADocumentIsIgnoredForABodyThatFalls()
    {
        var scene = Make(SceneOf(Floor(), Crate(new Vector3(0, 3, 0), new BodyComponent(trigger: true))));
        Run(scene, 2);

        Assert.Equal(0.5, MiddleOf(scene, "crate").Y, 0.05);
    }

    [Fact]
    public void HearsABodyInsideItEveryStepWhenItAsksTo()
    {
        var scene = Make(SceneOf(
            Placed("pool", new Vector3(0, 1, 0), Place(new Vector3(4, 2, 4), trigger: true, stay: true)),
            Crate(new Vector3(0, 1, 0), new BodyComponent(motion: BodyMotion.Driven))));
        var seen = Listen(scene, 0.5);

        Assert.True(Only(seen, PhysicsEventKind.Inside).Count > 20);
    }

    [Fact]
    public void SaysNothingEveryStepWhenItDoesNotAsk()
    {
        var scene = Make(SceneOf(
            Placed("pool", new Vector3(0, 1, 0), Place(new Vector3(4, 2, 4), trigger: true)),
            Crate(new Vector3(0, 1, 0), new BodyComponent(motion: BodyMotion.Driven))));
        var seen = Listen(scene, 0.5);

        Assert.Empty(Only(seen, PhysicsEventKind.Inside));
        Assert.Single(Only(seen, PhysicsEventKind.Entered));
    }

    [Fact]
    public void ABodyThatAsksHearsItsContactEveryStepItIsAwake()
    {
        var scene = Make(SceneOf(Floor(), Crate(new Vector3(0, 0.5, 0), new BodyComponent(stay: true))));
        var seen = Listen(scene, 0.25);

        Assert.True(Only(seen, PhysicsEventKind.TouchStay).Count > 10);
    }

    [Fact]
    public void ABodyThatDoesNotAskHearsNoStay()
    {
        var scene = Make(SceneOf(Floor(), Crate(new Vector3(0, 0.5, 0))));
        var seen = Listen(scene, 0.25);

        Assert.NotEmpty(Only(seen, PhysicsEventKind.TouchBegan));
        Assert.Empty(Only(seen, PhysicsEventKind.TouchStay));
    }

    private static SceneDocument Shaft(ZoneComponent? zone = null) => SceneOf(
        Placed("shaft", new Vector3(0, 3, 0), Place(new Vector3(4, 4, 4)), zone),
        Crate(new Vector3(0, 2, 0)));

    [Fact]
    public void AZoneMakesTheBodyUnderItATriggerWithoutBeingAsked()
    {
        var scene = Make(Shaft(new ZoneComponent(Vector3.Zero())));
        var seen = Listen(scene, 0.5);

        Assert.Single(Only(seen, PhysicsEventKind.Entered));
        Assert.Empty(Only(seen, PhysicsEventKind.TouchBegan));
    }

    [Fact]
    public void AZoneHoldsABodyWhereGravityIsNought()
    {
        var scene = Make(Shaft(new ZoneComponent(Vector3.Zero())));
        Run(scene, 1);

        Assert.Equal(2, MiddleOf(scene, "crate").Y, 0.01);
    }

    [Fact]
    public void AZoneLiftsABodyWhereGravityPointsUp()
    {
        var scene = Make(Shaft(new ZoneComponent(new Vector3(0, 20, 0))));
        Run(scene, 0.5);

        Assert.True(MiddleOf(scene, "crate").Y > 3);
    }

    [Fact]
    public void AZoneDoesNothingToABodyWhenItIsNotThere()
    {
        var scene = Make(Shaft());
        Run(scene, 0.5);

        Assert.True(MiddleOf(scene, "crate").Y < 1.5);
    }

    [Fact]
    public void AZoneLetsGoWhenTheZoneIsTakenOff()
    {
        var held = Shaft(new ZoneComponent(Vector3.Zero()));
        var scene = Make(held);
        Run(scene, 1);
        scene.Apply(SceneDiff.Between(held, Shaft()));
        Run(scene, 0.5);

        Assert.True(MiddleOf(scene, "crate").Y < 1.5);
    }

    [Fact]
    public void AZoneFollowsAnEditToTheZone()
    {
        var held = Shaft(new ZoneComponent(Vector3.Zero()));
        var scene = Make(held);
        Run(scene, 1);
        scene.Apply(SceneDiff.Between(held, Shaft(new ZoneComponent(new Vector3(0, 20, 0)))));
        Run(scene, 0.5);

        Assert.True(MiddleOf(scene, "crate").Y > 3);
    }

    [Fact]
    public void AZoneSlowsABodyWithDampingAlone()
    {
        var scene = Make(SceneOf(
            Placed("water", new Vector3(0, 3, 0), Place(new Vector3(4, 4, 4)), new ZoneComponent(linearDamping: 8)),
            Crate(new Vector3(0, 4, 0))));
        Run(scene, 0.5);
        var wet = MiddleOf(scene, "crate").Y;

        var dry = Make(SceneOf(Crate(new Vector3(0, 4, 0))));
        Run(dry, 0.5);

        Assert.True(wet > MiddleOf(dry, "crate").Y);
    }

    [Fact]
    public void AZoneWhereTwoOverlapTheHigherPriorityWins()
    {
        var scene = Make(SceneOf(
            Placed("low", new Vector3(0, 3, 0), Place(new Vector3(4, 4, 4)), new ZoneComponent(new Vector3(0, 20, 0))),
            Placed("high", new Vector3(0, 3, 0), Place(new Vector3(4, 4, 4)), new ZoneComponent(Vector3.Zero(), priority: 1)),
            Crate(new Vector3(0, 2, 0))));
        Run(scene, 1);

        Assert.Equal(2, MiddleOf(scene, "crate").Y, 0.01);
    }

    private static SceneDocument Belt(Vector3 surface) => SceneOf(
        Placed(
            "belt",
            new Vector3(0, -0.5, 0),
            Place(new Vector3(20, 1, 3)).CopyWith(surface: surface)),
        Crate(new Vector3(0, 0.6, 0)));

    [Fact]
    public void ABeltInADocumentCarriesWhatStandsOnIt()
    {
        var scene = Make(Belt(new Vector3(2, 0, 0)));
        Run(scene, 2);

        Assert.True(MiddleOf(scene, "crate").X > 1.5);
    }

    [Fact]
    public void ABeltInADocumentCarriesItTheOtherWayWhenTheSurfaceGoesTheOtherWay()
    {
        var scene = Make(Belt(new Vector3(-2, 0, 0)));
        Run(scene, 2);

        Assert.True(MiddleOf(scene, "crate").X < -1.5);
    }

    [Fact]
    public void ABeltInADocumentLeavesItWhereItIsWhenTheSurfaceIsStill()
    {
        var scene = Make(Belt(Vector3.Zero()));
        Run(scene, 2);

        Assert.Equal(0, MiddleOf(scene, "crate").X, 0.01);
    }
}
