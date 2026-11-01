using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class ScenePhysicsTests : SceneTestBase
{
    [Fact]
    public void FallsLandsAndSaysWhereItLanded()
    {
        var scene = Make(SceneOf(Floor(), Crate()));
        Run(scene, 3);

        var rested = TransformOf(scene.Document, "crate");
        Assert.Equal(0.5, rested.Position.Y, Resting);
        Assert.Equal(0, rested.Position.X, 0.01);
        Assert.Equal(0, rested.Rotation.X, 0.5);
        Assert.Equal(0, rested.Rotation.Z, 0.5);
        Assert.Equal(Vector3.All(1), rested.Scale);
    }

    [Fact]
    public void TheDiffsAreTheSimulationAppliedInTurnTheyReachTheDocumentItEndedWith()
    {
        var start = SceneOf(Floor(), Crate());
        var scene = Make(start);

        var followed = start;
        foreach (var diff in Run(scene, 2))
        {
            followed = diff.ApplyTo(followed);
        }

        Assert.Equal(scene.Document.Encode(), followed.Encode());
        Assert.NotEqual(start.Encode(), followed.Encode());
    }

    [Fact]
    public void EachDiffUndoes()
    {
        var scene = Make(SceneOf(Floor(), Crate()));
        Run(scene, 0.5);
        var before = scene.Document;
        var diff = scene.Advance(1.0 / 60);

        Assert.False(diff.IsEmpty);
        Assert.Equal(before.Encode(), diff.Inverse.ApplyTo(scene.Document).Encode());
    }

    [Fact]
    public void NothingThatCannotMoveIsEverWritten()
    {
        var scene = Make(SceneOf(
            Floor(),
            Crate("shelf", at: new Vector3(0, 2, 0), rotation: new Vector3(10, 37, 5), body: new BodyComponent(motion: BodyMotion.Fixed))));

        Assert.All(Run(scene, 1), diff => Assert.True(diff.IsEmpty));
    }

    [Fact]
    public void LessThanAStepOwedIsNoStepAndNoDiff()
    {
        var scene = Make(SceneOf(Floor(), Crate()));

        Assert.True(scene.Advance(0.001).IsEmpty);
        Assert.Equal(3, TransformOf(scene.Document, "crate").Position.Y);
    }

    [Fact]
    public void ALateFrameCatchesUpOnlySoFar()
    {
        var scene = Make(SceneOf(Floor(), Crate()), maxSteps: 2);
        scene.Advance(10);

        // Two sixtieths of falling from rest is a few millimetres, not a crate that has already landed.
        var y = TransformOf(scene.Document, "crate").Position.Y;
        Assert.True(y is < 3 and > 2.9, $"y was {y}");
    }

    private static List<(PhysicsEventKind Kind, string? A, string? B)> Heard(ScenePhysics scene, int frames, double seconds)
    {
        var heard = new List<(PhysicsEventKind, string?, string?)>();
        for (var i = 0; i < frames; i++)
        {
            scene.Advance(seconds);
            heard.AddRange(scene.Events.Select(e => (e.Kind, scene.EntityOf(e.A), scene.EntityOf(e.B))));
        }

        return heard;
    }

    [Fact]
    public void ASlowFrameHearsEveryStepItTookNotOnlyItsLast()
    {
        // A sixty-fourth, so three of them add up exactly and both runs take the same steps.
        const double step = 1.0 / 64;
        var everyStep = Heard(Make(SceneOf(Floor(), Crate(at: new Vector3(0, 1, 0))), step), 192, step);
        var everyThird = Heard(Make(SceneOf(Floor(), Crate(at: new Vector3(0, 1, 0))), step), 64, 3 * step);

        Assert.Contains((PhysicsEventKind.TouchBegan, "floor", "crate"), everyStep);
        Assert.Equal(everyStep, everyThird);
    }

    [Fact]
    public void AFrameThatTookNoStepHearsNothing()
    {
        var scene = Make(SceneOf(Floor(), Crate(at: new Vector3(0, 1, 0))));
        for (var i = 0; i < 600 && scene.Events.Count == 0; i++)
        {
            scene.Advance(1.0 / 60);
        }

        Assert.NotEmpty(scene.Events);

        scene.Advance(0.001);
        Assert.Empty(scene.Events);
    }

    [Fact]
    public void ACrateScaledToTwoIsABodyTwiceTheSize()
    {
        var scene = Make(SceneOf(Floor(), Crate(scale: Vector3.All(2))));
        Run(scene, 3);

        var rested = TransformOf(scene.Document, "crate");
        Assert.Equal(1, rested.Position.Y, Resting);
        Assert.Equal(Vector3.All(2), rested.Scale);
    }

    [Fact]
    public void ABallStretchedUprightGrowsRatherThanStretches()
    {
        var scene = Make(SceneOf(Floor(), Crate(scale: new Vector3(1, 3, 1), body: new BodyComponent(BodyShape.Sphere))));
        Run(scene, 3);

        Assert.Equal(1.5, TransformOf(scene.Document, "crate").Position.Y, Resting);
    }

    [Fact]
    public void ACapsuleRoundTheWaistLeavesItsEntityStandingOnTheFloor()
    {
        var scene = Make(SceneOf(
            Floor(),
            Crate(
                "hero",
                at: new Vector3(0, 2, 0),
                body: new BodyComponent(BodyShape.Capsule, radius: 0.5, height: 2, centre: new Vector3(0, 1, 0), angularDamping: 1))));
        Run(scene, 3);

        Assert.Equal(0, TransformOf(scene.Document, "hero").Position.Y, Resting);
    }

    [Fact]
    public void ABodyOffItsCentreAndTurnedFallsStraightDownWithIt()
    {
        // The box's middle is a metre along the entity's own x, and the entity is turned, so the box
        // starts a metre off to one side. Whichever side that is, reading the entity back from the box
        // has to undo the same turn that placed the box, or the entity jumps sideways.
        var scene = Make(SceneOf(
            Floor(),
            Crate(at: new Vector3(0, 2, 0), rotation: new Vector3(0, 90, 0), body: new BodyComponent(centre: new Vector3(1, 0, 0)))));
        Run(scene, 3);

        var rested = TransformOf(scene.Document, "crate").Position;
        Assert.Equal(0, rested.X, Resting);
        Assert.Equal(0.5, rested.Y, Resting);
        Assert.Equal(0, rested.Z, Resting);
    }

    [Fact]
    public void ARaisedPlaneIsRaisedGround()
    {
        var scene = Make(SceneOf(Floor(1), Crate()));
        Run(scene, 3);

        Assert.Equal(1.5, TransformOf(scene.Document, "crate").Position.Y, Resting);
    }

    [Fact]
    public void ACrateTurnedAboutItsUprightStaysTurned()
    {
        var scene = Make(SceneOf(Floor(), Crate(rotation: new Vector3(0, 30, 0))));
        Run(scene, 3);

        var rested = TransformOf(scene.Document, "crate");
        Assert.Equal(30, rested.Rotation.Y, 0.5);
        Assert.Equal(0, rested.Rotation.X, 0.5);
        Assert.Equal(0, rested.Rotation.Z, 0.5);
    }

    [Fact]
    public void ACrateTurnedAQuarterKeepsTheAnglesItWasGiven()
    {
        var scene = Make(SceneOf(Floor(), Crate(rotation: new Vector3(0, 90, 0))));
        Run(scene, 3);

        var rested = TransformOf(scene.Document, "crate");
        Assert.Equal(0, rested.Rotation.X, 0.5);
        Assert.Equal(90, rested.Rotation.Y, 0.5);
        Assert.Equal(0, rested.Rotation.Z, 0.5);
    }

    private static SceneEntity Cart() => new(
        "cart",
        "Cart",
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(10, 0, 0), new Vector3(0, 90, 0)),
        });

    [Fact]
    public void ABodyUnderAParentIsWrittenRelativeToIt()
    {
        var scene = Make(SceneOf(Floor(), Cart(), Crate(parent: "cart", at: new Vector3(0, 3, 0))));
        Run(scene, 3);

        var rested = TransformOf(scene.Document, "crate");
        Assert.Equal(0, rested.Position.X, 0.01);
        Assert.Equal(0.5, rested.Position.Y, Resting);
        Assert.Equal(0, rested.Position.Z, 0.01);
        Assert.Equal(0, rested.Rotation.Y, 0.5);
    }

    [Fact]
    public void ABodyUnderAFallingBodyIsCarriedAndRewritten()
    {
        // The rider is fixed, so the world never moves it; but what it is relative to falls, and to
        // stay where the world has it its transform has to change.
        var scene = Make(SceneOf(
            Floor(),
            Crate(),
            Crate("rider", parent: "crate", at: new Vector3(0, 5, 0), body: new BodyComponent(motion: BodyMotion.Fixed))));
        Run(scene, 3);

        var rider = TransformOf(scene.Document, "rider");
        var crateAt = TransformOf(scene.Document, "crate").Position;
        Assert.Equal(8, rider.Position.Y + crateAt.Y, Resting);
    }

    [Fact]
    public void MovingAnEntityMovesItsBodyAndKeepsItsNumber()
    {
        var scene = Make(SceneOf(Floor(), Crate()));
        var number = scene.BodyOf("crate");
        Run(scene, 3);

        var lifted = new TransformComponent(new Vector3(4, 6, 0));
        scene.Apply(new SceneDiff([
            new SetComponent("crate", SceneComponents.Transform, TransformOf(scene.Document, "crate").ToJson(), lifted.ToJson()),
        ]));

        Assert.Equal(number, scene.BodyOf("crate"));
        Assert.Equal(6, scene.Physics.TransformOf(number!.Value)![1], 1e-6);

        Run(scene, 3);
        var rested = TransformOf(scene.Document, "crate");
        Assert.Equal(4, rested.Position.X, 0.01);
        Assert.Equal(0.5, rested.Position.Y, Resting);
    }

    [Fact]
    public void ABodyTakenAwayLeavesTheWorldAndNeverComesBackAsTheSameNumber()
    {
        var scene = Make(SceneOf(Floor(), Crate()));
        var number = scene.BodyOf("crate")!.Value;
        var body = scene.Document["crate"]![SceneComponents.Body]!;

        scene.Apply(new SceneDiff([new SetComponent("crate", SceneComponents.Body, body.ToJson(), null)]));
        Assert.Null(scene.BodyOf("crate"));
        Assert.Null(scene.EntityOf(number));
        Assert.Null(scene.Physics.TransformOf(number));
        Assert.All(Run(scene, 1), diff => Assert.True(diff.IsEmpty));

        scene.Apply(new SceneDiff([new SetComponent("crate", SceneComponents.Body, null, body.ToJson())]));
        Assert.NotEqual(number, scene.BodyOf("crate"));
        Assert.Equal("crate", scene.EntityOf(scene.BodyOf("crate")!.Value));
    }

    [Fact]
    public void ARemovedEntityTakesItsBodyWithIt()
    {
        var scene = Make(SceneOf(Floor(), Crate()));
        var number = scene.BodyOf("crate")!.Value;
        var entity = scene.Document["crate"]!;

        scene.Apply(new SceneDiff([new RemoveEntity(entity.ToJson())]));

        Assert.Null(scene.BodyOf("crate"));
        Assert.Null(scene.Physics.TransformOf(number));
    }

    [Fact]
    public void AnAddedEntityWithABodyIsSimulated()
    {
        var scene = Make(SceneOf(Floor()));
        scene.Apply(new SceneDiff([new AddEntity(Crate().ToJson())]));

        Assert.NotNull(scene.BodyOf("crate"));
        Run(scene, 3);
        Assert.Equal(0.5, TransformOf(scene.Document, "crate").Position.Y, Resting);
    }

    [Fact]
    public void MovingAParentMovesTheBodiesUnderIt()
    {
        var folder = new SceneEntity(
            "folder",
            "Folder",
            components: new Dictionary<string, SceneComponent> { [SceneComponents.Transform] = new TransformComponent() });
        var scene = Make(SceneOf(Floor(), folder, Crate(parent: "folder")));
        var number = scene.BodyOf("crate")!.Value;

        scene.Apply(new SceneDiff([
            new SetField(
                "folder",
                SceneComponents.Transform,
                "position",
                Values.VectorToJson(Vector3.Zero()),
                Values.VectorToJson(new Vector3(7, 0, 0))),
        ]));

        Assert.Equal(7, scene.Physics.TransformOf(number)![0], 1e-6);
    }
}
