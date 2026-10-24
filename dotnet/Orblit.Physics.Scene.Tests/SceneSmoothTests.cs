using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneSmoothTests : SceneTestBase
{
    /// <summary>A 144 Hz render over 60 Hz physics.</summary>
    private const double Frame = 1.0 / 144;

    private const double Step = 1.0 / 60;

    private static readonly SceneDocument Falling = SceneOf(Floor(), Crate("crate", at: new Vector3(0, 60, 0)));

    private static readonly SceneDocument Pair = SceneOf(
        Floor(),
        Crate("a", at: new Vector3(0, 60, 0)),
        Crate("b", at: new Vector3(5, 60, 0)));

    private static double HeightOf(ScenePhysics scene, string id) => TransformOf(scene.Document, id).Position.Y;

    private static double SidewaysOf(ScenePhysics scene, string id) => TransformOf(scene.Document, id).Position.X;

    /// <summary>
    /// Where the world has the entity's body, which is not where the document shows it when the scene
    /// smooths.
    /// </summary>
    private static double TruthOf(ScenePhysics scene, string id) => scene.Physics.TransformOf(scene.BodyOf(id)!.Value)![1];

    private static SceneDiff Teleport(ScenePhysics scene, string id, Vector3 to) => new([
        new SetComponent(
            id,
            SceneComponents.Transform,
            TransformOf(scene.Document, id).ToJson(),
            new TransformComponent(to).ToJson()),
    ]);

    private static void Frames(ScenePhysics scene, int count)
    {
        for (var i = 0; i < count; i++)
        {
            scene.Advance(Frame);
        }
    }

    /// <summary>The height each whole step leaves a crate at, starting from where it began.</summary>
    private double[] Truth(SceneDocument document, int steps, Func<ScenePhysics, double> height)
    {
        var world = Make(document);
        var truth = new List<double> { height(world) };
        for (var k = 0; k < steps; k++)
        {
            world.Advance(Step);
            truth.Add(height(world));
        }

        return [.. truth];
    }

    private static double Blend(double[] truth, double behind)
    {
        var k = (int)Math.Floor(behind);
        return truth[k] + ((truth[k + 1] - truth[k]) * (behind - k));
    }

    [Fact]
    public void MovesABodyOnEveryFrameOfA144HzRender()
    {
        var scene = Make(Falling, smooth: true);

        var shown = new List<double> { HeightOf(scene, "crate") };
        for (var i = 0; i < 288; i++)
        {
            scene.Advance(Frame);
            shown.Add(HeightOf(scene, "crate"));
        }

        // The first step ends on frame three. Until it has, there is nothing to blend towards.
        for (var i = 3; i < shown.Count; i++)
        {
            Assert.True(shown[i] < shown[i - 1], $"frame {i}");
        }
    }

    [Fact]
    public void SaysSoInTheDiffOnAFrameThatTookNoStep()
    {
        var scene = Make(Falling, smooth: true);
        Frames(scene, 3);

        for (var i = 0; i < 100; i++)
        {
            Assert.False(scene.Advance(Frame).IsEmpty, $"frame {i}");
        }
    }

    [Fact]
    public void ShowsABodyAStepBehindTheWorldByTheTimeOwed()
    {
        var scene = Make(Falling, smooth: true);
        var truth = Truth(Falling, 130, world => HeightOf(world, "crate"));

        for (var n = 1; n <= 288; n++)
        {
            scene.Advance(Frame);

            // How many steps ago the render is showing.
            var behind = (n * Frame / Step) - 1;
            var shown = HeightOf(scene, "crate");
            Near(behind < 0 ? truth[0] : Blend(truth, behind), shown, 1e-4, $"frame {n}");
        }
    }

    [Fact]
    public void LeavesTheWorldExactlyAsItWouldHaveBeen()
    {
        var scene = Make(Falling, smooth: true);
        var world = Make(Falling);

        // 41 whole steps in a hundred frames, clear of a step landing on a frame.
        Frames(scene, 100);
        for (var k = 0; k < 41; k++)
        {
            world.Advance(Step);
        }

        Assert.Equal(
            world.Physics.TransformOf(world.BodyOf("crate")!.Value),
            scene.Physics.TransformOf(scene.BodyOf("crate")!.Value));
    }

    [Fact]
    public void KeepsTheWorldAheadOfWhatIsShown()
    {
        var scene = Make(Falling, smooth: true);
        Frames(scene, 100);

        Assert.True(TruthOf(scene, "crate") < HeightOf(scene, "crate"));
    }

    [Fact]
    public void DoesNotMoveABodyAtRest()
    {
        var scene = Make(SceneOf(Floor(), Crate("crate", at: new Vector3(0, 0.5, 0))), smooth: true);
        Frames(scene, 576);
        var settled = scene.Document.Encode();

        for (var i = 0; i < 288; i++)
        {
            Assert.True(scene.Advance(Frame).IsEmpty, $"frame {i}");
        }

        Assert.Equal(settled, scene.Document.Encode());
    }

    [Fact]
    public void ShowsALateFrameAStepBehind()
    {
        var scene = Make(SceneOf(Floor(), Crate("crate", at: new Vector3(0, 100, 0))), smooth: true);

        scene.Advance(10);

        Assert.True(HeightOf(scene, "crate") < 100);
        Assert.True(HeightOf(scene, "crate") > TruthOf(scene, "crate"));
    }

    [Fact]
    public void ShowsABodyUnderABodyWhereItsOwnBlendPutsIt()
    {
        var nested = SceneOf(
            Floor(),
            Crate("parent", at: new Vector3(0, 60, 0)),
            Crate("child", parent: "parent", at: new Vector3(3, 20, 0)));
        var scene = Make(nested, smooth: true);
        double Sum(ScenePhysics world) => HeightOf(world, "child") + HeightOf(world, "parent");
        var truth = Truth(nested, 60, Sum);

        // No step has ended yet, so the frames before frame three show the start.
        Frames(scene, 2);
        for (var n = 3; n <= 100; n++)
        {
            scene.Advance(Frame);

            // The child's transform is under the parent's, so what the document makes of it in the world is
            // the sum.
            Near(Blend(truth, (n * Frame / Step) - 1), Sum(scene), 1e-3, $"frame {n}");
        }
    }

    [Fact]
    public void ASceneThatDoesNotSmoothStandsStillBetweenStepsAsItDid()
    {
        var scene = Make(Falling);

        var still = 0;
        for (var i = 0; i < 288; i++)
        {
            if (scene.Advance(Frame).IsEmpty)
            {
                still++;
            }
        }

        // 120 steps in 288 frames leaves at least 168 with nothing to show.
        Assert.True(still >= 168, $"{still} frames stood still");
    }

    [Fact]
    public void ASceneThatDoesNotSmoothDoesNothingWithTheCallsAboutSmoothing()
    {
        var scene = Make(Falling);

        Assert.True(scene.StopSmoothing("crate"));
        Assert.True(scene.ResetSmoothing("crate"));
        Assert.True(scene.StartSmoothing("crate"));
        scene.Advance(Step);
        Assert.True(HeightOf(scene, "crate") < 60);
    }

    [Fact]
    public void ATeleportByAnEditDoesNotStreak()
    {
        var scene = Make(Falling, smooth: true);
        Frames(scene, 30);

        scene.Apply(Teleport(scene, "crate", new Vector3(100, 60, 0)));
        Assert.Equal(100.0, SidewaysOf(scene, "crate"));

        for (var i = 0; i < 20; i++)
        {
            scene.Advance(Frame);
            Near(100, SidewaysOf(scene, "crate"), 1e-3, $"{i}");
        }
    }

    [Fact]
    public void ATeleportThroughTheWorldDoesNotStreakOnceItIsReset()
    {
        var scene = Make(Falling, smooth: true);
        Frames(scene, 30);

        scene.Physics.Place(scene.BodyOf("crate")!.Value, [100, 60, 0]);
        Assert.True(scene.ResetSmoothing("crate"));

        for (var i = 0; i < 20; i++)
        {
            scene.Advance(Frame);
            Near(100, SidewaysOf(scene, "crate"), 1e-3, $"{i}");
        }
    }

    [Fact]
    public void ATeleportThroughTheWorldSlidesThereUntilItIsReset()
    {
        var scene = Make(Falling, smooth: true);
        Frames(scene, 30);

        scene.Physics.Place(scene.BodyOf("crate")!.Value, [100, 60, 0]);

        var seen = new List<double>();
        for (var i = 0; i < 6; i++)
        {
            scene.Advance(Frame);
            seen.Add(SidewaysOf(scene, "crate"));
        }

        Assert.True(seen.Any(x => x is > 1 and < 99), string.Join(", ", seen));
    }

    [Fact]
    public void ATeleportOfOneBodyLeavesTheBlendOfTheOthersAlone()
    {
        var scene = Make(Pair, smooth: true);
        var other = Make(Pair, smooth: true);
        Frames(scene, 30);
        Frames(other, 30);

        scene.Physics.Place(scene.BodyOf("a")!.Value, [100, 60, 0]);
        scene.ResetSmoothing("a");
        for (var i = 0; i < 30; i++)
        {
            scene.Advance(Frame);
            other.Advance(Frame);
            Assert.True(HeightOf(scene, "b") == HeightOf(other, "b"), $"{i}");
        }
    }

    [Fact]
    public void ATeleportIsNotResetForAnEntityTheDocumentDoesNotHave()
    {
        var scene = Make(Falling, smooth: true);

        Assert.False(scene.ResetSmoothing("nothing"));
    }

    [Fact]
    public void StopSmoothingShowsABodyAsTheWorldHasItFromThatMoment()
    {
        var scene = Make(Pair, smooth: true);
        Frames(scene, 20);
        Assert.True(HeightOf(scene, "a") > TruthOf(scene, "a"));

        Assert.True(scene.StopSmoothing("a"));
        scene.Advance(0);
        Near(TruthOf(scene, "a"), HeightOf(scene, "a"), 1e-4, "after the stop");

        var behind = 0;
        for (var i = 0; i < 100; i++)
        {
            scene.Advance(Frame);
            Near(TruthOf(scene, "a"), HeightOf(scene, "a"), 1e-4, $"frame {i}");
            if (HeightOf(scene, "b") > TruthOf(scene, "b") + 1e-4)
            {
                behind++;
            }
        }

        // The body beside it is still blended.
        Assert.True(behind > 50, $"{behind} frames behind");
    }

    [Fact]
    public void StopSmoothingTakesEverythingUnderTheEntityWithIt()
    {
        var folder = new SceneEntity(
            "group",
            "group",
            components: new Dictionary<string, SceneComponent> { [SceneComponents.Transform] = new TransformComponent() });
        var scene = Make(
            SceneOf(
                Floor(),
                folder,
                Crate("inside", parent: "group", at: new Vector3(0, 60, 0)),
                Crate("outside", at: new Vector3(5, 60, 0))),
            smooth: true);
        Frames(scene, 20);

        Assert.True(scene.StopSmoothing("group"));
        Frames(scene, 20);

        Near(TruthOf(scene, "inside"), HeightOf(scene, "inside"), 1e-4, "inside");
        Assert.True(HeightOf(scene, "outside") > TruthOf(scene, "outside") + 1e-4);
    }

    [Fact]
    public void StopSmoothingIsUndoneByStartSmoothingOnceWithoutAJump()
    {
        var scene = Make(Pair, smooth: true);
        scene.StopSmoothing("a");
        Frames(scene, 30);

        Assert.True(scene.StartSmoothing("a"));
        Assert.False(scene.StartSmoothing("a"));

        var last = HeightOf(scene, "a");
        var behind = 0;
        for (var i = 0; i < 30; i++)
        {
            scene.Advance(Frame);
            var shown = HeightOf(scene, "a");
            Assert.True(shown <= last, $"frame {i}");
            last = shown;
            if (shown > TruthOf(scene, "a") + 1e-4)
            {
                behind++;
            }
        }

        Assert.True(behind > 10, $"{behind} frames behind");
    }

    [Fact]
    public void StopSmoothingSaysNoToAnEntityTheDocumentDoesNotHave()
    {
        var scene = Make(Pair, smooth: true);

        Assert.False(scene.StopSmoothing("nothing"));
        Assert.False(scene.StartSmoothing("a"));
    }

    [Fact]
    public void StopSmoothingEndsWithTheEntity()
    {
        var scene = Make(Pair, smooth: true);
        scene.StopSmoothing("a");
        var entity = scene.Document["a"]!;

        scene.Apply(new SceneDiff([new RemoveEntity(entity.ToJson())]));
        scene.Apply(new SceneDiff([new AddEntity(entity.ToJson())]));

        Assert.False(scene.StartSmoothing("a"));
    }

    private static List<string> Replay(ScenePhysics scene)
    {
        var shown = new List<string>();
        for (var i = 0; i < 60; i++)
        {
            scene.Advance(Frame);
            shown.Add(scene.Document.Encode());
        }

        return shown;
    }

    [Fact]
    public void ASnapshotOfASceneThatSmoothsReplaysTheSameShownPosesHoweverOftenItIsRestored()
    {
        var scene = Make(Pair, smooth: true);

        // After 41 frames the next call takes no step, so the first frame replayed is a blend that reads
        // where the bodies were a step ago. A snapshot that shared that with the scene would show it moved
        // by the first replay.
        Frames(scene, 41);
        using var snapshot = scene.Snapshot();

        var first = Replay(scene);
        scene.Restore(snapshot);
        Assert.Equal(first, Replay(scene));
        scene.Restore(snapshot);
        Assert.Equal(first, Replay(scene));
    }

    [Fact]
    public void ASnapshotOfASceneThatSmoothsGoesBackToThePosesItWasTakenAt()
    {
        var scene = Make(Pair, smooth: true);
        Frames(scene, 40);
        var shown = scene.Document.Encode();
        using var snapshot = scene.Snapshot();

        Frames(scene, 40);
        scene.Restore(snapshot);

        Assert.Equal(shown, scene.Document.Encode());
    }

    [Fact]
    public void ASnapshotOfASceneThatSmoothsKeepsWhatWasSwitchedOffAndWhatWasNot()
    {
        var scene = Make(Pair, smooth: true);
        scene.StopSmoothing("a");
        using var stopped = scene.Snapshot();
        scene.StartSmoothing("a");
        using var blended = scene.Snapshot();

        scene.Restore(stopped);
        Assert.True(scene.StartSmoothing("a"));
        scene.StopSmoothing("a");
        scene.Restore(blended);
        Assert.False(scene.StartSmoothing("a"));
    }

    [Fact]
    public void ASnapshotOfASceneThatSmoothsDoesNotGoIntoASceneThatSmoothsDifferently()
    {
        var scene = Make(Pair, smooth: true);
        var plain = Make(Pair);
        using var smoothed = scene.Snapshot();
        using var unsmoothed = plain.Snapshot();
        Frames(plain, 10);
        var before = plain.Document.Encode();

        Assert.ThrowsAny<ArgumentException>(() => plain.Restore(smoothed));
        Assert.ThrowsAny<ArgumentException>(() => scene.Restore(unsmoothed));
        Assert.Equal(before, plain.Document.Encode());
    }
}
