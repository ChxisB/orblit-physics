using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneSnapshotTests : SceneTestBase
{
    private static SceneEntity CrateAt(string id, Vector3 at) => Crate(id, at: at);

    /// <summary>An entity that is only a weld, held to the crate it is under.</summary>
    private static SceneEntity Weld(string parent, double breakingForce, string id = "weld") => new(
        id,
        id,
        parent,
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(),
            [SceneComponents.Joint] = new JointComponent(JointKind.Fixed, breakingForce: breakingForce),
        });

    /// <summary>
    /// Takes the breaking force off the weld entity. Editing a broken joint's own entity is how it is
    /// asked for back.
    /// </summary>
    private static SceneDiff Mend() => new([new SetField("weld", SceneComponents.Joint, "breakingForce", 5, 0)]);

    private static double HeightOf(ScenePhysics scene, string id) => TransformOf(scene.Document, id).Position.Y;

    /// <summary>
    /// Everything a replay has to reproduce: the document, how much of it the advance wrote, which body
    /// each entity is, and what happened.
    /// </summary>
    private static string Recorded(ScenePhysics scene, SceneDiff diff)
    {
        var numbers = new[] { "a", "b", "late" }.Select(id => $"{id} {scene.BodyOf(id)}");
        var events = scene.Events.Select(e => $"{e.Kind} {e.A} {e.B}");
        return string.Join('\n', new[] { scene.Document.Encode(), diff.Operations.Count.ToString() }.Concat(numbers).Concat(events));
    }

    /// <summary>
    /// Frames <paramref name="from"/> up to <paramref name="to"/> of a script that keeps editing the
    /// scene: an entity arrives at frame 35 and another leaves at frame 50. Edits are keyed on the frame,
    /// so a replay does what the first run did.
    /// </summary>
    private static List<string> Play(ScenePhysics scene, int from, int to)
    {
        var frames = new List<string>();
        for (var frame = from; frame < to; frame++)
        {
            if (frame == 35)
            {
                scene.Apply(new SceneDiff([new AddEntity(CrateAt("late", new Vector3(0.2, 2, 0)).ToJson())]));
            }

            if (frame == 50)
            {
                scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["b"]!.ToJson())]));
            }

            frames.Add(Recorded(scene, scene.Advance(1.0 / 60)));
        }

        return frames;
    }

    private ScenePhysics Stacked() => Make(SceneOf(
        Floor(),
        CrateAt("a", new Vector3(0, 3, 0)),
        CrateAt("b", new Vector3(0.3, 5, 0))));

    private ScenePhysics Welded() => Make(SceneOf(CrateAt("a", new Vector3(0, 3, 0)), Weld("a", 5)));

    /// <summary>One crate on the floor and one dropped on it from above.</summary>
    private ScenePhysics AboveAnother() => Make(SceneOf(
        Floor(),
        CrateAt("low", new Vector3(0, 0.5, 0)),
        CrateAt("high", new Vector3(0, 3, 0))));

    [Fact]
    public void RestoresToTheFrameItWasTakenAtAndReplaysToTheSameFrames()
    {
        var scene = Stacked();
        Play(scene, 0, 30);
        using var snapshot = scene.Snapshot();
        var atThirty = scene.Document.Encode();
        var first = Play(scene, 30, 90);

        scene.Restore(snapshot);
        Assert.Equal(atThirty, scene.Document.Encode());
        Assert.Equal(first, Play(scene, 30, 90));

        // A snapshot is not used up, and what the replays did to the scene did not leak into it.
        scene.Restore(snapshot);
        Assert.Equal(first, Play(scene, 30, 90));
    }

    [Fact]
    public void HandsTheSameBodyNumbersOutAgain()
    {
        var scene = Stacked();
        using var snapshot = scene.Snapshot();
        Play(scene, 0, 40);
        var late = scene.BodyOf("late");
        Assert.NotNull(late);

        scene.Restore(snapshot);
        Assert.Null(scene.BodyOf("late"));
        Play(scene, 0, 40);
        Assert.Equal(late, scene.BodyOf("late"));
    }

    [Fact]
    public void AnswersWithTheDiffThatTakesTheDocumentBack()
    {
        var scene = Stacked();
        Play(scene, 0, 20);
        using var snapshot = scene.Snapshot();
        var atTwenty = scene.Document.Encode();
        Play(scene, 20, 60);
        var before = scene.Document;

        var diff = scene.Restore(snapshot);

        Assert.False(diff.IsEmpty);
        Assert.Equal(atTwenty, diff.ApplyTo(before).Encode());
        Assert.Equal(atTwenty, scene.Document.Encode());
    }

    [Fact]
    public void UndoesAnEditRemovalsIncluded()
    {
        var scene = Stacked();
        Play(scene, 0, 10);
        using var snapshot = scene.Snapshot();
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["a"]!.ToJson())]));
        Assert.Null(scene.BodyOf("a"));

        scene.Restore(snapshot);

        var body = scene.BodyOf("a");
        Assert.NotNull(body);
        Assert.NotNull(scene.Physics.TransformOf(body.Value));
        Assert.Equal("a", scene.EntityOf(body.Value));
    }

    [Fact]
    public void KeepsTheTimeAFrameStillOwesTheNext()
    {
        var scene = Stacked();
        scene.Advance(0.01);
        using var snapshot = scene.Snapshot();
        scene.Advance(0.01);

        scene.Restore(snapshot);

        // A step is 0.0167 s. The ten milliseconds owed and these seven add up to one, and the seven alone
        // would not.
        Assert.False(scene.Advance(0.0067).IsEmpty);
    }

    [Fact]
    public void IsNotDisturbedByWhereABodyWentAfterIt()
    {
        var scene = Make(SceneOf(Floor(), CrateAt("a", new Vector3(0, 3, 0))));
        Run(scene, 4);
        using var snapshot = scene.Snapshot();

        // The second round runs on what the first restore gave the scene. If that were the snapshot's own
        // copy, the run would change it.
        for (var round = 0; round < 2; round++)
        {
            scene.Physics.Push(scene.BodyOf("a")!.Value, [5, 0, 0]);
            Run(scene, 1);
            scene.Restore(snapshot);
        }

        // The crate is asleep where the snapshot left it. Writing it again would mean the snapshot had
        // been moved along with the scene.
        Assert.True(scene.Advance(1.0 / 60).IsEmpty);
    }

    [Fact]
    public void MendsAJointThatBrokeAfterItAndLetsItBreakAgain()
    {
        var scene = Welded();
        var number = scene.JointOf("weld")!.Value;
        using var snapshot = scene.Snapshot();
        scene.Advance(1.0 / 60);
        Assert.Equal(["weld"], scene.Broken);
        Assert.Null(scene.Physics.JointStateOf(number));

        scene.Restore(snapshot);

        Assert.Empty(scene.Broken);
        Assert.NotNull(scene.Physics.JointStateOf(number));
        scene.Advance(1.0 / 60);
        Assert.Equal(["weld"], scene.Broken);
        Assert.Single(scene.Events, e => e.Kind == PhysicsEventKind.Broke);
    }

    [Fact]
    public void KeepsAJointBrokenThatHadBrokenBeforeIt()
    {
        var scene = Welded();
        var number = scene.JointOf("weld")!.Value;
        scene.Advance(1.0 / 60);
        using var snapshot = scene.Snapshot();
        scene.Apply(Mend());
        Assert.Empty(scene.Broken);

        scene.Restore(snapshot);

        Assert.Equal(["weld"], scene.Broken);
        Assert.Null(scene.Physics.JointStateOf(number));
    }

    [Fact]
    public void BringsBackAJointTakenOutAfterItAndEditsItAsOne()
    {
        var scene = Welded();
        var number = scene.JointOf("weld")!.Value;
        using var snapshot = scene.Snapshot();
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["weld"]!.ToJson())]));
        Assert.Null(scene.JointOf("weld"));

        scene.Restore(snapshot);

        Assert.Equal(number, scene.JointOf("weld"));
        Assert.Equal("weld", scene.EntityOfJoint(number));

        // Editing it takes the world's joint out before making it again. One the scene had forgotten it
        // held would stay as it was, and still break at the old force.
        scene.Apply(Mend());
        scene.Advance(1.0 / 60);
        Assert.Empty(scene.Broken);
        Assert.NotNull(scene.Physics.JointStateOf(number));
    }

    [Fact]
    public void HandsTheSameJointNumbersOutAgain()
    {
        var scene = Welded();
        using var snapshot = scene.Snapshot();
        var added = new SceneDiff([new AddEntity(Weld("a", 0, "second").ToJson())]);
        scene.Apply(added);
        var second = scene.JointOf("second");
        Assert.NotNull(second);

        scene.Restore(snapshot);
        Assert.Null(scene.JointOf("second"));
        scene.Apply(added);

        Assert.Equal(second, scene.JointOf("second"));
    }

    [Fact]
    public void GivesBackTheEventsOfTheAdvanceItWasTakenAfter()
    {
        var scene = Welded();
        scene.Advance(1.0 / 60);
        using var snapshot = scene.Snapshot();
        scene.Advance(1.0 / 60);
        Assert.Empty(scene.Events);

        scene.Restore(snapshot);

        Assert.Equal([PhysicsEventKind.Broke], scene.Events.Select(e => e.Kind).ToArray());
    }

    [Fact]
    public void ForgetsAPairToldToPassThroughEachOtherAfterIt()
    {
        var scene = AboveAnother();
        using var snapshot = scene.Snapshot();
        scene.Ignore("low", "high");

        scene.Restore(snapshot);

        Assert.False(scene.Unignore("low", "high"));
        Run(scene, 2);
        Assert.Equal(1.5, HeightOf(scene, "high"), 0.05);
    }

    [Fact]
    public void KeepsAPairToldToPassThroughEachOtherBeforeIt()
    {
        var scene = AboveAnother();
        scene.Ignore("low", "high");
        using var snapshot = scene.Snapshot();
        scene.Unignore("low", "high");

        scene.Restore(snapshot);

        Run(scene, 2);
        Assert.Equal(0.5, HeightOf(scene, "high"), 0.05);
        Assert.True(scene.Unignore("low", "high"));
    }

    [Fact]
    public void ASnapshotLetGoOfCannotBeRestoredAndTheSceneIsAsItWas()
    {
        var scene = Stacked();
        var snapshot = scene.Snapshot();
        Play(scene, 0, 20);
        var before = Recorded(scene, SceneDiff.None);
        snapshot.Dispose();

        Assert.True(snapshot.Disposed);
        Assert.ThrowsAny<InvalidOperationException>(() => scene.Restore(snapshot));
        Assert.Equal(before, Recorded(scene, SceneDiff.None));
    }

    [Fact]
    public void ASnapshotCanBeLetGoOfTwice()
    {
        var scene = Stacked();
        var snapshot = scene.Snapshot();
        snapshot.Dispose();

        Assert.Null(Record.Exception(snapshot.Dispose));
    }
}
