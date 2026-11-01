using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneShapesTests : SceneTestBase
{
    /// <summary>
    /// How far into the ground the solver lets a body rest, which is the same for every shape: a box and
    /// a ball sink as far as these do.
    /// </summary>
    private const double Slop = 0.03;

    private static SceneEntity Ground() =>
        Entity("floor", new BodyComponent(BodyShape.Plane, motion: BodyMotion.Fixed), Vector3.Zero());

    /// <summary>The eight corners of a cube <paramref name="half"/> either side of its middle.</summary>
    private static double[] Cube(double half)
    {
        double[] sides = [-half, half];
        return [.. sides.SelectMany(x => sides.SelectMany(y => sides.SelectMany(z => new[] { x, y, z })))];
    }

    private static BodyComponent HullOf(IReadOnlyList<double> corners) => new(BodyShape.Hull, hull: corners);

    private static double HeightOf(ScenePhysics scene, string entity) =>
        scene.Physics.TransformOf(scene.BodyOf(entity)!.Value)![1];

    private static void Remove(ScenePhysics scene, string entity) =>
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document[entity]!.ToJson())]));

    /// <summary>
    /// Whether the world has a hull under <paramref name="id"/>, told by trying to lay one there. When
    /// it has none the try lays one, so ask last.
    /// </summary>
    private static bool HasHull(ScenePhysics scene, ulong id) => !scene.Physics.LayHull(id, Cube(1));

    [Fact]
    public void CylinderStandsOnItsFlatEndAsTallAsItSays()
    {
        var scene = Make(SceneOf(
            Ground(),
            Entity("barrel", new BodyComponent(BodyShape.Cylinder, radius: 0.4, height: 1.2), new Vector3(0, 3, 0))));
        Run(scene, 2);

        Assert.Equal(0.6, HeightOf(scene, "barrel"), Slop);
    }

    [Fact]
    public void CylinderGrowsWithItsEntityTheSidewaysScaleForTheRadius()
    {
        var scene = Make(SceneOf(
            Ground(),
            Entity("barrel", new BodyComponent(BodyShape.Cylinder, radius: 0.4, height: 1.2), new Vector3(0, 4, 0), new Vector3(2, 3, 1))));
        Run(scene, 2);

        // Three times as tall, so 3.6 metres from end to end.
        Assert.Equal(1.8, HeightOf(scene, "barrel"), Slop);
    }

    [Fact]
    public void HullRestsOnTheGroundAsTheSolidRoundItsCorners()
    {
        var scene = Make(SceneOf(Ground(), Entity("rock", HullOf(Cube(0.5)), new Vector3(0, 3, 0))));
        Run(scene, 2);

        Assert.Equal(0.5, HeightOf(scene, "rock"), Slop);
    }

    [Fact]
    public void HullIsStretchedByItsEntityAlongEachAxis()
    {
        var scene = Make(SceneOf(Ground(), Entity("slab", HullOf(Cube(0.5)), new Vector3(0, 4, 0), new Vector3(1, 0.2, 3))));
        Run(scene, 2);

        Assert.Equal(0.1, HeightOf(scene, "slab"), Slop);
    }

    [Fact]
    public void HullIsPlacedFromItsCentreWhereTheCornersAreMeasured()
    {
        var scene = Make(SceneOf(
            Ground(),
            Entity("rock", HullOf(Cube(0.5)).CopyWith(centre: new Vector3(0, 0.5, 0)), new Vector3(0, 3, 0))));

        Assert.Equal(3.5, HeightOf(scene, "rock"), 1e-4);
    }

    [Fact]
    public void HullThatEnclosesNothingMakesNoBodyAndTheEntityKeepsItsNumber()
    {
        var scene = Make(SceneOf(
            Ground(),
            Entity("flat", HullOf([0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1]), new Vector3(0, 3, 0)),
            Entity("ragged", HullOf([0, 1, 2, 3]), new Vector3(4, 3, 0))));

        Assert.NotNull(scene.BodyOf("flat"));
        Assert.Null(scene.Physics.VelocityOf(scene.BodyOf("flat")!.Value));
        Assert.Null(scene.Physics.VelocityOf(scene.BodyOf("ragged")!.Value));
        Assert.True(scene.Advance(1.0 / 60).IsEmpty);
    }

    private static SceneDocument Crates() => SceneOf(
        Ground(),
        Entity("a", HullOf(Cube(0.5)), new Vector3(0, 2, 0)),
        Entity("b", HullOf(Cube(0.5)), new Vector3(3, 2, 0)),
        Entity("c", HullOf(Cube(0.5)), new Vector3(6, 2, 0), new Vector3(2, 2, 2)));

    [Fact]
    public void LaidHullsAreSharedByBodiesCutFromTheSameCornersAtTheSameSize()
    {
        var scene = Make(Crates());

        // Two hulls, not three: the bridge numbers its own from one.
        Assert.True(HasHull(scene, 1));
        Assert.True(HasHull(scene, 2));
        Assert.False(HasHull(scene, 3));
    }

    [Fact]
    public void LaidHullsStayWhileABodyIsMadeOfThemAndGoWithTheLast()
    {
        var scene = Make(Crates());
        Remove(scene, "a");
        Assert.True(HasHull(scene, 1));

        Remove(scene, "b");
        Assert.True(HasHull(scene, 2));
        Assert.False(HasHull(scene, 1));
    }

    [Fact]
    public void LaidHullsChangeWhenTheBodyIsEditedToOtherCorners()
    {
        var scene = Make(Crates());
        var before = scene.Document["c"]![SceneComponents.Body]!;
        scene.Apply(new SceneDiff([
            new SetComponent("c", SceneComponents.Body, before.ToJson(), HullOf(Cube(0.3)).ToJson()),
        ]));

        // The cube at twice the size is gone, and the one at the new size is laid. The two unedited
        // bodies still have theirs.
        Assert.True(HasHull(scene, 1));
        Assert.True(HasHull(scene, 3));
        Assert.False(HasHull(scene, 2));
    }

    [Fact]
    public void LaidHullsComeBackWithASnapshotAndGoAgainWithTheLastBody()
    {
        var scene = Make(SceneOf(Ground(), Entity("rock", HullOf(Cube(0.5)), new Vector3(0, 3, 0))));
        using var snapshot = scene.Snapshot();
        Remove(scene, "rock");
        Assert.False(HasHull(scene, 1));

        scene.Restore(snapshot);
        Run(scene, 2);
        Assert.Equal(0.5, HeightOf(scene, "rock"), Slop);

        Remove(scene, "rock");
        Assert.False(HasHull(scene, 1));
    }
}
