using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneCompoundTests : SceneTestBase
{
    [Fact]
    public void CompoundPartsSurviveSceneFilesAndPreserveTheirEmptyGaps()
    {
        var body = new BodyComponent(
            BodyShape.Compound,
            motion: BodyMotion.Fixed,
            parts: [new BodyPart(centre: new Vector3(-2, 0, 0)), new BodyPart(centre: new Vector3(2, 0, 0))]);
        var loaded = BodyComponent.FromJson(body.ToJson());
        var scene = Make(new SceneDocument(entities: [Entity("arch", loaded)]));

        Assert.Empty(scene.Physics.Overlap([0, 0, 0]));
        Assert.Equal([scene.BodyOf("arch")!.Value], scene.Physics.Overlap([2, 0, 0]));
    }

    [Fact]
    public void ShapeScaleStretchesGeometrySeparatelyFromTheSceneTransform()
    {
        var scene = Make(new SceneDocument(entities:
        [
            Entity(
                "egg",
                new BodyComponent(BodyShape.Sphere, radius: 1, shapeScale: new Vector3(2, 1, 0.5), motion: BodyMotion.Fixed),
                scale: new Vector3(2, 1, 1)),
        ]));

        Assert.Equal(2, scene.Physics.Cast([6, 0, 0], [-1, 0, 0], 8)!.Distance, 0.002);
        Assert.Empty(scene.Physics.Overlap([0, 0, 0.6]));
        var scale = (TransformComponent)scene.Document["egg"]![SceneComponents.Transform]!;
        Assert.Equal(new Vector3(2, 1, 1), scale.Scale);
    }

    [Fact]
    public void RebuildingAndRemovingACompoundReleasesAllItsHullAssets()
    {
        double[] points =
        [
            -0.5, -0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, -0.5, -0.5, 0.5, 0.5,
            0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5,
        ];
        var body = new BodyComponent(
            BodyShape.Compound,
            motion: BodyMotion.Fixed,
            parts: [new BodyPart(BodyShape.Hull, hull: points)]);
        var scene = Make(new SceneDocument(entities: [Entity("rock", body)]));
        var number = scene.BodyOf("rock")!.Value;
        using var saved = scene.Snapshot();
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["rock"]!.ToJson())]));
        Assert.Empty(scene.Physics.Overlap([0, 0, 0]));
        Assert.True(scene.Physics.LayHull(1, points));

        scene.Restore(saved);
        Assert.Equal([number], scene.Physics.Overlap([0, 0, 0]));

        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["rock"]!.ToJson())]));
        Assert.True(scene.Physics.LayHull(1, points));
    }

    [Fact]
    public void InvalidPartsRemainEditableWithoutAWorldBody()
    {
        var scene = Make(new SceneDocument(entities:
        [
            Entity("broken", new BodyComponent(BodyShape.Compound, parts: [new BodyPart(BodyShape.Plane)])),
        ]));

        Assert.Equal(0, scene.Physics.Stats.Bodies);
        Assert.IsType<BodyComponent>(scene.Document["broken"]![SceneComponents.Body]);
    }
}
