using Orblit.Asset;
using Orblit.Mesh;
using Orblit.Scene;
using Orblit.VectorMath;
using static Orblit.Physics.Scene.Tests.SceneKit;
using MeshShape = Orblit.Mesh.Shape;
using MeshShapeKind = Orblit.Mesh.ShapeKind;

namespace Orblit.Physics.Scene.Tests;

public sealed class SceneMeshTests : SceneTestBase
{
    [Fact]
    public async Task AnImportedModelRestsOnAnImportedMeshFloorAcrossItsSeam()
    {
        var source = new MemoryAssetSource();
        var cube = new MeshShape { Kind = MeshShapeKind.Cube }.Build();
        var plane = new MeshShape { Kind = MeshShapeKind.Plane, Width = 20, Depth = 20 }.Build();
        var imported = await CollisionMesh.FromGltfAsync(AssetId.Parse("crate.glb"), cube.ToGlb(), source);
        var ground = await CollisionMesh.FromGltfAsync(AssetId.Parse("floor.glb"), plane.ToGlb(), source);
        var scene = Make(new SceneDocument(entities:
        [
            Entity(
                "floor",
                new BodyComponent(
                    BodyShape.Mesh,
                    meshVertices: ground.Vertices,
                    meshIndices: [.. ground.Indices.Select(i => (long)i)]),
                Vector3.Zero()),
            Entity("crate", new BodyComponent(BodyShape.Hull, hull: imported.Vertices), new Vector3(0, 4, 0)),
        ]));

        for (var step = 0; step < 360; step++)
        {
            scene.Advance(1.0 / 60);
        }

        var crate = scene.BodyOf("crate")!.Value;
        Assert.Equal(0, scene.Physics.TransformOf(crate)![1], 0.03);
        Assert.True(scene.Physics.Asleep(crate));
        using var saved = scene.Snapshot();
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["floor"]!.ToJson())]));
        scene.Restore(saved);
        Assert.NotNull(scene.Physics.Cast([4, 3, 0], [0, -1, 0], 6));
    }

    [Fact]
    public void MeshScalingRemovalAndRestorationKeepTheAssetBookkeeping()
    {
        var body = new BodyComponent(
            BodyShape.Mesh,
            meshVertices: [-1, 0, -1, 1, 0, -1, 0, 0, 1],
            meshIndices: [0, 2, 1],
            shapeScale: new Vector3(2, 1, 1));
        var scene = Make(new SceneDocument(entities: [Entity("ramp", body, new Vector3(0, 2, 0), new Vector3(2, 1, 1))]));

        Assert.Equal(2, scene.Physics.Cast([3, 4, -0.9], [0, -1, 0], 5)!.Distance, 0.002);
        using var saved = scene.Snapshot();
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["ramp"]!.ToJson())]));
        Assert.False(scene.Physics.DropMesh(scene.BodyOf("ramp") ?? 1));

        scene.Restore(saved);
        Assert.NotNull(scene.Physics.Cast([0, 4, 0], [0, -1, 0], 5));
        scene.Apply(new SceneDiff([new RemoveEntity(scene.Document["ramp"]!.ToJson())]));
        Assert.Equal(0, scene.Physics.Stats.Bodies);
    }

    [Fact]
    public void MalformedMeshesRemainEditableAndInert()
    {
        var scene = Make(new SceneDocument(entities:
        [
            Entity(
                "broken",
                new BodyComponent(BodyShape.Mesh, meshVertices: [0, 0, 0, 1, 0, 0, 0, 1, 0], meshIndices: [0, 1, 7]),
                Vector3.Zero()),
        ]));

        Assert.Equal(0, scene.Physics.Stats.Bodies);
        Assert.IsType<BodyComponent>(scene.Document["broken"]![SceneComponents.Body]);
    }
}
