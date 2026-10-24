namespace Orblit.Physics.Tests;

public class PhysicsMeshTests : PhysicsTestBase
{
    private static readonly double[] Sheet = [-10.0, 0.0, -10.0, 10.0, 0.0, -10.0, 10.0, 0.0, 10.0, -10.0, 0.0, 10.0];
    private static readonly int[] Triangles = [0, 2, 1, 0, 3, 2];

    private void Lay()
    {
        Assert.True(World.LayMesh(7, Sheet, Triangles));
        World.Add(1, Shape.Mesh(7), PhysicsMotion.Fixed);
    }

    [Fact]
    public void MeshRaysMeetBothSidesAndPreserveTheFiniteFootprint()
    {
        Lay();
        foreach (var side in new[] { -1.0, 1.0 })
        {
            var hit = World.Cast([0.0, side * 3, 0.0], [0.0, -side, 0.0], 6.0)!;
            Assert.Equal(3.0, hit.Distance, 0.002);
            Assert.Equal(side, hit.Normal[1], 0.002);
        }

        Assert.Null(World.Cast([11.0, 3.0, 0.0], [0.0, -1.0, 0.0], 6.0));
    }

    [Fact]
    public void BoxCylinderAndHullSettleAcrossTriangleSeams()
    {
        Lay();
        World.LayHull(3, Cube(0.5));
        Shape[] shapes = [Shape.Box(0.5, 0.5, 0.5), Shape.Cylinder(0.5, 0.5), Shape.Hull(3)];
        for (var i = 0; i < shapes.Length; i++)
        {
            World.Add((ulong)(i + 2), shapes[i], at: [i * 2.0, 3.0, i * 2.0]);
        }

        Run(6);
        for (var i = 0; i < shapes.Length; i++)
        {
            Assert.Equal(0.5, HeightOf((ulong)(i + 2)), 0.025);
            Assert.True(World.Asleep((ulong)(i + 2)));
        }
    }

    [Fact]
    public void ABallCrossingAWeldedSeamGetsNoSidewaysImpulseOrBounce()
    {
        Assert.True(World.LayMesh(
            7,
            [-10.0, 0.0, -10.0, 10.0, 0.0, -10.0, 10.0, 0.0, 10.0, -10.0, 0.0, -10.0, -10.0, 0.0, 10.0, 10.0, 0.0, 10.0],
            [0, 2, 1, 3, 4, 5]));
        World.Add(1, Shape.Mesh(7), PhysicsMotion.Fixed, friction: 0.0);
        World.Add(2, Shape.Sphere(0.5), at: [-3.0, 0.5, 0.0], friction: 0.0, linearDamping: 0.0);
        World.Drive(2, [2.0, 0.0, 0.0]);
        for (var step = 0; step < 180; step++)
        {
            World.Step(1.0 / 60);
            Assert.Equal(0.5, HeightOf(2), 0.015);
            Assert.Equal(2.0, World.VelocityOf(2)![0], 0.01);
        }
    }

    [Fact]
    public void ConvexCastsAndOverlapsMeetARotatedSheetFromEitherSide()
    {
        Assert.True(World.LayMesh(7, Sheet, Triangles));
        World.Add(1, Shape.Mesh(7), PhysicsMotion.Fixed, [2.0, 0.0, 0.0], [0.0, 0.0, -0.7071067812, 0.7071067812]);
        foreach (var side in new[] { -1.0, 1.0 })
        {
            var hit = World.Cast([2 + (side * 3), 0.0, 0.0], [-side, 0.0, 0.0], 6.0, Shape.Sphere(0.5))!;
            Assert.Equal(2.5, hit.Distance, 0.003);
            Assert.Equal(side, hit.Normal[0], 0.003);
        }

        Assert.Equal(new ulong[] { 1 }, World.Overlap([2.2, 0.0, 0.0], Shape.Box(0.5, 0.5, 0.5)));
        var inside = World.Cast([2.2, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0, Shape.Sphere(0.5))!;
        Assert.True(inside.Started);
        Assert.Equal(0.0, inside.Distance);
    }

    [Fact]
    public void TheGapBetweenSeparatedTrianglePatchesStaysEmpty()
    {
        Assert.True(World.LayMesh(
            7,
            [-3.0, 0.0, -1.0, -1.0, 0.0, -1.0, -2.0, 0.0, 1.0, 1.0, 0.0, -1.0, 3.0, 0.0, -1.0, 2.0, 0.0, 1.0],
            [0, 2, 1, 3, 5, 4]));
        World.Add(1, Shape.Mesh(7), PhysicsMotion.Fixed);
        Assert.Null(World.Cast([0.0, 2.0, 0.0], [0.0, -1.0, 0.0], 4.0));
        Assert.Empty(World.Overlap([0.0, 0.0, 0.0], Shape.Sphere(0.5)));
        Assert.NotNull(World.Cast([-2.0, 2.0, 0.0], [0.0, -1.0, 0.0], 4.0));
    }

    [Fact]
    public void CharactersAndScaledCompoundsUseTheMeshCollisionPath()
    {
        Lay();
        World.AddCharacter(2, at: [-3.0, 0.91, 0.0]);
        for (var step = 0; step < 180; step++)
        {
            var footing = World.FootingOf(2)!;
            World.Drive(2, [2.0, footing.Velocity[1] - (9.81 / 60), 0.0]);
            World.Step(1.0 / 60);
            Assert.True(World.FootingOf(2)!.Grounded);
            Assert.Equal(0.91, HeightOf(2), 0.02);
        }

        Assert.Equal(3.0, World.TransformOf(2)![0], 0.02);
        Assert.True(World.LayCompound(8, [new ShapePart(Shape.Sphere(0.5), scale: [2.0, 1.0, 1.0])]));
        World.Add(3, Shape.Compound(8), at: [0.0, 3.0, 3.0]);
        Run(6);
        Assert.Equal(0.5, HeightOf(3), 0.025);
        Assert.True(World.Asleep(3));
    }

    [Fact]
    public void MeshOwnershipSurvivesRestorationIntoAnotherWorld()
    {
        Lay();
        Assert.False(World.DropMesh(7));
        using var saved = World.Snapshot();
        World.Remove(1);
        Assert.True(World.DropMesh(7));
        using var other = new Physics();
        other.Restore(saved);
        saved.Dispose();
        Assert.Equal(1UL, other.Cast([0.0, 2.0, 0.0], [0.0, -1.0, 0.0], 3.0)!.Body);
    }

    [Fact]
    public void InvalidAndEmptyMeshesAreRefusedAndMeshBodiesStayFixed()
    {
        Assert.False(World.LayMesh(7, Sheet, [0, 1, 9]));
        Assert.False(World.LayMesh(7, Sheet, [0, 0, 0]));
        Lay();
        World.Add(2, Shape.Mesh(7));
        Assert.Null(World.TransformOf(2));
        World.SetMotion(1, PhysicsMotion.Free);
        World.Step(1.0 / 60);
        Assert.Equal(0.0, World.TransformOf(1)![1]);
    }
}
