namespace Orblit.Physics.Tests;

public class PhysicsCompoundTests : PhysicsTestBase
{
    private static ShapePart[] Pillars() =>
    [
        new ShapePart(Shape.Box(0.5, 1.0, 0.5), at: [-2.0, 0.0, 0.0]),
        new ShapePart(Shape.Box(0.5, 1.0, 0.5), at: [2.0, 0.0, 0.0]),
    ];

    private void RunFour()
    {
        for (var i = 0; i < 240; i++)
        {
            World.Step(1.0 / 60);
        }
    }

    [Fact]
    public void RaysAndOverlapsSeePartsAndTheEmptySpaceBetweenThem()
    {
        Assert.True(World.LayCompound(7, Pillars()));
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);
        Assert.Null(World.Cast([0.0, 0.0, 3.0], [0.0, 0.0, -1.0], 6.0));
        var hit = World.Cast([2.0, 0.0, 3.0], [0.0, 0.0, -1.0], 6.0)!;
        Assert.Equal(1UL, hit.Body);
        Assert.Equal(2.5, hit.Distance, 0.001);
        Assert.Equal(1.0, hit.Normal[2], 0.001);
        Assert.Empty(World.Overlap([0.0, 0.0, 0.0]));
        Assert.Equal(new ulong[] { 1 }, World.Overlap([2.0, 0.0, 0.0]));
        Assert.Single(World.CastAll([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0], 8.0));
    }

    [Fact]
    public void ACompoundCastFindsTheEarliestChildIncludingAnInitialOverlap()
    {
        World.LayCompound(7, Pillars());
        World.Add(1, Shape.Sphere(0.25), PhysicsMotion.Fixed);
        var hit = World.Cast([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0], 8.0, Shape.Compound(7))!;
        Assert.Equal(1.25, hit.Distance, 0.001);
        Assert.True(World.Cast([-2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0, Shape.Compound(7))!.Started);
    }

    [Fact]
    public void ABallFallsThroughACompoundGapWhileTheCompoundRestsOnBothParts()
    {
        Floor();
        World.LayCompound(7, Pillars());
        World.Add(2, Shape.Compound(7), at: [0.0, 4.0, 0.0]);
        World.Add(3, Shape.Sphere(0.3), at: [0.0, 3.0, 0.0]);
        RunFour();
        Assert.Equal(1.0, HeightOf(2), 0.03);
        Assert.Equal(0.3, HeightOf(3), 0.03);
        Assert.True(World.Asleep(2));
    }

    [Fact]
    public void NonuniformScalingMakesAnEllipsoidWithExactBoundsAndRayDistances()
    {
        World.LayCompound(7, [new ShapePart(Shape.Sphere(1.0), scale: [2.0, 1.0, 0.5])]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);
        var x = World.Cast([4.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 5.0)!;
        var z = World.Cast([0.0, 0.0, 4.0], [0.0, 0.0, -1.0], 5.0)!;
        Assert.Equal(2.0, x.Distance, 0.002);
        Assert.Equal(3.5, z.Distance, 0.002);
        Assert.Empty(World.Overlap([0.0, 0.0, 1.0]));
        World.Add(2, Shape.Sphere(0.2), PhysicsMotion.Fixed, [0.0, 0.0, 0.6]);
        Assert.Contains(2UL, World.Overlap([0.0, 0.0, 0.0], Shape.Compound(7)));
    }

    [Fact]
    public void OuterScalingPreservesARotatedChildAndItsContactNormal()
    {
        var angle = Math.PI / 4;
        World.LayCompound(
            7,
            [new ShapePart(Shape.Box(1.0, 0.1, 0.2), rotation: [0.0, 0.0, Math.Sin(angle / 2), Math.Cos(angle / 2)])],
            scale: [2.0, 1.0, 1.0]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);
        var hit = World.Cast([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 6.0)!;
        Assert.Equal(-1 / Math.Sqrt(5), hit.Normal[0], 0.01);
        Assert.Equal(2 / Math.Sqrt(5), hit.Normal[1], 0.01);
    }

    [Fact]
    public void APushThroughATranslatedShapeCentreCausesNoSpinOrOriginJump()
    {
        World.LayCompound(7, [new ShapePart(Shape.Box(0.5, 0.5, 0.5), at: [2.0, 0.0, 0.0])]);
        World.Add(1, Shape.Compound(7), at: [0.0, 4.0, 0.0]);
        World.Push(1, [0.0, 0.0, 1.0], [2.0, 4.0, 0.0]);
        World.Step(1.0 / 60);
        Assert.True(World.VelocityOf(1)!.Skip(3).All(v => Math.Abs(v) < 1e-6));
        Assert.Equal(0.0, World.TransformOf(1)![0], 1e-6);
    }

    [Fact]
    public void AssetOwnershipAndSnapshotsRetainCompoundHulls()
    {
        World.LayHull(3, Cube(0.5));
        Assert.True(World.LayCompound(7, [new ShapePart(Shape.Hull(3))]));
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);
        Assert.False(World.DropHull(3));
        Assert.False(World.DropCompound(7));
        using var snapshot = World.Snapshot();
        World.Remove(1);
        Assert.True(World.DropCompound(7));
        Assert.True(World.DropHull(3));
        using var other = new Physics();
        other.Restore(snapshot);
        snapshot.Dispose();
        Assert.Equal(new ulong[] { 1 }, other.Overlap([0.0, 0.0, 0.0]));
        other.Add(2, Shape.Compound(7), PhysicsMotion.Fixed, [3.0, 0.0, 0.0]);
        Assert.Equal(new ulong[] { 2 }, other.Overlap([3.0, 0.0, 0.0]));
    }

    [Fact]
    public void RestoringAMovingCompoundReproducesItsPosesAndVelocities()
    {
        Floor();
        World.LayCompound(7, Pillars());
        World.Add(2, Shape.Compound(7), at: [0.0, 4.0, 0.0]);
        Run(50.0 / 60);
        using var saved = World.Snapshot();
        Run(50.0 / 60);
        var pose = World.TransformOf(2)!.ToArray();
        var velocity = World.VelocityOf(2)!.ToArray();
        World.Restore(saved);
        Run(50.0 / 60);
        Assert.Equal(pose, World.TransformOf(2));
        Assert.Equal(velocity, World.VelocityOf(2));
    }

    [Fact]
    public void ScaledCapsulesAndCylindersKeepTheirCurvedShapeAndFlatEnds()
    {
        World.LayCompound(7, [new ShapePart(Shape.Capsule(0.25, 0.5), scale: [2.0, 1.0, 0.5])]);
        World.LayCompound(8, [new ShapePart(Shape.Cylinder(0.5, 1.0), scale: [2.0, 1.0, 0.5])]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);
        World.Add(2, Shape.Compound(8), PhysicsMotion.Fixed, [4.0, 0.0, 0.0]);
        Assert.Equal(2.25, World.Cast([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 5.0)!.Distance, 0.002);
        var cap = World.Cast([4.0, 3.0, 0.0], [0.0, -1.0, 0.0], 5.0)!;
        Assert.Equal(2.0, cap.Distance, 0.002);
        Assert.Equal(1.0, cap.Normal[1], 0.002);
    }

    [Fact]
    public void ScaledCompoundGeometryRestsOnAHeightField()
    {
        World.LayGround(1, [0.0, 0.0, 0.0, 0.0], 2, 2, spacing: 20.0, at: [-10.0, 0.0, -10.0]);
        World.LayCompound(7, [new ShapePart(Shape.Sphere(1.0), scale: [2.0, 1.0, 0.5])]);
        World.Add(2, Shape.Compound(7), at: [0.0, 3.0, 0.0]);
        RunFour();
        Assert.Equal(1.0, HeightOf(2), 0.03);
        Assert.True(Math.Abs(FallSpeedOf(2)) < 0.01);
    }

    [Fact]
    public void CompoundTriggersReportTheirPartsWhileGapsStayOutside()
    {
        World.LayCompound(7, Pillars());
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed, trigger: true);
        World.Add(2, Shape.Sphere(0.2), at: [0.0, 0.0, 0.0]);
        World.Step(1.0 / 60);
        Assert.DoesNotContain(World.Events, e => e.Kind == PhysicsEventKind.Entered);
        World.Place(2, [2.0, 0.0, 0.0]);
        World.Step(1.0 / 60);
        var entered = World.Events.Single(e => e.Kind == PhysicsEventKind.Entered);
        Assert.Equal(new ulong[] { 1, 2 }, new[] { entered.A, entered.B });
        Assert.Equal(new ulong[] { 1 }, World.Overlap([2.0, 0.0, 0.0], triggers: true));
        Assert.Empty(World.Contacts);
    }

    [Fact]
    public void InvalidPartsAreRefusedBeforeRegistration()
    {
        Assert.False(World.LayCompound(0, Pillars()));
        Assert.False(World.LayCompound(7, []));
        Assert.False(World.LayCompound(7, [new ShapePart(Shape.Box(0.0, 1.0, 1.0))]));
        Assert.False(World.LayCompound(7, [new ShapePart(Shape.Hull(99))]));
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Plane(0.0, 1.0, 0.0)));
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), scale: [1.0, 0.0, 1.0]));
        Assert.True(World.LayCompound(7, Pillars()));
        Assert.False(World.LayCompound(7, Pillars()));
    }
}
