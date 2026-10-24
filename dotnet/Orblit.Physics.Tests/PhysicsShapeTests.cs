namespace Orblit.Physics.Tests;

public class PhysicsShapeTests : PhysicsTestBase
{
    [Fact]
    public void ACylinderCrossesAsACylinderFlatAtBothEnds()
    {
        Floor();
        // 0.3 of radius and 0.5 of half length, on its end. A capsule of the
        // same two numbers would rest at 0.8, and a sphere at 0.3.
        World.Add(2, Shape.Cylinder(0.3, 0.5), at: [0.0, 2.0, 0.0]);
        Run(3);

        Assert.Equal(0.5, HeightOf(2), 0.02);
        Assert.True(World.Asleep(2));
    }

    [Fact]
    public void AHullIsCookedOnceAndSharedByTheBodiesThatNameIt()
    {
        Floor();
        Assert.True(World.LayHull(7, Cube(0.5)));
        World.Add(2, Shape.Hull(7), at: [0.0, 2.0, 0.0]);
        World.Add(3, Shape.Hull(7), at: [3.0, 3.0, 0.0]);
        Run(3);

        Assert.Equal(0.5, HeightOf(2), 0.03);
        Assert.Equal(0.5, HeightOf(3), 0.03);
    }

    [Fact]
    public void AHullRefusesPointsItCannotMakeASolidOf()
    {
        bool Lay(IReadOnlyList<double> points, ulong id = 7) => World.LayHull(id, points);

        Assert.False(Lay(Cube(0.5), id: 0));
        Assert.False(Lay([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]));
        Assert.False(Lay(Cube(0.5).Take(11).ToArray()));
        Assert.False(Lay([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0]));
        Assert.False(Lay([.. Cube(0.5).Take(21), double.NaN]));
        Assert.True(Lay(Cube(0.5)));
        Assert.False(Lay(Cube(1.0)));
    }

    [Fact]
    public void ABodyNamingAHullThatWasNotLaidIsNotMade()
    {
        World.Add(2, Shape.Hull(7), at: [0.0, 2.0, 0.0]);
        Assert.False(World.Alive(2));
        Assert.Equal(0, World.Count);

        // Laid afterwards does not bring it back, and does not stop a new one.
        World.LayHull(7, Cube(0.5));
        Assert.False(World.Alive(2));
        World.Add(2, Shape.Hull(7), at: [0.0, 2.0, 0.0]);
        Assert.True(World.Alive(2));
    }

    [Fact]
    public void AHullStaysUntilTheLastBodyThatUsesItHasGone()
    {
        World.LayHull(7, Cube(0.5));
        World.Add(2, Shape.Hull(7), at: [0.0, 2.0, 0.0]);
        Assert.False(World.DropHull(7));
        Assert.False(World.DropHull(8));

        World.Remove(2);
        Assert.True(World.DropHull(7));
        Assert.False(World.DropHull(7));
        Assert.True(World.LayHull(7, Cube(1.0)));
    }

    [Fact]
    public void ACastMeetsAHullAndACylinderAndCanBeOne()
    {
        World.LayHull(7, Cube(0.5));
        World.Add(2, Shape.Hull(7), PhysicsMotion.Fixed, [0.0, 0.5, 0.0]);
        World.Add(3, Shape.Cylinder(0.5, 0.5), PhysicsMotion.Fixed, [10.0, 0.5, 0.0]);

        var onHull = World.Cast([0.1, 5.0, 0.2], [0.0, -1.0, 0.0], 20.0)!;
        Assert.Equal(2UL, onHull.Body);
        Assert.Equal(4.0, onHull.Distance, 0.01);
        Assert.Equal(1.0, onHull.Normal[1], 0.01);

        var onCylinder = World.Cast([10.3, 0.5, -5.0], [0.0, 0.0, 1.0], 20.0)!;
        Assert.Equal(3UL, onCylinder.Body);
        Assert.Equal(4.6, onCylinder.Distance, 0.01);

        // A hull cast onto a hull: its bottom starts at 4.5 and meets the top at 1.
        var sweptHull = World.Cast([0.0, 5.0, 0.0], [0.0, -1.0, 0.0], 20.0, Shape.Hull(7))!;
        Assert.Equal(2UL, sweptHull.Body);
        Assert.Equal(3.5, sweptHull.Distance, 0.01);

        // A hull that was not laid meets nothing.
        Assert.Null(World.Cast([0.0, 5.0, 0.0], [0.0, -1.0, 0.0], 20.0, Shape.Hull(99)));
        Assert.Equal(new ulong[] { 2 }, World.Overlap([0.1, 0.5, 0.1], Shape.Sphere(0.2)));
    }

    [Fact]
    public void ASnapshotKeepsAHullBodyAndAWorldRestoredFromIt()
    {
        Floor();
        World.LayHull(7, Cube(0.5));
        World.Add(2, Shape.Hull(7), at: [0.0, 2.0, 0.0]);
        Run(0.5);
        var falling = HeightOf(2);
        using var snapshot = World.Snapshot();
        Run(2.5);
        var settled = HeightOf(2);

        World.Restore(snapshot);
        Assert.Equal(falling, HeightOf(2));
        Run(2.5);
        Assert.Equal(settled, HeightOf(2));
    }
}
