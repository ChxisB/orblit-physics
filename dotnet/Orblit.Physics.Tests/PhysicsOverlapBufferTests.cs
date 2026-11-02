namespace Orblit.Physics.Tests;

/// <summary>Overlap written into a buffer the caller owns, so a query every frame makes no garbage.</summary>
public class PhysicsOverlapBufferTests : PhysicsTestBase
{
    private static readonly double[] Here = [1.2, 0.0, 0.0];

    private void TwoCrates()
    {
        World.Add(1, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed);
        World.Add(2, Shape.Sphere(0.5), PhysicsMotion.Fixed, [1.4, 0.0, 0.0]);
    }

    [Fact]
    public void TheBufferHoldsWhatTheListWouldHaveHeld()
    {
        TwoCrates();
        Span<ulong> found = stackalloc ulong[8];

        var count = World.OverlapInto(Here, found, Shape.Sphere(0.3));

        Assert.Equal(2, count);
        Assert.Equal(World.Overlap(Here, Shape.Sphere(0.3)).Order(), found[..count].ToArray().Order());
    }

    [Fact]
    public void ABufferTooShortHoldsAsManyAsFit()
    {
        TwoCrates();
        Span<ulong> found = stackalloc ulong[1];

        Assert.Equal(1, World.OverlapInto(Here, found, Shape.Sphere(0.3)));
    }

    [Fact]
    public void ABufferWithNoRoomFindsNothing()
    {
        TwoCrates();

        Assert.Equal(0, World.OverlapInto(Here, Span<ulong>.Empty, Shape.Sphere(0.3)));
    }

    [Fact]
    public void WhatTheBufferHeldBeforeIsLeftBeyondTheCount()
    {
        TwoCrates();
        Span<ulong> found = [99, 99, 99, 99];

        var count = World.OverlapInto(Here, found, Shape.Sphere(0.3), ignore: 1);

        Assert.Equal(1, count);
        Assert.Equal(2UL, found[0]);
        Assert.Equal(99UL, found[1]);
    }

    [Fact]
    public void ABufferOfTheWorldsOwnQueriesTakesTheSameOptions()
    {
        World.Add(1, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed, trigger: true);
        Span<ulong> found = stackalloc ulong[4];

        Assert.Equal(0, World.OverlapInto([0.0, 0.0, 0.0], found));
        Assert.Equal(1, World.OverlapInto([0.0, 0.0, 0.0], found, triggers: true));
    }

    [Fact]
    public void ANullShapeStillMeansAPointForTheListForm()
    {
        TwoCrates();

        // A literal null for the shape has to keep binding to the list form.
        Assert.Equal(1UL, Assert.Single(World.Overlap([0.1, 0.0, 0.0], null)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-3)]
    public void AListWithNoRoomFindsNothing(int limit)
    {
        TwoCrates();

        Assert.Empty(World.Overlap(Here, Shape.Sphere(0.3), limit: limit));
    }

    [Fact]
    public void TheListHoldsTheDefaultLimitWhenMoreBodiesOverlap()
    {
        for (ulong id = 1; id <= Physics.DefaultOverlapLimit + 8; id++)
        {
            World.Add(id, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed);
        }

        Assert.Equal(Physics.DefaultOverlapLimit, World.Overlap([0.0, 0.0, 0.0]).Count);
    }

    [Fact]
    public void ADisposedWorldRefusesTheQuery()
    {
        var gone = new Physics();
        gone.Dispose();

        Assert.Throws<InvalidOperationException>(() => gone.OverlapInto([0.0, 0.0, 0.0], new ulong[4]));
    }

    [Fact]
    public void AQueryIntoABufferMakesNoGarbageWhereTheListMakesSome()
    {
        TwoCrates();
        var shape = Shape.Sphere(0.3);
        var found = new ulong[8];
        World.OverlapInto(Here, found, shape);
        World.Overlap(Here, shape);

        var buffered = Allocated(() => World.OverlapInto(Here, found, shape));
        var listed = Allocated(() => World.Overlap(Here, shape));

        Assert.Equal(0, buffered);
        Assert.True(listed > 0, "the list overload is the one that allocates");
    }

    private static long Allocated(Action query)
    {
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 1000; i++)
        {
            query();
        }

        return GC.GetAllocatedBytesForCurrentThread() - before;
    }
}
