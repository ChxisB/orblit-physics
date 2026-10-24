using System.Runtime.InteropServices;

namespace Orblit.Physics.Tests;

public class PhysicsShapeAddedTests : PhysicsTestBase
{
    private static readonly double[] Patches =
    [
        -5.0, 0.0, -1.0, -3.0, 0.0, -1.0, -4.0, 0.0, 1.0,
        -1.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 1.0,
        3.0, 0.0, -1.0, 5.0, 0.0, -1.0, 4.0, 0.0, 1.0,
    ];

    /// <summary>Casts straight down from above <paramref name="x"/>, 0, 0 and answers how far it went.</summary>
    private double? DropOnto(double x, double height = 3.0) =>
        World.Cast([x, height, 0.0], [0.0, -1.0, 0.0], 2 * height)?.Distance;

    private static ShapePart Turned(double[] rotation) =>
        new(Shape.Box(1.0, 0.2, 0.5), rotation: rotation);

    [Fact]
    public void ItsPartIsTheSizeTheEngineReads() =>
        Assert.Equal(80, Marshal.SizeOf<NativePart>());

    [Fact]
    public void AShapeNamesItsAssetAndItsKindAsTheEngineNumbersThem()
    {
        Assert.Equal((7U, 5UL), (Shape.Hull(5).Kind, Shape.Hull(5).Asset));
        Assert.Equal((8U, 6UL), (Shape.Compound(6).Kind, Shape.Compound(6).Asset));
        Assert.Equal((9U, 7UL), (Shape.Mesh(7).Kind, Shape.Mesh(7).Asset));
        Assert.Equal(0UL, Shape.Sphere(1.0).Asset);
    }

    [Fact]
    public void ABodyQueuedBeforeAHullIsLaidDoesNotGetIt()
    {
        World.Add(2, Shape.Hull(7), at: [0.0, 2.0, 0.0]);
        World.LayHull(7, Cube(0.5));

        Assert.False(World.Alive(2));
    }

    [Fact]
    public void ABodyQueuedBeforeAMeshIsLaidDoesNotGetIt()
    {
        World.Add(2, Shape.Mesh(7), PhysicsMotion.Fixed);
        World.LayMesh(7, Patches, [0, 2, 1]);

        Assert.False(World.Alive(2));
    }

    [Fact]
    public void ABodyQueuedBeforeACompoundIsLaidDoesNotGetIt()
    {
        World.Add(2, Shape.Compound(7), PhysicsMotion.Fixed);
        World.LayCompound(7, [new ShapePart(Shape.Sphere(1.0))]);

        Assert.False(World.Alive(2));
    }

    [Fact]
    public void ABodyQueuedBeforeAMeshOrACompoundIsDroppedKeepsItFromGoing()
    {
        World.LayMesh(7, Patches, [0, 2, 1]);
        World.LayCompound(8, [new ShapePart(Shape.Sphere(1.0))]);
        World.Add(2, Shape.Mesh(7), PhysicsMotion.Fixed);
        Assert.False(World.DropMesh(7));
        World.Add(3, Shape.Compound(8), PhysicsMotion.Fixed);
        Assert.False(World.DropCompound(8));

        World.Remove(2);
        World.Remove(3);
        Assert.True(World.DropMesh(7));
        Assert.True(World.DropCompound(8));
    }

    [Fact]
    public void AHullNeedsFourPointsAtTheLeast()
    {
        Assert.False(World.LayHull(7, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]));
        Assert.True(World.LayHull(7, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]));
    }

    [Fact]
    public void AMeshRefusesWhatItCannotCook()
    {
        Assert.False(World.LayMesh(0, Patches, [0, 2, 1]));
        Assert.False(World.LayMesh(7, Patches, [0, 2, -1]));
        Assert.False(World.LayMesh(7, Patches, [0, 2, 9]));
        Assert.False(World.LayMesh(7, Patches[..^1], [0, 2, 1]));
        Assert.False(World.LayMesh(7, Patches, [0, 2, 1, 3]));
        Assert.False(World.LayMesh(7, [.. Patches[..8], double.NaN], [0, 2, 1]));
        Assert.False(World.LayMesh(7, [.. Patches[..8], double.PositiveInfinity], [0, 2, 1]));
        Assert.False(World.LayMesh(7, [], []));
        Assert.True(World.LayMesh(7, Patches, [0, 2, 1]));
        Assert.False(World.LayMesh(7, Patches, [0, 2, 1]));
    }

    [Fact]
    public void EveryTriangleOfAMeshIsLaid()
    {
        Assert.True(World.LayMesh(7, Patches, [0, 2, 1, 3, 5, 4, 6, 8, 7]));
        World.Add(1, Shape.Mesh(7), PhysicsMotion.Fixed);

        Assert.Equal(3.0, DropOnto(-4.0));
        Assert.Equal(3.0, DropOnto(0.0));
        Assert.Equal(3.0, DropOnto(4.0));
        Assert.Null(DropOnto(-2.0));
    }

    [Fact]
    public void AnUnknownMeshOrCompoundIsNotDropped()
    {
        Assert.False(World.DropMesh(99));
        Assert.False(World.DropCompound(99));
    }

    [Fact]
    public void ACompoundTakesSixtyFourPartsAndNoMore()
    {
        ShapePart[] parts(int count) => [.. Enumerable.Range(0, count).Select(i => new ShapePart(Shape.Sphere(0.1), at: [i, 0.0, 0.0]))];

        Assert.False(World.LayCompound(7, parts(65)));
        Assert.True(World.LayCompound(7, parts(64)));
    }

    [Fact]
    public void ACompoundRefusesAScaleItCannotUse()
    {
        ShapePart[] one = [new ShapePart(Shape.Sphere(1.0))];

        Assert.False(World.LayCompound(7, one, [0.0, 1.0, 1.0]));
        Assert.False(World.LayCompound(7, one, [1.0, -1.0, 1.0]));
        Assert.Equal("scale", Assert.Throws<ArgumentException>(() => World.LayCompound(7, one, [1.0, 1.0])).ParamName);
        Assert.Throws<ArgumentException>(() => World.LayCompound(7, one, [1.0, double.NaN, 1.0]));
        Assert.True(World.LayCompound(7, one, [1.0, 2.0, 3.0]));
    }

    [Fact]
    public void AnOuterScaleStretchesAPartsOffsetAsWellAsItsSize()
    {
        World.LayCompound(7, [new ShapePart(Shape.Box(0.2, 0.2, 0.2), at: [1.0, 0.0, 0.0])], [3.0, 1.0, 1.0]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        Assert.Equal(new ulong[] { 1 }, World.Overlap([3.0, 0.0, 0.0], Shape.Sphere(0.1)));
        Assert.Empty(World.Overlap([1.0, 0.0, 0.0], Shape.Sphere(0.1)));
        Assert.Equal(new ulong[] { 1 }, World.Overlap([3.5, 0.0, 0.0], Shape.Sphere(0.1)));
    }

    [Fact]
    public void AnOffsetPartInAllThreeWaysIsPlacedWhereItWasPut()
    {
        World.LayCompound(7, [new ShapePart(Shape.Sphere(0.2), at: [1.0, 2.0, 3.0])]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        Assert.Equal(new ulong[] { 1 }, World.Overlap([1.0, 2.0, 3.0], Shape.Sphere(0.1)));
        Assert.Empty(World.Overlap([2.0, 1.0, 3.0], Shape.Sphere(0.1)));
        Assert.Empty(World.Overlap([1.0, 3.0, 2.0], Shape.Sphere(0.1)));
        Assert.Empty(World.Overlap([3.0, 2.0, 1.0], Shape.Sphere(0.1)));
    }

    [Theory]
    [InlineData(0, 1.0, 3.0 - 1.0, 3.0 - 0.5, 3.0 - 0.2)]
    [InlineData(1, 1.0, 3.0 - 0.5, 3.0 - 0.2, 3.0 - 1.0)]
    [InlineData(2, 1.0, 3.0 - 0.2, 3.0 - 1.0, 3.0 - 0.5)]
    [InlineData(0, 4.0, 3.0 - 1.0, 3.0 - 0.5, 3.0 - 0.2)]
    [InlineData(1, 4.0, 3.0 - 0.5, 3.0 - 0.2, 3.0 - 1.0)]
    [InlineData(2, 4.0, 3.0 - 0.2, 3.0 - 1.0, 3.0 - 0.5)]
    public void APartTurnedAQuarterAboutAnAxisSwapsTheOtherTwoExtentsWhateverTheLengthOfTheTurn(
        int axis, double length, double ex, double ey, double ez)
    {
        var half = length * Math.Sqrt(0.5);
        double[] turn = [axis == 0 ? half : 0.0, axis == 1 ? half : 0.0, axis == 2 ? half : 0.0, half];
        World.LayCompound(7, [Turned(turn)]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        Assert.Equal(ex, World.Cast([3.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 6.0)!.Distance, 0.002);
        Assert.Equal(ey, World.Cast([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 6.0)!.Distance, 0.002);
        Assert.Equal(ez, World.Cast([0.0, 0.0, 3.0], [0.0, 0.0, -1.0], 6.0)!.Distance, 0.002);
    }

    [Fact]
    public void ATurnAboutADiagonalAxisLaysTheLongSideAcrossTheOtherTwoWays()
    {
        var half = Math.Sqrt(0.5);
        World.LayCompound(7, [new ShapePart(Shape.Box(1.0, 0.1, 0.1), rotation: [0.5, 0.0, 0.5, half])]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        // The ray down the middle meets the face a tenth deep, tilted 45 degrees.
        Assert.Equal(3.0 - (0.1 / half), World.Cast([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 6.0)!.Distance, 0.002);

        // A point 0.09 along both thin sides of the box lies inside it.
        double[] inside = [0.09 * (0.5 - half), -0.09 * half, 0.09 * (0.5 + half)];
        Assert.Equal(new ulong[] { 1 }, World.Overlap(inside, Shape.Sphere(0.005)));
    }

    [Fact]
    public void AnOuterScaleStretchesEachWayOfAnOffsetByItsOwnFactor()
    {
        World.LayCompound(7, [new ShapePart(Shape.Sphere(0.2), at: [1.0, 2.0, 3.0])], [2.0, 3.0, 5.0]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        Assert.Equal(new ulong[] { 1 }, World.Overlap([2.0, 6.0, 15.0], Shape.Sphere(0.05)));
    }

    [Fact]
    public void ATurnAboutXOrYTipsAFacesNormalTheWayARightHandedTurnDoes()
    {
        var angle = Math.PI / 4;
        var s = Math.Sin(angle / 2);
        var c = Math.Cos(angle / 2);
        World.LayCompound(7, [new ShapePart(Shape.Box(1.0, 0.1, 1.0), rotation: [s, 0.0, 0.0, c])]);
        World.LayCompound(8, [new ShapePart(Shape.Box(0.1, 1.0, 1.0), rotation: [0.0, s, 0.0, c])]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);
        World.Add(2, Shape.Compound(8), PhysicsMotion.Fixed, [20.0, 0.0, 0.0]);

        var top = World.Cast([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 6.0)!;
        Assert.Equal(Math.Sqrt(0.5), top.Normal[1], 0.01);
        Assert.Equal(Math.Sqrt(0.5), top.Normal[2], 0.01);
        var side = World.Cast([23.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 6.0)!;
        Assert.Equal(Math.Sqrt(0.5), side.Normal[0], 0.01);
        Assert.Equal(-Math.Sqrt(0.5), side.Normal[2], 0.01);
    }

    [Fact]
    public void ATurnThatIsNotUnitLengthIsTakenAsTheSameTurn()
    {
        var angle = Math.PI / 4;
        double[] turn = [0.0, 0.0, 3 * Math.Sin(angle / 2), 3 * Math.Cos(angle / 2)];
        World.LayCompound(7, [new ShapePart(Shape.Box(1.0, 0.1, 0.2), rotation: turn)], [2.0, 1.0, 1.0]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        var hit = World.Cast([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 6.0)!;
        Assert.Equal(-1 / Math.Sqrt(5), hit.Normal[0], 0.01);
        Assert.Equal(2 / Math.Sqrt(5), hit.Normal[1], 0.01);
    }

    [Fact]
    public void APartScalesBeforeItTurns()
    {
        var half = Math.Sqrt(0.5);
        World.LayCompound(7, [new ShapePart(Shape.Box(1.0, 1.0, 1.0), rotation: [0.0, 0.0, half, half], scale: [3.0, 1.0, 1.0])]);
        World.Add(1, Shape.Compound(7), PhysicsMotion.Fixed);

        Assert.Equal(0.0, World.Cast([0.0, 4.0, 0.0], [0.0, -1.0, 0.0], 8.0)!.Distance - 1.0, 0.002);
        Assert.Equal(2.0, World.Cast([3.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 6.0)!.Distance, 0.002);
    }

    [Fact]
    public void APartIsMadeOfWhatItWasGiven()
    {
        double[] at = [1.0, 2.0, 3.0];
        var part = new ShapePart(Shape.Sphere(1.0), at: at);
        at[0] = 9.0;

        Assert.Equal(new[] { 1.0, 2.0, 3.0 }, part.At);
        Assert.Throws<NotSupportedException>(() => ((IList<double>)part.At).Add(1.0));
        Assert.Equal(new[] { 0.0, 0.0, 0.0, 1.0 }, new ShapePart(Shape.Sphere(1.0)).Rotation);
        Assert.Equal(new[] { 1.0, 1.0, 1.0 }, new ShapePart(Shape.Sphere(1.0)).Scale);
        Assert.Equal(new[] { 0.0, 0.0, 0.0 }, new ShapePart(Shape.Sphere(1.0)).At);
    }

    [Fact]
    public void APartRefusesWhatIsNotConvexOrNotNumbers()
    {
        Assert.Equal("shape", Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Compound(1))).ParamName);
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Mesh(1)));
        Assert.Equal("at", Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), at: [0.0, 0.0])).ParamName);
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), at: [0.0, 0.0, 0.0, 0.0]));
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), rotation: [0.0, 0.0, 0.0, 1.0, 0.0]));
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), at: [0.0, double.NaN, 0.0]));
        Assert.Equal("rotation", Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), rotation: [0.0, 0.0, 1.0])).ParamName);
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), rotation: [0.0, 0.0, 0.0, 0.0]));
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), scale: [1.0, -1.0, 1.0]));
        Assert.Throws<ArgumentException>(() => new ShapePart(Shape.Sphere(1.0), scale: [1.0, double.PositiveInfinity, 1.0]));
        Assert.Equal(1.0, new ShapePart(Shape.Hull(3), rotation: [0.0, 0.0, 0.0, -1.0]).Rotation[3] * -1);
    }

    [Fact]
    public void ADisposedWorldRefusesToLayOrDropAnyShape()
    {
        var gone = new Physics();
        gone.Dispose();

        Assert.Throws<InvalidOperationException>(() => gone.LayHull(1, Cube(0.5)));
        Assert.Throws<InvalidOperationException>(() => gone.DropHull(1));
        Assert.Throws<InvalidOperationException>(() => gone.LayMesh(1, Patches, [0, 2, 1]));
        Assert.Throws<InvalidOperationException>(() => gone.DropMesh(1));
        Assert.Throws<InvalidOperationException>(() => gone.LayCompound(1, [new ShapePart(Shape.Sphere(1.0))]));
        Assert.Throws<InvalidOperationException>(() => gone.DropCompound(1));
    }
}
