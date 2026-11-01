using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene.Tests;

public sealed class CompoundPartsAddedTests
{
    private const uint Sphere = 1;
    private const uint Box = 2;
    private const uint Capsule = 4;
    private const uint Cylinder = 6;
    private const uint Hull = 7;

    private static ShapePart PartOf(BodyPart part) => CompoundParts.PartOf(part, _ => 77);

    private static void AssertShape(uint kind, double x, double y, double z, Shape shape)
    {
        Assert.Equal(kind, shape.Kind);
        Assert.Equal(x, shape.X);
        Assert.Equal(y, shape.Y);
        Assert.Equal(z, shape.Z);
    }

    [Fact]
    public void ABoxPartTakesHalfOfEachSizeInItsOwnAxis()
    {
        var part = PartOf(new BodyPart(BodyShape.Box, size: new Vector3(2, 4, 6)));

        AssertShape(Box, 1, 2, 3, part.Shape);
    }

    [Fact]
    public void ASpherePartTakesItsRadius()
    {
        var part = PartOf(new BodyPart(BodyShape.Sphere, radius: 0.7));

        AssertShape(Sphere, 0.7, 0, 0, part.Shape);
    }

    [Fact]
    public void ACapsulePartKeepsItsStraightLengthBetweenTheEndCaps()
    {
        var part = PartOf(new BodyPart(BodyShape.Capsule, radius: 0.25, height: 3));

        Assert.Equal(Capsule, part.Shape.Kind);
        Assert.Equal(0.25, part.Shape.X);
        Assert.Equal(1.25, part.Shape.Y);
    }

    [Fact]
    public void ACapsulePartShorterThanItsCapsHasNoStraightLength()
    {
        var part = PartOf(new BodyPart(BodyShape.Capsule, radius: 0.75, height: 1));

        Assert.Equal(Capsule, part.Shape.Kind);
        Assert.Equal(0.75, part.Shape.X);
        Assert.Equal(0.0, part.Shape.Y);
    }

    [Fact]
    public void ACylinderPartTakesHalfItsHeight()
    {
        var part = PartOf(new BodyPart(BodyShape.Cylinder, radius: 0.4, height: 3));

        Assert.Equal(Cylinder, part.Shape.Kind);
        Assert.Equal(0.4, part.Shape.X);
        Assert.Equal(1.5, part.Shape.Y);
    }

    [Fact]
    public void AHullPartAsksForItsHullByThePointsItWasGiven()
    {
        IReadOnlyList<double>? asked = null;
        var points = new[] { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

        var part = CompoundParts.PartOf(
            new BodyPart(BodyShape.Hull, hull: points),
            hull =>
            {
                asked = hull;
                return 41;
            });

        Assert.Equal(Hull, part.Shape.Kind);
        Assert.Equal(41UL, part.Shape.Asset);
        Assert.Equal(points, asked);
    }

    [Fact]
    public void APartKeepsItsCentreRotationAndScaleInTheOrderTheyAreWritten()
    {
        var turn = Quaternion.AxisAngle(new Vector3(0, 0, 1), 0.5);
        var part = PartOf(new BodyPart(
            BodyShape.Box,
            centre: new Vector3(1, 2, 3),
            rotation: turn,
            scale: new Vector3(4, 5, 6)));

        Assert.Equal([1.0, 2.0, 3.0], part.At);
        Assert.Equal([turn.X, turn.Y, turn.Z, turn.W], part.Rotation);
        Assert.Equal([4.0, 5.0, 6.0], part.Scale);
    }

    [Fact]
    public void APartThatIsNotConvexIsRefused()
    {
        Assert.Throws<ArgumentException>(() => PartOf(new BodyPart(BodyShape.Plane)));
    }
}
