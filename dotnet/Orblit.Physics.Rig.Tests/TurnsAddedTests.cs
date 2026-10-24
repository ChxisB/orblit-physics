using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public class TurnsAddedTests
{
    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z), angle);

    [Fact]
    public void NoTurnReadsAsNothing()
    {
        Assert.All(Turns.Angles(Quaternion.Identity()), angle => Assert.Equal(0, angle, 1e-12));
    }

    [Fact]
    public void AHalfTurnAboutYIsAllSwing()
    {
        // x and w are both nought, so there is no twist to take out.
        var angles = Turns.Angles(new Quaternion(0, 1, 0, 0));

        Assert.Equal(0, angles[0], 1e-9);
        Assert.Equal(Math.PI, angles[1], 1e-9);
        Assert.Equal(0, angles[2], 1e-9);
    }

    [Fact]
    public void AHalfTurnAboutXIsAllTwist()
    {
        var angles = Turns.Angles(new Quaternion(1, 0, 0, 0));

        Assert.Equal(Math.PI, angles[0], 1e-9);
        Assert.Equal(0, angles[1], 1e-9);
        Assert.Equal(0, angles[2], 1e-9);
    }

    [Fact]
    public void ATinySwingIsReadAsTwiceItsVectorPart()
    {
        var angles = Turns.Angles(About(0, 0, 1, 1e-10));

        Assert.Equal(1e-10, angles[2], 1e-15);
    }

    [Fact]
    public void BetweenGivesTheAxisScaledByTheAngle()
    {
        var turn = Turns.Between(About(0, 1, 0, 0.3), About(0, 1, 0, 0.8));

        Assert.Equal(0, turn.X, 1e-12);
        Assert.Equal(0.5, turn.Y, 1e-12);
        Assert.Equal(0, turn.Z, 1e-12);
    }

    [Fact]
    public void BetweenTakesTheShorterWayWhateverSignTheQuaternionHas()
    {
        var after = About(1, 0, 0, 0.6);
        var negated = new Quaternion(-after.X, -after.Y, -after.Z, -after.W);

        var turn = Turns.Between(Quaternion.Identity(), negated);

        Assert.Equal(0.6, turn.X, 1e-12);
    }

    [Fact]
    public void BetweenAVeryNearPairIsTwiceTheVectorPart()
    {
        var turn = Turns.Between(Quaternion.Identity(), About(0, 0, 1, 1e-10));

        Assert.Equal(1e-10, turn.Z, 1e-15);
    }

    [Fact]
    public void BetweenTwoEqualTurnsIsNothing()
    {
        var turn = Turns.Between(About(1, 2, 3, 0.4), About(1, 2, 3, 0.4));

        Assert.Equal(0, turn.Length, 1e-12);
    }

    [Fact]
    public void SlerpStartsAtTheFirstTurnAndEndsAtTheSecond()
    {
        var from = About(0, 1, 0, 0.2);
        var to = About(0, 1, 0, 1.6);

        Assert.Equal(0, Turns.Between(from, Turns.Slerp(from, to, 0)).Length, 1e-12);
        Assert.Equal(0, Turns.Between(to, Turns.Slerp(from, to, 1)).Length, 1e-12);
    }

    [Fact]
    public void SlerpTakesAnEvenStepAlongTheArc()
    {
        var from = About(0, 1, 0, 0.2);
        var to = About(0, 1, 0, 1.6);

        var quarter = Turns.Slerp(from, to, 0.25);

        Assert.Equal(0.35, Turns.Between(from, quarter).Y, 1e-12);
        Assert.Equal(1, Math.Sqrt((quarter.X * quarter.X) + (quarter.Y * quarter.Y) + (quarter.Z * quarter.Z) + (quarter.W * quarter.W)), 1e-12);
    }

    [Fact]
    public void SlerpGoesTheShorterWayWhenTheSecondTurnIsTheLongOne()
    {
        var from = About(0, 1, 0, 0.2);
        var to = About(0, 1, 0, 1.6);
        var negated = new Quaternion(-to.X, -to.Y, -to.Z, -to.W);

        var mid = Turns.Slerp(from, negated, 0.5);

        Assert.Equal(0.7, Turns.Between(from, mid).Y, 1e-12);
    }

    [Fact]
    public void SlerpBetweenNearEqualTurnsStaysAUnitTurnBetweenThem()
    {
        var from = About(1, 0, 0, 0.3);
        var to = About(1, 0, 0, 0.3004);

        var mid = Turns.Slerp(from, to, 0.5);

        Assert.Equal(0.0002, Turns.Between(from, mid).X, 1e-9);
        Near(1, Math.Sqrt((mid.X * mid.X) + (mid.Y * mid.Y) + (mid.Z * mid.Z) + (mid.W * mid.W)), 1e-12);
    }

    [Fact]
    public void SlerpBetweenNearOppositeSignsOfTheSameTurnStaysPut()
    {
        var from = About(1, 0, 0, 0.3);
        var negated = new Quaternion(-from.X, -from.Y, -from.Z, -from.W);

        var mid = Turns.Slerp(from, negated, 0.5);

        Assert.Equal(0, Turns.Between(from, mid).Length, 1e-9);
    }
}
