using Orblit.VectorMath;

namespace Orblit.Physics.Scene.Tests;

public sealed class ScenePoseAddedTests
{
    private static float[] Turned(double yaw, double x = 0, double y = 0, double z = 0)
    {
        var turn = Quaternion.AxisAngle(new Vector3(0, 1, 0), Doubles.Radians(yaw));
        return [(float)x, (float)y, (float)z, (float)turn.X, (float)turn.Y, (float)turn.Z, (float)turn.W];
    }

    private static float[] Blended(float[] from, float[] to, double alpha)
    {
        var output = new float[7];
        ScenePose.BlendPose(from, to, alpha, output);
        return output;
    }

    [Fact]
    public void ATurnAndItsNegativeAreOneTurnSoBlendingThemGivesThatTurn()
    {
        var turn = Turned(40);
        var negative = turn.Select((v, i) => i < 3 ? v : -v).ToArray();

        var middle = Blended(turn, negative, 0.5);

        for (var i = 3; i < 7; i++)
        {
            Assert.Equal(turn[i], middle[i], 1e-6);
        }
    }

    [Fact]
    public void ABlendToTheNegativeOfATurnFollowsTheShortArc()
    {
        var to = Turned(60);
        var negative = to.Select((v, i) => i < 3 ? v : -v).ToArray();

        var quarter = Blended(Turned(0), negative, 0.25);

        // A quarter of the way from 0 to 60 degrees is 15, and a quaternion holds half the angle.
        var half = Doubles.Radians(7.5);
        Assert.Equal(Math.Sin(half), quarter[4], 1e-6);
        Assert.Equal(Math.Cos(half), quarter[6], 1e-6);
    }

    [Fact]
    public void ATurnHalfwayAroundBlendsTowardItFromTheNearSide()
    {
        var quarter = Blended(Turned(0), Turned(180), 0.25);

        Assert.Equal(Math.Sin(Doubles.Radians(22.5)), quarter[4], 1e-6);
        Assert.Equal(Math.Cos(Doubles.Radians(22.5)), quarter[6], 1e-6);
    }

    [Fact]
    public void TwoTurnsAlmostTheSameStillBlendToAUnitTurn()
    {
        var middle = Blended(Turned(0), Turned(3), 0.5);

        var length = Math.Sqrt(middle.Skip(3).Sum(v => (double)v * v));
        Assert.Equal(1.0, length, 1e-6);
    }

    [Fact]
    public void ABlendDoesNotTouchTheFloatsPastTheSeventh()
    {
        var output = new float[8];
        output[7] = 5;

        ScenePose.BlendPose(Turned(0, x: 1, y: 2, z: 3), Turned(30, x: 3, y: 4, z: 5), 0.5, output);

        Assert.Equal(5f, output[7]);
        Assert.Equal([2f, 3f, 4f], output[..3]);
    }

    [Fact]
    public void ASizeIsMadePositiveInEveryAxis()
    {
        var size = ScenePose.Absolute(new Vector3(-1, -2, -3));

        Assert.Equal([1.0, 2.0, 3.0], [size.X, size.Y, size.Z]);
    }
}
