using Orblit.Rig;
using Orblit.VectorMath;
using static Orblit.Physics.Rig.Tests.Bounds;

namespace Orblit.Physics.Rig.Tests;

public sealed class TurnsFramesAddedTests
{
    private static readonly Matrix4 Moved = Matrix4.Compose(
        new Vector3(1, 0.5, -2),
        Quaternion.AxisAngle(new Vector3(1, 2, 0.5).Normalized(), 0.8),
        new Vector3(1.5, 1.5, 1.5));

    private static Quaternion About(double x, double y, double z, double angle) =>
        Quaternion.AxisAngle(new Vector3(x, y, z).Normalized(), angle);

    private static void SameTurn(Quaternion expected, Quaternion actual)
    {
        var dot = (expected.X * actual.X) + (expected.Y * actual.Y) + (expected.Z * actual.Z) + (expected.W * actual.W);
        Near(1, Math.Abs(dot), 1e-9, "turn");
    }

    private static void SameFrame(Frame expected, Frame actual)
    {
        Near(0, (expected.Position - actual.Position).Length, 1e-9, "position");
        SameTurn(expected.Rotation, actual.Rotation);
    }

    [Fact]
    public void ATurnAndItsNegativeReadAsTheSameAngles()
    {
        var q = About(0.3, 0.5, 0.8, 1.2);

        var angles = Turns.Angles(q);
        var negated = Turns.Angles(new Quaternion(-q.X, -q.Y, -q.Z, -q.W));

        for (var i = 0; i < 3; i++)
        {
            Near(angles[i], negated[i], 1e-12, "angle");
        }
    }

    [Fact]
    public void ASwingFollowedByATwistIsReadAsItsTwistAndItsSwing()
    {
        var swing = About(0, 0.6, 0.8, 0.8);
        var twist = About(1, 0, 0, 0.5);

        var angles = Turns.Angles(swing * twist);

        Near(0.5, angles[0], 1e-9, "twist");
        Near(0.48, angles[1], 1e-9, "swing about y");
        Near(0.64, angles[2], 1e-9, "swing about z");
    }

    [Fact]
    public void ATinySwingAboutYIsReadAsTwiceItsVectorPart()
    {
        Near(1e-10, Turns.Angles(About(0, 1, 0, 1e-10))[1], 1e-15, "swing about y");
    }

    [Fact]
    public void ASwingNearlyAHalfTurnKeepsItsTwistAndItsSwing()
    {
        var angles = Turns.Angles(About(0, 1, 0, Math.PI - 2e-6) * About(1, 0, 0, Math.PI / 2));

        Near(Math.PI / 2, angles[0], 1e-8, "twist");
        Near(Math.PI - 2e-6, angles[1], 1e-8, "swing about y");
        Near(0, angles[2], 1e-8, "swing about z");
    }

    [Fact]
    public void BetweenIsTheTurnThatCarriesTheFirstOntoTheSecond()
    {
        var before = About(1, 2, 3, 0.7);
        var after = About(-2, 1, 0.5, 1.9);

        var turn = Turns.Between(before, after);

        SameTurn(after, Quaternion.AxisAngle(turn.Normalized(), turn.Length) * before);
    }

    [Fact]
    public void BetweenAHalfTurnAboutYIsPositive()
    {
        Near(Math.PI, Turns.Between(Quaternion.Identity(), new Quaternion(0, 1, 0, 0)).Y, 1e-12, "half turn");
    }

    [Fact]
    public void SlerpToAHalfTurnGoesTheWayTheAxisPoints()
    {
        var mid = Turns.Slerp(Quaternion.Identity(), new Quaternion(0, 1, 0, 0), 0.5);

        Near(Math.Sqrt(0.5), mid.Y, 1e-12, "y");
        Near(Math.Sqrt(0.5), mid.W, 1e-12, "w");
    }

    [Fact]
    public void SlerpToTheLongSignOfATurnStillGoesTheShortWay()
    {
        var far = About(0, 1, 0, 1.0);

        var quarter = Turns.Slerp(Quaternion.Identity(), new Quaternion(-far.X, -far.Y, -far.Z, -far.W), 0.25);

        SameTurn(About(0, 1, 0, 0.25), quarter);
    }

    [Fact]
    public void SlerpKeepsToTheArcWhenTheTurnsAreExactlyAtTheEdgeOfTheStraightBlend()
    {
        var to = new Quaternion(Math.Sqrt(1 - (0.9995 * 0.9995)), 0, 0, 0.9995);
        var quarterOfTheWay = Math.Acos(0.9995) / 4;

        var turn = Turns.Slerp(Quaternion.Identity(), to, 0.25);

        Near(Math.Sin(quarterOfTheWay), turn.X, 1e-9, "x");
        Near(Math.Cos(quarterOfTheWay), turn.W, 1e-9, "w");
    }

    [Fact]
    public void SlerpBetweenNearEqualTurnsMovesTheAskedShareOfTheWay()
    {
        var axis = new Vector3(1, 2, 3).Normalized();
        var from = About(1, 2, 3, 0.1);
        var to = About(1, 2, 3, 0.14);

        var turn = Turns.Between(from, Turns.Slerp(from, to, 0.75));

        // A straight blend between turns this close is out by less than a millionth.
        Near(0.03, turn.Length, 1e-6, "angle");
        Near(0, (turn.Normalized() - axis).Length, 1e-6, "axis");
    }

    [Fact]
    public void ATurnedFrameFollowedByAnotherHasBothTurnsAndTheSecondsPositionTurned()
    {
        var outer = new Frame(new Vector3(1, 2, 3), About(0, 1, 0, 0.5));
        var local = new Frame(new Vector3(0.5, 0, -1), About(1, 0, 0, 0.3));

        var both = outer.Then(local);

        SameTurn(About(0, 1, 0, 0.5) * About(1, 0, 0, 0.3), both.Rotation);
        var matrix = Matrix4.Compose(outer.Position, outer.Rotation, new Vector3(1, 1, 1));
        Near(0, (matrix.Transformed3(local.Position) - both.Position).Length, 1e-9, "position");
    }

    [Fact]
    public void AFrameSeenFromAnotherAndPutBackIsTheFrameItWas()
    {
        var outer = new Frame(new Vector3(1, 2, 3), About(0.2, 1, 0.4, 0.9));
        var inner = new Frame(new Vector3(-2, 0.5, 4), About(1, -1, 0.3, 1.4));

        SameFrame(inner, outer.Then(inner.Within(outer)));
    }

    [Fact]
    public void ScalingAFrameScalesItsPositionAndNotItsTurn()
    {
        var frame = new Frame(new Vector3(1, -2, 3), About(0, 1, 1, 0.7));

        var scaled = frame.Scaled(2);

        Near(0, (new Vector3(2, -4, 6) - scaled.Position).Length, 1e-12, "position");
        SameTurn(frame.Rotation, scaled.Rotation);
    }

    [Fact]
    public void AModelSpaceSendsAFrameToTheWorldWithTheModelsScaleAndTurn()
    {
        var space = new ModelSpace(new ModifierFrame(Harness.Dt, Moved));
        var model = new Frame(new Vector3(0.3, 1.1, -0.4), About(1, 1, 0, 0.6));

        var world = space.ToWorld(model);

        SameFrame(
            Frame.FromMatrix(Moved.Multiplied(Matrix4.Compose(model.Position, model.Rotation, new Vector3(1, 1, 1)))),
            world);
        SameFrame(model, space.ToModel(world));
        Near(1.5, space.Scale, 1e-9, "scale");
    }
}
