using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

/// <summary>
/// The sheets, steps and checks the scenarios share. It holds no tests of its own.
/// </summary>
public abstract class SoftTestBase
{
    protected const int Columns = 20;
    protected const double Spacing = 0.05;
    protected const double Mass = 0.01;
    protected const double Dt = 1.0 / 60;

    /// <summary>
    /// Steps <paramref name="body"/> for the given seconds, <paramref name="dt"/>
    /// at a time, and calls <paramref name="each"/> after every step.
    /// </summary>
    protected static void Run(SoftBody body, double seconds, double dt = Dt, Action? each = null)
    {
        var steps = (int)Math.Round(seconds / dt, MidpointRounding.AwayFromZero);
        for (var n = 0; n < steps; n++)
        {
            body.Step(dt);
            each?.Invoke();
        }
    }

    protected static void Less(double actual, double bound) =>
        Assert.True(actual < bound, $"{actual} is not below {bound}");

    protected static void Greater(double actual, double bound) =>
        Assert.True(actual > bound, $"{actual} is not above {bound}");

    protected static bool Same(Vector3 a, Vector3 b) => a.X == b.X && a.Y == b.Y && a.Z == b.Z;

    protected static List<Vector3> Points(SoftBody body)
    {
        var points = new List<Vector3>();
        for (var i = 0; i < body.Count; i++)
        {
            points.Add(body.PositionOf(i));
        }

        return points;
    }

    protected static Vector3 Centre(SoftBody body)
    {
        var total = Vector3.Zero();
        foreach (var p in Points(body))
        {
            total.Add(p);
        }

        return total / (double)body.Count;
    }

    protected static double MeanZ(SoftBody body) => Points(body).Average(p => p.Z);

    protected static double Lowest(SoftBody body) => Points(body).Min(p => p.Y);

    protected static double Highest(SoftBody body) => Points(body).Max(p => p.Y);

    /// <summary>A wind the test can switch.</summary>
    protected sealed class Breeze : ISoftAir
    {
        public Vector3 Wind { get; set; } = Vector3.Zero();

        public Vector3 WindAt(Vector3 point) => Wind;
    }

    /// <summary>Particles one metre apart, hanging down from the origin.</summary>
    protected static List<Vector3> Line(int count) =>
        [.. Enumerable.Range(0, count).Select(i => new Vector3(0, -i, 0))];

    protected static List<Vector3> Tetrahedron() => [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];

    protected static int[] TetrahedronTriangles() => [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];

    /// <summary>Fails unless every axis agrees to <paramref name="places"/> decimal places.</summary>
    protected static void Near(Vector3 expected, Vector3 actual, int places = 12)
    {
        Assert.Equal(expected.X, actual.X, places);
        Assert.Equal(expected.Y, actual.Y, places);
        Assert.Equal(expected.Z, actual.Z, places);
    }
}
