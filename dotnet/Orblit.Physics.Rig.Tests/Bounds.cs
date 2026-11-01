namespace Orblit.Physics.Rig.Tests;

/// <summary>
/// One-sided bounds that name both numbers when they fail.
/// </summary>
internal static class Bounds
{
    public static void Less(double actual, double bound, string reason = "") =>
        Assert.True(actual < bound, $"{actual} is not below {bound}. {reason}");

    public static void Greater(double actual, double bound, string reason = "") =>
        Assert.True(actual > bound, $"{actual} is not above {bound}. {reason}");

    public static void AtLeast(double actual, double bound, string reason = "") =>
        Assert.True(actual >= bound, $"{actual} is below {bound}. {reason}");

    public static void Near(double expected, double actual, double tolerance, string reason = "") =>
        Assert.True(Math.Abs(expected - actual) <= tolerance, $"{actual} is not within {tolerance} of {expected}. {reason}");
}
