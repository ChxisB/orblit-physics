namespace Orblit.Physics.Vehicle.Tests;

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
}
