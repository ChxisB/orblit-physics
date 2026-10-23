namespace Orblit.Physics;

/// <summary>
/// Which layers a body is in, and which it wants to be told about.
/// </summary>
/// <remarks>
/// A pair interacts when either cares about the other, not when both do, so a
/// bullet that cares about walls hits a wall that cares about nothing.
/// </remarks>
public sealed record Layers(uint Is = 1, uint Cares = 0xFFFFFFFF)
{
    /// <summary>In layer one, and caring about every layer.</summary>
    public static Layers Everything { get; } = new();
}
