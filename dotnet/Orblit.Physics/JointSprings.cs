namespace Orblit.Physics;

/// <summary>
/// A spring on each of the six ways, or none. Named as a six-axis joint names
/// them: metres along the frame's axes, then radians about them.
/// </summary>
// F2: six fields, one for each way the ABI names, all optional and set by name.
public sealed record JointSprings(
    JointSpring? AlongX = null,
    JointSpring? AlongY = null,
    JointSpring? AlongZ = null,
    JointSpring? AboutX = null,
    JointSpring? AboutY = null,
    JointSpring? AboutZ = null)
{
    /// <summary>No spring on any way.</summary>
    public static JointSprings None { get; } = new();

    /// <summary>In the order the ABI names the ways.</summary>
    public IReadOnlyList<JointSpring?> All => [AlongX, AlongY, AlongZ, AboutX, AboutY, AboutZ];
}
