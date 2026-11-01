namespace Orblit.Physics.Rig;

/// <summary>How a ragdoll body is held.</summary>
public enum RagdollMode
{
    /// <summary>
    /// A driven body that is set to where the pose puts it every step. It
    /// pushes what it meets and is pushed by nothing.
    /// </summary>
    Animated,

    /// <summary>
    /// A free body whose joint's springs pull it to the pose's angles. It
    /// gives under a blow and comes back. A root has no joint to spring, so a
    /// powered root is animated.
    /// </summary>
    Powered,

    /// <summary>A free body held by its joint's limits and nothing else.</summary>
    Limp,
}
