namespace Orblit.Physics;

/// <summary>What happened during a step, numbered as the ABI numbers them.</summary>
public enum PhysicsEventKind : uint
{
    TouchBegan = 1,
    TouchEnded = 2,
    Slept = 3,
    Woke = 4,

    /// <summary>A joint gave way and is gone. The event's A is the joint, not a body.</summary>
    Broke = 5,

    /// <summary>
    /// A body came into a trigger. A is the trigger and B the body, in that
    /// order whichever id is smaller.
    /// </summary>
    Entered = 6,

    /// <summary>A body left a trigger, or the trigger or the body was removed.</summary>
    Exited = 7,

    /// <summary>A touch that goes on, every step, for a body added with stay.</summary>
    TouchStay = 8,

    /// <summary>A body is still in a trigger, every step, when either was added with stay.</summary>
    Inside = 9,
}
