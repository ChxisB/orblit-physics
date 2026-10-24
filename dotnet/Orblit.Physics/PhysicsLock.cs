namespace Orblit.Physics;

/// <summary>The ways a free body can move, each held still in the world's axes.</summary>
[Flags]
public enum PhysicsLock : uint
{
    None = 0,
    MoveX = 1,
    MoveY = 2,
    MoveZ = 4,
    TurnX = 8,
    TurnY = 16,
    TurnZ = 32,
}
