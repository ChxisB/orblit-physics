namespace Orblit.Physics;

/// <summary>How the solver treats a body.</summary>
public enum PhysicsMotion : uint
{
    /// <summary>Never moves. The floor, the walls.</summary>
    Fixed = 0,

    /// <summary>
    /// Moves exactly where it is put, and pushes anything in the way without
    /// being pushed back. Lifts, doors, turntables. Something that walks wants
    /// <see cref="Physics.AddCharacter"/> instead.
    /// </summary>
    Driven = 1,

    /// <summary>Falls, is pushed, and pushes back.</summary>
    Free = 2,
}
