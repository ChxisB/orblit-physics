namespace Orblit.Physics;

/// <summary>
/// What a cast ran into: the body it met, where they touched, which way out of
/// the body towards the cast, and how far along the direction it got. Started
/// means the cast began inside the body, so the distance is zero and At is the
/// deepest point of the overlap.
/// </summary>
// F2: five fields, all required and built in one place from the native hit.
public sealed record PhysicsHit(
    ulong Body,
    IReadOnlyList<double> At,
    IReadOnlyList<double> Normal,
    double Distance,
    bool Started);
