namespace Orblit.Physics;

/// <summary>
/// One thing that happened in a step. A and B are the body it happened to and
/// for a touch the other one, smaller id first. At is where they met and
/// Normal which way out of B towards A, both zero for anything that is not a
/// touch or a trigger being entered. Force is newton-seconds along the normal.
/// </summary>
// F2: six fields, all required and built in one place from the native event.
public sealed record PhysicsEvent(
    PhysicsEventKind Kind,
    ulong A,
    ulong B,
    IReadOnlyList<double> At,
    IReadOnlyList<double> Normal,
    double Force);
