namespace Orblit.Physics;

/// <summary>
/// One point where two bodies touched in the last step. A and B are the pair,
/// smaller id first, as a touch event names it. At is midway between the two
/// surfaces where the bodies were when the step began, and Normal the way out
/// of B towards A. Depth is how far they overlapped along the normal when the
/// step began, in metres. Impulse is how hard the solver pushed along the
/// normal at this point over that step, in newton-seconds.
/// </summary>
// F2: six fields, all required and built in one place from the native contact.
public sealed record PhysicsContact(
    ulong A,
    ulong B,
    IReadOnlyList<double> At,
    IReadOnlyList<double> Normal,
    double Depth,
    double Impulse);
