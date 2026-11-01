namespace Orblit.Physics;

/// <summary>
/// What a character stood on at the end of the last step, and what survived of
/// what it asked for. Ground is what is under it, zero for nothing. Velocity is
/// what is left of the velocity it asked for, relative to its ground. Carried
/// and Turning are how far its ground moved and turned it, per second. Wall and
/// Ceiling say what stopped its walk besides the ground it stands on.
/// </summary>
// F2: eight fields, the last two optional and built in one place from the native footing.
public sealed record PhysicsFooting(
    ulong Ground,
    IReadOnlyList<double> Normal,
    IReadOnlyList<double> Velocity,
    IReadOnlyList<double> Carried,
    double Turning,
    bool Grounded,
    bool Wall = false,
    bool Ceiling = false);
