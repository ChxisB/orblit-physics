namespace Orblit.Physics;

/// <summary>
/// The least and most a joint lets one of its axes go: metres along it, or
/// radians about it, measured from where the bodies stood when it was made.
/// Kept in order, so a <paramref name="Low"/> above its
/// <paramref name="High"/> is read the other way round.
/// </summary>
public sealed record JointLimit(double Low, double High)
{
    /// <summary>
    /// Held exactly at <paramref name="at"/>, which is where it was made unless
    /// said otherwise.
    /// </summary>
    public static JointLimit Locked(double at = 0.0) => new(at, at);
}
