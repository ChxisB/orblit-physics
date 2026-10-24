namespace Orblit.Physics.Rig;

/// <summary>
/// A joint that turns any way within a cone: a shoulder, a hip, a neck.
/// </summary>
/// <remarks>
/// <see cref="Swing"/> is how far the bone may tip from where it rests, in
/// radians, in any direction. <see cref="TwistLow"/> and
/// <see cref="TwistHigh"/> are how far it may turn about its own length.
/// </remarks>
public sealed class RagdollBall : RagdollJoint
{
    public RagdollBall(double swing = 1.0, double twistLow = -0.5, double twistHigh = 0.5)
    {
        if (!double.IsFinite(swing) || swing < 0 || swing > 3)
        {
            throw new ArgumentOutOfRangeException(nameof(swing), swing, "Swing is 0 to 3 radians.");
        }

        if (!double.IsFinite(twistLow) || !double.IsFinite(twistHigh) || twistLow > twistHigh)
        {
            throw new ArgumentException("Twist goes from low to high, with low first.", nameof(twistLow));
        }

        Swing = swing;
        TwistLow = twistLow;
        TwistHigh = twistHigh;
    }

    public double Swing { get; }

    public double TwistLow { get; }

    public double TwistHigh { get; }
}
