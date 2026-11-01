using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>
/// How a soft body is solved, and how stiff it is.
/// </summary>
/// <remarks>
/// The solver steps in pieces no longer than <see cref="SubstepLength"/>, so a
/// body hangs the same at 30 and at 60 frames a second. Each piece makes
/// <see cref="Iterations"/> passes over the links. Stiffness is a compliance,
/// the metres a link gives under one newton.
/// A pass cannot carry a load down a long chain, so a body stretches more the
/// more of it hangs from few links, and less the shorter the sub-step. A 20 by
/// 20 sheet hung from its top edge stretches under 2 per cent at the default.
/// Hung from two corners it needs a sub-step of 1/960 s to stay under 5.
/// </remarks>
public sealed class SoftSpec
{
    /// <summary>The longest sub-step. Past it a body hangs in rags, whatever it is made of.</summary>
    public const double MaxSubstepLength = 1.0 / 120;

    /// <summary>
    /// The longest span a single <see cref="SoftBody.Step"/> simulates. A longer
    /// one is cut to this, so a hitch loses time rather than spending it.
    /// </summary>
    public const double MaxStep = 0.1;

    // F2: nine parameters, all optional and called by name.
    public SoftSpec(
        Vector3? gravity = null,
        double damping = 1.0,
        double stretchCompliance = 1e-6,
        double bendCompliance = 1e-3,
        double substepLength = 1.0 / 480,
        int iterations = 1,
        double thickness = 0.01,
        double friction = 0.5,
        double windDrag = 1.0)
    {
        NotNegative(damping, nameof(damping));
        NotNegative(stretchCompliance, nameof(stretchCompliance));
        NotNegative(bendCompliance, nameof(bendCompliance));
        if (!(substepLength > 0 && substepLength <= MaxSubstepLength))
        {
            throw new ArgumentOutOfRangeException(
                nameof(substepLength),
                substepLength,
                $"Must be above zero and at most {MaxSubstepLength} s.");
        }

        if (iterations < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(iterations), iterations, "Must be 1 or more.");
        }

        if (!(thickness > 0 && double.IsFinite(thickness)))
        {
            throw new ArgumentOutOfRangeException(nameof(thickness), thickness, "Must be above zero.");
        }

        NotNegative(friction, nameof(friction));
        if (!(windDrag >= 0 && double.IsFinite(windDrag)))
        {
            throw new ArgumentOutOfRangeException(nameof(windDrag), windDrag, "Must not be negative.");
        }

        Gravity = gravity?.Clone() ?? new Vector3(0, -9.81, 0);
        Damping = damping;
        StretchCompliance = stretchCompliance;
        BendCompliance = bendCompliance;
        SubstepLength = substepLength;
        Iterations = iterations;
        Thickness = thickness;
        Friction = friction;
        WindDrag = windDrag;
    }

    /// <summary>Metres a second squared. Do not change it after the spec is made.</summary>
    public Vector3 Gravity { get; }

    /// <summary>The fraction of a particle's speed shed each second, as air does.</summary>
    public double Damping { get; }

    /// <summary>Metres of give per newton in an edge. Zero is a rod.</summary>
    public double StretchCompliance { get; }

    /// <summary>The same for the links that resist folding. Larger folds more easily.</summary>
    public double BendCompliance { get; }

    /// <summary>The longest sub-step, in seconds.</summary>
    public double SubstepLength { get; }

    /// <summary>Passes over the links in each sub-step.</summary>
    public int Iterations { get; }

    /// <summary>
    /// How far, in metres, a particle keeps from a surface. It is the skin of
    /// the cloth, so a surface thinner than the gap between two particles still
    /// lets a particle through.
    /// </summary>
    public double Thickness { get; }

    /// <summary>
    /// How hard a particle grips what it rests on. Zero slides like ice. A
    /// particle slips when the push along the surface is more than this times
    /// the push out of it.
    /// </summary>
    public double Friction { get; }

    /// <summary>
    /// How fast, per second, the air takes a particle up to its own speed. It
    /// counts only for a body given an <see cref="ISoftAir"/>, and it is on top
    /// of <see cref="Damping"/>. A flag in a 10 m/s wind at 1.0 is pushed at
    /// 10 m/s², about gravity.
    /// </summary>
    public double WindDrag { get; }

    private static void NotNegative(double value, string name)
    {
        if (!(value >= 0))
        {
            throw new ArgumentOutOfRangeException(name, value, "Must not be negative.");
        }
    }
}
