namespace Orblit.Physics;

/// <summary>What a spring, a soft limit and a goal each demand of their numbers.</summary>
internal static class SpringRules
{
    internal static void Check(double hertz, double ratio, string what)
    {
        // Written as a negation so that NaN fails too.
        if (!(hertz > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(hertz), hertz, $"{what} needs a frequency above nought.");
        }

        if (!(ratio >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(ratio), ratio, "A damping ratio is nought or more.");
        }
    }
}
