namespace Orblit.Physics;

/// <summary>
/// A convex shape placed inside a compound. Scaling is applied before turning
/// and translating. A single part provides a scaled or offset shape.
/// </summary>
public sealed class ShapePart
{
    private static readonly double[] Origin = [0.0, 0.0, 0.0];
    private static readonly double[] Upright = [0.0, 0.0, 0.0, 1.0];
    private static readonly double[] Unscaled = [1.0, 1.0, 1.0];

    // Sphere, box, capsule, cylinder and hull. A plane, ground and a compound are not convex.
    private static readonly uint[] Convex = [1, 2, 4, 6, 7];

    public ShapePart(
        Shape shape,
        IReadOnlyList<double>? at = null,
        IReadOnlyList<double>? rotation = null,
        IReadOnlyList<double>? scale = null)
    {
        At = Vector(at ?? Origin, 3, nameof(at));
        Rotation = Vector(rotation ?? Upright, 4, nameof(rotation));
        Scale = Vector(scale ?? Unscaled, 3, nameof(scale));
        if (!Convex.Contains(shape.Kind))
        {
            throw new ArgumentException("A part must be convex.", nameof(shape));
        }

        if (Scale.Any(v => v <= 0) || Rotation.All(v => v == 0))
        {
            throw new ArgumentException("Scale must be positive and rotation nonzero.", nameof(scale));
        }

        Shape = shape;
    }

    public Shape Shape { get; }

    /// <summary>Where the part sits, before the compound's own scale.</summary>
    public IReadOnlyList<double> At { get; }

    /// <summary>A quaternion, x y z w, that need not be unit length.</summary>
    public IReadOnlyList<double> Rotation { get; }

    public IReadOnlyList<double> Scale { get; }

    /// <summary>A read-only copy of <paramref name="from"/>, which must be <paramref name="count"/> finite numbers.</summary>
    internal static IReadOnlyList<double> Vector(IReadOnlyList<double> from, int count, string name)
    {
        if (from.Count != count || from.Any(v => !double.IsFinite(v)))
        {
            throw new ArgumentException($"Expected {count} finite numbers.", name);
        }

        return Array.AsReadOnly(from.ToArray());
    }

    /// <summary>Where the part sits once the compound's own scale is applied.</summary>
    internal double[] Offset(IReadOnlyList<double> outer) =>
        [At[0] * outer[0], At[1] * outer[1], At[2] * outer[2]];

    /// <summary>
    /// The row-major matrix that scales the part, turns it, and then scales it
    /// again by <paramref name="outer"/>.
    /// </summary>
    internal double[] Linear(IReadOnlyList<double> outer)
    {
        var sum = 0.0;
        foreach (var v in Rotation)
        {
            sum += v * v;
        }

        var length = Math.Sqrt(sum);
        var x = Rotation[0] / length;
        var y = Rotation[1] / length;
        var z = Rotation[2] / length;
        var w = Rotation[3] / length;
        double[] turn =
        [
            1 - (2 * ((y * y) + (z * z))),
            2 * ((x * y) - (z * w)),
            2 * ((x * z) + (y * w)),
            2 * ((x * y) + (z * w)),
            1 - (2 * ((x * x) + (z * z))),
            2 * ((y * z) - (x * w)),
            2 * ((x * z) - (y * w)),
            2 * ((y * z) + (x * w)),
            1 - (2 * ((x * x) + (y * y))),
        ];
        var linear = new double[9];
        for (var i = 0; i < 9; i++)
        {
            linear[i] = outer[i / 3] * turn[i] * Scale[i % 3];
        }

        return linear;
    }
}
