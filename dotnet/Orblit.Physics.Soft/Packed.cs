using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>A vector kept as three numbers in a flat list, which is how a body holds its particles.</summary>
internal static class Packed
{
    public static Vector3 Read(IReadOnlyList<double> values, int offset) =>
        new(values[offset], values[offset + 1], values[offset + 2]);

    public static void Write(Vector3 value, double[] values, int offset)
    {
        values[offset] = value.X;
        values[offset + 1] = value.Y;
        values[offset + 2] = value.Z;
    }
}
