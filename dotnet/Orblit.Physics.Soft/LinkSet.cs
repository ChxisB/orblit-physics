namespace Orblit.Physics.Soft;

/// <summary>
/// The distance links of a body: where each runs from and to, the length it
/// holds and how much it gives. A bend link resists folding and the rest
/// resist stretching. They run in the order they were added, which for a
/// cloth is from the top row down, so a load is carried down in one pass.
/// </summary>
internal sealed class LinkSet(int[] a, int[] b, double[] rest, double[] alpha, byte[] bends)
{
    public int[] A { get; } = a;

    public int[] B { get; } = b;

    public double[] Rest { get; } = rest;

    public double[] Alpha { get; } = alpha;

    public byte[] Bends { get; } = bends;

    /// <summary>What each link has pushed in the sub-step being solved.</summary>
    public double[] Lambda { get; } = new double[a.Length];

    public int Length => A.Length;
}
