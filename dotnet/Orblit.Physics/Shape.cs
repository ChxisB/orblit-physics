namespace Orblit.Physics;

/// <summary>What a body is, geometrically.</summary>
public sealed class Shape
{
    private Shape(uint kind) => Kind = kind;

    /// <summary>Which kind it is, as the ABI numbers them.</summary>
    public uint Kind { get; }

    /// <summary>
    /// The ABI's four size floats, read as the kind describes: a radius, three
    /// half extents, a radius and a half height, or a normal and an offset.
    /// </summary>
    public double X { get; private init; }

    /// <inheritdoc cref="X"/>
    public double Y { get; private init; }

    /// <inheritdoc cref="X"/>
    public double Z { get; private init; }

    /// <inheritdoc cref="X"/>
    public double W { get; private init; }

    /// <summary>The asset a hull or compound names, and zero otherwise.</summary>
    public ulong Asset { get; private init; }

    /// <summary>A ball of <paramref name="radius"/> metres.</summary>
    public static Shape Sphere(double radius) => new(1) { X = radius };

    /// <summary>
    /// A box reaching the given half extents from its centre. Halves, not
    /// widths, because everything the solver does with them is measured from
    /// the middle outwards.
    /// </summary>
    public static Shape Box(double halfX, double halfY, double halfZ) =>
        new(2) { X = halfX, Y = halfY, Z = halfZ };

    /// <summary>
    /// A cylinder of <paramref name="radius"/> with <paramref name="halfHeight"/>
    /// metres of straight either side of the middle, capped with a hemisphere
    /// at each end, standing along the body's own y.
    /// </summary>
    public static Shape Capsule(double radius, double halfHeight) =>
        new(4) { X = radius, Y = halfHeight };

    /// <summary>
    /// A solid cylinder of <paramref name="radius"/> with
    /// <paramref name="halfHeight"/> metres of side either side of the middle,
    /// flat at both ends, standing along the body's own y.
    /// </summary>
    public static Shape Cylinder(double radius, double halfHeight) =>
        new(6) { X = radius, Y = halfHeight };

    /// <summary>
    /// An endless flat surface: everything behind the normal at
    /// <paramref name="offset"/> along it is solid. Never dynamic, because a
    /// half-space has no centre to spin about.
    /// </summary>
    public static Shape Plane(double nx, double ny, double nz, double offset = 0.0) =>
        new(3) { X = nx, Y = ny, Z = nz, W = offset };
}
