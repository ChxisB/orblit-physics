namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    private const int MostParts = 64;

    private static readonly double[] Unit = [1.0, 1.0, 1.0];

    /// <summary>
    /// Cooks <paramref name="points"/> into the convex hull round them and
    /// keeps it under <paramref name="id"/> for <see cref="Shape.Hull"/> to
    /// name, and answers whether it was laid. <paramref name="points"/> is
    /// flat, three numbers a point: x0, y0, z0, x1, y1, z1 and so on.
    /// </summary>
    /// <remarks>
    /// A hull is made once and shared: a hundred crates cut from one mesh name
    /// one hull. Cooking is the slow part, so it belongs at load and not in a
    /// tick. A hull is never changed once laid. To change one, lay another
    /// under a new id and make the bodies again.
    /// More than 255 corners are cooked down to the 255 that stand out
    /// furthest, so the solid is a little smaller than the points and never
    /// larger.
    /// False, and nothing laid, for a zero id, an id that already names a
    /// hull, a length that is not a multiple of three, fewer than four points
    /// or more than 100,000, a number that is not finite, or points that
    /// enclose no volume because they all lie in one plane or on one line.
    /// </remarks>
    public bool LayHull(ulong id, IReadOnlyList<double> points)
    {
        RequireAlive();
        // Sent now rather than queued, so a body queued before this that names the
        // hull is made before the hull exists, as it would be for any other order.
        Flush();
        if (points.Count % 3 != 0)
        {
            return false;
        }

        fixed (float* xyz = Floats(points))
        {
            return NativeMethods.Hull(World, id, xyz, (uint)(points.Count / 3));
        }
    }

    /// <summary>
    /// Takes a hull away and answers whether it did. False for an id that
    /// names no hull, and for one a body still uses: take the bodies away
    /// first. The id may then be laid again.
    /// </summary>
    public bool DropHull(ulong id)
    {
        RequireAlive();
        // So a body removed a moment ago has gone before the hull is asked about.
        Flush();
        return NativeMethods.HullDrop(World, id);
    }

    /// <summary>
    /// Copies indexed triangle geometry and cooks its spatial tree and seams.
    /// Degenerate triangles are omitted. Invalid indices, nonfinite positions,
    /// empty geometry and repeated ids return false without changing the world.
    /// </summary>
    public bool LayMesh(ulong id, IReadOnlyList<double> vertices, IReadOnlyList<int> indices)
    {
        RequireAlive();
        Flush();
        if (!IsMesh(vertices, indices))
        {
            return false;
        }

        var triangles = new uint[Math.Max(indices.Count, 1)];
        for (var i = 0; i < indices.Count; i++)
        {
            triangles[i] = (uint)indices[i];
        }

        fixed (float* points = Floats(vertices))
        fixed (uint* corners = triangles)
        {
            return NativeMethods.Mesh(World, id, points, (uint)(vertices.Count / 3), corners, (uint)indices.Count);
        }
    }

    /// <summary>Removes an unused mesh. Remove its bodies first. Snapshots keep theirs.</summary>
    public bool DropMesh(ulong id)
    {
        RequireAlive();
        Flush();
        return NativeMethods.MeshDrop(World, id);
    }

    /// <summary>
    /// Keeps 1 to 64 convex parts under <paramref name="id"/>, shared by
    /// bodies and snapshots. Parts have uniform density. Overlapping parts
    /// count their mass twice. <paramref name="scale"/> stretches the whole
    /// compound, including its offsets and turns.
    /// </summary>
    /// <remarks>
    /// False for invalid geometry or an existing id. A hull part must be laid
    /// first. Nested compounds, ground and planes cannot be parts.
    /// </remarks>
    public bool LayCompound(ulong id, IReadOnlyList<ShapePart> parts, IReadOnlyList<double>? scale = null)
    {
        RequireAlive();
        Flush();
        var outer = ShapePart.Vector(scale ?? Unit, 3, nameof(scale));
        if (outer.Any(v => v <= 0) || parts.Count == 0 || parts.Count > MostParts)
        {
            return false;
        }

        var laid = new NativePart[parts.Count];
        fixed (NativePart* into = laid)
        {
            for (var i = 0; i < parts.Count; i++)
            {
                WritePart(into + i, parts[i], outer);
            }

            return NativeMethods.Compound(World, id, into, (uint)parts.Count);
        }
    }

    /// <summary>
    /// Removes an unused compound. Remove its bodies first. Hulls it held can
    /// then be dropped too. A snapshot still retains its own copy.
    /// </summary>
    public bool DropCompound(ulong id)
    {
        RequireAlive();
        Flush();
        return NativeMethods.CompoundDrop(World, id);
    }

    private static bool IsMesh(IReadOnlyList<double> vertices, IReadOnlyList<int> indices) =>
        vertices.Count % 3 == 0
        && indices.Count % 3 == 0
        && vertices.All(double.IsFinite)
        && indices.All(i => i >= 0 && i < vertices.Count / 3);

    // One float of room for an empty list, so the pointer is never null.
    private static float[] Floats(IReadOnlyList<double> from)
    {
        var floats = new float[Math.Max(from.Count, 1)];
        for (var i = 0; i < from.Count; i++)
        {
            floats[i] = (float)from[i];
        }

        return floats;
    }

    private static void WritePart(NativePart* into, ShapePart part, IReadOnlyList<double> outer)
    {
        into->Shape = part.Shape.Kind;
        into->Hull = part.Shape.Asset;
        WriteSize(into->Size, part.Shape);
        Write(into->At, part.Offset(outer), 3);
        Write(into->Linear, part.Linear(outer), 9);
    }
}
