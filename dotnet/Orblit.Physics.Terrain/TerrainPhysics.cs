using Orblit.Terrain;
using Orblit.VectorMath;
using TerrainData = Orblit.Terrain.Terrain;

namespace Orblit.Physics.Terrain;

/// <summary>
/// A terrain as ground in a physics world.
/// </summary>
/// <remarks>
/// <para>
/// Each region is laid as a height field of its own, one fixed body a region,
/// and only the regions near a point are laid at all. A world a hundred
/// regions across has the few around the camera in it, not millions of
/// triangles nobody is standing on. <see cref="Sync"/> keeps it that way as
/// the point moves, and lays a region again when the terrain under it changes.
/// </para>
/// <para>
/// A region's field is its own texels and the first texel of the regions
/// after it. That is the ground the renderer draws for it, with a margin of
/// one texel all round that is never stood on. The margin is the neighbours'
/// heights, and it is what makes the fields one ground. A ridge running along
/// the line between two regions holds a ball up the way it would if the line
/// were not there. So a region is laid again when a neighbour changes, not
/// only when it does.
/// </para>
/// <para>
/// A hole is a hole, and so is ground whose region does not exist. A texel
/// with no height to give takes every triangle it is a corner of with it,
/// which is the rule <see cref="TerrainData.HeightAt"/> and the renderer keep.
/// </para>
/// <para>
/// The world is the caller's. This adds and removes its own bodies in it and
/// touches nothing else. They are numbered by <see cref="IdOf"/> unless the
/// constructor is given another numbering, and <see cref="RegionOf"/> turns an
/// event's body back into the region it hit.
/// </para>
/// </remarks>
public sealed class TerrainPhysics
{
    private const long First = -(1L << 62);
    private const long Field = (1L << 28) - 1;

    private readonly Func<RegionKey, ulong> _numbering;
    private readonly OrderedDictionary<RegionKey, Placement> _laid = [];
    private readonly Dictionary<ulong, RegionKey> _regionOf = [];

    // F2: six parameters, the first two required and the rest called by name.
    public TerrainPhysics(
        Physics physics,
        TerrainData terrain,
        double friction = 0.5,
        double restitution = 0.0,
        Layers? layers = null,
        Func<RegionKey, ulong>? numbering = null)
    {
        Physics = physics;
        Terrain = terrain;
        Friction = friction;
        Restitution = restitution;
        Layers = layers ?? Layers.Everything;
        _numbering = numbering ?? IdOf;
    }

    /// <summary>The world the ground is laid in.</summary>
    public Physics Physics { get; }

    /// <summary>The ground being laid. Its regions are read, never written.</summary>
    public TerrainData Terrain { get; }

    /// <summary>As for any body, and the same for every region.</summary>
    public double Friction { get; }

    public double Restitution { get; }

    public Layers Layers { get; }

    /// <summary>The regions laid now, in the order they were first laid.</summary>
    public IReadOnlyCollection<RegionKey> Laid => _laid.Keys;

    private int Samples => Terrain.RegionSize + 3;

    /// <summary>
    /// The body region <paramref name="key"/> is laid as, unless the constructor
    /// was given another numbering.
    /// </summary>
    /// <remarks>
    /// Far below zero when read as a signed number, so it never meets the
    /// numbers an entity store or a scene's physics hands out, or the small
    /// negative numbers a game gives bodies of its own, like a player's
    /// character. Packed from the key's two numbers, so no two regions within
    /// 2^27 of the origin share one.
    /// </remarks>
    public static ulong IdOf(RegionKey key) =>
        unchecked((ulong)(First + (((key.X & Field) << 28) | (key.Z & Field))));

    /// <summary>The body region <paramref name="key"/> is laid as, or null while it is not laid.</summary>
    public ulong? BodyOf(RegionKey key) => _laid.TryGetValue(key, out var placed) ? placed.Body : null;

    /// <summary>The region laid as <paramref name="body"/>, or null when it is not one of these.</summary>
    public RegionKey? RegionOf(ulong body) => _regionOf.TryGetValue(body, out var key) ? key : null;

    /// <summary>
    /// Lays the regions within <paramref name="radius"/> metres of world
    /// (<paramref name="x"/>, <paramref name="z"/>), takes up the ones that
    /// have gone far, and answers how many it laid or laid again.
    /// </summary>
    /// <remarks>
    /// A region is near when any of its square is within the radius. It is
    /// taken up once it is a region's width further than that, so a camera
    /// wandering back and forth along the line between two regions does not
    /// lay and take up the same one every frame. It is also taken up as soon
    /// as it is gone from the terrain.
    /// A region already laid is laid again when its heights or cover, or any
    /// neighbour's, have changed since. With <paramref name="refresh"/> false
    /// it is left as it was, however it has changed. That is how an editor
    /// holds off relaying the ground under a stroke that is still being drawn:
    /// sync with it false while the stroke goes on, and true once it is done.
    /// </remarks>
    public int Sync(double x, double z, double radius, bool refresh = true)
    {
        Check(x, z, radius);
        var width = Terrain.RegionSize * Terrain.Spacing;
        TakeUpFar(x, z, radius, width);
        var again = refresh ? LayAgain() : 0;
        return again + LayNear(x, z, radius, width);
    }

    /// <summary>Takes up every region laid.</summary>
    public void Clear()
    {
        foreach (var key in _laid.Keys.ToArray())
        {
            TakeUp(key);
        }
    }

    private static void Check(double x, double z, double radius)
    {
        if (!double.IsFinite(x))
        {
            throw new ArgumentOutOfRangeException(nameof(x), x, "Must be a finite number.");
        }

        if (!double.IsFinite(z))
        {
            throw new ArgumentOutOfRangeException(nameof(z), z, "Must be a finite number.");
        }

        if (!double.IsFinite(radius) || radius < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "Must be a finite number, not negative.");
        }
    }

    /// <summary>
    /// How far world (<paramref name="x"/>, <paramref name="z"/>) is from the
    /// nearest point of region <paramref name="key"/>'s square,
    /// <paramref name="width"/> metres a side.
    /// </summary>
    private static double Distance(RegionKey key, double x, double z, double width)
    {
        var dx = Math.Max(0.0, Math.Max((key.X * width) - x, x - ((key.X + 1) * width)));
        var dz = Math.Max(0.0, Math.Max((key.Z * width) - z, z - ((key.Z + 1) * width)));
        return Math.Sqrt((dx * dx) + (dz * dz));
    }

    private void TakeUpFar(double x, double z, double radius, double width)
    {
        foreach (var key in _laid.Keys.ToArray())
        {
            if (Terrain.RegionAt(key) is null || Distance(key, x, z, width) > radius + width)
            {
                TakeUp(key);
            }
        }
    }

    private int LayAgain()
    {
        var laid = 0;
        foreach (var (key, placed) in _laid.ToArray())
        {
            var now = RevisionsAround(key);
            if (!placed.Revisions.AsSpan().SequenceEqual(now) && Lay(key, now))
            {
                laid++;
            }
        }

        return laid;
    }

    private int LayNear(double x, double z, double radius, double width)
    {
        var x0 = Doubles.Floor((x - radius) / width);
        var x1 = Doubles.Floor((x + radius) / width);
        var z0 = Doubles.Floor((z - radius) / width);
        var z1 = Doubles.Floor((z + radius) / width);
        var laid = 0;
        for (var kz = z0; kz <= z1; kz++)
        {
            for (var kx = x0; kx <= x1; kx++)
            {
                var key = new RegionKey(kx, kz);
                if (_laid.ContainsKey(key) || Terrain.RegionAt(key) is null || Distance(key, x, z, width) > radius)
                {
                    continue;
                }

                if (Lay(key, RevisionsAround(key)))
                {
                    laid++;
                }
            }
        }

        return laid;
    }

    private bool Lay(RegionKey key, long[] revisions)
    {
        // Texel (i0, j0) of the whole terrain is sample (0, 0): a texel short
        // of the region's own corner, for the margin.
        var i0 = (key.X * Terrain.RegionSize) - 1;
        var j0 = (key.Z * Terrain.RegionSize) - 1;
        var heights = Gather(key, i0, j0);
        var body = _numbering(key);
        var spacing = Terrain.Spacing;
        var laid = Physics.LayGround(
            body,
            heights,
            columns: Samples,
            rows: Samples,
            spacing: spacing,
            margin: true,
            at: [i0 * spacing, 0.0, j0 * spacing],
            friction: Friction,
            restitution: Restitution,
            layers: Layers);
        if (laid)
        {
            _laid[key] = new Placement(body, revisions);
            _regionOf[body] = key;
        }

        return laid;
    }

    /// <summary>
    /// The heights of the grid whose first sample is texel (<paramref name="i0"/>,
    /// <paramref name="j0"/>), from the region and its eight neighbours. A hole,
    /// or a region that is not there, is left as NaN, which the solver reads as a hole.
    /// </summary>
    private double[] Gather(RegionKey key, long i0, long j0)
    {
        var heights = new double[Samples * Samples];
        Array.Fill(heights, double.NaN);
        for (var dz = -1; dz <= 1; dz++)
        {
            for (var dx = -1; dx <= 1; dx++)
            {
                if (Terrain.RegionAt(new RegionKey(key.X + dx, key.Z + dz)) is { } region)
                {
                    Copy(region, heights, i0, j0);
                }
            }
        }

        return heights;
    }

    /// <summary>
    /// Copies the texels of <paramref name="region"/> that fall in the grid
    /// whose first sample is texel (<paramref name="i0"/>, <paramref name="j0"/>):
    /// one column or row of a neighbour before, the whole of this one, two of a
    /// neighbour after.
    /// </summary>
    private void Copy(TerrainRegion region, double[] heights, long i0, long j0)
    {
        var size = Terrain.RegionSize;
        var ri = region.Key.X * size;
        var rj = region.Key.Z * size;
        var iTo = Math.Min(i0 + Samples, ri + size);
        var jTo = Math.Min(j0 + Samples, rj + size);
        for (var j = Math.Max(j0, rj); j < jTo; j++)
        {
            for (var i = Math.Max(i0, ri); i < iTo; i++)
            {
                var at = ((j - rj) * size) + (i - ri);
                if (!new Cover(region.Cover[at]).Hole)
                {
                    heights[((j - j0) * Samples) + (i - i0)] = region.Heights[at];
                }
            }
        }
    }

    private void TakeUp(RegionKey key)
    {
        if (_laid.Remove(key, out var placed))
        {
            _regionOf.Remove(placed.Body);
            Physics.Remove(placed.Body);
        }
    }

    /// <summary>
    /// The revision of the region at <paramref name="key"/> and of each of its
    /// eight neighbours, zero for one that is not there. Revisions are never
    /// reused, so a region taken out and a different one put back is a change.
    /// </summary>
    private long[] RevisionsAround(RegionKey key)
    {
        var revisions = new long[9];
        for (var n = 0; n < 9; n++)
        {
            var around = new RegionKey(key.X + (n % 3) - 1, key.Z + (n / 3) - 1);
            revisions[n] = Terrain.RegionAt(around)?.Revision ?? 0;
        }

        return revisions;
    }

    /// <summary>A region as it was laid: the body it is, and the revisions around it then.</summary>
    private sealed record Placement(ulong Body, long[] Revisions);
}
