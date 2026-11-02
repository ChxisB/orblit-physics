namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    /// <summary>
    /// Fires <paramref name="shape"/> from <paramref name="from"/> along
    /// <paramref name="direction"/> for <paramref name="distance"/> metres and
    /// answers the first body it meets, or null if it meets none. Leaving the
    /// shape out fires a point, the ray cast everything else is named after.
    /// </summary>
    /// <remarks>
    /// The direction need not be a unit vector, so the distance is always
    /// metres. A plane cannot be cast, because a half-space reaches everywhere
    /// along any line, and asking for one gives null. A cast sees solid bodies
    /// and never a trigger. With <paramref name="triggers"/> it sees triggers
    /// and nothing solid.
    /// </remarks>
    // F2: eight parameters, the first three required and the rest called by name.
    public PhysicsHit? Cast(
        IReadOnlyList<double> @from,
        IReadOnlyList<double> direction,
        double distance,
        Shape? shape = null,
        IReadOnlyList<double>? rotation = null,
        Layers? layers = null,
        ulong ignore = 0,
        bool triggers = false)
    {
        Flush();
        var query = QueryOf(@from, direction, distance, shape, rotation, layers, ignore, triggers);
        return NativeMethods.Cast(World, in query, out var hit) ? HitAt(&hit) : null;
    }

    /// <summary>
    /// Every body <see cref="Cast"/> would meet along its way, nearest first,
    /// each once, up to <paramref name="limit"/> of them. A full list keeps the
    /// nearest, not the first found. A body the cast begins inside is one of
    /// them, with <see cref="PhysicsHit.Started"/> set.
    /// </summary>
    // F2: nine parameters, the first three required and the rest called by name.
    public IReadOnlyList<PhysicsHit> CastAll(
        IReadOnlyList<double> @from,
        IReadOnlyList<double> direction,
        double distance,
        Shape? shape = null,
        IReadOnlyList<double>? rotation = null,
        Layers? layers = null,
        ulong ignore = 0,
        bool triggers = false,
        int limit = 32)
    {
        Flush();
        if (limit <= 0)
        {
            return [];
        }

        var query = QueryOf(@from, direction, distance, shape, rotation, layers, ignore, triggers);
        var found = new NativeHit[limit];
        fixed (NativeHit* hits = found)
        {
            var written = (int)NativeMethods.CastAll(World, in query, hits, (uint)limit);
            var made = new PhysicsHit[written];
            for (var i = 0; i < written; i++)
            {
                made[i] = HitAt(&hits[i]);
            }

            return made;
        }
    }

    /// <summary>
    /// Whether <see cref="Cast"/> would meet anything at all, found without
    /// working out which was nearest: the cheap way to ask whether there is a
    /// wall in the way.
    /// </summary>
    // F2: eight parameters, the first three required and the rest called by name.
    public bool CastAny(
        IReadOnlyList<double> @from,
        IReadOnlyList<double> direction,
        double distance,
        Shape? shape = null,
        IReadOnlyList<double>? rotation = null,
        Layers? layers = null,
        ulong ignore = 0,
        bool triggers = false)
    {
        Flush();
        var query = QueryOf(@from, direction, distance, shape, rotation, layers, ignore, triggers);
        return NativeMethods.CastAny(World, in query);
    }

    /// <summary>How many bodies <see cref="Overlap"/> lists when no limit is given.</summary>
    public const int DefaultOverlapLimit = 32;

    /// <summary>
    /// The bodies <paramref name="shape"/> overlaps where it stands at
    /// <paramref name="at"/>, in no order, up to <paramref name="limit"/> of
    /// them. Leaving the shape out asks about a point: every body that contains
    /// it, a plane included, which contains everything below its surface. A
    /// plane cannot be asked about, and that finds nothing.
    /// </summary>
    /// <remarks>
    /// Solid bodies unless <paramref name="triggers"/> is set, and then only
    /// triggers, as for <see cref="Cast"/>. Nothing is moved and nothing woken.
    /// </remarks>
    // F2: seven parameters, the first required and the rest called by name.
    public IReadOnlyList<ulong> Overlap(
        IReadOnlyList<double> at,
        Shape? shape = null,
        IReadOnlyList<double>? rotation = null,
        Layers? layers = null,
        ulong ignore = 0,
        bool triggers = false,
        int limit = DefaultOverlapLimit)
    {
        if (limit <= 0)
        {
            Flush();
            return [];
        }

        var found = new ulong[limit];
        var written = OverlapInto(at, found, shape, rotation, layers, ignore, triggers);
        return found[..written];
    }

    /// <summary>
    /// The same query as <see cref="Overlap"/>, written into
    /// <paramref name="found"/> instead of a new list. It answers how many ids
    /// it wrote, up to the length of the buffer, and leaves the rest of the
    /// buffer as it was. A query made every frame, such as wind at each point of
    /// a cloth, makes no garbage this way.
    /// </summary>
    /// <remarks>
    /// Its own name, because a call that writes <c>null</c> for the shape would
    /// otherwise not say which of the two forms it meant.
    /// </remarks>
    // F2: seven parameters, the first two required and the rest called by name.
    public int OverlapInto(
        IReadOnlyList<double> at,
        Span<ulong> found,
        Shape? shape = null,
        IReadOnlyList<double>? rotation = null,
        Layers? layers = null,
        ulong ignore = 0,
        bool triggers = false)
    {
        Flush();
        if (found.IsEmpty)
        {
            return 0;
        }

        var query = QueryOf(at, null, 0.0, shape, rotation, layers, ignore, triggers);
        fixed (ulong* ids = found)
        {
            return (int)NativeMethods.Overlap(World, in query, ids, (uint)found.Length);
        }
    }

    // F2: eight parameters, the same ones every query takes.
    private static NativeCast QueryOf(
        IReadOnlyList<double> @from,
        IReadOnlyList<double>? direction,
        double distance,
        Shape? shape,
        IReadOnlyList<double>? rotation,
        Layers? layers,
        ulong ignore,
        bool triggers)
    {
        var layering = layers ?? Layers.Everything;
        var query = new NativeCast
        {
            Distance = (float)distance,
            LayerIs = layering.Is,
            LayerCares = layering.Cares,
            Ignore = ignore,
            Triggers = Flag(triggers),
        };
        if (shape is not null)
        {
            query.Shape = shape.Kind;
            query.Hull = shape.Asset;
            WriteSize(query.Size, shape);
        }

        Write(query.From, @from, 3);
        Write(query.Rotation, rotation ?? Identity, 4);
        Write(query.Direction, direction, 3);
        return query;
    }

    private static PhysicsHit HitAt(NativeHit* it) => new(
        it->Body,
        Vector(it->At),
        Vector(it->Normal),
        it->Distance,
        it->Started != 0);
}
