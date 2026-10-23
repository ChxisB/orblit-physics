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
        return NativeMethods.Cast(World, in query, out var hit) ? HitAt(&hit) : null;
    }

    private static PhysicsHit HitAt(NativeHit* it) => new(
        it->Body,
        Vector(it->At),
        Vector(it->Normal),
        it->Distance,
        it->Started != 0);
}
