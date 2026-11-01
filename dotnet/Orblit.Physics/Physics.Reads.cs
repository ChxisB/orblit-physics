namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    /// <summary>
    /// Advances the world by <paramref name="delta"/> seconds, applying anything
    /// submitted first. One call per tick. A caller wanting a fixed step calls
    /// it several times with the same delta rather than once with a varying
    /// one: a solver handed a wobbling delta produces a wobbling simulation.
    /// </summary>
    public void Step(double delta)
    {
        Flush();
        NativeMethods.Step(World, (float)delta);
    }

    /// <summary>How many bodies there are.</summary>
    public int Count
    {
        get
        {
            Flush();
            return (int)NativeMethods.Count(World);
        }
    }

    public bool Alive(ulong id)
    {
        Flush();
        return NativeMethods.Alive(World, id);
    }

    public bool Asleep(ulong id)
    {
        Flush();
        return NativeMethods.Asleep(World, id);
    }

    /// <summary>
    /// Seven floats, translation xyz then rotation xyzw, or null if the world
    /// does not know <paramref name="id"/>.
    /// </summary>
    public float[]? TransformOf(ulong id)
    {
        Flush();
        var found = new float[7];
        fixed (float* into = found)
        {
            return NativeMethods.Transform(World, id, into) ? found : null;
        }
    }

    /// <summary>
    /// Six floats, linear xyz then angular xyz, or null if the world does not
    /// know <paramref name="id"/>.
    /// </summary>
    public float[]? VelocityOf(ulong id)
    {
        Flush();
        var found = new float[6];
        fixed (float* into = found)
        {
            return NativeMethods.Velocity(World, id, into) ? found : null;
        }
    }

    /// <summary>
    /// Writes where each of <paramref name="ids"/> ended up into
    /// <paramref name="output"/>: seven floats per body, <paramref name="stride"/>
    /// floats apart, starting <paramref name="offset"/> floats in. Returns how
    /// many were written. An id the world does not know is skipped and its
    /// place left exactly as it was. Throws when the last row would not fit.
    /// </summary>
    public int ReadInto(ReadOnlySpan<ulong> ids, Span<float> output, int stride = 7, int offset = 0)
    {
        Flush();
        if (stride < 7)
        {
            return 0;
        }

        var needed = ids.IsEmpty ? 0L : ((long)(ids.Length - 1) * stride) + offset + 7;
        if (offset < 0 || needed > output.Length)
        {
            throw new ArgumentException("The output cannot hold a row for every id.", nameof(output));
        }

        fixed (ulong* keys = ids)
        fixed (float* into = output)
        {
            return (int)NativeMethods.Read(World, keys, (uint)ids.Length, into, (uint)stride, (uint)offset);
        }
    }

    /// <summary>Everything that happened in the last step. Dropped by the next one.</summary>
    public IReadOnlyList<PhysicsEvent> Events
    {
        get
        {
            Flush();
            var found = NativeMethods.Events(World, out var length);
            if (found is null || length == 0)
            {
                return [];
            }

            var events = new PhysicsEvent[length];
            for (var i = 0; i < events.Length; i++)
            {
                events[i] = EventAt(found + i);
            }

            return events;
        }
    }

    /// <summary>
    /// What character <paramref name="id"/> stood on at the end of the last
    /// step, or null if it is not a character.
    /// </summary>
    public PhysicsFooting? FootingOf(ulong id) => FootingsOf([id])[0];

    /// <summary>
    /// The footing of each of <paramref name="ids"/>, in the same order, in one
    /// crossing. A slot is null where that id is not a character.
    /// </summary>
    public PhysicsFooting?[] FootingsOf(ReadOnlySpan<ulong> ids)
    {
        Flush();
        if (ids.IsEmpty)
        {
            return [];
        }

        var slots = new NativeFooting[ids.Length];
        for (var i = 0; i < slots.Length; i++)
        {
            // The engine writes every field of a character's slot and none of
            // anyone else's, so a slot still holding NaN afterwards is one it
            // passed over. Turning is a number for every character there is.
            slots[i].Turning = float.NaN;
        }

        fixed (ulong* keys = ids)
        fixed (NativeFooting* into = slots)
        {
            // The count of characters found is not needed: a passed over slot is still NaN.
            _ = NativeMethods.Footing(World, keys, (uint)ids.Length, into);
            var footings = new PhysicsFooting?[ids.Length];
            for (var i = 0; i < footings.Length; i++)
            {
                footings[i] = FootingAt(into + i);
            }

            return footings;
        }
    }

    private static PhysicsEvent EventAt(NativeEvent* it) => new(
        // The ABI numbers them from one. A kind from a newer engine than this
        // binding knows about is reported as a touch ending, which is the safe
        // thing to mistake something for.
        it->Kind is >= 1 and <= 9 ? (PhysicsEventKind)it->Kind : PhysicsEventKind.TouchEnded,
        it->A,
        it->B,
        Vector(it->At),
        Vector(it->Normal),
        it->Force);

    private static PhysicsFooting? FootingAt(NativeFooting* it) => float.IsNaN(it->Turning)
        ? null
        : new PhysicsFooting(
            it->Ground,
            Vector(it->Normal),
            Vector(it->Velocity),
            Vector(it->Carried),
            it->Turning,
            it->Grounded != 0,
            it->Wall != 0,
            it->Ceiling != 0);
}
