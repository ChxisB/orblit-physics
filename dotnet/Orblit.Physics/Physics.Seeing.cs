namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    /// <summary>
    /// Copies everything the world remembers, anything submitted and not yet
    /// stepped included. A world restored from it and given the same commands
    /// and the same deltas reaches, on the same build, the same bits the
    /// original did. That is the whole promise: a replay across builds,
    /// compilers or machines would drift, which is why the engine does not
    /// claim cross-platform determinism.
    /// </summary>
    /// <remarks>
    /// The copy is native memory the caller lets go of with
    /// <see cref="PhysicsSnapshot.Dispose"/>. A height field or a hull is
    /// shared with the world and not copied, so a snapshot of a large terrain
    /// costs its bodies.
    /// </remarks>
    public PhysicsSnapshot Snapshot()
    {
        Flush();
        var taken = NativeMethods.Snapshot(World);
        if (taken == nint.Zero)
        {
            throw new InvalidOperationException("Could not copy the Orblit physics world.");
        }

        return new PhysicsSnapshot(taken);
    }

    /// <summary>
    /// Makes this world the one <paramref name="snapshot"/> was taken from: its
    /// bodies, joints, ground, hulls, zones, rules, characters and settings,
    /// and the contacts and events the last step left. Anything done since is
    /// gone, commands not yet stepped included.
    /// </summary>
    /// <remarks>
    /// The snapshot is left as it was, so it may be restored again, and it may
    /// be restored into a different world, which then becomes that one.
    /// </remarks>
    public void Restore(PhysicsSnapshot snapshot)
    {
        var from = snapshot.Handle;
        var into = World;
        // What was queued belongs to the timeline being left.
        _pending = 0;
        NativeMethods.Restore(into, from);
    }

    /// <summary>
    /// The contact points of the last step, pairs in the order of their ids.
    /// Only pairs the solver looked at are here, so a pair that has gone to
    /// sleep is not. A body removed since the step takes its pairs with it.
    /// </summary>
    public IReadOnlyList<PhysicsContact> Contacts
    {
        get
        {
            Flush();
            var total = (int)NativeMethods.Contacts(World, null, 0);
            if (total == 0)
            {
                return [];
            }

            var found = new NativeContact[total];
            fixed (NativeContact* from = found)
            {
                // Nothing runs between the two calls, so this lists what the
                // first counted. The bound is for a count that grew anyway.
                var written = Math.Min((int)NativeMethods.Contacts(World, from, (uint)total), total);
                var listed = new PhysicsContact[written];
                for (var i = 0; i < written; i++)
                {
                    listed[i] = ContactAt(from + i);
                }

                return listed;
            }
        }
    }

    /// <summary>
    /// How much the world holds, and how much its last step looked at. There is
    /// no count of islands, because the solver has none yet.
    /// </summary>
    public PhysicsStats Stats
    {
        get
        {
            Flush();
            NativeMethods.Stats(World, out var it);
            return new PhysicsStats(
                (int)it.Bodies,
                (int)it.StaticBodies,
                (int)it.KinematicBodies,
                (int)it.DynamicBodies,
                (int)it.Asleep,
                (int)it.Triggers,
                (int)it.Characters,
                (int)it.Joints,
                (int)it.Zones,
                (int)it.Rules,
                (int)it.Pairs,
                (int)it.Touching,
                (int)it.Points,
                it.StepMicroseconds);
        }
    }

    private static PhysicsContact ContactAt(NativeContact* it) => new(
        it->A,
        it->B,
        Vector(it->At),
        Vector(it->Normal),
        it->Depth,
        it->Impulse);
}
