using System.Runtime.InteropServices;
using System.Text;

namespace Orblit.Physics.Tests;

public class PhysicsSeeingTests : PhysicsTestBase
{
    /// <summary>Every body the scene below moves, or removes and adds, by name.</summary>
    private static readonly ulong[] Watched = [2, 3, 4, 5, 7, 10, 21, 30, 50];

    private static readonly Shape Crate = Shape.Box(0.5, 0.5, 0.5);

    /// <summary>
    /// Most of what a snapshot has to hold: a stack, a body that sleeps, a
    /// joint, a zone over a body, and a character that is driven every step.
    /// </summary>
    private static void RichWorld(Physics world)
    {
        world.Add(Ground, Shape.Plane(0.0, 1.0, 0.0), PhysicsMotion.Fixed);
        world.Add(2, Crate, at: [0.0, 0.5, 0.0], stay: true);
        world.Add(3, Crate, at: [0.0, 1.6, 0.0]);
        world.Add(4, Crate, at: [0.05, 2.7, 0.0]);
        world.Add(5, Shape.Sphere(0.5), at: [3.0, 2.0, 0.0]);
        world.Add(7, Shape.Sphere(0.5), at: [-3.0, 0.5, 0.0], asleep: true);
        world.Add(10, Shape.Sphere(0.2), at: [1.0, 6.0, 4.0]);
        world.Join(100, Joint.Point(), a: 10, at: [0.0, 6.0, 4.0]);
        world.Add(20, Shape.Box(2.0, 2.0, 2.0), PhysicsMotion.Fixed, at: [6.0, 1.0, 0.0], trigger: true);
        world.SetZone(20, new PhysicsZone(Gravity: [0.0, 2.0, 0.0]));
        world.Add(21, Shape.Sphere(0.25), at: [6.0, 1.0, 0.0]);
        world.AddCharacter(30, at: [-6.0, 1.0, 0.0]);
    }

    /// <summary>
    /// What the game does on step <paramref name="at"/>, then the step.
    /// Scripted by number, so a replay does the same thing at the same point
    /// of the run.
    /// </summary>
    private static void Play(Physics world, int at)
    {
        if (at == 55)
        {
            world.Add(50, Shape.Sphere(0.3), at: [1.0, 4.0, 1.0]);
        }

        if (at == 60)
        {
            world.Remove(3);
        }

        if (at == 70)
        {
            world.Push(7, [0.0, 3.0, 0.0]);
        }

        if (at == 75)
        {
            world.Remove(2);
        }

        var left = world.FootingOf(30)?.Velocity ?? [0.0, 0.0, 0.0];
        world.Drive(30, [1.0, left[1] - (9.81 / 60), 0.0]);
        world.Step(1.0 / 60);
    }

    private static string Text(IEnumerable<float>? values) => values is null ? "null" : string.Join(',', values);

    private static string Text(IEnumerable<double> values) => string.Join(',', values);

    /// <summary>
    /// Everything a caller can read about the world after a step, as text.
    /// Numbers print as the shortest string that reads back as the same
    /// number, so two records are equal only if every bit is.
    /// </summary>
    private static string Record(Physics world)
    {
        var text = new StringBuilder();
        foreach (var id in Watched)
        {
            text.Append($"{id} {Text(world.TransformOf(id))} {Text(world.VelocityOf(id))} ");
            text.AppendLine((world.Alive(id) && world.Asleep(id)).ToString());
        }

        foreach (var it in world.Events)
        {
            text.AppendLine($"{it.Kind} {it.A} {it.B} {Text(it.At)} {Text(it.Normal)} {it.Force}");
        }

        // Step time is the machine's, and differs between two identical runs.
        var stats = world.Stats;
        text.AppendLine(string.Join(
            ',',
            stats.Bodies,
            stats.StaticBodies,
            stats.KinematicBodies,
            stats.DynamicBodies,
            stats.Asleep,
            stats.Triggers,
            stats.Characters,
            stats.Joints,
            stats.Zones,
            stats.Rules,
            stats.Pairs,
            stats.Touching,
            stats.Points));
        foreach (var it in world.Contacts)
        {
            text.AppendLine($"{it.A} {it.B} {Text(it.At)} {Text(it.Normal)} {it.Depth} {it.Impulse}");
        }

        return text.ToString();
    }

    /// <summary>Plays steps <paramref name="from"/> up to <paramref name="to"/> and keeps a record of each.</summary>
    private static List<string> Replay(Physics world, int from, int to)
    {
        var records = new List<string>();
        for (var i = from; i < to; i++)
        {
            Play(world, i);
            records.Add(Record(world));
        }

        return records;
    }

    [Fact]
    public void ItsStructsAreTheSizeTheEngineReads()
    {
        Assert.Equal(48, Marshal.SizeOf<NativeContact>());
        Assert.Equal(56, Marshal.SizeOf<NativeStats>());
    }

    [Fact]
    public void AWorldRestoredFromStepFiftyReachesTheSameStepOneHundred()
    {
        RichWorld(World);
        for (var i = 0; i < 50; i++)
        {
            Play(World, i);
        }

        using var snapshot = World.Snapshot();
        var atFifty = Record(World);
        Assert.True(World.Asleep(7));

        var first = Replay(World, 50, 100);
        // The run has to do something, or repeating it proves nothing.
        Assert.False(World.Asleep(7)); // woken at step 70
        Assert.False(World.Alive(3)); // removed at step 60
        Assert.True(World.Alive(50)); // added at step 55

        World.Restore(snapshot);

        Assert.Equal(atFifty, Record(World));
        Assert.True(World.Asleep(7));
        Assert.True(World.Alive(3));
        Assert.False(World.Alive(50));
        Assert.Equal(first, Replay(World, 50, 100));

        // The snapshot is left as it was, so it goes back again.
        World.Restore(snapshot);
        Assert.Equal(first, Replay(World, 50, 100));
    }

    [Fact]
    public void ASnapshotOutlivesItsWorldAndRestoresIntoAnother()
    {
        RichWorld(World);
        for (var i = 0; i < 50; i++)
        {
            Play(World, i);
        }

        var snapshot = World.Snapshot();
        var expected = Replay(World, 50, 100);
        World.Dispose();

        // Settings that are nothing like the first world's: a restore brings
        // its own, since a replay under different ones would not be a replay.
        using var other = new Physics(new PhysicsSettings
        {
            Gravity = [0.0, -1.0, 0.0],
            SubSteps = 2,
            VelocitySteps = 2,
            PositionSteps = 1,
            JointSteps = 1,
            Sleeping = false,
        });
        other.Restore(snapshot);
        snapshot.Dispose();

        Assert.Equal(expected, Replay(other, 50, 100));
    }

    [Fact]
    public void WhatWasQueuedIsInASnapshotAndIsUndoneByARestore()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0]);
        Run(0.5);

        World.Add(3, Shape.Sphere(0.5), at: [4.0, 3.0, 0.0]);
        using var snapshot = World.Snapshot();
        Assert.True(World.Alive(3)); // queued before the copy

        World.Add(4, Shape.Sphere(0.5), at: [8.0, 3.0, 0.0]);
        World.Remove(2);
        World.Restore(snapshot);
        World.Step(1.0 / 60);

        Assert.Equal(3, World.Count); // the ground, 2 and 3
        Assert.True(World.Alive(2));
        Assert.True(World.Alive(3));
        Assert.False(World.Alive(4)); // queued after the copy
    }

    [Fact]
    public void ASnapshotOrAWorldThatIsGoneSaysSo()
    {
        Floor();
        var snapshot = World.Snapshot();
        Assert.False(snapshot.Disposed);
        snapshot.Dispose();
        snapshot.Dispose();

        Assert.True(snapshot.Disposed);
        Assert.Throws<InvalidOperationException>(() => World.Restore(snapshot));

        using var live = World.Snapshot();
        var gone = new Physics();
        gone.Dispose();
        Assert.Throws<InvalidOperationException>(() => gone.Snapshot());
        Assert.Throws<InvalidOperationException>(() => gone.Restore(live));
        Assert.Throws<InvalidOperationException>(() => gone.Contacts);
        Assert.Throws<InvalidOperationException>(() => gone.Stats);
    }

    [Fact]
    public void ARestingBoxListsItsContactsFacingOutOfTheSecondBody()
    {
        Floor();
        World.Add(2, Crate, at: [0.0, 1.0, 0.0]);
        // It lands at about a third of a second and sleeps half a second after.
        Run(0.5);

        var listed = World.Contacts;

        Assert.NotEmpty(listed);
        foreach (var it in listed)
        {
            Assert.Equal(new[] { Ground, 2UL }, new[] { it.A, it.B });
            // Out of the box, towards the ground: down.
            Assert.True(it.Normal[1] < -0.9);
        }

        // Holding a one kilogram box still takes its weight over the step.
        var held = listed.Sum(it => it.Impulse);
        Assert.InRange(held, (9.81 / 60) - 0.05, (9.81 / 60) + 0.05);

        var stats = World.Stats;
        Assert.Equal(listed.Count, stats.Points);
        Assert.Equal(1, stats.Touching);

        Run(3.0);
        Assert.True(World.Asleep(2));
        Assert.Empty(World.Contacts); // nothing looked at a sleeper
    }

    [Fact]
    public void TheCountsAddUpAndSurviveARestore()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 1.0, 0.0]);
        World.Add(3, Shape.Sphere(0.5), at: [2.0, 1.0, 0.0]);
        World.Add(4, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Driven, at: [6.0, 1.0, 0.0]);
        World.Add(5, Shape.Box(1.0, 1.0, 1.0), PhysicsMotion.Fixed, at: [9.0, 1.0, 0.0], trigger: true);
        World.AddCharacter(6, at: [12.0, 1.0, 0.0]);
        Run(0.1);

        var stats = World.Stats;
        Assert.Equal(6, stats.Bodies);
        Assert.Equal(2, stats.StaticBodies);
        Assert.Equal(2, stats.KinematicBodies); // a driven body and a character
        Assert.Equal(2, stats.DynamicBodies);
        Assert.Equal(1, stats.Triggers);
        Assert.Equal(1, stats.Characters);
        Assert.True(stats.StepMicroseconds > 0);

        using var snapshot = World.Snapshot();
        World.Remove(2);
        World.Remove(3);
        World.Step(1.0 / 60);
        Assert.Equal(0, World.Stats.DynamicBodies);

        World.Restore(snapshot);
        var restored = World.Stats;
        Assert.Equal(6, restored.Bodies);
        Assert.Equal(2, restored.DynamicBodies);
        // The clock belongs to the handle, not to the world that was saved.
        Assert.True(restored.StepMicroseconds > 0);
    }
}
