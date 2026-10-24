namespace Orblit.Physics.Tests;

public class PhysicsSeeingAddedTests : PhysicsTestBase
{
    private static readonly Shape Crate = Shape.Box(0.5, 0.5, 0.5);

    /// <summary>A spot high above the floor, apart from every other spot, so nothing here touches.</summary>
    private static double[] Spot(ulong number) => [number * 5.0, 10.0, 0.0];

    [Fact]
    public void EveryCountIsReadFromItsOwnField()
    {
        // Each count here is a different number, so a field read into the
        // wrong place shows.
        Floor();
        for (ulong id = 10; id < 12; id++)
        {
            World.Add(id, Crate, PhysicsMotion.Fixed, at: Spot(id));
        }

        for (ulong id = 20; id < 24; id++)
        {
            World.Add(id, Crate, PhysicsMotion.Fixed, at: Spot(id), trigger: true);
        }

        World.SetZone(20, new PhysicsZone(Gravity: [0.0, 2.0, 0.0]));
        World.SetZone(21, new PhysicsZone(Gravity: [0.0, 2.0, 0.0]));
        for (ulong id = 40; id < 45; id++)
        {
            World.AddCharacter(id, at: Spot(id));
        }

        World.Add(45, Crate, PhysicsMotion.Driven, at: Spot(45));
        for (ulong id = 50; id < 59; id++)
        {
            World.Add(id, Shape.Sphere(0.5), at: Spot(id), asleep: id < 58);
        }

        for (ulong id = 100; id < 110; id++)
        {
            World.Join(id, Joint.Point(), a: 58, at: Spot(58));
        }

        World.SetRule(10, 11, new PhysicsRule(friction: 0.2));
        // Awake bodies only: a rule wakes the two it names.
        World.SetRule(10, 58, new PhysicsRule(friction: 0.2));
        World.SetRule(11, 58, new PhysicsRule(friction: 0.2));
        Run(0.1);

        var it = World.Stats;

        // The one awake sphere is looked at against the floor and touches nothing.
        Assert.Equal(
            new[] { 22, 7, 6, 9, 8, 4, 5, 10, 2, 3, 1, 0, 0 },
            new[]
            {
                it.Bodies, it.StaticBodies, it.KinematicBodies, it.DynamicBodies, it.Asleep, it.Triggers,
                it.Characters, it.Joints, it.Zones, it.Rules, it.Pairs, it.Touching, it.Points,
            });
    }

    [Fact]
    public void AContactSaysWhereItIsHowDeepAndHowHard()
    {
        Floor();
        World.Add(2, Crate, at: [0.0, 3.0, 0.0]);

        // The step the box lands on is the one that pushes hardest, about
        // the speed it fell at in newton-seconds.
        IReadOnlyList<PhysicsContact> landing = [];
        for (var i = 0; i < 120 && landing.Sum(it => it.Impulse) < 3.0; i++)
        {
            World.Step(1.0 / 60);
            landing = World.Contacts;
        }

        Assert.True(landing.Sum(it => it.Impulse) >= 3.0);
        foreach (var it in landing)
        {
            Assert.InRange(it.At[0], -0.6, 0.6);
            Assert.InRange(it.At[1], -0.3, 0.3);
            Assert.InRange(it.At[2], -0.6, 0.6);
            Assert.InRange(Math.Abs(it.Normal[0]), 0.0, 0.1);
            Assert.True(it.Normal[1] < -0.9);
            Assert.InRange(Math.Abs(it.Normal[2]), 0.0, 0.1);
            Assert.InRange(it.Depth, -0.3, 0.3);
            Assert.True(it.Impulse >= 0.0);
        }
    }

    [Fact]
    public void ABodyRemovedSinceTheStepTakesItsContacts()
    {
        Floor();
        World.Add(2, Crate, at: [0.0, 1.0, 0.0]);
        Run(0.5);
        Assert.NotEmpty(World.Contacts);

        World.Remove(2);

        Assert.Empty(World.Contacts);
    }

    [Fact]
    public void TheStepClockIsZeroBeforeTheFirstStep()
    {
        Floor();

        Assert.Equal(0.0, World.Stats.StepMicroseconds);
    }

    [Fact]
    public void ABodyAddedSinceTheStepIsCounted()
    {
        Floor();
        World.Add(2, Crate, at: [0.0, 1.0, 0.0]);

        Assert.Equal(2, World.Stats.Bodies);
    }

    [Fact]
    public void ARestoreFromBeforeAnyStepLeavesTheClockWhereTheHandleHasIt()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0]);
        using var snapshot = World.Snapshot();
        Run(0.1);

        World.Restore(snapshot);

        Assert.True(World.Stats.StepMicroseconds > 0);
    }

    [Fact]
    public void ARestoreRefusedForADisposedSnapshotLeavesWhatWasQueued()
    {
        Floor();
        var snapshot = World.Snapshot();
        snapshot.Dispose();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0]);

        Assert.Throws<InvalidOperationException>(() => World.Restore(snapshot));

        Assert.True(World.Alive(2));
    }
}
