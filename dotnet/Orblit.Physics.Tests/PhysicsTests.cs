namespace Orblit.Physics.Tests;

public class PhysicsTests : PhysicsTestBase
{
    [Fact]
    public void ADroppedSphereComesToRestOnTheGround()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0]);

        Run(3);

        // Resting means both things: sitting a radius above the plane, and no
        // longer moving. Either alone can be true of a sphere still falling.
        Assert.Equal(0.5, HeightOf(2), 0.03);
        Assert.True(Math.Abs(FallSpeedOf(2)) < 0.01);
    }

    [Fact]
    public void ARestingBodyGoesToSleepAndCanBeWoken()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 1.0, 0.0]);

        Run(3);
        Assert.True(World.Asleep(2));

        World.Wake(2);
        World.Step(1.0 / 60);
        Assert.False(World.Asleep(2));
    }

    [Fact]
    public void ALandingReportsATouchAndRemovalEndsIt()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 1.0, 0.0]);

        var began = new List<PhysicsEvent>();
        for (var i = 0; i < 120 && began.Count == 0; i++)
        {
            World.Step(1.0 / 60);
            began = World.Events.Where(it => it.Kind == PhysicsEventKind.TouchBegan).ToList();
        }

        var touch = Assert.Single(began);
        Assert.Equal(new HashSet<ulong> { Ground, 2 }, new HashSet<ulong> { touch.A, touch.B });
        Assert.True(touch.Force > 0.0);
        Assert.Equal(0.0, touch.At[1], 0.05);

        World.Remove(2);
        World.Step(1.0 / 60);
        Assert.Contains(PhysicsEventKind.TouchEnded, World.Events.Select(it => it.Kind));
    }

    [Fact]
    public void RestitutionDecidesWhetherASphereBounces()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 3.0, 0.0], restitution: 0.8);
        World.Add(3, Shape.Sphere(0.5), at: [2.0, 3.0, 0.0], restitution: 0.0);

        var highestBouncy = 0.0;
        var highestDead = 0.0;
        for (var i = 0; i < 180; i++)
        {
            World.Step(1.0 / 60);
            if (i > 60)
            {
                highestBouncy = Math.Max(highestBouncy, HeightOf(2));
                highestDead = Math.Max(highestDead, HeightOf(3));
            }
        }

        Assert.True(highestBouncy > 1.0);
        Assert.True(highestDead < 0.6);
    }

    [Fact]
    public void FrictionStopsASlidingBox()
    {
        Floor();
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [0.0, 0.5, 0.0], friction: 0.8);
        World.Push(2, [5.0, 0.0, 0.0]);

        Run(3);
        Assert.True(Math.Abs(World.VelocityOf(2)![0]) < 0.05);
    }

    [Fact]
    public void LayersDecideWhatABodyNotices()
    {
        // A pair interacts when either one cares about the other, so ignoring
        // something takes both of them: this ground watches only bit one, and
        // the body on bit two watches only bit two.
        World.Add(
            Ground,
            Shape.Plane(0.0, 1.0, 0.0),
            PhysicsMotion.Fixed,
            layers: new Layers(Is: 1, Cares: 1));
        World.Add(2, Shape.Sphere(0.5), at: [0.0, 2.0, 0.0], layers: new Layers(Is: 2, Cares: 2));
        World.Add(3, Shape.Sphere(0.5), at: [2.0, 2.0, 0.0]);

        Run(2);
        Assert.True(HeightOf(2) < -0.5, "should have fallen through");
        Assert.Equal(0.5, HeightOf(3), 0.05);
    }

    [Fact]
    public void ADrivenBodyCarriesWhatStandsOnIt()
    {
        Floor();
        World.Add(2, Shape.Box(2.0, 0.25, 2.0), PhysicsMotion.Driven, at: [0.0, 0.25, 0.0]);
        World.Add(3, Shape.Sphere(0.5), at: [0.0, 1.05, 0.0]);

        World.Drive(2, [0.0, 0.5, 0.0]);
        Run(2);

        Assert.Equal(1.25, HeightOf(2), 0.02);
        Assert.True(HeightOf(3) > 1.5);
    }

    [Fact]
    public void AnOffCentrePushImpartsSpin()
    {
        Floor();
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), at: [0.0, 5.0, 0.0]);

        World.Push(2, [0.0, 0.0, 2.0], at: [0.0, 5.5, 0.0]);
        World.Step(1.0 / 60);

        // Six floats: linear xyz, then angular xyz. A shove along Z landing
        // above the centre sends the top of the box that way and the bottom
        // the other, which is a turn about X.
        var velocity = World.VelocityOf(2)!;
        Assert.Equal(2.0, velocity[2], 0.01);
        Assert.Equal(6.0, velocity[3], 0.05);
        Assert.True(Math.Abs(velocity[4]) < 0.001);
        Assert.True(Math.Abs(velocity[5]) < 0.001);
    }

    [Fact]
    public void ACapsuleCrossesAsACapsuleNotAsTheSphereItResembles()
    {
        Floor();
        // Standing up, 0.3 of radius with 0.5 of straight either side, so it
        // comes to rest with its middle 0.8 above the floor. A sphere of the
        // same radius would rest at 0.3, which is exactly what a half height
        // arriving in the wrong size slot would look like.
        World.Add(2, Shape.Capsule(0.3, 0.5), at: [0.0, 3.0, 0.0]);

        Run(3);
        Assert.Equal(0.8, HeightOf(2), 0.05);
    }

    [Fact]
    public void ACastReportsWhatItMetAndItsStructsSurviveTheCrossing()
    {
        Floor();
        World.Add(2, Shape.Box(0.5, 0.5, 0.5), PhysicsMotion.Fixed, at: [0.0, 5.0, 0.0]);
        // A crate off to the side that watches nothing, for the layer cases.
        World.Add(
            3,
            Shape.Box(0.5, 0.5, 0.5),
            PhysicsMotion.Fixed,
            at: [3.0, 5.0, 0.0],
            layers: new Layers(Is: 2, Cares: 0));

        // Not stepped first on purpose: a cast flushes what is waiting, so both
        // crates are already there to be hit.
        var down = World.Cast([0.0, 10.0, 0.0], [0.0, -1.0, 0.0], 100.0)!;
        Assert.Equal(2UL, down.Body);
        Assert.Equal(4.5, down.Distance, 0.01);
        Assert.Equal(5.5, down.At[1], 0.01);
        Assert.Equal(1.0, down.Normal[1], 0.01);
        Assert.False(down.Started);

        // Each case below reaches a different field of the query struct. Taken
        // together they are the only way to know every one of them landed where
        // the C is looking for it, rather than a neighbour's.
        PhysicsHit? Fire(
            double[]? from = null,
            double[]? direction = null,
            double distance = 100.0,
            Shape? shape = null,
            double[]? rotation = null,
            Layers? layers = null,
            ulong ignore = 0) => World.Cast(
                from ?? [0.0, 10.0, 0.0],
                direction ?? [0.0, -1.0, 0.0],
                distance,
                shape,
                rotation,
                layers,
                ignore);

        Assert.Null(Fire(direction: [0.0, 1.0, 0.0]));
        Assert.Null(Fire(distance: 1.0));
        Assert.Equal(Ground, Fire(ignore: 2)?.Body);

        // A body that cares about nothing is reached only by a cast that cares
        // about it, which is also what tells the two layer words apart: swap
        // them and the first of these misses.
        double[] beside = [3.0, 10.0, 0.0];
        var cares = Fire(from: beside, layers: new Layers(Is: 4, Cares: 2));
        var passesBy = Fire(from: beside, layers: new Layers(Is: 4, Cares: 8));
        Assert.Equal(3UL, cares?.Body);
        Assert.Equal(Ground, passesBy?.Body);

        Assert.Equal(4.25, Fire(shape: Shape.Sphere(0.25))!.Distance, 0.02);

        // Laid along x by a quarter turn about z, so it is 0.2 thick underneath
        // rather than 0.7. Upright it would stop at 3.8.
        var laid = Fire(shape: Shape.Capsule(0.2, 0.5), rotation: [0.0, 0.0, 0.70710678, 0.70710678]);
        Assert.Equal(4.3, laid!.Distance, 0.02);

        var inside = Fire(from: [0.0, 5.0, 0.0], direction: [1.0, 0.0, 0.0], distance: 10.0)!;
        Assert.True(inside.Started);
        Assert.Equal(0.0, inside.Distance);

        // A half-space reaches everywhere along a line, so it is refused.
        Assert.Null(Fire(shape: Shape.Plane(0.0, 1.0, 0.0)));
    }

    [Fact]
    public void ACharacterWalksUpAStepAndReadsBackWhatItStandsOn()
    {
        // Ids past 32 bits on both sides, so a footing whose ground came back
        // through the wrong width of field fails here rather than in a game.
        const ulong walker = 0x100000002;
        const ulong step = 0x700000005;
        Floor();
        World.Add(step, Shape.Box(2.0, 0.1, 1.0), PhysicsMotion.Fixed, at: [3.0, 0.1, 0.0]);
        World.AddCharacter(walker, at: [0.0, 0.91, 0.0]);

        Assert.Equal(0, Walk(walker, 1.5, 0.0, 2.0));

        Assert.Equal(3.0, World.TransformOf(walker)![0], 0.05);
        Assert.Equal(1.11, HeightOf(walker), 0.005);
        var footing = World.FootingOf(walker)!;
        Assert.True(footing.Grounded);
        Assert.Equal(step, footing.Ground);
        Assert.Equal(0.0, footing.Normal[0], 1e-4);
        Assert.Equal(1.0, footing.Normal[1], 1e-4);
        Assert.Equal(0.0, footing.Normal[2]);
        Assert.Equal(1.5, footing.Velocity[0], 1e-4);
        Assert.Equal(0.0, footing.Velocity[1]);
        Assert.Equal(0.0, footing.Velocity[2]);
        Assert.All(footing.Carried, v => Assert.Equal(0.0, v));
        Assert.Equal(0.0, footing.Turning);
    }

    [Fact]
    public void ACharacterStopsAtAWallAndSoDoesWhatItAskedFor()
    {
        Floor();
        World.Add(3, Shape.Box(0.25, 1.0, 2.0), PhysicsMotion.Fixed, at: [2.0, 1.0, 0.0]);
        World.AddCharacter(2, at: [0.0, 0.91, 0.0]);

        Walk(2, 1.0, 0.0, 3.0);

        // The face is at 1.75; a radius and a skin short of it is 1.44.
        Assert.Equal(1.44, World.TransformOf(2)![0], 0.005);
        Assert.Equal(0.0, World.FootingOf(2)!.Velocity[0], 1e-4);
    }

    [Fact]
    public void ACharacterCrouchesUnderABeamItCannotThenStandUpUnder()
    {
        Floor();
        // A beam 0.2 m thick with its underside 1.2 m up, across x from -2 to 2.
        World.Add(3, Shape.Box(2.0, 0.1, 2.0), PhysicsMotion.Fixed, at: [0.0, 1.3, 0.0]);
        World.AddCharacter(2, at: [-3.0, 0.91, 0.0]);
        Run(0.2);

        Assert.True(World.SetStance(2, Shape.Capsule(0.3, 0.2)));
        Assert.Equal(0.51, HeightOf(2), 0.005);

        Walk(2, 1.5, 0.0, 2.0);
        Assert.Equal(0.0, World.TransformOf(2)![0], 0.2);
        Assert.False(World.SetStance(2, Shape.Capsule(0.3, 0.6)));
        Assert.Equal(0.51, HeightOf(2), 0.005);

        Walk(2, 1.5, 0.0, 2.0);
        Assert.True(World.SetStance(2, Shape.Capsule(0.3, 0.6)));
        Assert.Equal(0.91, HeightOf(2), 0.005);
    }

    [Fact]
    public void AStanceIsRefusedForWhatIsNotACharacterOrARigidShape()
    {
        Floor();
        World.Add(3, Shape.Sphere(0.5), at: [3.0, 0.5, 0.0]);
        World.AddCharacter(2, at: [0.0, 0.91, 0.0]);

        Assert.False(World.SetStance(3, Shape.Capsule(0.3, 0.6)));
        Assert.False(World.SetStance(2, Shape.Plane(0.0, 1.0, 0.0)));
        Assert.False(World.SetStance(99, Shape.Capsule(0.3, 0.6)));
    }

    [Fact]
    public void ACharacterIsToldWhenItsWalkMetAWallOrACeiling()
    {
        Floor();
        World.Add(3, Shape.Box(0.25, 1.0, 2.0), PhysicsMotion.Fixed, at: [2.0, 1.0, 0.0]);
        World.Add(4, Shape.Box(8.0, 0.1, 4.0), PhysicsMotion.Fixed, at: [0.0, 2.3, 0.0]);
        World.AddCharacter(2, at: [0.0, 0.91, 0.0]);

        Walk(2, 1.0, 0.0, 0.5);
        Assert.False(World.FootingOf(2)!.Wall);
        Assert.False(World.FootingOf(2)!.Ceiling);

        Walk(2, 1.0, 0.0, 2.0);
        Assert.True(World.FootingOf(2)!.Wall);
        Assert.False(World.FootingOf(2)!.Ceiling);

        var bumped = false;
        for (var i = 0; i < 12; i++)
        {
            World.Drive(2, [0.0, 8.0, 0.0]);
            World.Step(1.0 / 60);
            bumped = bumped || World.FootingOf(2)!.Ceiling;
        }

        Assert.True(bumped);
    }

    [Fact]
    public void ACharacterRidesAPlatformAndIsToldHowFarItWasCarried()
    {
        World.Add(3, Shape.Box(2.0, 0.1, 2.0), PhysicsMotion.Driven, at: [0.0, 1.0, 0.0]);
        World.AddCharacter(2, at: [0.0, 2.01, 0.0]);
        World.Drive(3, [1.0, 0.0, 0.0]);

        Assert.Equal(0, Walk(2, 0.0, 0.0, 1.0));

        Assert.Equal(1.0, World.TransformOf(2)![0], 0.02);
        Assert.Equal(2.01, HeightOf(2), 0.005);
        var footing = World.FootingOf(2)!;
        Assert.Equal(3UL, footing.Ground);
        Assert.Equal(1.0, footing.Carried[0], 0.01);
        // The ride is not a walk.
        Assert.Equal(0.0, footing.Velocity[0], 1e-4);
    }

    [Fact]
    public void FootingsAreReadTogetherAndOnlyCharactersHaveOne()
    {
        Floor();
        World.AddCharacter(2, at: [0.0, 0.91, 0.0]);
        World.AddCharacter(3, at: [2.0, 5.0, 0.0]);
        World.Add(4, Shape.Sphere(0.5), at: [4.0, 0.5, 0.0]);
        World.Step(1.0 / 60);

        var footings = World.FootingsOf([4, 2, 99, 3]);
        Assert.Equal(4, footings.Length);
        Assert.Null(footings[0]);
        Assert.Null(footings[2]);
        Assert.True(footings[1]!.Grounded);
        Assert.Equal(Ground, footings[1]!.Ground);
        Assert.False(footings[3]!.Grounded);
        Assert.Equal(0UL, footings[3]!.Ground);
        Assert.Empty(World.FootingsOf([]));

        // A half-space has no underneath to stand anything on.
        World.AddCharacter(5, Shape.Plane(1.0, 0.0, 0.0));
        Assert.Null(World.FootingOf(5));

        World.Remove(2);
        Assert.Null(World.FootingOf(2));
    }

    [Fact]
    public void GroundHoldsUpWhatLandsOnItAndItsStructSurvivesTheCrossing()
    {
        // Each body below lands somewhere only one field of the struct puts
        // ground: take any field from its neighbour's slot and one of them falls.
        const ulong field = 0x300000007;
        Assert.True(World.LayGround(
            field,
            HeightsOf(5, height: 1.0),
            columns: 5,
            rows: 5,
            spacing: 2.0,
            at: [-4.0, 0.5, -4.0],
            friction: 0.0,
            restitution: 0.8,
            layers: new Layers(Is: 2, Cares: 2)));
        Assert.True(World.Alive(field));

        // Past x = 2 only because the spacing is two, and at 1.5 only because
        // the field is raised half a metre.
        World.Add(2, Shape.Sphere(0.5), at: [3.0, 4.0, 3.5]);
        // Off the edge of it, where there is nothing.
        World.Add(3, Shape.Sphere(0.5), at: [4.5, 4.0, 0.0]);
        // Watching nothing the ground is in, and nothing watching it.
        World.Add(4, Shape.Sphere(0.5), at: [0.0, 4.0, 0.0], layers: new Layers(Is: 4, Cares: 4));
        World.Add(5, Shape.Sphere(0.5), at: [-2.0, 4.0, 0.0], restitution: 0.0);

        var highest = 0.0;
        for (var i = 0; i < 180; i++)
        {
            World.Step(1.0 / 60);
            if (i > 60 && HeightOf(5) > highest)
            {
                highest = HeightOf(5);
            }
        }

        // Bouncing takes the ground's restitution, which is only there if it
        // landed in its own slot and not in friction's.
        Assert.True(highest > 2.0, "the ground is springy");
        Assert.True(HeightOf(3) < 0.0, "past the edge");
        Assert.True(HeightOf(4) < 0.0, "on a layer it ignores");

        World.Remove(5);
        Run(3);
        Assert.Equal(2.0, HeightOf(2), 0.05);

        var down = World.Cast([1.0, 9.0, 1.0], [0.0, -1.0, 0.0], 20.0)!;
        Assert.Equal(field, down.Body);
        Assert.Equal(7.5, down.Distance, 0.01);
        Assert.Equal(1.0, down.Normal[1], 0.001);
    }

    [Fact]
    public void GroundWithAHoleLetsThingsThroughAndRelayingItMovesThem()
    {
        const ulong field = 7;
        var holed = HeightsOf(9, heightAt: (c, r) => c == 4 && r == 4 ? double.NaN : 0.0);
        Assert.True(World.LayGround(field, holed, columns: 9, rows: 9, at: [-4.0, 0.0, -4.0]));
        World.Add(2, Shape.Sphere(0.3), at: [0.0, 2.0, 0.0]);
        World.Add(3, Shape.Box(0.3, 0.3, 0.3), at: [2.5, 1.0, 2.5]);
        Run(3);
        Assert.True(HeightOf(2) < -1.0, "down the hole");
        Assert.Equal(0.3, HeightOf(3), 0.03);
        Assert.True(World.Asleep(3));

        // Laid again half a metre down, whole: the sleeping crate is woken and
        // follows it rather than hanging where the ground was.
        World.LayGround(field, HeightsOf(9, height: -0.5), columns: 9, rows: 9, at: [-4.0, 0.0, -4.0]);
        Assert.Equal(3, World.Count);
        Run(2);
        Assert.Equal(-0.2, HeightOf(3), 0.03);

        // Taken away, it wakes nothing: ground streamed out from under a
        // sleeping crate far off leaves it where it was for when it comes back.
        Run(2);
        Assert.True(World.Asleep(3));
        World.Remove(field);
        Run(1);
        Assert.Equal(-0.2, HeightOf(3), 0.03);
        World.Wake(3);
        Run(1);
        Assert.True(HeightOf(3) < -1.0, "the ground is gone");
    }

    [Fact]
    public void AMarginIsTheNeighboursGroundNeverStoodOn()
    {
        // Six by six with a margin is four by four of its own, from one to four.
        // The margin is a wall of heights that would stop anything if it were
        // ground. As a margin it only says which way the ground carries on.
        var walled = HeightsOf(6, heightAt: (c, r) => c == 0 || r == 0 || c == 5 || r == 5 ? 3.0 : 0.0);
        Assert.True(World.LayGround(10, walled, columns: 6, rows: 6, margin: true));
        World.Add(2, Shape.Sphere(0.2), at: [2.5, 1.0, 2.5]);
        World.Add(3, Shape.Sphere(0.2), at: [0.5, 4.0, 2.5]);
        Run(2);
        Assert.Equal(0.2, HeightOf(2), 0.02);
        Assert.True(HeightOf(3) < 0.0, "over the margin, so nothing");
    }

    [Fact]
    public void GroundRefusesAGridItCannotLay()
    {
        var four = HeightsOf(4);
        bool Lay(
            ulong id = 9,
            int columns = 4,
            int rows = 4,
            double spacing = 1.0,
            bool margin = false,
            double[]? heights = null) => World.LayGround(
                id,
                heights ?? four,
                columns,
                rows,
                spacing,
                margin);

        Assert.False(Lay(id: 0), "nameless");
        Assert.False(Lay(columns: 1), "no square to stand on");
        Assert.False(Lay(spacing: 0.0));
        Assert.False(Lay(spacing: double.NaN));
        Assert.False(Lay(heights: HeightsOf(3)), "too few heights");
        Assert.False(Lay(columns: 3, rows: 3, margin: true), "all margin");
        Assert.Equal(0, World.Count);
        Assert.True(Lay(margin: true), "four is enough with a margin");
        Assert.True(Lay(columns: 2, rows: 2, id: 10), "two without");
        Assert.Equal(2, World.Count);
    }

    [Fact]
    public void ReadIntoFillsAStridedBufferAndSkipsWhatItDoesNotKnow()
    {
        Floor();
        World.Add(2, Shape.Sphere(0.5), at: [1.0, 2.0, 3.0]);
        World.Add(4, Shape.Sphere(0.5), at: [4.0, 5.0, 6.0]);

        // Ten floats a row, starting one in: the shape of a transform column
        // that holds translation, rotation and scale.
        var output = new float[30];
        Array.Fill(output, -1.0f);
        var written = World.ReadInto([2, 3, 4], output, stride: 10, offset: 1);

        Assert.Equal(2, written);
        Assert.Equal([1.0f, 2.0f, 3.0f], output[1..4]);
        Assert.Equal([4.0f, 5.0f, 6.0f], output[21..24]);
        // The unknown id's row, and every float outside a row, is untouched.
        Assert.All(output[11..18], v => Assert.Equal(-1.0f, v));
        Assert.Equal(-1.0f, output[0]);
        Assert.Equal(-1.0f, output[8]);

        Assert.Equal(0, World.ReadInto([2], output, stride: 6));
    }

    [Fact]
    public void TheWorldRefusesANamelessOrRepeatedBody()
    {
        World.Add(0, Shape.Sphere(0.5));
        World.Add(2, Shape.Sphere(0.5));
        World.Add(2, Shape.Box(1.0, 1.0, 1.0));
        Assert.Equal(1, World.Count);

        Assert.True(World.Alive(2));
        Assert.False(World.Alive(3));
        Assert.Null(World.TransformOf(3));
        Assert.Null(World.VelocityOf(3));

        // A command naming a body that is no longer there is simply dropped.
        World.Remove(2);
        World.Place(2, [0.0, 9.0, 0.0]);
        Assert.Equal(0, World.Count);
    }

    [Fact]
    public void SettingsLeftNullKeepTheEngineDefaults()
    {
        using var light = new Physics(new PhysicsSettings { Gravity = [0.0, -1.0, 0.0], Sleeping = false });

        light.Add(2, Shape.Sphere(0.5), at: [0.0, 10.0, 0.0]);
        for (var i = 0; i < 60; i++)
        {
            light.Step(1.0 / 60);
        }

        // A second under Earth's gravity costs about 4.9 metres. Under this
        // one, about half of one. Damping trims both a little.
        Assert.True(10.0 - light.TransformOf(2)![1] < 1.0);
        Assert.False(light.Asleep(2));
    }

    [Fact]
    public void ADisposedWorldSaysSoRatherThanCrashing()
    {
        var short_ = new Physics();
        short_.Dispose();
        short_.Dispose();

        Assert.Throws<InvalidOperationException>(() => short_.Step(1.0 / 60));
        Assert.Throws<InvalidOperationException>(() => short_.Add(2, Shape.Sphere(0.5)));
        Assert.Throws<InvalidOperationException>(() => short_.Count);
    }
}
