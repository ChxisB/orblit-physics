using System.Diagnostics;
using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftCollisionTests : SoftTestBase
{
    private const double Skin = 0.01;

    /// <summary>A world with a floor whose top is at y = 0, and whatever <paramref name="fill"/> adds.</summary>
    private static Physics World(Action<Physics>? fill = null, double floor = 1)
    {
        var physics = new Physics();
        physics.Add(1, Shape.Box(5, floor, 5), PhysicsMotion.Fixed, at: [0, -floor, 0]);
        fill?.Invoke(physics);
        physics.Step(Dt);
        return physics;
    }

    /// <summary>A flat 20 by 20 sheet with its corner at <paramref name="origin"/>, its sides along x and z.</summary>
    private static SoftBody Sheet(
        Physics physics,
        Vector3 origin,
        SoftSpec? spec = null,
        Vector3? across = null,
        Vector3? down = null) =>
        SoftBody.Cloth(
            Columns,
            Columns,
            origin,
            across ?? new Vector3(Spacing, 0, 0),
            down ?? new Vector3(0, 0, Spacing),
            spec: spec,
            surface: new PhysicsSurface(physics));

    private static int Middle => (Columns / 2 * Columns) + (Columns / 2);

    private static Physics WithSphere(Vector3 middle) =>
        World(w => w.Add(2, Shape.Sphere(0.25), PhysicsMotion.Fixed, at: [middle.X, middle.Y, middle.Z]));

    [Fact]
    public void ASphereCatchesAClothWhichDrapesOverItAndNeverEndsInside()
    {
        var middle = new Vector3(0.475, 0.25, 0.475);
        using var physics = WithSphere(middle);
        var cloth = Sheet(physics, new Vector3(0, 0.8, 0));
        Run(cloth, 5);
        foreach (var p in Points(cloth))
        {
            Greater(p.DistanceTo(middle), 0.25 + Skin - 0.002);
            Greater(p.Y, Skin - 0.002);
        }

        Assert.Equal(0.5 + Skin, cloth.PositionOf(Middle).Y, 0.01);
        Less(cloth.MaxSpeed, 0.05);
    }

    [Fact]
    public void ASphereIsNotEnteredBetweenTheParticlesThatTouchIt()
    {
        var middle = new Vector3(0.475, 0.25, 0.475);
        using var physics = WithSphere(middle);
        var cloth = Sheet(physics, new Vector3(0, 0.8, 0));
        Run(cloth, 5);
        for (var j = 0; j < Columns; j++)
        {
            for (var i = 0; i + 1 < Columns; i++)
            {
                var a = cloth.PositionOf((j * Columns) + i);
                var b = cloth.PositionOf((j * Columns) + i + 1);
                Greater(((a + b) * 0.5).DistanceTo(middle), 0.25 - 0.003);
            }
        }
    }

    [Fact]
    public void ABoxCatchesAClothWhichHangsOverItsEdges()
    {
        const double half = 0.2;
        var centre = new Vector3(0.475, half, 0.475);
        using var physics = World(w => w.Add(2, Shape.Box(half, half, half), PhysicsMotion.Fixed, at: [centre.X, centre.Y, centre.Z]));
        var cloth = Sheet(physics, new Vector3(0, 0.8, 0));
        Run(cloth, 5);
        foreach (var p in Points(cloth))
        {
            var inside = Enumerable.Range(0, 3).All(c => Math.Abs(p[c] - centre[c]) < half + Skin - 0.002);
            Assert.False(inside, $"({p.X}, {p.Y}, {p.Z}) is inside the box");
            Greater(p.Y, Skin - 0.002);
        }

        Assert.Equal((2 * half) + Skin, cloth.PositionOf(Middle).Y, 0.01);
        Less(cloth.PositionOf(0).Y, 2 * half);
    }

    [Fact]
    public void AClothDroppedFromAHeightDoesNotGoThroughAThinFloor()
    {
        using var physics = World(floor: 0.05);
        var cloth = Sheet(physics, new Vector3(0, 20, 0), new SoftSpec(damping: 0));
        var fastest = 0.0;
        Run(
            cloth,
            4,
            each: () =>
            {
                fastest = Math.Max(fastest, cloth.MaxSpeed);
                foreach (var p in Points(cloth))
                {
                    Greater(p.Y, Skin - 0.002);
                }
            });

        // It was going fast enough to cross a skin many times over in one step.
        Greater(fastest, 10);
        Less(cloth.MaxSpeed, 0.05);
    }

    [Fact]
    public void AClothLandingFlatStopsDeadAndDoesNotBounce()
    {
        using var physics = World();
        var cloth = Sheet(physics, new Vector3(0, 5, 0), new SoftSpec(damping: 0));

        // It lands at 1.01 s, at 9.9 m/s.
        Run(cloth, 1.2);
        Less(cloth.MaxSpeed, 1);
        Less(cloth.PositionOf(0).Y, 0.03);
    }

    [Fact]
    public void ABodyThatBeginsInsideIsPushedOutWithoutBeingThrown()
    {
        using var physics = World(w => w.Add(2, Shape.Box(2, 0.3, 2), PhysicsMotion.Fixed, at: [0, 0.8, 0]));
        var cloth = Sheet(physics, new Vector3(0, 1.0, 0));

        // It is 10 cm deep. A skin a sub-step is all it may be moved, so it
        // cannot have come out in the first step, which has 8 sub-steps.
        cloth.Step(Dt);
        Less(cloth.PositionOf(Middle).Y, 1.0 + (8 * Skin) + 0.001);
        var fastest = 0.0;
        Run(cloth, 1, each: () => fastest = Math.Max(fastest, cloth.MaxSpeed));
        Less(fastest, 8);
        foreach (var p in Points(cloth))
        {
            Greater(p.Y, 1.1 + Skin - 0.002);
        }
    }

    [Fact]
    public void APinnedParticleIsNotMovedByASurface()
    {
        using var physics = World();
        var cloth = Sheet(physics, new Vector3(0, 0.005, 0));
        cloth.Pin(0);
        var held = cloth.PositionOf(0);
        Run(cloth, 1);
        Assert.True(Same(cloth.PositionOf(0), held));
    }

    [Fact]
    public void FrictionHoldsAClothOnASlopeAndLetsGoOfItWithoutAny()
    {
        const double angle = 20 * Math.PI / 180;
        var along = new Vector3(Math.Cos(angle), Math.Sin(angle), 0);
        var normal = new Vector3(-Math.Sin(angle), Math.Cos(angle), 0);

        // A sheet lying on a ramp that rises towards +x, and how far down the
        // ramp its middle goes in the given seconds.
        double Slide(double friction, double seconds)
        {
            using var physics = new Physics();
            physics.Add(
                1,
                Shape.Box(3, 0.1, 3),
                PhysicsMotion.Fixed,
                rotation: [0, 0, Math.Sin(angle / 2), Math.Cos(angle / 2)]);
            physics.Step(Dt);
            var cloth = Sheet(
                physics,
                (normal * (0.1 + Skin + 0.001)) - (along * 0.475) - new Vector3(0, 0, 0.475),
                new SoftSpec(friction: friction),
                across: along * Spacing);
            var start = Centre(cloth);
            Run(cloth, seconds);
            return (start - Centre(cloth)).Dot(along);
        }

        Less(Slide(2.0, 2), 0.01);
        Greater(Slide(0.05, 1), 0.5);
    }

    [Fact]
    public void FourHundredParticlesWithASurfaceCostUnderAFewMillisecondsAStep()
    {
        using var physics = WithSphere(new Vector3(0.475, 0.25, 0.475));
        var cloth = Sheet(physics, new Vector3(0, 0.55, 0));
        Run(cloth, 0.3);
        var clock = Stopwatch.StartNew();
        Run(cloth, 1);
        var each = clock.Elapsed.TotalMilliseconds / 60;

        // The aim is 1 ms. The bound only catches a step that has got far worse.
        Less(each, 5);
    }
}
