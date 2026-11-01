using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftBodyTests : SoftTestBase
{
    /// <summary>A 20 by 20 sheet standing in the x-y plane, its top row at y = 1.</summary>
    private static SoftBody Curtain(SoftSpec? spec = null) =>
        SoftBody.Cloth(
            Columns,
            Columns,
            new Vector3(0, 1, 0),
            new Vector3(Spacing, 0, 0),
            new Vector3(0, -Spacing, 0),
            Mass,
            spec);

    /// <summary>
    /// The worst way to hang a sheet: all of it from the two top corners. A
    /// finer sub-step than the default keeps the links near the pins from giving.
    /// </summary>
    private static SoftSpec Corners(double compliance = 1e-6) =>
        new(substepLength: 1.0 / 960, stretchCompliance: compliance);

    private static SoftBody Hung(SoftSpec? spec = null)
    {
        var cloth = Curtain(spec ?? Corners());
        cloth.Pin(0);
        cloth.Pin(Columns - 1);
        return cloth;
    }

    private static double Energy(SoftBody cloth, SoftSpec spec)
    {
        var total = 0.0;
        for (var i = 0; i < cloth.Count; i++)
        {
            if (cloth.IsPinned(i))
            {
                continue;
            }

            var p = cloth.PositionOf(i);
            total += (0.5 * Mass * cloth.VelocityOf(i).Length2) - (Mass * spec.Gravity.Dot(p));
        }

        return total;
    }

    [Fact]
    public void ACurtainPinnedAtTwoCornersSettlesWithLittleStretch()
    {
        var cloth = Hung();
        Run(cloth, 5);
        Less(cloth.MaxStretch, 0.05);
        Less(cloth.MaxSpeed, 0.01);
    }

    [Fact]
    public void ACurtainPinnedAlongItsTopEdgeStretchesLessAtTheDefault()
    {
        var cloth = Curtain();
        for (var i = 0; i < Columns; i++)
        {
            cloth.Pin(i);
        }

        Run(cloth, 5);
        Less(cloth.MaxStretch, 0.05);
        Less(cloth.MaxSpeed, 0.01);
    }

    [Fact]
    public void ItHangsTheSameAt30And60FramesASecond()
    {
        var fast = Hung();
        var slow = Hung();
        Run(fast, 5, 1.0 / 60);
        Run(slow, 5, 1.0 / 30);
        Less(Math.Abs(fast.MaxStretch - slow.MaxStretch), 0.01);
        var low = (Columns * Columns) - (Columns / 2);
        Less(fast.PositionOf(low).DistanceTo(slow.PositionOf(low)), 0.02);
    }

    [Fact]
    public void AStifferLinkStretchesLess()
    {
        var soft = Hung(Corners(1e-3));
        var stiff = Hung(Corners(1e-7));
        Run(soft, 3);
        Run(stiff, 3);
        Greater(soft.MaxStretch, stiff.MaxStretch);
    }

    [Fact]
    public void TighterBendLinksLetAClampedSheetDroopLess()
    {
        static double Droop(double compliance)
        {
            var cloth = SoftBody.Cloth(
                Columns,
                Columns,
                new Vector3(0, 1, 0),
                new Vector3(Spacing, 0, 0),
                new Vector3(0, 0, Spacing),
                Mass,
                new SoftSpec(bendCompliance: compliance));

            // Two columns clamp it. One only lets the sheet swing about its edge.
            for (var j = 0; j < Columns; j++)
            {
                cloth.Pin(j * Columns);
                cloth.Pin((j * Columns) + 1);
            }

            Run(cloth, 3);

            // A fold squeezes bend links a long way. That is not stretch.
            Less(cloth.MaxStretch, 0.05);
            return 1 - cloth.PositionOf(Columns - 1).Y;
        }

        Less(Droop(1e-5), Droop(10) - 0.02);
    }

    [Fact]
    public void APinnedCurtainNeverGainsEnergy()
    {
        var spec = new SoftSpec();
        var cloth = Hung(spec);
        var start = Energy(cloth, spec);
        var worst = double.NegativeInfinity;
        for (var n = 0; n < 300; n++)
        {
            cloth.Step(Dt);
            worst = Math.Max(worst, Energy(cloth, spec) - start);
        }

        Less(worst, 0.01 * Math.Abs(start));
    }

    [Fact]
    public void AFreeSheetFallsAsABodyDoes()
    {
        var cloth = Curtain(new SoftSpec(damping: 0));
        var top = cloth.PositionOf(0).Y;
        Run(cloth, 0.5);
        var dropped = top - cloth.PositionOf(0).Y;
        Assert.Equal(0.5 * 9.81 * 0.25, dropped, 0.02);
    }

    [Fact]
    public void ABodyThatHasComeToRestStopsBeingStepped()
    {
        var cloth = Hung();
        Run(cloth, 10);
        Assert.True(cloth.Asleep);
    }

    [Fact]
    public void MovingAPinWakesIt()
    {
        var cloth = Hung();
        Run(cloth, 10);
        cloth.Place(0, cloth.PositionOf(0) + new Vector3(0.1, 0, 0));
        Assert.False(cloth.Asleep);
        var low = (Columns * Columns) - 1;
        var before = cloth.PositionOf(low);
        Run(cloth, 1);
        Assert.False(Same(cloth.PositionOf(low), before));
    }

    [Fact]
    public void APinnedParticleStaysAndAMovedOneIsFollowed()
    {
        var cloth = Hung();
        var held = cloth.PositionOf(0);
        Run(cloth, 1);
        Assert.True(Same(cloth.PositionOf(0), held));
        cloth.Place(0, held + new Vector3(0, 0, 0.2));
        Assert.Equal(0.2, cloth.PositionOf(0).Z, 1e-12);
    }

    [Fact]
    public void AReleasedParticleIsFreeAgain()
    {
        var cloth = Curtain();
        cloth.Pin(0);
        cloth.Release(0);
        Assert.False(cloth.IsPinned(0));
    }

    [Fact]
    public void PlacingAFreeParticleIsRefused()
    {
        Assert.Throws<InvalidOperationException>(() => Curtain().Place(3, Vector3.Zero()));
    }

    [Fact]
    public void ARopeHungFromOneEndSwingsDownAndHangsStraight()
    {
        var rope = SoftBody.Rope([.. Enumerable.Range(0, 20).Select(i => new Vector3(i * 0.05, 1, 0))]);
        rope.Pin(0);
        Run(rope, 8);
        var tip = rope.PositionOf(19);
        Assert.Equal(0, tip.X, 0.05);
        Assert.Equal(1 - (19 * 0.05), tip.Y, 0.05);
    }

    [Fact]
    public void ASpecOutsideItsRange()
    {
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(damping: -1));
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(stretchCompliance: -1));
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(bendCompliance: -1));
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(substepLength: 0));
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(substepLength: 0.1));
        Assert.ThrowsAny<ArgumentException>(() => new SoftSpec(iterations: 0));
    }

    [Fact]
    public void AClothOrRopeTooSmallToHoldTogether()
    {
        static Vector3 V() => new(1, 0, 0);
        Assert.ThrowsAny<ArgumentException>(() => SoftBody.Cloth(1, 4, Vector3.Zero(), V(), V()));
        Assert.ThrowsAny<ArgumentException>(() => SoftBody.Rope([Vector3.Zero()]));
        Assert.ThrowsAny<ArgumentException>(() => SoftBody.Rope([Vector3.Zero(), V()], particleMass: 0));
    }

    [Fact]
    public void AStepThatGoesNowhere()
    {
        Assert.ThrowsAny<ArgumentException>(() => Curtain().Step(0));
        Assert.ThrowsAny<ArgumentException>(() => Curtain().Step(double.NaN));
    }
}
