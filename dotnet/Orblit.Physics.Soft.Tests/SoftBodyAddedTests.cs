using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftBodyAddedTests : SoftTestBase
{
    private const double EighthOfASecond = 0.0625;

    /// <summary>
    /// A hanging pair that gravity can barely move, so it comes to rest in
    /// about a second.
    /// </summary>
    private static SoftBody Quiet(double gravity, ISoftAir? air = null) =>
        SoftBody.Rope(Line(2), spec: new SoftSpec(gravity: new Vector3(0, -gravity, 0), damping: 10), air: air);

    private static void SleepAway(SoftBody body)
    {
        for (var n = 0; n < 200 && !body.Asleep; n++)
        {
            body.Step(EighthOfASecond);
        }

        Assert.True(body.Asleep);
    }

    private static int[] TetrahedronWith(int at, int index)
    {
        var triangles = TetrahedronTriangles();
        triangles[at] = index;
        return triangles;
    }

    private static string Refused(Action make) => Assert.Throws<ArgumentOutOfRangeException>(make).ParamName!;

    [Fact]
    public void ABodyKeepsTheSpecSurfaceAndAirItWasGiven()
    {
        var spec = new SoftSpec();
        var breeze = new Breeze();
        var rope = SoftBody.Rope(Line(2), spec: spec, air: breeze);
        Assert.Same(spec, rope.Spec);
        Assert.Same(breeze, rope.Air);
        Assert.Null(rope.Surface);
        Assert.Null(rope.Volume);
    }

    [Fact]
    public void AClothNeedsTwoColumnsAndTwoRows()
    {
        var across = new Vector3(1, 0, 0);
        var down = new Vector3(0, 1, 0);
        Assert.Throws<ArgumentException>(() => SoftBody.Cloth(1, 2, Vector3.Zero(), across, down));
        Assert.Throws<ArgumentException>(() => SoftBody.Cloth(2, 1, Vector3.Zero(), across, down));
        Assert.Equal(4, SoftBody.Cloth(2, 2, Vector3.Zero(), across, down).Count);
    }

    [Fact]
    public void AClothIsLaidOutAlongItsRowsFromTheOrigin()
    {
        var cloth = SoftBody.Cloth(3, 2, new Vector3(1, 2, 3), new Vector3(0.1, 0, 0), new Vector3(0, 0, 0.2));
        Assert.Equal(6, cloth.Count);
        var corner = cloth.PositionOf(4);
        Assert.Equal(1.1, corner.X, 12);
        Assert.Equal(2.0, corner.Y, 12);
        Assert.Equal(3.2, corner.Z, 12);
    }

    [Fact]
    public void ARopeNeedsTwoParticles()
    {
        var one = Assert.Throws<ArgumentException>(() => SoftBody.Rope(Line(1)));
        Assert.Equal("points", one.ParamName);
        Assert.Equal(2, SoftBody.Rope(Line(2)).Count);
    }

    [Fact]
    public void AParticleMassMustBeFiniteAndAboveZero()
    {
        foreach (var bad in new[] { 0.0, -1.0, double.NaN, double.PositiveInfinity })
        {
            Assert.Equal("mass", Refused(() => SoftBody.Rope(Line(2), bad)));
            Assert.Equal("mass", Refused(() => SoftBody.Balloon(Tetrahedron(), TetrahedronTriangles(), bad)));
            Assert.Equal(
                "mass",
                Refused(() => SoftBody.Cloth(2, 2, Vector3.Zero(), new Vector3(1, 0, 0), new Vector3(0, 1, 0), bad)));
        }
    }

    [Fact]
    public void AnIndexOutsideTheBodyIsRefusedByEveryMethodThatTakesOne()
    {
        var calls = new (string Name, Action<SoftBody, int> Call)[]
        {
            ("PositionOf", (b, i) => b.PositionOf(i)),
            ("VelocityOf", (b, i) => b.VelocityOf(i)),
            ("IsPinned", (b, i) => b.IsPinned(i)),
            ("Pin", (b, i) => b.Pin(i)),
            ("Release", (b, i) => b.Release(i)),
            ("Attach", (b, i) => b.Attach(i, Vector3.Zero)),
            ("Place", (b, i) => b.Place(i, Vector3.Zero())),
        };
        foreach (var (name, call) in calls)
        {
            var body = SoftBody.Rope(Line(3));
            body.Pin(0);
            body.Pin(2);
            call(body, 0);
            call(body, 2);
            foreach (var bad in new[] { -1, 3 })
            {
                var error = Assert.Throws<ArgumentOutOfRangeException>(() => call(body, bad));
                Assert.True(error.ParamName == "i", $"{name}({bad}) named {error.ParamName}");
            }
        }
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(0, 2)]
    [InlineData(0, 3)]
    [InlineData(1, 2)]
    [InlineData(1, 3)]
    [InlineData(2, 3)]
    public void EachEdgeAndDiagonalOfASheetCountsInTheStretch(int a, int b)
    {
        var cloth = SoftBody.Cloth(2, 2, Vector3.Zero(), new Vector3(1, 0, 0), new Vector3(0, 1, 0));
        for (var i = 0; i < 4; i++)
        {
            cloth.Pin(i);
        }

        cloth.Place(a, cloth.PositionOf(b));
        Assert.Equal(1.0, cloth.MaxStretch);
    }

    [Fact]
    public void AFoldedRopeIsNotStretched()
    {
        var rope = SoftBody.Rope(Line(3));
        for (var i = 0; i < 3; i++)
        {
            rope.Pin(i);
        }

        rope.Place(2, rope.PositionOf(0));
        Assert.Equal(0.0, rope.MaxStretch);
    }

    [Theory]
    [InlineData(10, 0, 0)]
    [InlineData(0, 10, 0)]
    [InlineData(0, 0, 10)]
    [InlineData(0, -10, 0)]
    public void AFreeBodyGainsSpeedForNoMoreThanTheLongestStep(double x, double y, double z)
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(gravity: new Vector3(x, y, z), damping: 0));
        rope.Step(1.0);
        var v = rope.VelocityOf(0);
        Assert.Equal(x * 0.1, v.X, 6);
        Assert.Equal(y * 0.1, v.Y, 6);
        Assert.Equal(z * 0.1, v.Z, 6);
        Assert.Equal(1.0, rope.MaxSpeed, 6);
    }

    [Theory]
    [InlineData(1e-4)]
    [InlineData(0.005)]
    [InlineData(0.1)]
    public void ASpanIsSpentWholeHoweverItIsCutIntoSubsteps(double span)
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(damping: 0));
        rope.Step(span);
        Assert.Equal(-9.81 * span, rope.VelocityOf(0).Y, 9);
    }

    [Fact]
    public void AHeavyDampingStopsAFreeBodyAndNeverReversesIt()
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(damping: 1e6));
        rope.Step(0.1);
        Assert.Equal(0.0, rope.VelocityOf(0).Y);
        Assert.Equal(0.0, rope.PositionOf(0).Y);
    }

    [Fact]
    public void TheAirDragsAFallingBodyBackEvenWhenTheWindIsStill()
    {
        var open = SoftBody.Rope(Line(2), spec: new SoftSpec(damping: 0));
        var dragged = SoftBody.Rope(Line(2), spec: new SoftSpec(damping: 0, windDrag: 10), air: new Breeze());
        open.Step(0.1);
        dragged.Step(0.1);
        Less(-dragged.VelocityOf(0).Y, -open.VelocityOf(0).Y);
        Greater(-dragged.VelocityOf(0).Y, 0.5);
    }

    private static double HangStretch(int iterations, double mass)
    {
        var points = Enumerable.Range(0, 10).Select(i => new Vector3(0, -0.1 * i, 0)).ToList();
        var rope = SoftBody.Rope(points, mass, new SoftSpec(iterations: iterations));
        rope.Pin(0);
        Run(rope, 1.0);
        return rope.MaxStretch;
    }

    [Fact]
    public void MorePassesHoldALoadWithLessStretch()
    {
        Less(HangStretch(3, 0.01), HangStretch(1, 0.01));
    }

    [Fact]
    public void AHeavierParticleStretchesAChainMore()
    {
        Greater(HangStretch(1, 0.1), HangStretch(1, 0.01));
    }

    [Fact]
    public void ABodyStillForOneSecondSleepsAtOneSecondAndNotBefore()
    {
        var pair = SoftBody.Rope(Line(2));
        pair.Pin(0);
        pair.Pin(1);
        for (var n = 0; n < 15; n++)
        {
            pair.Step(EighthOfASecond);
        }

        Assert.False(pair.Asleep);
        pair.Step(EighthOfASecond);
        Assert.True(pair.Asleep);
    }

    [Fact]
    public void ABodyMovingSlowerThanTheSleepSpeedSleeps()
    {
        SleepAway(Quiet(0.045));
    }

    [Fact]
    public void ABodyMovingFasterThanTheSleepSpeedStaysAwake()
    {
        var body = Quiet(0.055);
        Run(body, 3.0, EighthOfASecond);
        Assert.False(body.Asleep);
    }

    [Fact]
    public void AFallingBodyNeverSleeps()
    {
        var body = SoftBody.Rope(Line(2));
        Run(body, 3.0);
        Assert.False(body.Asleep);
    }

    [Fact]
    public void ASleepingBodyStaysWhereItIsUntilItIsWoken()
    {
        var body = Quiet(0.045);
        SleepAway(body);
        var before = body.PositionOf(1);
        body.Step(EighthOfASecond);
        Assert.True(Same(before, body.PositionOf(1)));
        body.Wake();
        Assert.False(body.Asleep);
        body.Step(EighthOfASecond);
        Assert.False(Same(before, body.PositionOf(1)));
    }

    [Fact]
    public void LettingGoOfAParticleThatWasNeverAttachedLeavesABodyAsleep()
    {
        var body = Quiet(0.045);
        SleepAway(body);
        body.Detach(1);
        Assert.True(body.Asleep);
    }

    [Fact]
    public void LettingGoOfAnAttachedParticleFreesItAndWakesTheBody()
    {
        var body = Quiet(0.045);
        body.Attach(0, Vector3.Zero);
        SleepAway(body);
        body.Detach(0);
        Assert.False(body.IsAttached(0));
        Assert.False(body.IsPinned(0));
        Assert.False(body.Asleep);
    }

    [Fact]
    public void TheForceOnAnAttachedParticleIsACopy()
    {
        var rope = SoftBody.Rope(Line(2));
        rope.Attach(0, Vector3.Zero);
        Run(rope, 0.5);
        var pull = rope.PullOf(0);
        var y = pull.Y;
        Less(y, -0.05);
        pull.Y = 123;
        Assert.Equal(y, rope.PullOf(0).Y);
    }

    [Theory]
    [InlineData(1e-7, false)]
    [InlineData(1e-5, true)]
    public void OnlyAChangeInTheWindOfAMicrometreOrMoreWakesABody(double change, bool wakes)
    {
        var breeze = new Breeze();
        var body = Quiet(0.045, breeze);
        SleepAway(body);
        breeze.Wind = new Vector3(change, 0, 0);
        body.Step(Dt);
        Assert.Equal(wakes, !body.Asleep);
    }

    [Theory]
    [InlineData(1e-10, false)]
    [InlineData(1e-8, true)]
    public void OnlyAnAnchorThatMovesFarEnoughWakesABody(double move, bool wakes)
    {
        var body = Quiet(0.045);
        var at = Vector3.Zero();
        body.Attach(0, () => at);
        SleepAway(body);
        at.X = move;
        body.Step(Dt);
        Assert.Equal(wakes, !body.Asleep);
    }

    private static double FoldedSpan(double bendCompliance)
    {
        var spec = new SoftSpec(gravity: Vector3.Zero(), bendCompliance: bendCompliance);
        var rope = SoftBody.Rope([new(0, 0, 0), new(1, 0, 0), new(2, 0, 0)], spec: spec);
        rope.Pin(0);
        rope.Pin(1);
        rope.Pin(2);
        rope.Place(2, new Vector3(1, 1, 0));
        rope.Release(2);
        Run(rope, 1.0);
        return rope.PositionOf(0).DistanceTo(rope.PositionOf(2));
    }

    [Fact]
    public void AStiffLinkTwoApartStraightensAFoldAndASoftOneLeavesIt()
    {
        Greater(FoldedSpan(0), 1.9);
        Less(FoldedSpan(1e6), 1.5);
    }

    [Fact]
    public void ATetrahedronIsTheSmallestClosedBalloon()
    {
        var balloon = SoftBody.Balloon(Tetrahedron(), TetrahedronTriangles());
        Assert.Equal(4, balloon.Count);
        Assert.Equal(1.0 / 6, balloon.Volume!.Value, 12);
    }

    [Fact]
    public void ABalloonNeedsAWholeNumberOfTrianglesAndFourOfThemAtLeast()
    {
        var nine = TetrahedronTriangles().Take(9).ToArray();
        var thirteen = TetrahedronTriangles().Append(0).ToArray();
        Assert.Equal("triangles", Refused(() => SoftBody.Balloon(Tetrahedron(), nine)));
        Assert.Equal("triangles", Refused(() => SoftBody.Balloon(Tetrahedron(), thirteen)));
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(0, 4)]
    [InlineData(1, -1)]
    [InlineData(1, 4)]
    public void ATriangleCornerOutsideThePointsIsRefused(int at, int index)
    {
        var triangles = TetrahedronWith(at, index);
        var error = Assert.Throws<ArgumentOutOfRangeException>(() => SoftBody.Balloon(Tetrahedron(), triangles));
        Assert.Equal("triangles", error.ParamName);
        Assert.Equal((object)index, error.ActualValue);
    }

    [Fact]
    public void AnEdgeRunTwiceTheSameWayIsRefused()
    {
        var triangles = TetrahedronTriangles().Concat([0, 2, 1]).ToArray();
        Assert.Equal("triangles", Refused(() => SoftBody.Balloon(Tetrahedron(), triangles)));
    }

    [Fact]
    public void AnEdgeFromAParticleToItselfIsRefused()
    {
        var points = Enumerable.Range(0, 8).Select(i => new Vector3(i, i * i, 1)).ToList();
        int[] triangles = [0, 0, 1, 2, 3, 4, 2, 4, 3, 5, 6, 7, 5, 7, 6];
        Assert.Equal("triangles", Refused(() => SoftBody.Balloon(points, triangles)));
    }

    [Fact]
    public void ATetrahedronWoundInsideOutIsRefused()
    {
        int[] inside = [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2];
        var error = Assert.Throws<ArgumentException>(() => SoftBody.Balloon(Tetrahedron(), inside));
        Assert.Equal("mesh", error.ParamName);
    }

    [Fact]
    public void AMeshThatEnclosesNothingIsRefused()
    {
        List<Vector3> flat = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(1, 1, 0)];
        var error = Assert.Throws<ArgumentException>(() => SoftBody.Balloon(flat, TetrahedronTriangles()));
        Assert.Equal("mesh", error.ParamName);
    }

    [Fact]
    public void AVolumeComplianceMustNotBeNegative()
    {
        foreach (var bad in new[] { -1e-9, double.NaN })
        {
            Assert.Equal(
                "volumeCompliance",
                Refused(() => SoftBody.Balloon(Tetrahedron(), TetrahedronTriangles(), volumeCompliance: bad)));
        }

        Assert.Equal(4, SoftBody.Balloon(Tetrahedron(), TetrahedronTriangles(), volumeCompliance: 0).Count);
    }

    [Fact]
    public void AnAnchorLimitMustBeAboveZero()
    {
        var rope = SoftBody.Rope(Line(2));
        foreach (var bad in new[] { 0.0, -1.0, double.NaN })
        {
            Assert.Equal("limit", Refused(() => rope.Attach(0, Vector3.Zero, bad)));
        }

        rope.Attach(0, Vector3.Zero, 1e-9);
        Assert.True(rope.IsAttached(0));
    }

    [Fact]
    public void ABadIndexIsNamedBeforeABadLimit()
    {
        var rope = SoftBody.Rope(Line(2));
        Assert.Equal("i", Refused(() => rope.Attach(99, Vector3.Zero, -1)));
    }

    [Fact]
    public void AStepMustSpanSomeTime()
    {
        var rope = SoftBody.Rope(Line(2));
        foreach (var bad in new[] { 0.0, -Dt, double.NaN })
        {
            Assert.Equal("dt", Refused(() => rope.Step(bad)));
        }
    }

    [Fact]
    public void TheFastestParticleCountsWhereverItIs()
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(gravity: new Vector3(10, 0, 0), damping: 0));
        rope.Pin(0);
        rope.Step(Dt);
        Greater(rope.MaxSpeed, 0.1);
    }

    [Fact]
    public void EveryEdgeOfABalloonCountsInTheStretch()
    {
        var balloon = SoftBody.Balloon(Tetrahedron(), TetrahedronTriangles());
        for (var i = 0; i < balloon.Count; i++)
        {
            balloon.Pin(i);
        }

        balloon.Place(0, balloon.PositionOf(1));
        Assert.Equal(1.0, balloon.MaxStretch);
    }

    [Fact]
    public void PinningAParticleStopsItOnEveryAxis()
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(gravity: new Vector3(3, 4, 5), damping: 0));
        rope.Step(Dt);
        Greater(Math.Min(Math.Min(rope.VelocityOf(1).X, rope.VelocityOf(1).Y), rope.VelocityOf(1).Z), 0.01);
        rope.Pin(1);
        Near(Vector3.Zero(), rope.VelocityOf(1));
    }

    [Fact]
    public void PinningAParticleWakesTheBody()
    {
        var body = Quiet(0.045);
        SleepAway(body);
        body.Pin(1);
        Assert.False(body.Asleep);
    }

    private sealed class Probe : ISoftAir
    {
        public Vector3? Asked { get; private set; }

        public Vector3 WindAt(Vector3 point)
        {
            Asked = point.Clone();
            return Vector3.Zero();
        }
    }

    [Fact]
    public void TheWindIsAskedAtTheMeanOfAllThreeAxes()
    {
        var probe = new Probe();
        var rope = SoftBody.Rope([new(1, 2, 3), new(3, 6, 9)], air: probe);
        rope.Step(Dt);
        Near(new Vector3(2, 4, 6), probe.Asked!);
    }

    [Fact]
    public void AShortLinkStillCarriesItsLoad()
    {
        var rope = SoftBody.Rope([new(0, 0, 0), new(0, -0.005, 0)]);
        rope.Attach(0, Vector3.Zero);
        Run(rope, 0.5);
        Assert.InRange(rope.PullOf(0).Y, -0.11, -0.09);
    }

    [Fact]
    public void TwoPinnedEndsOfALinkWithNoGiveStayWhereTheyAre()
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(stretchCompliance: 0));
        rope.Pin(0);
        rope.Pin(1);
        rope.Place(1, new Vector3(0, -2, 0));
        rope.Step(Dt);
        Near(Vector3.Zero(), rope.PositionOf(0));
        Near(new Vector3(0, -2, 0), rope.PositionOf(1));
    }

    [Fact]
    public void ALinkWhoseEndsMeetHasNoDirectionAndPushesNothing()
    {
        var rope = SoftBody.Rope(Line(2), spec: new SoftSpec(gravity: Vector3.Zero()));
        rope.Pin(0);
        rope.Pin(1);
        rope.Place(1, Vector3.Zero());
        rope.Release(1);
        rope.Step(Dt);
        Near(Vector3.Zero(), rope.PositionOf(1));
    }

    private static SoftBody Stretched(int iterations)
    {
        var spec = new SoftSpec(gravity: Vector3.Zero(), stretchCompliance: 1e-4, iterations: iterations);
        var rope = SoftBody.Rope(Line(2), spec: spec);
        rope.Pin(0);
        rope.Pin(1);
        rope.Place(1, new Vector3(0, -1.5, 0));
        rope.Release(0);
        rope.Release(1);
        return rope;
    }

    [Fact]
    public void ASecondPassOverASingleLinkChangesNothing()
    {
        var once = Stretched(1);
        var often = Stretched(5);
        Run(once, 0.1);
        Run(often, 0.1);
        Greater(once.PositionOf(0).DistanceTo(Vector3.Zero()), 0.01);
        for (var i = 0; i < 2; i++)
        {
            Near(once.PositionOf(i), often.PositionOf(i), 9);
        }
    }

    private static SoftBody Hung(double? limit)
    {
        var rope = SoftBody.Rope(Line(3));
        rope.Attach(0, Vector3.Zero, limit);
        return rope;
    }

    [Fact]
    public void ALimitEqualToThePullHolds()
    {
        var unlimited = Hung(null);
        unlimited.Step(Dt);
        var pull = unlimited.PullOf(0).Length;
        Greater(pull, 0);

        var limited = Hung(pull);
        limited.Step(Dt);

        Assert.True(limited.IsAttached(0));
    }
}
