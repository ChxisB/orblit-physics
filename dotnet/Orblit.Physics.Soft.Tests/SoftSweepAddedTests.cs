using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftSweepAddedTests : SoftTestBase
{
    private sealed record SweepCall(Vector3 From, Vector3 Direction, double Distance, double Radius);

    /// <summary>A surface that answers every sweep with one contact, or none, and keeps what it was asked.</summary>
    private sealed class FakeSurface : ISoftSurface
    {
        public SoftContact? Contact { get; set; }

        public List<SweepCall> Asked { get; } = [];

        public SoftContact? Sweep(Vector3 from, Vector3 direction, double distance, double radius)
        {
            Asked.Add(new SweepCall(from.Clone(), direction.Clone(), distance, radius));
            return Contact;
        }
    }

    private static Vector3 Tilt() => new(0.36, 0.48, 0.8);

    /// <summary>Two particles a metre apart along the plane, 0.3 m behind it.</summary>
    private static SoftBody Buried(FakeSurface surface, SoftSpec spec)
    {
        var p0 = Tilt() * -0.3;
        var p1 = p0 + new Vector3(0.8, -0.6, 0);
        return SoftBody.Rope([p0, p1], spec: spec, surface: surface);
    }

    [Fact]
    public void EachFreeParticleLooksAlongItsTravelForTheStep()
    {
        const double Span = 1.0 / 64;
        var spec = new SoftSpec(gravity: new Vector3(0, -10, 0), damping: 0, windDrag: 1, thickness: 0.02);
        var surface = new FakeSurface();
        var body = SoftBody.Rope(Line(2), spec: spec, surface: surface, air: new Breeze { Wind = new Vector3(30, -30, 0) });
        body.Step(Span);
        var from = Points(body);
        var velocity = new[] { body.VelocityOf(0), body.VelocityOf(1) };
        surface.Asked.Clear();
        body.Step(Span);

        Assert.Equal(2, surface.Asked.Count);
        var fall = new Vector3(30, -40, 0) * (0.5 * Span * Span);
        for (var i = 0; i < 2; i++)
        {
            var move = (velocity[i] * Span) + fall;
            var call = surface.Asked[i];
            Near(from[i], call.From);
            Near(move / move.Length, call.Direction);
            Assert.Equal(move.Length, call.Distance, 12);
            Assert.Equal(0.02 * 1.1, call.Radius, 12);
        }
    }

    [Fact]
    public void AParticleWithNowhereToGoStillLooksDown()
    {
        var surface = new FakeSurface();
        var body = SoftBody.Rope(Line(2), spec: new SoftSpec(gravity: Vector3.Zero()), surface: surface);
        body.Step(Dt);
        Assert.Equal(2, surface.Asked.Count);
        foreach (var call in surface.Asked)
        {
            Near(new Vector3(0, -1, 0), call.Direction);
            Assert.Equal(0.0, call.Distance);
        }
    }

    [Fact]
    public void ABuriedParticleIsMovedOutOneSkinASubstepAndNotThrown()
    {
        var spec = new SoftSpec(gravity: Vector3.Zero(), thickness: 0.02);
        var surface = new FakeSurface { Contact = new SoftContact(Vector3.Zero(), Tilt()) };
        var body = Buried(surface, spec);
        var before = Points(body);
        body.Step(Dt);

        var shift = 0.02 * Math.Ceiling(Dt / spec.SubstepLength);
        var n = Tilt();
        for (var i = 0; i < 2; i++)
        {
            var at = before[i];
            Near(new Vector3(at.X + (n.X * shift), at.Y + (n.Y * shift), at.Z + (n.Z * shift)), body.PositionOf(i));
            Near(Vector3.Zero(), body.VelocityOf(i), 9);
        }
    }

    [Fact]
    public void AParticleIsNotHeldByAPlaneTheSurfaceNoLongerAnswers()
    {
        var surface = new FakeSurface { Contact = new SoftContact(Vector3.Zero(), Tilt()) };
        var body = Buried(surface, new SoftSpec(gravity: Vector3.Zero(), thickness: 0.02));
        body.Step(Dt);
        var held = Points(body);
        surface.Contact = null;
        body.Step(Dt);
        for (var i = 0; i < 2; i++)
        {
            Near(held[i], body.PositionOf(i));
        }
    }

    [Fact]
    public void ASlideOverATiltedPlaneLosesOnlyItsSurfaceParts()
    {
        var n = Tilt();
        var p0 = n * 0.010004;
        var p1 = p0 + new Vector3(0.8, -0.6, 0);
        var spec = new SoftSpec(gravity: new Vector3(3, -10, 2), friction: 4, damping: 0);
        var surface = new FakeSurface { Contact = new SoftContact(Vector3.Zero(), n) };
        var body = SoftBody.Rope([p0, p1], spec: spec, surface: surface);
        body.Step(spec.SubstepLength);

        Near(new Vector3(0.0036088169219674862, 0.004778954846970425, 0.008008659476932375), body.PositionOf(0));
        Near(new Vector3(0.8036088169219675, -0.5952210451530295, 0.008008659476932365), body.PositionOf(1));
        Near(new Vector3(0.0035409225443933795, -0.011023273454195959, 0.002620548927539934), body.VelocityOf(0));
    }
}
