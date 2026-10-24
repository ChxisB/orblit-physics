using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftSpecAddedTests
{
    private static string Refused(Func<SoftSpec> make) =>
        Assert.Throws<ArgumentOutOfRangeException>(make).ParamName!;

    [Fact]
    public void ADefaultSpecHasTheDocumentedValues()
    {
        var spec = new SoftSpec();
        Assert.Equal(0, spec.Gravity.X);
        Assert.Equal(-9.81, spec.Gravity.Y);
        Assert.Equal(0, spec.Gravity.Z);
        Assert.Equal(1.0, spec.Damping);
        Assert.Equal(1e-6, spec.StretchCompliance);
        Assert.Equal(1e-3, spec.BendCompliance);
        Assert.Equal(1.0 / 480, spec.SubstepLength);
        Assert.Equal(1, spec.Iterations);
        Assert.Equal(0.01, spec.Thickness);
        Assert.Equal(0.5, spec.Friction);
        Assert.Equal(1.0, spec.WindDrag);
    }

    [Fact]
    public void TheLimitsAreThePublishedOnes()
    {
        Assert.Equal(1.0 / 120, SoftSpec.MaxSubstepLength);
        Assert.Equal(0.1, SoftSpec.MaxStep);
        Assert.Equal(0.005, SoftBody.SleepSpeed);
        Assert.Equal(1.0, SoftBody.SleepTime);
    }

    [Fact]
    public void TheLongestSubstepIsAcceptedAndAnythingLongerIsNot()
    {
        Assert.Equal(1.0 / 120, new SoftSpec(substepLength: 1.0 / 120).SubstepLength);
        Assert.Equal("substepLength", Refused(() => new SoftSpec(substepLength: (1.0 / 120) + 1e-9)));
        Assert.Equal("substepLength", Refused(() => new SoftSpec(substepLength: 0)));
        Assert.Equal("substepLength", Refused(() => new SoftSpec(substepLength: -1.0 / 480)));
        Assert.Equal("substepLength", Refused(() => new SoftSpec(substepLength: double.NaN)));
    }

    [Fact]
    public void ZeroIsAStiffnessAndADampingAndAFrictionAndADragThatAreAllowed()
    {
        var spec = new SoftSpec(damping: 0, stretchCompliance: 0, bendCompliance: 0, friction: 0, windDrag: 0);
        Assert.Equal(0, spec.Damping);
        Assert.Equal(0, spec.StretchCompliance);
        Assert.Equal(0, spec.BendCompliance);
        Assert.Equal(0, spec.Friction);
        Assert.Equal(0, spec.WindDrag);
    }

    [Fact]
    public void ANegativeOrNotANumberValueIsRefusedByTheNameOfTheField()
    {
        foreach (var bad in new[] { -1e-9, double.NaN })
        {
            Assert.Equal("damping", Refused(() => new SoftSpec(damping: bad)));
            Assert.Equal("stretchCompliance", Refused(() => new SoftSpec(stretchCompliance: bad)));
            Assert.Equal("bendCompliance", Refused(() => new SoftSpec(bendCompliance: bad)));
            Assert.Equal("friction", Refused(() => new SoftSpec(friction: bad)));
            Assert.Equal("windDrag", Refused(() => new SoftSpec(windDrag: bad)));
        }
    }

    [Fact]
    public void ThicknessMustBeAFiniteNumberAboveZero()
    {
        foreach (var bad in new[] { 0.0, -1.0, double.NaN, double.PositiveInfinity })
        {
            Assert.Equal("thickness", Refused(() => new SoftSpec(thickness: bad)));
        }
    }

    [Fact]
    public void WindDragMustBeFinite()
    {
        Assert.Equal("windDrag", Refused(() => new SoftSpec(windDrag: double.PositiveInfinity)));
    }

    [Fact]
    public void IterationsMustBeOneOrMore()
    {
        Assert.Equal(1, new SoftSpec(iterations: 1).Iterations);
        Assert.Equal("iterations", Refused(() => new SoftSpec(iterations: 0)));
        Assert.Equal("iterations", Refused(() => new SoftSpec(iterations: -1)));
    }

    [Fact]
    public void TheGravityIsACopyOfTheOneGiven()
    {
        var given = new Vector3(0, -1, 0);
        var spec = new SoftSpec(gravity: given);
        given.Y = -50;
        Assert.Equal(-1, spec.Gravity.Y);
    }

    [Fact]
    public void TwoDefaultSpecsDoNotShareTheirGravity()
    {
        var first = new SoftSpec();
        var second = new SoftSpec();
        first.Gravity.Y = 0;
        Assert.Equal(-9.81, second.Gravity.Y);
    }
}
