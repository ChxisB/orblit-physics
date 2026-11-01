using System.Runtime.InteropServices;

namespace Orblit.Physics.Tests;

public class NativeSettingsAddedTests
{
    [Fact]
    public void SettingsAreTheSizeTheSolverReads()
    {
        Assert.Equal(52, Marshal.SizeOf<NativeSettings>());
    }

    [Fact]
    public void TheSolverFillsInItsDefaults()
    {
        NativeMethods.Defaults(out var settings);
        Assert.Equal(-9.81f, settings.GravityY);
        Assert.Equal(6u, settings.SubSteps);
        Assert.Equal(8u, settings.JointSteps);
        Assert.Equal(1, settings.Sleeping);
    }

    [Fact]
    public void AWorldIsMadeFromSettingsAndDestroyed()
    {
        NativeMethods.Defaults(out var settings);
        var world = NativeMethods.Create(in settings);
        Assert.NotEqual(nint.Zero, world);
        NativeMethods.Destroy(world);
    }
}
