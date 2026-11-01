namespace Orblit.Physics;

/// <summary>
/// How a world behaves. Anything left null is the engine's own default, so
/// there is one place the numbers live and it is not this file.
/// </summary>
public sealed record PhysicsSettings
{
    public IReadOnlyList<double>? Gravity { get; init; }

    public uint? SubSteps { get; init; }

    public uint? VelocitySteps { get; init; }

    public uint? PositionSteps { get; init; }

    /// <summary>
    /// Passes over the joints in each correcting pass. More of them hold a
    /// chain with something heavy at its tip, and cost nothing without joints.
    /// </summary>
    public uint? JointSteps { get; init; }

    public double? Slop { get; init; }

    public double? ContactHertz { get; init; }

    public double? BounceThreshold { get; init; }

    public double? SleepSpeed { get; init; }

    public double? SleepAfter { get; init; }

    public bool? Sleeping { get; init; }

    /// <summary>The engine's defaults with these fields laid over them.</summary>
    internal NativeSettings ToNative()
    {
        NativeMethods.Defaults(out var settings);
        if (Gravity is { } gravity)
        {
            settings.GravityX = (float)gravity[0];
            settings.GravityY = (float)gravity[1];
            settings.GravityZ = (float)gravity[2];
        }

        settings.SubSteps = SubSteps ?? settings.SubSteps;
        settings.VelocitySteps = VelocitySteps ?? settings.VelocitySteps;
        settings.PositionSteps = PositionSteps ?? settings.PositionSteps;
        settings.JointSteps = JointSteps ?? settings.JointSteps;
        return Tuned(settings);
    }

    private NativeSettings Tuned(NativeSettings settings)
    {
        settings.Slop = (float?)Slop ?? settings.Slop;
        settings.ContactHertz = (float?)ContactHertz ?? settings.ContactHertz;
        settings.BounceThreshold = (float?)BounceThreshold ?? settings.BounceThreshold;
        settings.SleepSpeed = (float?)SleepSpeed ?? settings.SleepSpeed;
        settings.SleepAfter = (float?)SleepAfter ?? settings.SleepAfter;
        if (Sleeping is { } sleeping)
        {
            settings.Sleeping = sleeping ? (byte)1 : (byte)0;
        }

        return settings;
    }
}
