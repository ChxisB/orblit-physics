using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// One wheel. It has no mass of its own, but it has a spin to hold.
/// </summary>
public sealed class WheelSpec
{
    // F2: nine parameters, all but the first two optional and called by name.
    public WheelSpec(
        Vector3 mount,
        double radius,
        bool steers = false,
        bool drives = false,
        double brakeTorque = 0.0,
        bool handbrake = false,
        double inertia = 1.0,
        SuspensionSpec? suspension = null,
        TyreSpec? tyre = null)
    {
        if (!(radius > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "Must be above zero.");
        }

        if (!(brakeTorque >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(brakeTorque), brakeTorque, "Must not be negative.");
        }

        if (handbrake && brakeTorque == 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(brakeTorque),
                brakeTorque,
                "A wheel on the handbrake needs a brake.");
        }

        if (!(inertia > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(inertia), inertia, "Must be above zero.");
        }

        Mount = mount.Clone();
        Radius = radius;
        Steers = steers;
        Drives = drives;
        BrakeTorque = brakeTorque;
        Handbrake = handbrake;
        Inertia = inertia;
        Suspension = suspension ?? new SuspensionSpec();
        Tyre = tyre ?? new TyreSpec();
    }

    /// <summary>
    /// The top of the suspension, in the chassis frame. The wheel hangs below it
    /// along the chassis' own down. Not to be changed.
    /// </summary>
    public Vector3 Mount { get; }

    public double Radius { get; }

    /// <summary>It turns with the steering.</summary>
    public bool Steers { get; }

    /// <summary>The engine turns it, through the differential of its axle.</summary>
    public bool Drives { get; }

    /// <summary>Newton metres the brake gives at full pressure. Zero is a wheel with no brake.</summary>
    public double BrakeTorque { get; }

    /// <summary>The handbrake holds this wheel. It uses the same brake.</summary>
    public bool Handbrake { get; }

    /// <summary>Kilogram square metres the wheel and tyre turn with.</summary>
    public double Inertia { get; }

    public SuspensionSpec Suspension { get; }

    public TyreSpec Tyre { get; }
}
