namespace Orblit.Physics.Vehicle;

/// <summary>
/// What the driver asks of the car for the next step.
/// </summary>
public sealed class VehicleInput
{
    public VehicleInput(
        double steer = 0.0,
        double throttle = 0.0,
        double brake = 0.0,
        bool handbrake = false,
        int? gear = null)
    {
        if (!(steer >= -1 && steer <= 1))
        {
            throw new ArgumentOutOfRangeException(nameof(steer), steer, "Must be -1 to 1.");
        }

        if (!(throttle >= 0 && throttle <= 1))
        {
            throw new ArgumentOutOfRangeException(nameof(throttle), throttle, "Must be 0 to 1.");
        }

        if (!(brake >= 0 && brake <= 1))
        {
            throw new ArgumentOutOfRangeException(nameof(brake), brake, "Must be 0 to 1.");
        }

        if (gear < -1)
        {
            throw new ArgumentOutOfRangeException(nameof(gear), gear, "Must be -1 or more.");
        }

        Steer = steer;
        Throttle = throttle;
        Brake = brake;
        Handbrake = handbrake;
        Gear = gear;
    }

    /// <summary>Full lock to the left is one and to the right is minus one.</summary>
    public double Steer { get; }

    /// <summary>How far the accelerator is pressed.</summary>
    public double Throttle { get; }

    /// <summary>How hard the brake pedal is pressed.</summary>
    public double Brake { get; }

    /// <summary>Holds the wheels that have a handbrake, with the whole of their brake.</summary>
    public bool Handbrake { get; }

    /// <summary>
    /// Reverse is minus one, neutral is zero and the forward gears count up
    /// from one. Null lets the box pick among the forward gears.
    /// </summary>
    public int? Gear { get; }
}
