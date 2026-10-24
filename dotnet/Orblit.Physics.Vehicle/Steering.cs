namespace Orblit.Physics.Vehicle;

/// <summary>
/// The angle each wheel is turned to.
/// </summary>
internal static class Steering
{
    /// <summary>
    /// The angle each wheel is turned to, in radians and in the order of
    /// <paramref name="wheels"/>, for a steering angle of <paramref name="angle"/>
    /// at the middle of the steering axle. Left is positive.
    /// </summary>
    /// <remarks>
    /// The wheels that steer turn so that their lines across the road meet on the
    /// line through the wheels that do not. The inner wheel turns more than the
    /// outer one, and the car follows a circle with no wheel pushed sideways. With
    /// no wheel left fixed, or none to steer, they turn together.
    /// </remarks>
    public static double[] Angles(IReadOnlyList<WheelSpec> wheels, double angle)
    {
        var steering = wheels.Where(w => w.Steers).ToList();
        var fixedWheels = wheels.Where(w => !w.Steers).ToList();
        if (steering.Count == 0)
        {
            return new double[wheels.Count];
        }

        // Ahead is along the chassis' negative z, left along its negative x.
        var rear = Mean(fixedWheels.Select(w => -w.Mount.Z));
        var baseline = Mean(steering.Select(w => -w.Mount.Z)) - rear;
        if (fixedWheels.Count == 0 || Math.Abs(baseline) < 1e-6)
        {
            return [.. wheels.Select(w => w.Steers ? angle : 0.0)];
        }

        var lean = Math.Tan(angle);
        return
        [
            .. wheels.Select(w =>
                w.Steers ? Math.Atan((-w.Mount.Z - rear) * lean / (baseline + (w.Mount.X * lean))) : 0.0),
        ];
    }

    private static double Mean(IEnumerable<double> values)
    {
        var sum = 0.0;
        var count = 0;
        foreach (var value in values)
        {
            sum += value;
            count++;
        }

        return sum / count;
    }
}
