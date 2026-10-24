using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// The rubber of one wheel and the wheel's spin.
/// </summary>
/// <remarks>
/// The patch is held to the ground by bristles. A bristle bends as the patch
/// slips and pulls the patch back, so a car that is parked is held in place,
/// not slowed to a crawl. As the wheel rolls the bristles are carried away and
/// fresh ones take the ground, which is why a tyre turning a corner gives a
/// steady force for a steady slip. The force is limited to the grip.
/// <see cref="Step"/> solves the bristles and the wheel's spin together, so a
/// wheel that weighs a few kilograms is stable at the step of a game.
/// </remarks>
internal sealed class Tyre
{
    private readonly TyreSpec _spec;
    private readonly double _radius;
    private readonly double _inertia;

    private double _spin;
    private Vector3 _force = Vector3.Zero();

    /// <summary>How far the bristles are bent, in the world, along the ground.</summary>
    private Vector3 _bent = Vector3.Zero();

    /// <summary>
    /// How fast the last step expected the chassis to move at the end of it,
    /// counting only the tyre's own force, and the share of a step's slip that
    /// bent the bristles. Null when the wheel had no ground.
    /// </summary>
    private Vector3? _expected;
    private double _carry;

    public Tyre(TyreSpec spec, double radius, double inertia)
    {
        _spec = spec;
        _radius = radius;
        _inertia = inertia;
    }

    /// <summary>Radians a second the wheel turns at. Positive rolls the car forward.</summary>
    public double Spin => _spin;

    /// <summary>Newtons the patch pushes the chassis with, in the world.</summary>
    public Vector3 Force => _force;

    /// <summary>
    /// Moves the tyre on by <paramref name="dt"/> seconds. <paramref name="drive"/>
    /// and <paramref name="brake"/> are torques in newton metres: the first turns
    /// the wheel forward and the second holds it. A wheel off the ground or with
    /// no load only turns freely and brakes.
    /// </summary>
    public void Step(Patch? patch, double dt, double drive, double brake)
    {
        var next = Solve(patch, dt, drive, brake);
        _spin = next.Spin;
        _force = next.Force;
        _bent = next.Bent;
        _expected = next.Expected;
        _carry = next.Carry;
    }

    /// <summary>
    /// The spin the wheel would end a step at, for a <paramref name="drive"/>
    /// torque, without taking the step. The driveline asks it to find how the
    /// ground answers the torque it is about to send.
    /// </summary>
    public double SpinAfter(Patch? patch, double dt, double drive, double brake) =>
        Solve(patch, dt, drive, brake).Spin;

    private Outcome Solve(Patch? patch, double dt, double drive, double brake)
    {
        var start = _spin + (drive * dt / _inertia);
        var ground = patch == null ? null : Ground.Of(patch);
        if (patch == null || ground == null || !(patch.Load > 0))
        {
            return Outcome.Free(Braked(start, brake * dt));
        }

        // The bristles solved the step as if the tyre were the only force on the
        // chassis. Whatever else pushed it (weight on a slope, a spring) moved the
        // patch further, and the bend has to follow or a tyre holds a car that is
        // still sliding at the speed those forces give it in one step.
        var expected = _expected;
        var wheel = new Wheel(start, expected == null ? _bent : _bent + ((patch.Velocity - expected) * _carry));

        var free = GripOf(patch, ground, dt, wheel, turning: true);
        var turned = start - (free.Along * _radius * dt / _inertia);
        var locked = brake * dt >= _inertia * Math.Abs(turned);
        var held = locked ? GripOf(patch, ground, dt, wheel, turning: false) : free;
        var force = (ground.Heading * held.Along) + (ground.Side * held.Across);
        return new Outcome(
            locked ? 0.0 : Braked(turned, brake * dt),
            force,
            (ground.Heading * held.BentAlong) + (ground.Side * held.BentAcross),
            patch.Velocity + (force * (dt / patch.Mass)),
            dt * held.Keep);
    }

    private double Braked(double spin, double impulse)
    {
        var slowed = impulse / _inertia;
        return Math.Abs(spin) <= slowed ? 0.0 : spin - (Doubles.Sign(spin) * slowed);
    }

    /// <summary>What the patch gives for a step with the wheel free to turn, or held.</summary>
    // F2: the wheel at the start of the step and whether it turns are both needed.
    private Grip GripOf(Patch patch, Ground ground, double dt, Wheel wheel, bool turning)
    {
        var spin = turning ? wheel.Spin : 0.0;
        var rolling = Math.Abs(spin) * _radius;
        var slip = new Vector2(ground.Along - (spin * _radius), ground.Across);

        // A wheel that turns also takes the force into its spin, which makes the
        // car feel much lighter along the road.
        var mass = new Vector2(
            turning ? 1 / ((1 / patch.Mass) + (_radius * _radius / _inertia)) : patch.Mass,
            patch.Mass);
        var most = _spec.Grip * patch.Surface * patch.Load;
        var bristles = new Bristles(
            most / (_spec.PeakSlip * _spec.Relaxation),
            1 / (1 + (dt * rolling / _spec.Relaxation)),
            dt);

        var bent = new Vector2(wheel.Bent.Dot(ground.Heading), wheel.Bent.Dot(ground.Side));
        var force = new Vector2(
            bristles.Force(bent.X, slip.X, mass.X),
            bristles.Force(bent.Y, slip.Y, mass.Y));
        var reference = Math.Max(Math.Max(ground.Speed, rolling), 1.0);
        var past = Doubles.Clamp((slip.Length / reference / _spec.PeakSlip) - 1, 0.0, 1.0);
        var limit = most * (1 - ((1 - _spec.Sliding) * past));
        if (force.Length > limit)
        {
            force *= limit / force.Length;
        }

        return new Grip(
            force.X,
            force.Y,
            bristles.Bent(bent.X, force.X, mass.X),
            bristles.Bent(bent.Y, force.Y, mass.Y),
            bristles.Keep);
    }

    /// <summary>
    /// A wheel at the start of a step: how fast it turns and how bent its
    /// bristles are, in the world.
    /// </summary>
    private sealed record Wheel(double Spin, Vector3 Bent);

    /// <summary>What a step leaves the tyre with.</summary>
    // F2: five fields, all set by every step.
    private sealed record Outcome(double Spin, Vector3 Force, Vector3 Bent, Vector3? Expected, double Carry)
    {
        /// <summary>A wheel with no ground turning at <paramref name="spin"/>.</summary>
        public static Outcome Free(double spin) => new(spin, Vector3.Zero(), Vector3.Zero(), null, 0.0);
    }

    /// <summary>A force along the wheel and across it, and the bristles it leaves.</summary>
    /// <param name="Along">Newtons along the wheel.</param>
    /// <param name="Across">Newtons across the wheel.</param>
    /// <param name="BentAlong">Metres the bristles are bent along the wheel.</param>
    /// <param name="BentAcross">Metres the bristles are bent across the wheel.</param>
    /// <param name="Keep">The share of the bend that stayed with the wheel through the step.</param>
    // F2: five fields, all set by every grip.
    private sealed record Grip(double Along, double Across, double BentAlong, double BentAcross, double Keep);
}
