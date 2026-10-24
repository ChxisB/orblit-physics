using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// A car: a chassis body whose wheels are casts.
/// </summary>
/// <remarks>
/// Call <see cref="Update"/> once before each <see cref="Physics.Step"/>, with
/// that step's length. It reads the chassis, casts from each wheel, and pushes
/// the chassis with each spring's force and each tyre's grip as an impulse. The
/// world is never told about the wheels.
/// </remarks>
public sealed class Vehicle
{
    /// <summary>The longest step a spring of <see cref="SuspensionSpec.MaxFrequency"/> is stable at.</summary>
    public const double MaxStep = 1.0 / 30;

    private readonly Tyre[] _tyres;
    private readonly double[] _rolled;
    private readonly Driveline? _driveline;
    private WheelState[] _wheels;
    private VehicleInput _input = new();

    /// <summary>Where the chassis stood at the last update.</summary>
    private Chassis? _before;

    /// <summary>
    /// A car with a new chassis body, a <paramref name="chassis"/> shape at
    /// <paramref name="at"/>, in the world.
    /// </summary>
    // F2: eight parameters, all but the first four optional and called by name.
    public Vehicle(
        Physics physics,
        ulong id,
        Shape chassis,
        VehicleSpec spec,
        IReadOnlyList<double>? at = null,
        IReadOnlyList<double>? rotation = null,
        Layers? layers = null,
        SurfaceGrip? surface = null)
        : this(physics, id, spec, surface)
    {
        physics.Add(id, chassis, at: at, rotation: rotation, mass: spec.Mass, layers: layers);
        physics.SetControls(id, new PhysicsControls(Centre: spec.Centre));
    }

    private Vehicle(Physics physics, ulong id, VehicleSpec spec, SurfaceGrip? surface)
    {
        Physics = physics;
        Id = id;
        Spec = spec;
        Surface = surface ?? Dry;
        _wheels = [.. spec.Wheels.Select(_ => new WheelState())];
        _rolled = new double[spec.Wheels.Count];
        _tyres = [.. spec.Wheels.Select(w => new Tyre(w.Tyre, w.Radius, w.Inertia))];
        _driveline = spec.Drive == null ? null : new Driveline(spec.Drive, spec.Wheels);
    }

    public Physics Physics { get; }

    public ulong Id { get; }

    public VehicleSpec Spec { get; }

    /// <summary>How well the tyres hold what a wheel stands on, by the body it stands on.</summary>
    public SurfaceGrip Surface { get; }

    /// <summary>Each wheel as the last update left it, in the order of the spec.</summary>
    public IReadOnlyList<WheelState> Wheels => Array.AsReadOnly(_wheels);

    /// <summary>What the driver is asking for.</summary>
    public VehicleInput Input => _input;

    /// <summary>Revolutions a minute the engine turns at. Zero for a car with no engine.</summary>
    public double Rpm => _driveline?.Rpm ?? 0.0;

    /// <summary>
    /// The gear the box is in: reverse is minus one and neutral is zero. Zero for
    /// a car with no engine.
    /// </summary>
    public int Gear => _driveline?.Gear ?? 0;

    /// <summary>
    /// A car for a chassis body that is already in <paramref name="physics"/> as
    /// <paramref name="id"/>. The body keeps its shape, its layers and its
    /// controls, and wants the car's <see cref="VehicleSpec.Mass"/> and
    /// <see cref="VehicleSpec.Centre"/>.
    /// </summary>
    public static Vehicle Attach(Physics physics, ulong id, VehicleSpec spec, SurfaceGrip? surface = null) =>
        new(physics, id, spec, surface);

    /// <summary>
    /// Changes what the driver asks for, from the next update. A gear the car
    /// does not have is refused.
    /// </summary>
    public void SetInput(VehicleInput input)
    {
        var top = Spec.Drive?.Ratios.Count ?? 0;
        if (input.Gear > top)
        {
            throw new ArgumentOutOfRangeException(nameof(input), input.Gear, $"The car has {top}.");
        }

        _input = input;
    }

    /// <summary>Pushes the chassis with every wheel's force for a step of <paramref name="dt"/> seconds.</summary>
    public void Update(double dt)
    {
        if (!(dt > 0 && dt <= MaxStep))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, $"Must be above 0 and at most {MaxStep}.");
        }

        var chassis = Chassis.Read(Physics, Id, Spec.Centre);
        if (chassis == null)
        {
            return;
        }

        var steer = Steering.Angles(Spec.Wheels, _input.Steer * Spec.MaxSteer);
        var probes = new Probe?[_wheels.Length];
        for (var i = 0; i < probes.Length; i++)
        {
            probes[i] = ProbeOf(i, chassis, dt);
        }

        var loads = Forces(probes, dt);
        var patches = new Patch?[probes.Length];
        for (var i = 0; i < patches.Length; i++)
        {
            var probe = probes[i];
            patches[i] = probe == null ? null : PatchOf(i, probe, chassis, dt, steer[i], loads[i]);
        }

        _driveline?.Step(
            [.. _tyres.Select(tyre => tyre.Spin)],
            [
                .. Enumerable
                    .Range(0, patches.Length)
                    .Select<int, Func<double, double>>(i =>
                        torque => _tyres[i].SpinAfter(patches[i], dt, torque, Brake(i))),
            ],
            _input,
            dt);
        var rolled = new WheelState[probes.Length];
        for (var i = 0; i < rolled.Length; i++)
        {
            rolled[i] = Roll(i, probes[i], patches[i], dt, steer[i], loads[i]);
        }

        _wheels = rolled;
        _before = chassis;
    }

    private static double Dry(ulong body) => 1.0;

    private static double[] Listed(Vector3 v) => [v.X, v.Y, v.Z];

    /// <summary>
    /// Turns wheel <paramref name="i"/> for a step, pushes the chassis with what
    /// it gives, and reports the wheel.
    /// </summary>
    // F2: six parameters, the wheel and what the step found for it.
    private WheelState Roll(int i, Probe? probe, Patch? patch, double dt, double steer, double load)
    {
        var tyre = _tyres[i];
        tyre.Step(patch, dt, _driveline?.Torque(i) ?? 0.0, Brake(i));
        _rolled[i] = Doubles.Mod(_rolled[i] + (tyre.Spin * dt), 2 * Math.PI);
        if (probe != null)
        {
            var push = (probe.Normal * load) + tyre.Force;
            if (push.Length2 > 0)
            {
                Physics.Push(Id, Listed(push * dt), probe.At);
            }
        }

        return new WheelState(probe != null, probe?.Compression ?? 0.0, load, tyre.Spin, steer, _rolled[i]);
    }

    /// <summary>Newton metres the brake of wheel <paramref name="i"/> gives.</summary>
    private double Brake(int i)
    {
        var wheel = Spec.Wheels[i];
        var held = wheel.Handbrake && _input.Handbrake;
        return wheel.BrakeTorque * (held ? 1.0 : _input.Brake);
    }

    // F2: six parameters, the wheel and what the step found for it.
    private Patch PatchOf(int i, Probe probe, Chassis chassis, double dt, double steer, double load) =>
        new(
            chassis.MotionAt(new Vector3(probe.At[0], probe.At[1], probe.At[2]), _before, dt),
            probe.Normal,
            chassis.Direct(new Vector3(-Math.Sin(steer), 0, -Math.Cos(steer))),
            load,
            Surface(probe.Body),
            Spec.Sprung[i]);

    private Probe? ProbeOf(int i, Chassis chassis, double dt)
    {
        var wheel = Spec.Wheels[i];
        var reach = wheel.Suspension.Travel + wheel.Radius;
        var hit = Physics.Cast(
            Listed(chassis.ToWorld(wheel.Mount)),
            Listed(chassis.Down),
            reach,
            layers: Spec.Ground,
            ignore: Id);

        // A mount already inside the ground has nothing to measure from. The
        // chassis' own contact holds the car up there.
        if (hit == null || hit.Started)
        {
            return null;
        }

        var compression = reach - hit.Distance;

        // The rate is the change since the last update, not the chassis' velocity:
        // the solver reports a velocity at the end of its sub-steps, which for a
        // body held by one impulse a step is a sawtooth that is not its motion.
        var before = _wheels[i];
        return new Probe(
            hit.Body,
            compression,
            before.Grounded ? (compression - before.Compression) / dt : 0.0,
            hit.At,
            new Vector3(hit.Normal[0], hit.Normal[1], hit.Normal[2]));
    }

    /// <summary>
    /// A spring, a damper and half an anti-roll bar for each wheel on the
    /// ground, in newtons.
    /// </summary>
    private double[] Forces(Probe?[] probes, double dt)
    {
        var forces = new double[probes.Length];
        for (var i = 0; i < forces.Length; i++)
        {
            var probe = probes[i];
            forces[i] = probe == null ? 0.0 : Spring(Spec.Wheels[i].Suspension, probe, Spec.Sprung[i], dt);
        }

        foreach (var (a, b) in Spec.Axles)
        {
            var (pa, pb) = (probes[a], probes[b]);
            var bar = Spec.AntiRoll * ((pa?.Compression ?? 0) - (pb?.Compression ?? 0));
            if (pa != null)
            {
                forces[a] += bar;
            }

            if (pb != null)
            {
                forces[b] -= bar;
            }
        }

        return [.. forces.Select(force => Math.Max(0.0, force))];
    }

    private static double Spring(SuspensionSpec spring, Probe probe, double sprung, double dt)
    {
        var omega = 2 * Math.PI * spring.Frequency;
        var push = sprung * omega * omega * probe.Compression;
        var damper = 2 * spring.Damping * omega * sprung * probe.Rate;

        // The damper may take the compression rate to zero in one step and no
        // further. Past that it pushes the wheel the other way and the car buzzes.
        var most = sprung * Math.Abs(probe.Rate) / dt;
        return push + Doubles.Clamp(damper, -most, most);
    }
}
