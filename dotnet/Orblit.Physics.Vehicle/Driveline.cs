using Orblit.VectorMath;

namespace Orblit.Physics.Vehicle;

/// <summary>
/// The engine, the clutch and the gearbox, and the torque they send to each
/// wheel.
/// </summary>
/// <remarks>
/// The clutch is a spring on the difference in speed, limited to what it can
/// hold. A wheel on the ground is turned with the whole car behind it and a
/// wheel in the air is not, so the step asks each tyre how it will answer a
/// torque and solves the clutch against that. The engine, the clutch and the
/// wheels are then stable at any gear ratio, spinning or gripping.
/// </remarks>
internal sealed class Driveline
{
    /// <summary>Revolutions a minute for a radian a second.</summary>
    private const double RpmPerRadian = 60 / (2 * Math.PI);

    /// <summary>Radians a second the clutch may slip by while it holds all it can.</summary>
    private const double Slip = 2.0;

    /// <summary>
    /// The clutch is open up to this many times idle. The governor holds the
    /// engine just under it, so a car at rest with the pedal up does not creep.
    /// </summary>
    private const double Open = 1.1;

    /// <summary>
    /// Halvings of the range the clutch torque is found in. Twelve leave it
    /// within a four-thousandth of the clutch's grip.
    /// </summary>
    private const int Passes = 12;

    private readonly DriveSpec _spec;
    private readonly IReadOnlyList<WheelSpec> _wheels;
    private readonly List<int> _driven;
    private readonly double[] _torques;
    private double _speed;
    private int _gear = 1;
    private double _shifting;

    public Driveline(DriveSpec spec, IReadOnlyList<WheelSpec> wheels)
    {
        _spec = spec;
        _wheels = wheels;
        _torques = new double[wheels.Count];
        _driven = [.. Enumerable.Range(0, wheels.Count).Where(i => wheels[i].Drives)];
        _speed = spec.Engine.Idle / RpmPerRadian;
    }

    /// <summary>Revolutions a minute the engine turns at.</summary>
    public double Rpm => _speed * RpmPerRadian;

    /// <summary>Reverse is minus one, neutral is zero.</summary>
    public int Gear => _gear;

    /// <summary>Newton metres the driveline turns wheel <paramref name="i"/> with.</summary>
    public double Torque(int i) => _torques[i];

    /// <summary>
    /// Moves the engine and the gearbox on by <paramref name="dt"/> seconds.
    /// <paramref name="spins"/> is how fast each wheel turns, and
    /// <paramref name="spinAfter"/> gives, for each wheel, the spin it would end
    /// the step at for a drive torque.
    /// </summary>
    public void Step(
        IReadOnlyList<double> spins,
        IReadOnlyList<Func<double, double>> spinAfter,
        VehicleInput input,
        double dt)
    {
        var shaft = Mean(i => spins[i]);
        Shift(input, shaft, dt);
        var ratio = Ratio();
        var engine = _spec.Engine;

        var matched = Matched(ratio, shaft, dt);
        if (matched != null)
        {
            _speed = matched.Value;
        }

        var pedal = Math.Max(input.Throttle, Governor());
        var cut = Doubles.Clamp(((engine.Redline * 1.05) - Rpm) / (engine.Redline * 0.05), 0.0, 1.0);
        var drag = engine.Drag * Rpm / engine.Redline;
        var push = (pedal * engine.Full(Rpm) * cut) - ((1 - pedal) * drag);

        var held = Held(ratio, spinAfter, push, dt);
        _speed = Math.Max(0.0, _speed + ((push - held) * dt / engine.Inertia));
        Split(ratio * held, spins, dt);
    }

    private double Mean(Func<int, double> of)
    {
        var sum = 0.0;
        foreach (var i in _driven)
        {
            sum += of(i);
        }

        return sum / _driven.Count;
    }

    /// <summary>Changes gear, by the driver's word or the box's own, and starts the shift.</summary>
    private void Shift(VehicleInput input, double shaft, double dt)
    {
        _shifting = Math.Max(0.0, _shifting - dt);
        var wanted = input.Gear ?? Auto(input.Throttle, shaft);
        if (wanted == _gear)
        {
            return;
        }

        _gear = wanted;
        _shifting = _spec.ShiftTime;
    }

    /// <summary>
    /// The gear the box wants for a driveshaft turning at <paramref name="shaft"/>
    /// radians a second. It reads the engine speed the wheels would give, not the
    /// engine's own, which flares while the clutch is slipping and would shift
    /// again.
    /// </summary>
    private int Auto(double throttle, double shaft)
    {
        if (_gear < 1)
        {
            return 1;
        }

        if (_shifting > 0)
        {
            return _gear;
        }

        var redline = _spec.Engine.Redline;
        var turning = Math.Abs(Ratio() * shaft) * RpmPerRadian;
        if (_gear < _spec.Ratios.Count && throttle > 0 && turning > redline * _spec.Upshift)
        {
            return _gear + 1;
        }

        return _gear > 1 && turning < redline * _spec.Downshift ? _gear - 1 : _gear;
    }

    /// <summary>Turns of the engine for a turn of the wheels.</summary>
    private double Ratio()
    {
        if (_gear == 0)
        {
            return 0.0;
        }

        var gear = _gear < 0 ? -_spec.Reverse : _spec.Ratios[_gear - 1];
        return gear * _spec.FinalDrive;
    }

    /// <summary>
    /// While a shift is under way the engine is brought to the speed the new gear
    /// will give, so the clutch comes back on a matched engine and does not
    /// jolt. Null when there is no shift or the gearbox is in neutral.
    /// </summary>
    private double? Matched(double ratio, double shaft, double dt)
    {
        if (_shifting <= 0 || ratio == 0)
        {
            return null;
        }

        var target = Math.Max(_spec.Engine.Idle / RpmPerRadian, Math.Abs(ratio * shaft));
        return _speed + ((target - _speed) * Math.Min(1.0, dt / _shifting));
    }

    /// <summary>Opens the throttle by itself to keep the engine from stalling at idle.</summary>
    private double Governor()
    {
        var idle = _spec.Engine.Idle;
        return Doubles.Clamp(((idle * 1.05) - Rpm) / (idle * 0.1), 0.0, 1.0);
    }

    /// <summary>
    /// Newton metres the clutch holds between the engine and the driveshaft, for
    /// an engine that gives <paramref name="push"/>. Positive drives the car.
    /// </summary>
    private double Held(double ratio, IReadOnlyList<Func<double, double>> spinAfter, double push, double dt)
    {
        var open = _spec.Engine.Idle * Open;
        var take = (Rpm - open) / (_spec.Engage - open);
        var shifted = _spec.ShiftTime == 0 ? 1.0 : 1 - (_shifting / _spec.ShiftTime);
        var most = ratio == 0 ? 0.0 : _spec.Clutch * Doubles.Clamp(take, 0.0, 1.0) * shifted;
        if (most == 0)
        {
            return 0.0;
        }

        // The clutch holds `stiffness` newton metres for each radian a second of
        // slip at the end of the step, and the wheels answer a torque as the tyres
        // say. The residual rises with the torque, since a wheel that is driven
        // harder ends faster, so a bisection finds the one torque that fits both.
        var stiffness = most / Slip;
        var inertia = _spec.Engine.Inertia;
        var share = ratio / _driven.Count;
        double Residual(double held)
        {
            var shaft = Mean(i => spinAfter[i](share * held));
            return held - (stiffness * (_speed + ((push - held) * dt / inertia) - (ratio * shaft)));
        }

        if (Residual(most) <= 0)
        {
            return most;
        }

        if (Residual(-most) >= 0)
        {
            return -most;
        }

        var (low, high) = (-most, most);
        for (var pass = 0; pass < Passes; pass++)
        {
            var mid = (low + high) / 2;
            if (Residual(mid) < 0)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }

        return (low + high) / 2;
    }

    /// <summary>
    /// Shares <paramref name="total"/> newton metres out: evenly between the
    /// axles, then evenly between the wheels of an axle, with the differential
    /// moving torque to the slower wheel.
    /// </summary>
    private void Split(double total, IReadOnlyList<double> spins, double dt)
    {
        var axles = new OrderedDictionary<double, List<int>>();
        foreach (var i in _driven)
        {
            var z = _wheels[i].Mount.Z;
            if (!axles.TryGetValue(z, out var axle))
            {
                axle = [];
                axles[z] = axle;
            }

            axle.Add(i);
        }

        foreach (var axle in axles.Values)
        {
            foreach (var i in axle)
            {
                _torques[i] = total / axles.Count / axle.Count;
            }

            if (axle.Count != 2)
            {
                continue;
            }

            var (a, b) = (axle[0], axle[1]);
            var catchUp = _wheels[a].Inertia * (spins[b] - spins[a]) / (2 * dt);
            var moved = Doubles.Clamp(catchUp, -_spec.Lock, _spec.Lock);
            _torques[a] += moved;
            _torques[b] -= moved;
        }
    }
}
