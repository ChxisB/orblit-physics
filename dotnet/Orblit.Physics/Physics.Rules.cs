namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    private const uint ZoneGravity = 1;
    private const uint ZoneLinearDamping = 2;
    private const uint ZoneAngularDamping = 4;
    private const uint ZoneFluid = 8;

    private const uint RuleFriction = 1;
    private const uint RuleRestitution = 2;
    private const uint RuleMoveScale = 4;
    private const uint RuleIgnore = 8;
    private const uint RuleOneWay = 16;

    /// <summary>
    /// Puts <paramref name="zone"/> over the trigger <paramref name="id"/>,
    /// replacing any it had, and wakes what is inside it. False, and nothing
    /// changed, for a body that is not there or not a trigger, a number that is
    /// not a number, or a fluid density or drag that is negative or infinite.
    /// </summary>
    /// <remarks>
    /// It changes what the solver moves, so it acts on free bodies only. A
    /// character asks for its own gravity and is not touched by one.
    /// </remarks>
    public bool SetZone(ulong id, PhysicsZone zone)
    {
        var fields = Bit(zone.Gravity is not null, ZoneGravity)
            | Bit(zone.LinearDamping is not null, ZoneLinearDamping)
            | Bit(zone.AngularDamping is not null, ZoneAngularDamping)
            | Bit(zone.FluidDensity is not null, ZoneFluid);
        return SendZone(id, fields, zone);
    }

    /// <summary>
    /// Takes the zone off a trigger, leaving the trigger, and wakes what was
    /// inside it. False if there was none.
    /// </summary>
    public bool RemoveZone(ulong id) => SendZone(id, 0, new PhysicsZone());

    private bool SendZone(ulong id, uint fields, PhysicsZone zone)
    {
        RequireAlive();
        Flush();
        var made = new NativeZone { Body = id, Overrides = fields, Priority = zone.Priority };
        Write(made.Gravity, zone.Gravity, 3);
        made.Damping[0] = (float)(zone.LinearDamping ?? 0.0);
        made.Damping[1] = (float)(zone.AngularDamping ?? 0.0);
        made.Fluid[0] = (float)(zone.FluidDensity ?? 0.0);
        made.Fluid[1] = (float)zone.FluidDrag;
        return NativeMethods.Zone(World, in made);
    }

    /// <summary>
    /// Makes <paramref name="rule"/> hold whenever bodies <paramref name="a"/>
    /// and <paramref name="b"/> touch, until it is removed or either body goes,
    /// and wakes both. False, and nothing changed, for a body that is not
    /// there, a body against itself, a number that is not a number or a
    /// negative move scale. The order matters only to the move scales, which
    /// follow <paramref name="a"/> and <paramref name="b"/> however the world
    /// keeps them.
    /// </summary>
    public bool SetRule(ulong a, ulong b, PhysicsRule rule)
    {
        var fields = Bit(rule.Friction is not null, RuleFriction)
            | Bit(rule.Restitution is not null, RuleRestitution)
            | Bit(rule.MoveScaleA != 1.0 || rule.MoveScaleB != 1.0, RuleMoveScale)
            | Bit(rule.Ignore, RuleIgnore)
            | Bit(rule.OneWay is not null, RuleOneWay);
        return SendRule(a, b, fields, rule);
    }

    /// <summary>Takes away the rule between two bodies, waking both. False if there was none.</summary>
    public bool RemoveRule(ulong a, ulong b) => SendRule(a, b, 0, null);

    private bool SendRule(ulong a, ulong b, uint fields, PhysicsRule? rule)
    {
        RequireAlive();
        Flush();
        var made = new NativeRule
        {
            A = a,
            B = b,
            Overrides = fields,
            Friction = (float)(rule?.Friction ?? 0.0),
            Restitution = (float)(rule?.Restitution ?? 0.0),
        };
        made.MoveScale[0] = (float)(rule?.MoveScaleA ?? 1.0);
        made.MoveScale[1] = (float)(rule?.MoveScaleB ?? 1.0);
        Write(made.Through, rule?.OneWay, 3);
        return NativeMethods.Rule(World, in made);
    }

    private static uint Bit(bool on, uint bit) => on ? bit : 0u;
}
