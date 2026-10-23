namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    private static readonly double[] CentreOfMass = [double.NaN, double.NaN, double.NaN];

    /// <summary>
    /// Adds a body. Ignored if <paramref name="id"/> is zero or already here.
    /// </summary>
    /// <remarks>
    /// A trigger is where things are rather than something they hit: nothing
    /// bounces off it and it pushes nothing, and solid casts, overlaps and
    /// characters do not see it. It is ignored for a free body, which the
    /// solver has to move. A body with stay hears about its contacts every step.
    /// </remarks>
    // F2: fourteen parameters, all but the first two optional and called by name.
    public void Add(
        ulong id,
        Shape shape,
        PhysicsMotion motion = PhysicsMotion.Free,
        IReadOnlyList<double>? at = null,
        IReadOnlyList<double>? rotation = null,
        double mass = 1.0,
        double friction = 0.5,
        double restitution = 0.0,
        double linearDamping = 0.05,
        double angularDamping = 0.05,
        Layers? layers = null,
        bool asleep = false,
        bool trigger = false,
        bool stay = false)
    {
        var command = Queue(1, id);
        var layering = layers ?? Layers.Everything;
        command->Shape = shape.Kind;
        command->Hull = shape.Asset;
        WriteSize(command->Size, shape);
        Write(command->At, at, 3);
        Write(command->Rotation, rotation ?? Identity, 4);
        command->Motion = (uint)motion;
        command->Mass = (float)mass;
        command->Friction = (float)friction;
        command->Restitution = (float)restitution;
        command->Damping[0] = (float)linearDamping;
        command->Damping[1] = (float)angularDamping;
        command->LayerIs = layering.Is;
        command->LayerCares = layering.Cares;
        command->Asleep = Flag(asleep);
        command->Sensor = Flag(trigger);
        command->Stay = Flag(stay);
    }

    /// <summary>Adds a character: a body that walks.</summary>
    /// <remarks>
    /// It is moved by sweeping its shape through the world rather than by the
    /// solver, so it slides along walls instead of bouncing off them, climbs
    /// anything up to <paramref name="stepHeight"/> metres tall, stands on
    /// slopes up to <paramref name="steepest"/> radians and slides down
    /// steeper ones, and rides whatever it is standing on. It pushes free
    /// bodies it walks into with at most <paramref name="strength"/> newtons
    /// and keeps <paramref name="skin"/> metres from everything. Tell it where
    /// to go with <see cref="Drive"/>, every step, and read what came of it
    /// with <see cref="FootingOf"/>. It does not fall on its own: gravity is
    /// part of what it asks for.
    /// </remarks>
    // F2: ten parameters, all but the first optional and called by name.
    public void AddCharacter(
        ulong id,
        Shape? shape = null,
        IReadOnlyList<double>? at = null,
        IReadOnlyList<double>? rotation = null,
        double stepHeight = 0.3,
        double steepest = Math.PI / 4,
        double skin = 0.01,
        double strength = 500.0,
        double friction = 0.5,
        Layers? layers = null)
    {
        Add(
            id,
            shape ?? Shape.Capsule(0.3, 0.6),
            PhysicsMotion.Driven,
            at,
            rotation,
            friction: friction,
            layers: layers);
        var command = Queue(7, id);
        command->Size[0] = (float)stepHeight;
        command->Size[1] = (float)steepest;
        command->Size[2] = (float)skin;
        command->Size[3] = (float)strength;
    }

    /// <summary>
    /// Lays ground as heights on a grid, as one fixed body named
    /// <paramref name="id"/>, and answers whether it was laid.
    /// </summary>
    /// <remarks>
    /// Sample (c, r) is <c>heights[r * columns + c]</c> and sits at
    /// <c>at + (c * spacing, height, r * spacing)</c>. Each square of four
    /// samples is two triangles, split from (c, r) to (c + 1, r + 1). A height
    /// that is not a finite number is a hole. Laying it again under the same id
    /// replaces it and wakes everything over it. With <paramref name="margin"/>
    /// the outermost ring of heights is the neighbouring pieces' samples, never
    /// stood on. False, and nothing changed, for a zero id, a spacing that is
    /// not a positive number, fewer heights than columns times rows, or a grid
    /// with no square of its own.
    /// </remarks>
    // F2: ten parameters, the last six optional and called by name.
    public bool LayGround(
        ulong id,
        IReadOnlyList<double> heights,
        int columns,
        int rows,
        double spacing = 1.0,
        bool margin = false,
        IReadOnlyList<double>? at = null,
        double friction = 0.5,
        double restitution = 0.0,
        Layers? layers = null)
    {
        RequireAlive();
        // Sent now rather than queued, so it lands after whatever was queued
        // before it: a body added under this id a moment ago is the one replaced.
        Flush();
        if (columns < 0 || rows < 0 || heights.Count < (long)columns * rows)
        {
            return false;
        }

        var samples = new float[Math.Max(columns * rows, 1)];
        for (var i = 0; i < columns * rows; i++)
        {
            samples[i] = (float)heights[i];
        }

        var layering = layers ?? Layers.Everything;
        fixed (float* grid = samples)
        {
            var ground = new NativeGround
            {
                Id = id,
                Heights = grid,
                Columns = (uint)columns,
                Rows = (uint)rows,
                Spacing = (float)spacing,
                Margin = Flag(margin),
                Friction = (float)friction,
                Restitution = (float)restitution,
                LayerIs = layering.Is,
                LayerCares = layering.Cares,
            };
            Write(ground.At, at, 3);
            return NativeMethods.Ground(World, in ground);
        }
    }

    /// <summary>
    /// Takes a body out of the world. Anything that was touching it is told the
    /// touch has ended on the next step. Its joints go with it, silently, and
    /// whatever they held is woken, so it falls.
    /// </summary>
    public void Remove(ulong id) => Queue(2, id);

    /// <summary>Puts a body exactly here, forgetting how it was moving. The teleport, not the shove.</summary>
    public void Place(ulong id, IReadOnlyList<double> at, IReadOnlyList<double>? rotation = null)
    {
        var command = Queue(3, id);
        Write(command->At, at, 3);
        Write(command->Rotation, rotation ?? Identity, 4);
    }

    /// <summary>
    /// Sets how a body is moving, outright. This is how a driven body is told
    /// where to go. For a character it is a request rather than an order: the
    /// velocity is where it would like to go, relative to whatever it is
    /// standing on and with gravity in it, and the world decides how much of
    /// that it gets. Spin is ignored for a character.
    /// </summary>
    public void Drive(ulong id, IReadOnlyList<double>? velocity = null, IReadOnlyList<double>? spin = null)
    {
        var command = Queue(4, id);
        Write(command->Vector, velocity, 3);
        Write(command->Spin, spin, 3);
    }

    /// <summary>
    /// Hits a body with <paramref name="impulse"/> newton-seconds at world
    /// point <paramref name="at"/>, which defaults to its centre of mass. Away
    /// from the centre it spins as well as moves.
    /// </summary>
    public void Push(ulong id, IReadOnlyList<double> impulse, IReadOnlyList<double>? at = null)
    {
        var command = Queue(5, id);
        Write(command->Vector, impulse, 3);
        // A point that is not a number is how the engine is told to use the
        // centre of mass, which only it knows the place of.
        Write(command->Spin, at ?? CentreOfMass, 3);
    }

    /// <summary>Wakes a body, whether or not anything touched it.</summary>
    public void Wake(ulong id) => Queue(6, id);

    /// <summary>
    /// Changes the shape character <paramref name="id"/> is made of, keeping
    /// its feet where they are, and answers whether it did. A crouch is a
    /// shorter capsule from the same sole. False, and nothing changed, for a
    /// body that is not a character, for any shape but a ball, box, capsule or
    /// cylinder, and for a shape that would overlap something the character
    /// does not pass through.
    /// </summary>
    public bool SetStance(ulong id, Shape shape)
    {
        RequireAlive();
        Flush();
        var size = stackalloc float[4];
        WriteSize(size, shape);
        return NativeMethods.Stance(World, id, shape.Kind, size);
    }
}
