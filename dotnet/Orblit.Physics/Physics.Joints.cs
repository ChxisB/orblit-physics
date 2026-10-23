namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    /// <summary>
    /// Joins body <paramref name="a"/> to body <paramref name="b"/> as
    /// <paramref name="joint"/> says, and answers whether it was made. Both
    /// bodies are woken.
    /// </summary>
    /// <remarks>
    /// Either may be zero, for the world, but not both. Every measure is
    /// <paramref name="b"/> as <paramref name="a"/> sees it, so which one is
    /// the world decides the sense: hang a door with the world as
    /// <paramref name="a"/> and its angle is the door's, and with the world as
    /// <paramref name="b"/> it is the world's as the door sees it, the other
    /// way round.
    /// It is made where the bodies stand: <paramref name="at"/> is the joint's
    /// point in the world, and <paramref name="rotation"/> its frame, whose x
    /// axis is the one a hinge turns about, a slider slides along and a cone
    /// points along. What it holds is how they stood, so a hinge made with its
    /// door shut reads nought when the door is shut. A distance joint keeps
    /// <paramref name="at"/> on <paramref name="a"/> a distance from
    /// <paramref name="to"/> on <paramref name="b"/>, and a pulley takes both
    /// ends that way. Every other kind ignores <paramref name="to"/>.
    /// Past <paramref name="breakingForce"/> newtons, or
    /// <paramref name="breakingTorque"/> newton-metres, it breaks: it is
    /// removed, and a <see cref="PhysicsEventKind.Broke"/> event says so.
    /// Nought never breaks. The two bodies do not collide with each other
    /// unless <paramref name="collide"/> says they do, because a joint's
    /// bodies nearly always overlap where it is.
    /// Joint ids are apart from body ids, so a joint may share a number with a
    /// body. False, and nothing changed, for a zero id or one already a joint,
    /// an <paramref name="a"/> or <paramref name="b"/> that is neither zero nor
    /// a body, a body or the world joined to itself, or a number that is not a
    /// number. <paramref name="path"/> is a path joint's line, and
    /// <paramref name="over"/> a pulley's two fixed points. A point that is not
    /// x, y and z is an <see cref="ArgumentException"/>, as are not two points
    /// over a pulley.
    /// </remarks>
    // F2: thirteen parameters, all but the first four optional and called by name.
    public bool Join(
        ulong id,
        Joint joint,
        ulong a,
        IReadOnlyList<double> at,
        ulong b = 0,
        IReadOnlyList<double>? rotation = null,
        IReadOnlyList<double>? to = null,
        IReadOnlyList<double>? axisB = null,
        IReadOnlyList<IReadOnlyList<double>>? over = null,
        IReadOnlyList<IReadOnlyList<double>>? path = null,
        double breakingForce = 0.0,
        double breakingTorque = 0.0,
        bool collide = false)
    {
        RequireAlive();
        // Sent first so a body added or placed a moment ago is where the joint
        // is made.
        Flush();
        NativeJoint made = default;
        made.Id = id;
        made.A = a;
        made.B = b;
        made.Kind = (uint)joint.Kind;
        WriteLimits(&made, joint);
        Write(made.At, at, 3);
        Write(made.Rotation, rotation ?? Identity, 4);
        Write(made.To, to ?? at, 3);
        made.Swing = (float)joint.Swing;
        made.Speed = (float)joint.Speed;
        made.Strength = (float)joint.Strength;
        WriteGive(&made, joint, LengthOf(joint, at, to));
        WriteTie(&made, joint, axisB, over);
        made.BreakingForce = (float)breakingForce;
        made.BreakingTorque = (float)breakingTorque;
        made.Collide = Flag(collide);
        if (path is null)
        {
            return NativeMethods.Join(World, &made);
        }

        fixed (float* line = FlattenPath(path))
        {
            return NativeMethods.JoinPath(World, &made, line, (uint)path.Count);
        }
    }

    /// <summary>
    /// Removes joint <paramref name="id"/>, waking both its bodies so what it
    /// held up falls. False if there was no such joint.
    /// </summary>
    public bool Unjoin(ulong id)
    {
        Flush();
        return NativeMethods.Unjoin(World, id);
    }

    /// <summary>
    /// Moves where joint <paramref name="id"/>'s springs and goal hold, without
    /// making it again.
    /// </summary>
    /// <remarks>
    /// <paramref name="rest"/> is where each of the six ways is pulled to, in
    /// the order <see cref="JointSprings.All"/> gives them: metres along the
    /// frame's axes, then radians about them. A null leaves that way where it
    /// was, and a null list leaves all six. A way with no spring ignores its
    /// place, and <paramref name="goal"/> is ignored by a joint with no
    /// <see cref="JointGoal"/>, as they are when it is made.
    /// Both bodies wake when a place moved and not otherwise, so aiming at
    /// where a joint already holds lets what hangs from it sleep. False if
    /// there is no such joint or a value is infinite.
    /// </remarks>
    public bool Aim(ulong id, IReadOnlyList<double?>? rest = null, double? goal = null)
    {
        if (rest is not null && rest.Count != 6)
        {
            throw new ArgumentException("Give a place for each of six.", nameof(rest));
        }

        Flush();
        Span<float> places = stackalloc float[6];
        for (var i = 0; rest is not null && i < 6; i++)
        {
            places[i] = (float)(rest[i] ?? double.NaN);
        }

        fixed (float* held = places)
        {
            return NativeMethods.Aim(World, id, rest is null ? null : held, (float)(goal ?? double.NaN));
        }
    }

    /// <summary>How joint <paramref name="id"/> stands now, or null if there is no such joint.</summary>
    public JointState? JointStateOf(ulong id)
    {
        Flush();
        NativeJointState state = default;
        if (!NativeMethods.ReadJoint(World, id, &state))
        {
            return null;
        }

        return new JointState(Vector(state.Offset), Vector(state.Angles), state.Force, state.Torque);
    }

    /// <summary>A path's points as the flat floats the ABI takes.</summary>
    private static float[] FlattenPath(IReadOnlyList<IReadOnlyList<double>> path)
    {
        // An empty array pins to a null pointer, which the engine would refuse for the wrong reason.
        var flat = new float[Math.Max(path.Count * 3, 1)];
        for (var i = 0; i < path.Count; i++)
        {
            if (path[i].Count != 3)
            {
                throw new ArgumentException("A point is x, y and z.", nameof(path));
            }

            for (var k = 0; k < 3; k++)
            {
                flat[i * 3 + k] = (float)path[i][k];
            }
        }

        return flat;
    }

    private static void WriteLimits(NativeJoint* it, Joint joint)
    {
        var limits = joint.Limits;
        uint limited = 0;
        for (var i = 0; i < 6; i++)
        {
            if (limits[i] is not { } limit)
            {
                continue;
            }

            limited |= 1u << i;
            it->Low[i] = (float)limit.Low;
            it->High[i] = (float)limit.High;
        }

        it->Limited = limited;
    }

    /// <summary>
    /// Writes what a coupling or a path adds to a joint: its ratio, the axis of
    /// its second body, the two fixed points of a pulley, and whether a path
    /// turns its body to follow.
    /// </summary>
    private static void WriteTie(
        NativeJoint* it,
        Joint joint,
        IReadOnlyList<double>? axisB,
        IReadOnlyList<IReadOnlyList<double>>? over)
    {
        it->Ratio = (float)joint.Ratio;
        it->Follow = Flag(joint.Follow);
        Write(it->AxisB, axisB, 3);
        if (over is null)
        {
            return;
        }

        if (over.Count != 2)
        {
            throw new ArgumentException("A pulley has two points.", nameof(over));
        }

        for (var i = 0; i < 2; i++)
        {
            Write(it->Over + i * 3, over[i], 3);
        }
    }

    /// <summary>
    /// The length a distance joint is made at, which is where a spring on it
    /// rests unless it says otherwise.
    /// </summary>
    private static double LengthOf(Joint joint, IReadOnlyList<double> at, IReadOnlyList<double>? to)
    {
        if (joint.Kind != PhysicsJointKind.Distance || to is null)
        {
            return 0.0;
        }

        var dx = to[0] - at[0];
        var dy = to[1] - at[1];
        var dz = to[2] - at[2];
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    /// <summary>Writes the goal, the springs and the soft limit of a joint.</summary>
    private static void WriteGive(NativeJoint* it, Joint joint, double length)
    {
        if (joint.Goal is { } goal)
        {
            it->Goal = (float)goal.At;
            it->GoalHertz = (float)goal.Hertz;
            it->GoalRatio = (float)goal.Ratio;
        }

        var springs = joint.Springs.All;
        for (var i = 0; i < 6; i++)
        {
            if (springs[i] is not { } spring)
            {
                continue;
            }

            it->SpringHertz[i] = (float)spring.Hertz;
            it->SpringRatio[i] = (float)spring.Ratio;
            it->SpringRest[i] = (float)(spring.Rest ?? (i == 0 ? length : 0.0));
        }

        if (joint.SoftLimit is { } soft)
        {
            it->LimitHertz = (float)soft.Hertz;
            it->LimitRatio = (float)soft.Ratio;
        }
    }
}
