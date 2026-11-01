namespace Orblit.Physics;

public sealed unsafe partial class Physics
{
    /// <summary>
    /// Makes a body fixed, driven or free. This is how a crate is lifted by a
    /// cutscene and dropped again, and how a door is locked by making it fixed.
    /// </summary>
    /// <remarks>
    /// A body made fixed or driven keeps where it is and stops feeling forces.
    /// Fixed stops it, and driven keeps the velocity it had. A body made free
    /// has the mass it was created with and the controls it was given, then
    /// falls and is pushed. What rested on it or is resting on it is woken.
    /// Ignored for a trigger, a character, ground, a body that is not there and
    /// a body that is already what it is asked to be.
    /// </remarks>
    public void SetMotion(ulong id, PhysicsMotion motion)
    {
        var command = Queue(9, id);
        command->Motion = (uint)motion;
    }

    /// <summary>
    /// Sets how a body moves, all of <see cref="PhysicsControls"/> at once, and
    /// wakes it and what rests on it. False, and nothing changed, for a body
    /// that is not there, ground, a number that is not finite, or a negative
    /// cap or inertia.
    /// </summary>
    /// <remarks>
    /// To change one field, send the others again. <paramref name="quiet"/>
    /// wakes nothing, for a body just added: one added asleep stays asleep with
    /// the controls it was added with.
    /// </remarks>
    public bool SetControls(ulong id, PhysicsControls controls, bool quiet = false)
    {
        RequireAlive();
        Flush();
        var made = new NativeControls
        {
            Body = id,
            Quiet = Flag(quiet),
            Continuous = Flag(controls.Continuous),
            Locks = (uint)controls.Locks,
            GravityScale = (float)controls.GravityScale,
            MaxSpeed = (float)controls.MaxSpeed,
            MaxSpin = (float)controls.MaxSpin,
        };
        Write(made.Centre, controls.Centre, 3);
        Write(made.Inertia, controls.Inertia, 3);
        return NativeMethods.Controls(World, in made);
    }

    /// <summary>
    /// Sets the world's gravity in metres per second squared, and wakes every
    /// body that is not fixed or driven. False, and nothing changed, for a
    /// number that is not finite. Zones and each body's gravity scale still
    /// apply on top of it.
    /// </summary>
    public bool SetGravity(IReadOnlyList<double> gravity)
    {
        RequireAlive();
        Flush();
        var made = stackalloc float[3];
        Write(made, gravity, 3);
        return NativeMethods.Gravity(World, made);
    }
}
