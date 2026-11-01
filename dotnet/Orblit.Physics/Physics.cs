using System.Runtime.InteropServices;

namespace Orblit.Physics;

/// <summary>
/// A rigid body world.
/// </summary>
/// <remarks>
/// Bodies are named by whatever the caller calls them, an Orblit entity handle
/// or any number that is not zero. The world never interprets one. It only
/// matches them, which is what lets this package know nothing about the entity
/// store it is usually driven by.
/// </remarks>
public sealed unsafe partial class Physics : IDisposable
{
    // A full buffer is sent rather than grown. Commands arrive in the same
    // order either way, and one allocation that never moves beats a doubling
    // one that does.
    private const int Room = 64;

    private static readonly double[] Identity = [0.0, 0.0, 0.0, 1.0];

    private nint _world;
    private NativeCommand* _commands;
    private int _pending;

    public Physics(PhysicsSettings? settings = null)
    {
        var native = (settings ?? new PhysicsSettings()).ToNative();
        _world = NativeMethods.Create(in native);
        if (_world == nint.Zero)
        {
            throw new InvalidOperationException("Could not create an Orblit physics world.");
        }

        _commands = (NativeCommand*)NativeMemory.AllocZeroed(Room, (nuint)sizeof(NativeCommand));
    }

    private nint World
    {
        get
        {
            RequireAlive();
            return _world;
        }
    }

    public void Dispose()
    {
        if (_world == nint.Zero)
        {
            return;
        }

        NativeMethods.Destroy(_world);
        _world = nint.Zero;
        NativeMemory.Free(_commands);
        _commands = null;
        _pending = 0;
    }

    private void RequireAlive()
    {
        if (_world == nint.Zero)
        {
            throw new InvalidOperationException("This Physics world has been disposed.");
        }
    }

    /// <summary>The next free command, zeroed, with its kind and body already set.</summary>
    private NativeCommand* Queue(uint kind, ulong id)
    {
        RequireAlive();
        if (_pending == Room)
        {
            Flush();
        }

        var command = _commands + _pending++;
        *command = default;
        command->Kind = kind;
        command->Id = id;
        return command;
    }

    private void Flush()
    {
        if (_pending == 0)
        {
            return;
        }

        var sending = (uint)_pending;
        // Cleared before the call rather than after, so a command submitted
        // from inside whatever this triggers cannot be sent twice.
        _pending = 0;
        NativeMethods.Submit(World, _commands, sending);
    }

    private static void Write(float* into, IReadOnlyList<double>? from, int count)
    {
        if (from is null)
        {
            return;
        }

        for (var i = 0; i < count; i++)
        {
            into[i] = (float)from[i];
        }
    }

    private static void WriteSize(float* into, Shape shape)
    {
        into[0] = (float)shape.X;
        into[1] = (float)shape.Y;
        into[2] = (float)shape.Z;
        into[3] = (float)shape.W;
    }

    private static double[] Vector(float* from) => [from[0], from[1], from[2]];

    private static byte Flag(bool value) => value ? (byte)1 : (byte)0;
}
