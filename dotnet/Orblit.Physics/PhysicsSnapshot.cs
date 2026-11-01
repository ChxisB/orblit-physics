namespace Orblit.Physics;

/// <summary>A world at one instant, to go back to with <see cref="Physics.Restore"/>.</summary>
/// <remarks>
/// It is native memory, so it is let go of with <see cref="Dispose"/>. It is a
/// copy that owns itself: disposing the world it came from leaves it whole, and
/// it may be restored into another world, or into the same one any number of
/// times.
/// </remarks>
public sealed class PhysicsSnapshot : IDisposable
{
    private nint _handle;

    internal PhysicsSnapshot(nint handle) => _handle = handle;

    public bool Disposed => _handle == nint.Zero;

    internal nint Handle
    {
        get
        {
            if (_handle == nint.Zero)
            {
                throw new InvalidOperationException("This PhysicsSnapshot has been disposed.");
            }

            return _handle;
        }
    }

    public void Dispose()
    {
        if (_handle == nint.Zero)
        {
            return;
        }

        NativeMethods.SnapshotDestroy(_handle);
        _handle = nint.Zero;
    }
}
