using System.Runtime.InteropServices;
using Orblit.Interop;

namespace Orblit.Physics;

/// <summary>
/// The C ABI of <c>orblit_physics.h</c>, declared and nothing else. A world is
/// a <see cref="nint"/>. Each part of the port declares only the calls it makes.
/// </summary>
internal static partial class NativeMethods
{
    private const string Library = "orblit_physics";

    static NativeMethods() => NativeLoader.Register(typeof(NativeMethods).Assembly, Library);

    [LibraryImport(Library, EntryPoint = "orblit_physics_defaults")]
    internal static partial void Defaults(out NativeSettings settings);

    [LibraryImport(Library, EntryPoint = "orblit_physics_create")]
    internal static partial nint Create(in NativeSettings settings);

    [LibraryImport(Library, EntryPoint = "orblit_physics_destroy")]
    internal static partial void Destroy(nint physics);

    [LibraryImport(Library, EntryPoint = "orblit_physics_submit")]
    internal static unsafe partial void Submit(nint physics, NativeCommand* commands, uint count);

    [LibraryImport(Library, EntryPoint = "orblit_physics_step")]
    internal static partial void Step(nint physics, float delta);

    [LibraryImport(Library, EntryPoint = "orblit_physics_read")]
    internal static unsafe partial uint Read(nint physics, ulong* ids, uint count, float* output, uint stride, uint offset);

    [LibraryImport(Library, EntryPoint = "orblit_physics_events")]
    internal static unsafe partial NativeEvent* Events(nint physics, out uint count);

    [LibraryImport(Library, EntryPoint = "orblit_physics_count")]
    internal static partial uint Count(nint physics);

    [LibraryImport(Library, EntryPoint = "orblit_physics_alive")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static partial bool Alive(nint physics, ulong id);

    [LibraryImport(Library, EntryPoint = "orblit_physics_asleep")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static partial bool Asleep(nint physics, ulong id);

    [LibraryImport(Library, EntryPoint = "orblit_physics_transform")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static unsafe partial bool Transform(nint physics, ulong id, float* output);

    [LibraryImport(Library, EntryPoint = "orblit_physics_velocity")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static unsafe partial bool Velocity(nint physics, ulong id, float* output);

    [LibraryImport(Library, EntryPoint = "orblit_physics_cast")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static partial bool Cast(nint physics, in NativeCast query, out NativeHit hit);

    [LibraryImport(Library, EntryPoint = "orblit_physics_ground")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static partial bool Ground(nint physics, in NativeGround ground);

    [LibraryImport(Library, EntryPoint = "orblit_physics_footing")]
    internal static unsafe partial uint Footing(nint physics, ulong* ids, uint count, NativeFooting* output);

    [LibraryImport(Library, EntryPoint = "orblit_physics_stance")]
    [return: MarshalAs(UnmanagedType.U1)]
    internal static unsafe partial bool Stance(nint physics, ulong id, uint shape, float* size);
}
