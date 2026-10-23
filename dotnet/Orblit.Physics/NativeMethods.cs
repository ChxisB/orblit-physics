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

    /// <summary>
    /// <c>OrblitPhysicsSettings</c>. The header has a C <c>bool</c> and three
    /// reserved bytes where this has four bytes, because a <see cref="bool"/> is
    /// not blittable.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct Settings
    {
        public float GravityX;
        public float GravityY;
        public float GravityZ;
        public uint SubSteps;
        public uint VelocitySteps;
        public uint PositionSteps;
        public uint JointSteps;
        public float Slop;
        public float ContactHertz;
        public float BounceThreshold;
        public float SleepSpeed;
        public float SleepAfter;
        public byte Sleeping;
        public byte Reserved0;
        public byte Reserved1;
        public byte Reserved2;
    }

    [LibraryImport(Library, EntryPoint = "orblit_physics_defaults")]
    internal static partial void Defaults(out Settings settings);

    [LibraryImport(Library, EntryPoint = "orblit_physics_create")]
    internal static partial nint Create(in Settings settings);

    [LibraryImport(Library, EntryPoint = "orblit_physics_destroy")]
    internal static partial void Destroy(nint physics);
}
