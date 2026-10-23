using System.Runtime.InteropServices;

namespace Orblit.Physics;

// The structs of orblit_physics.h, field for field. A C bool is a byte here,
// because a bool field is not blittable, and a float array is a fixed buffer.

/// <summary><c>OrblitPhysicsSettings</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct NativeSettings
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

/// <summary><c>OrblitPhysicsCommand</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeCommand
{
    public uint Kind;
    public uint Shape;
    public ulong Id;
    public fixed float Size[4];
    public fixed float At[3];
    public fixed float Rotation[4];
    public fixed float Vector[3];
    public fixed float Spin[3];
    public uint Motion;
    public float Mass;
    public float Friction;
    public float Restitution;
    public fixed float Damping[2];
    public uint LayerIs;
    public uint LayerCares;
    public byte Asleep;
    public byte Sensor;
    public byte Stay;
    public byte Reserved;
    public ulong Hull;
}

/// <summary><c>OrblitPhysicsEvent</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeEvent
{
    public uint Kind;
    public uint Pad;
    public ulong A;
    public ulong B;
    public fixed float At[3];
    public fixed float Normal[3];
    public float Force;
}

/// <summary><c>OrblitPhysicsCast</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeCast
{
    public uint Shape;
    public uint LayerIs;
    public uint LayerCares;
    public byte Triggers;
    public fixed byte Reserved[3];
    public fixed float Size[4];
    public fixed float From[3];
    public fixed float Rotation[4];
    public fixed float Direction[3];
    public float Distance;
    public ulong Ignore;
    public ulong Hull;
}

/// <summary><c>OrblitPhysicsHit</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeHit
{
    public ulong Body;
    public fixed float At[3];
    public fixed float Normal[3];
    public float Distance;
    public byte Started;
    public fixed byte Reserved[3];
}

/// <summary><c>OrblitPhysicsGround</c>. The heights are the caller's memory.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeGround
{
    public ulong Id;
    public float* Heights;
    public uint Columns;
    public uint Rows;
    public float Spacing;
    public byte Margin;
    public fixed byte Reserved[3];
    public fixed float At[3];
    public float Friction;
    public float Restitution;
    public uint LayerIs;
    public uint LayerCares;
    public uint Pad;
}

/// <summary><c>OrblitPhysicsFooting</c>.</summary>
[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeFooting
{
    public ulong Ground;
    public fixed float Normal[3];
    public fixed float Velocity[3];
    public fixed float Carried[3];
    public float Turning;
    public byte Grounded;
    public byte Wall;
    public byte Ceiling;
    public fixed byte Reserved[5];
}
