namespace Orblit.Physics.Vehicle;

/// <summary>
/// The grip of the ground under a body, as a number to scale a tyre's by. One
/// is dry ground.
/// </summary>
public delegate double SurfaceGrip(ulong body);
