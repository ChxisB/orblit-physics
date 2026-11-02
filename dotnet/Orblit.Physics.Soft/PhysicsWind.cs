using Orblit.VectorMath;

namespace Orblit.Physics.Soft;

/// <summary>
/// Wind from the triggers of a physics world.
/// </summary>
/// <remarks>
/// <paramref name="zones"/> maps the id of a trigger body to the wind blowing
/// inside it. Where triggers overlap their winds add. A trigger that is not in
/// the map is not wind, so a trigger made for something else blows nothing.
/// </remarks>
public sealed class PhysicsWind(Physics physics, IReadOnlyDictionary<ulong, Vector3> zones) : ISoftAir
{
    public Physics Physics { get; } = physics;

    public IReadOnlyDictionary<ulong, Vector3> Zones { get; } = zones;

    public Vector3 WindAt(Vector3 point)
    {
        var wind = Vector3.Zero();
        // Asked once per particle every step, so the ids go on the stack. The room
        // is what the list form holds, so both forms agree on how many zones count.
        Span<ulong> found = stackalloc ulong[Physics.DefaultOverlapLimit];
        var count = Physics.OverlapInto([point.X, point.Y, point.Z], found, triggers: true);
        foreach (var id in found[..count])
        {
            if (Zones.TryGetValue(id, out var zone))
            {
                wind.Add(zone);
            }
        }

        return wind;
    }
}
