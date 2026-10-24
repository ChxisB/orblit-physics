using Orblit.Scene;

namespace Orblit.Physics.Scene;

/// <summary>
/// A <see cref="ScenePhysics"/> at one instant, to go back to with <see cref="ScenePhysics.Restore"/>.
/// </summary>
/// <remarks>
/// It holds the world's own <see cref="PhysicsSnapshot"/>, so like that one it is native memory and is
/// let go of with <see cref="Dispose"/>. Beside the world it keeps what the scene knows and the world
/// does not: the document, which entity is which body and joint, the numbers still to hand out, and
/// what has broken or been told to pass through. In a scene that smooths, it also keeps how each body
/// was moving in the blend.
/// </remarks>
public sealed class ScenePhysicsSnapshot : IDisposable
{
    internal ScenePhysicsSnapshot(PhysicsSnapshot world, SceneDocument document)
    {
        World = world;
        Document = document;
    }

    /// <summary>Whether <see cref="Dispose"/> has let the world's copy go.</summary>
    public bool Disposed => World.Disposed;

    internal PhysicsSnapshot World { get; }

    internal SceneDocument Document { get; }

    internal bool Smooth { get; init; }

    internal OrderedDictionary<string, ulong> BodyOf { get; init; } = [];

    internal Dictionary<ulong, string> EntityOf { get; init; } = [];

    internal OrderedDictionary<ulong, float[]> Known { get; init; } = [];

    internal OrderedDictionary<ulong, float[]> Previous { get; init; } = [];

    internal OrderedDictionary<ulong, float[]> Latest { get; init; } = [];

    internal HashSet<string> Sharp { get; init; } = [];

    internal ulong Next { get; init; }

    internal Dictionary<string, ulong> HullIdOf { get; init; } = [];

    internal Dictionary<ulong, string> HullKeyOf { get; init; } = [];

    internal ulong NextHull { get; init; }

    internal HashSet<ulong> CompoundBodies { get; init; } = [];

    internal HashSet<ulong> MeshBodies { get; init; } = [];

    internal Dictionary<ulong, HashSet<string>> PartHullKeys { get; init; } = [];

    internal OrderedDictionary<string, ulong> JointOf { get; init; } = [];

    internal Dictionary<ulong, string> EntityOfJoint { get; init; } = [];

    internal OrderedDictionary<string, (ulong A, ulong B)> Held { get; init; } = [];

    internal HashSet<string> Broken { get; init; } = [];

    internal ulong NextJoint { get; init; }

    internal HashSet<(string A, string B)> Ignored { get; init; } = [];

    internal HashSet<ulong> Crouching { get; init; } = [];

    internal double Owed { get; init; }

    internal IReadOnlyList<PhysicsEvent> Events { get; init; } = [];

    public void Dispose() => World.Dispose();
}
