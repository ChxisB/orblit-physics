namespace Orblit.Physics.Soft;

/// <summary>
/// A triangle mesh with no holes, and the volume it holds.
/// </summary>
/// <remarks>
/// Every edge belongs to two triangles that run along it in opposite
/// directions, which is what a surface wound the same way all over does. The
/// volume is positive when the normals point out.
/// </remarks>
internal sealed class ClosedMesh
{
    /// <summary>
    /// Throws unless <paramref name="triangles"/>, three indices into
    /// <paramref name="particles"/> each, close up.
    /// </summary>
    public ClosedMesh(int particles, IReadOnlyList<int> triangles)
    {
        Triangles = [.. triangles];
        if (triangles.Count < 12 || triangles.Count % 3 != 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(triangles),
                triangles.Count,
                "Must be three indices for each of 4 triangles at least.");
        }

        var seen = Tally(particles, triangles);
        foreach (var key in seen.Keys)
        {
            if (!IsPaired(seen, particles, key))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(triangles),
                    (key / particles, key % particles),
                    "Must close up, each edge run once in each direction.");
            }
        }

        Edges =
        [
            .. seen.Keys
                .Where(key => key / particles < key % particles)
                .Select(key => ((int)(key / particles), (int)(key % particles))),
        ];
    }

    /// <summary>Three indices for each triangle, wound so the normals point out.</summary>
    public int[] Triangles { get; }

    /// <summary>Each edge once, as the lower index and then the higher.</summary>
    public IReadOnlyList<(int Low, int High)> Edges { get; }

    /// <summary>
    /// How many times each directed edge is run, keyed by the first index times
    /// the particle count plus the second, in the order the edges are first met.
    /// </summary>
    private static OrderedDictionary<long, int> Tally(int particles, IReadOnlyList<int> triangles)
    {
        var seen = new OrderedDictionary<long, int>();
        for (var t = 0; t < triangles.Count; t += 3)
        {
            for (var e = 0; e < 3; e++)
            {
                var a = triangles[t + e];
                var b = triangles[t + ((e + 1) % 3)];
                if (a < 0 || a >= particles)
                {
                    throw OutOfRange(a, particles, nameof(triangles));
                }

                if (b < 0 || b >= particles)
                {
                    throw OutOfRange(b, particles, nameof(triangles));
                }

                var key = ((long)a * particles) + b;
                seen[key] = Runs(seen, key) + 1;
            }
        }

        return seen;
    }

    private static ArgumentOutOfRangeException OutOfRange(int index, int particles, string name) =>
        new(name, index, $"Must be 0 to {particles - 1}.");

    private static int Runs(OrderedDictionary<long, int> seen, long key) => seen.TryGetValue(key, out var runs) ? runs : 0;

    /// <summary>Whether the edge at <paramref name="key"/> is run once, and its reverse once.</summary>
    private static bool IsPaired(OrderedDictionary<long, int> seen, int particles, long key)
    {
        var a = key / particles;
        var b = key % particles;
        return a != b && Runs(seen, key) == 1 && Runs(seen, (b * particles) + a) == 1;
    }
}
