using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

public class SoftBalloonTests : SoftTestBase
{
    /// <summary>The plane y = 0, for a body to land on.</summary>
    internal sealed class Floor : ISoftSurface
    {
        public SoftContact? Sweep(Vector3 from, Vector3 direction, double distance, double radius)
        {
            var lowest = from.Y + Math.Min(0.0, direction.Normalized().Y * distance);
            return lowest - radius > 0
                ? null
                : new SoftContact(new Vector3(from.X, 0, from.Z), new Vector3(0, 1, 0));
        }
    }

    /// <summary>
    /// A sphere of <paramref name="radius"/> about <paramref name="centre"/>,
    /// made from an octahedron cut into four triangles twice over, wound with
    /// the normals out.
    /// </summary>
    private static (List<Vector3> Points, List<int> Triangles) Sphere(double radius, Vector3 centre)
    {
        var points = new List<Vector3>
        {
            new(1, 0, 0), new(-1, 0, 0), new(0, 1, 0), new(0, -1, 0), new(0, 0, 1), new(0, 0, -1),
        };
        var triangles = new List<int>();
        foreach (var sx in new[] { 0, 1 })
        {
            foreach (var sy in new[] { 2, 3 })
            {
                foreach (var sz in new[] { 4, 5 })
                {
                    // A face with an odd number of negative axes runs the other way round.
                    var flip = (sx + (sy - 2) + (sz - 4)) % 2 == 1;
                    triangles.AddRange(flip ? [sx, sz, sy] : [sx, sy, sz]);
                }
            }
        }

        for (var round = 0; round < 2; round++)
        {
            var middles = new Dictionary<(int, int), int>();
            int Middle(int a, int b)
            {
                var key = a < b ? (a, b) : (b, a);
                if (!middles.TryGetValue(key, out var found))
                {
                    var half = (points[a] + points[b]) / 2.0;
                    half.Normalize();
                    points.Add(half);
                    found = points.Count - 1;
                    middles[key] = found;
                }

                return found;
            }

            var cut = new List<int>();
            for (var t = 0; t < triangles.Count; t += 3)
            {
                var (a, b, c) = (triangles[t], triangles[t + 1], triangles[t + 2]);
                var (ab, bc, ca) = (Middle(a, b), Middle(b, c), Middle(c, a));
                cut.AddRange([a, ab, ca, ab, b, bc, ca, bc, c, ab, bc, ca]);
            }

            triangles = cut;
        }

        return ([.. points.Select(p => (p * radius) + centre)], triangles);
    }

    /// <summary>
    /// A rubber balloon: its edges give, so only the volume holds it up. Edges
    /// as stiff as the default make the mesh a rigid shell that needs no volume.
    /// </summary>
    private static SoftBody Balloon(double give = 0, ISoftSurface? surface = null)
    {
        var (points, triangles) = Sphere(0.3, new Vector3(0, 0.6, 0));
        return SoftBody.Balloon(
            points,
            triangles,
            volumeCompliance: give,
            spec: new SoftSpec(stretchCompliance: 1e-2),
            surface: surface);
    }

    [Fact]
    public void AClosedMeshReadsAsTheVolumeItEnclosesWhereverItIs()
    {
        var body = Balloon();
        var sphere = 4.0 / 3 * Math.PI * 0.3 * 0.3 * 0.3;

        // Flat faces cut off a tenth of a sphere.
        Assert.Equal(sphere, body.Volume!.Value, sphere * 0.1);

        var (points, triangles) = Sphere(0.3, new Vector3(40, -25, 7));
        var far = SoftBody.Balloon(points, triangles);
        Assert.Equal(body.Volume.Value, far.Volume!.Value, 1e-9);
    }

    [Fact]
    public void ABalloonOnAFloorKeepsItsVolumeAndDoesNotGoFlat()
    {
        var body = Balloon(surface: new Floor());
        var before = body.Volume!.Value;
        Run(body, 3);
        Assert.Equal(before, body.Volume!.Value, before * 0.02);
        Greater(Highest(body) - Lowest(body), 0.4);
    }

    [Fact]
    public void AClothEnclosesNothing()
    {
        var sheet = SoftBody.Cloth(2, 2, Vector3.Zero(), new Vector3(1, 0, 0), new Vector3(0, 0, 1));
        Assert.Null(sheet.Volume);
    }

    [Fact]
    public void ABalloonWithGiveIsSqueezedMoreThanOneWithout()
    {
        var firm = Balloon(surface: new Floor());
        var soft = Balloon(give: 1e-4, surface: new Floor());
        Run(firm, 3);
        Run(soft, 3);
        Less(soft.Volume!.Value, firm.Volume!.Value * 0.99);
    }

    [Fact]
    public void ASurfaceWithAHoleOrWoundInsideOutIsRefused()
    {
        var (points, triangles) = Sphere(0.3, Vector3.Zero());
        Assert.ThrowsAny<ArgumentException>(() => SoftBody.Balloon(points, [.. triangles.Skip(3)]));
        var inside = new List<int>();
        for (var t = 0; t < triangles.Count; t += 3)
        {
            inside.AddRange([triangles[t], triangles[t + 2], triangles[t + 1]]);
        }

        Assert.ThrowsAny<ArgumentException>(() => SoftBody.Balloon(points, inside));
        Assert.Throws<ArgumentOutOfRangeException>(() => SoftBody.Balloon(points, [.. triangles, 0, 1, 999]));
    }
}
