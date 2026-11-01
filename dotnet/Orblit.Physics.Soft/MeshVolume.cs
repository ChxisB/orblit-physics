namespace Orblit.Physics.Soft;

/// <summary>
/// Holds the volume of a <see cref="ClosedMesh"/> as a position constraint.
/// </summary>
/// <remarks>
/// The volume is the sum, over the triangles, of the signed volume of the
/// tetrahedron each makes with the origin. A closed mesh gives the same sum
/// wherever it is. The gradient at a particle is the sum, over the triangles
/// it is in, of the cross product of the other two corners over six.
/// </remarks>
internal sealed class MeshVolume
{
    private readonly ClosedMesh _mesh;
    private readonly double _compliance;
    private readonly double[] _gradient;

    /// <summary>What the constraint has pushed in the sub-step being solved.</summary>
    private double _lambda;

    public MeshVolume(ClosedMesh mesh, double[] x, double compliance)
    {
        _mesh = mesh;
        _compliance = compliance;
        _gradient = new double[x.Length];
        Rest = VolumeOf(x);
        if (!(Rest > 0))
        {
            throw new ArgumentException($"The triangles are wound inside out: volume {Rest}.", nameof(mesh));
        }
    }

    /// <summary>The volume to hold, in cubic metres: the one the mesh began with.</summary>
    public double Rest { get; }

    /// <summary>
    /// The volume the particles in <paramref name="x"/> enclose, in cubic
    /// metres. Reads <paramref name="x"/> and changes nothing, so it can be
    /// asked at any time.
    /// </summary>
    public double VolumeOf(double[] x)
    {
        var triangles = _mesh.Triangles;
        var sum = 0.0;
        for (var t = 0; t < triangles.Length; t += 3)
        {
            var a = triangles[t] * 3;
            var b = triangles[t + 1] * 3;
            var c = triangles[t + 2] * 3;
            sum +=
                (x[a] * ((x[b + 1] * x[c + 2]) - (x[b + 2] * x[c + 1])))
                + (x[a + 1] * ((x[b + 2] * x[c]) - (x[b] * x[c + 2])))
                + (x[a + 2] * ((x[b] * x[c + 1]) - (x[b + 1] * x[c])));
        }

        return sum / 6;
    }

    public void Restart() => _lambda = 0;

    /// <summary>
    /// Moves the particles, <paramref name="weight"/> being the inverse mass of
    /// each, towards the volume to hold by an amount that follows the compliance.
    /// </summary>
    public void Project(double[] x, double[] weight, double inverseSquare)
    {
        var error = Gather(x) - Rest;
        var alpha = _compliance * inverseSquare;
        var stiffness = alpha;
        for (var i = 0; i < weight.Length; i++)
        {
            var k = i * 3;
            stiffness +=
                weight[i]
                * ((_gradient[k] * _gradient[k])
                    + (_gradient[k + 1] * _gradient[k + 1])
                    + (_gradient[k + 2] * _gradient[k + 2]));
        }

        if (stiffness == 0)
        {
            return;
        }

        var change = (-error - (alpha * _lambda)) / stiffness;
        _lambda += change;
        for (var i = 0; i < weight.Length; i++)
        {
            for (var c = i * 3; c < (i * 3) + 3; c++)
            {
                x[c] += weight[i] * change * _gradient[c];
            }
        }
    }

    /// <summary>
    /// Fills the gradient, and answers the volume. The volume is a polynomial
    /// of degree three in the coordinates, so a third of the gradient dotted
    /// with the position is the volume itself.
    /// </summary>
    private double Gather(double[] x)
    {
        var triangles = _mesh.Triangles;
        Array.Clear(_gradient);
        for (var t = 0; t < triangles.Length; t += 3)
        {
            var a = triangles[t] * 3;
            var b = triangles[t + 1] * 3;
            var c = triangles[t + 2] * 3;
            Accumulate(x, a, b, c);
            Accumulate(x, b, c, a);
            Accumulate(x, c, a, b);
        }

        var sum = 0.0;
        for (var k = 0; k < x.Length; k++)
        {
            sum += x[k] * _gradient[k];
        }

        return sum / 3;
    }

    /// <summary>
    /// Adds a sixth of the cross product of particles <paramref name="p"/> and
    /// <paramref name="q"/> to the gradient of the particle at <paramref name="target"/>.
    /// </summary>
    private void Accumulate(double[] x, int target, int p, int q)
    {
        _gradient[target] += ((x[p + 1] * x[q + 2]) - (x[p + 2] * x[q + 1])) / 6;
        _gradient[target + 1] += ((x[p + 2] * x[q]) - (x[p] * x[q + 2])) / 6;
        _gradient[target + 2] += ((x[p] * x[q + 1]) - (x[p + 1] * x[q])) / 6;
    }
}
