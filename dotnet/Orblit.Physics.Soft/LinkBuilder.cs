using System.Diagnostics.CodeAnalysis;

namespace Orblit.Physics.Soft;

/// <summary>Collects links with the length they have in the starting positions.</summary>
internal sealed class LinkBuilder(double[] x)
{
    private readonly List<int> _from = [];
    private readonly List<int> _to = [];
    private readonly List<double> _rest = [];
    private readonly List<double> _alpha = [];
    private readonly List<byte> _bend = [];

    /// <summary>The links of a sheet of <paramref name="columns"/> by <paramref name="rows"/> particles, numbered along its rows.</summary>
    [SuppressMessage("Maintainability", "CA1502", Justification = "F2: a list of cases is data.")]
    public static LinkSet Cloth(int columns, int rows, double[] x, SoftSpec spec)
    {
        var links = new LinkBuilder(x);
        var stretch = spec.StretchCompliance;
        var bend = spec.BendCompliance;
        for (var j = 0; j < rows; j++)
        {
            for (var i = 0; i < columns; i++)
            {
                var right = i + 1 < columns;
                var below = j + 1 < rows;
                if (right)
                {
                    links.Stretch(At(i, j), At(i + 1, j), stretch);
                }

                if (below)
                {
                    links.Stretch(At(i, j), At(i, j + 1), stretch);
                }

                if (right && below)
                {
                    links.Stretch(At(i, j), At(i + 1, j + 1), stretch);
                }

                if (i > 0 && below)
                {
                    links.Stretch(At(i, j), At(i - 1, j + 1), stretch);
                }

                if (i + 2 < columns)
                {
                    links.Bend(At(i, j), At(i + 2, j), bend);
                }

                if (j + 2 < rows)
                {
                    links.Bend(At(i, j), At(i, j + 2), bend);
                }
            }
        }

        return links.Build();

        int At(int i, int j) => (j * columns) + i;
    }

    public void Stretch(int a, int b, double alpha) => Add(a, b, alpha, 0);

    public void Bend(int a, int b, double alpha) => Add(a, b, alpha, 1);

    public LinkSet Build() => new([.. _from], [.. _to], [.. _rest], [.. _alpha], [.. _bend]);

    private void Add(int a, int b, double alpha, byte bend)
    {
        var dx = x[a * 3] - x[b * 3];
        var dy = x[(a * 3) + 1] - x[(b * 3) + 1];
        var dz = x[(a * 3) + 2] - x[(b * 3) + 2];
        _from.Add(a);
        _to.Add(b);
        _rest.Add(Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz)));
        _alpha.Add(alpha);
        _bend.Add(bend);
    }
}
