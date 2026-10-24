using System.Globalization;
using Orblit.VectorMath;

namespace Orblit.Physics.Soft.Tests;

/// <summary>
/// A square sheet hung from one corner that falls onto a floor, written as the
/// raw bits of every particle at three steps. The same sheet was run by an
/// independent build of the original solver, so a line here can be compared
/// with one there word for word.
/// </summary>
internal static class SteppedClothScene
{
    private const double Dt = 1.0 / 60;

    public static IReadOnlyList<string> Run()
    {
        var cloth = SoftBody.Cloth(
            8,
            8,
            new Vector3(0, 0.5, 0),
            new Vector3(0.15, 0, 0),
            new Vector3(0, 0, 0.15),
            surface: new SoftBalloonTests.Floor());
        cloth.Pin(0);
        List<string> lines = [];
        for (var step = 1; step <= 180; step++)
        {
            cloth.Step(Dt);
            if (step is 30 or 90 or 180)
            {
                lines.AddRange(Enumerable.Range(0, cloth.Count).Select(i => Line(step, i, cloth)));
            }
        }

        return lines;
    }

    private static string Line(int step, int particle, SoftBody cloth)
    {
        var p = cloth.PositionOf(particle);
        var v = cloth.VelocityOf(particle);
        return $"cloth {step} {particle} P {Bits(p.X, p.Y, p.Z)} V {Bits(v.X, v.Y, v.Z)}";
    }

    private static string Bits(params double[] words) =>
        string.Join(' ', words.Select(w => BitConverter.DoubleToUInt64Bits(w).ToString("x16", CultureInfo.InvariantCulture)));
}
