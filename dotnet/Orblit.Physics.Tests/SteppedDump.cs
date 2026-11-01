using System.Globalization;

namespace Orblit.Physics.Tests;

/// <summary>
/// Compares a stepped run with the dump pinned from the original solver on macOS. On macOS every word is the
/// same bit for bit. Another system fuses a multiply and an add, or widens a vector, in other places, so a word
/// there is off in its last bits and a body that has been pushed about for seconds carries that on. There each
/// word must agree with the pinned one to <see cref="Allowed"/>.
/// </summary>
internal static class SteppedDump
{
    private const int FloatDigits = 8;
    private const int DoubleDigits = 16;
    private const int Shown = 8;

    /// <summary>
    /// How far a value may be from <paramref name="pinned"/>: the rounding of the figure pinned on macOS, and a
    /// thousandth of its size, or of one for a small value, on another system.
    /// </summary>
    public static double Allowed(double pinned) =>
        OperatingSystem.IsMacOS() ? 1e-9 : 1e-3 * Math.Max(1, Math.Abs(pinned));

    public static void AssertSame(IEnumerable<string> pinned, IEnumerable<string> ran)
    {
        var want = pinned.ToArray();
        var got = ran.ToArray();
        if (OperatingSystem.IsMacOS())
        {
            Assert.Equal(want, got);
            return;
        }

        Assert.Equal(want.Length, got.Length);
        var far = want.Zip(got).SelectMany((pair, line) => FarWords(line, pair.First, pair.Second)).ToList();
        Assert.True(far.Count == 0, $"{far.Count} words are off by more than allowed:\n{string.Join('\n', far.Take(Shown))}");
    }

    private static IEnumerable<string> FarWords(int line, string pinned, string ran)
    {
        var want = pinned.Split(' ');
        var got = ran.Split(' ');
        if (want.Length != got.Length)
        {
            return [$"line {line} has {got.Length} words, not {want.Length}: {ran}"];
        }

        return want.Zip(got)
            .Select((pair, word) => (Word: word, pair.First, pair.Second))
            .Where(one => one.First != one.Second && !Near(one.First, one.Second))
            .Select(one => $"line {line} word {one.Word}: pinned {one.First}, ran {one.Second}");
    }

    // A word that differs and is not the bits of a float is a count or a name, and those must be the same.
    private static bool Near(string pinned, string ran) =>
        pinned.Length == ran.Length
        && ValueOf(pinned) is { } want
        && ValueOf(ran) is { } got
        && Math.Abs(want - got) <= Allowed(want);

    private static double? ValueOf(string word)
    {
        var parsed = ulong.TryParse(word, NumberStyles.AllowHexSpecifier, CultureInfo.InvariantCulture, out var bits);
        return !parsed ? null : word.Length switch
        {
            FloatDigits => BitConverter.UInt32BitsToSingle((uint)bits),
            DoubleDigits => BitConverter.UInt64BitsToDouble(bits),
            _ => null,
        };
    }
}
