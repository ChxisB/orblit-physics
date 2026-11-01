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
    private const double Resting = 1e-3;
    private const double Scattered = 5e-2;
    private const int HeightWord = 5;
    private const double Ceiling = 5;

    /// <summary>
    /// How far a value may be from <paramref name="pinned"/>: the rounding of the figure pinned on macOS, and a
    /// thousandth of its size, or of one for a small value, on another system.
    /// </summary>
    public static double Allowed(double pinned) =>
        OperatingSystem.IsMacOS() ? 1e-9 : Within(pinned, Resting);

    public static void AssertSame(IEnumerable<string> pinned, IEnumerable<string> ran) =>
        AssertEach(pinned, ran, pair => FarWords(pair.First, pair.Second, Resting));

    /// <summary>
    /// For a scene that throws bodies about, which no other system keeps in step with the pinned one. Off
    /// macOS the first mark is compared to a twentieth, taken as a fraction as <see cref="Allowed"/> is. Every
    /// later mark need only hold whole bodies above the ground and below a ceiling.
    /// </summary>
    public static void AssertScatteredSame(IEnumerable<string> pinned, IEnumerable<string> ran)
    {
        var want = pinned.ToArray();
        var first = StepOf(want[0]);
        AssertEach(want, ran, pair => StepOf(pair.First) == first
            ? FarWords(pair.First, pair.Second, Scattered)
            : NotWhole(pair.Second));
    }

    private static void AssertEach(
        IEnumerable<string> pinned,
        IEnumerable<string> ran,
        Func<(string First, string Second), IEnumerable<string>> check)
    {
        var want = pinned.ToArray();
        var got = ran.ToArray();
        if (OperatingSystem.IsMacOS())
        {
            Assert.Equal(want, got);
            return;
        }

        Assert.Equal(want.Length, got.Length);
        var far = want.Zip(got)
            .SelectMany((pair, line) => check(pair).Select(one => $"line {line} {one}"))
            .ToList();
        Assert.True(far.Count == 0, $"{far.Count} words are off by more than allowed:\n{string.Join('\n', far.Take(Shown))}");
    }

    private static IEnumerable<string> FarWords(string pinned, string ran, double fraction)
    {
        var want = pinned.Split(' ');
        var got = ran.Split(' ');
        if (want.Length != got.Length)
        {
            return [$"has {got.Length} words, not {want.Length}: {ran}"];
        }

        return want.Zip(got)
            .Select((pair, word) => (Word: word, pair.First, pair.Second))
            .Where(one => one.First != one.Second && !Near(one.First, one.Second, fraction))
            .Select(one => $"word {one.Word}: pinned {one.First}, ran {one.Second}");
    }

    private static IEnumerable<string> NotWhole(string ran)
    {
        var words = ran.Split(' ');
        var bad = words.Select((word, at) => (Word: at, Value: ValueOf(word)))
            .Where(one => one.Value is { } value && !double.IsFinite(value));
        foreach (var one in bad)
        {
            yield return $"word {one.Word} is not a number: {words[one.Word]}";
        }

        var height = words.Length > HeightWord ? ValueOf(words[HeightWord]) : null;
        if (height is not { } above || above <= 0 || above >= Ceiling)
        {
            yield return $"height is not above the ground and below {Ceiling}: {ran}";
        }
    }

    private static int StepOf(string line) => int.Parse(line.Split(' ')[1], CultureInfo.InvariantCulture);

    private static double Within(double pinned, double fraction) => fraction * Math.Max(1, Math.Abs(pinned));

    // A word that differs and is not the bits of a float is a count or a name, and those must be the same.
    private static bool Near(string pinned, string ran, double fraction) =>
        pinned.Length == ran.Length
        && ValueOf(pinned) is { } want
        && ValueOf(ran) is { } got
        && Math.Abs(want - got) <= Within(want, fraction);

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
