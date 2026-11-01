using System.Globalization;

namespace Orblit.Physics.Tests;

/// <summary>
/// Three worlds run for a fixed number of sixtieths, written as the raw bits of
/// every body at a few steps. The same worlds were run by an independent build
/// of the original solver, so a line here can be compared with one there word
/// for word.
/// </summary>
internal static class SteppedScenes
{
    private const double Dt = 1.0 / 60;

    public static IReadOnlyList<string> Wall()
    {
        using var physics = new Physics();
        physics.Add(1, Shape.Plane(0.0, 1.0, 0.0), PhysicsMotion.Fixed);
        List<ulong> ids = [];
        for (var row = 0; row < 5; row++)
        {
            for (var column = 0; column < 4; column++)
            {
                var id = (ulong)(10 + (row * 4) + column);
                ids.Add(id);
                var x = ((column - 1.5) * 0.62) + (row % 2 == 1 ? 0.31 : 0.0);
                physics.Add(id, Shape.Box(0.3, 0.2, 0.2), at: [x, 0.2 + (row * 0.4), 0.0], mass: 2.0, friction: 0.6);
            }
        }

        ids.Add(100);
        physics.Add(100, Shape.Sphere(0.35), at: [0.2, 0.9, 6.0], mass: 8.0);
        physics.Push(100, [0.0, 0.0, -80.0]);
        return Run("wall", physics, ids, 240, [30, 60, 120, 240], null);
    }

    public static IReadOnlyList<string> Character()
    {
        using var physics = new Physics();
        physics.Add(1, Shape.Plane(0.0, 1.0, 0.0), PhysicsMotion.Fixed);
        physics.Add(2, Shape.Box(1.0, 0.1, 1.0), PhysicsMotion.Fixed, at: [0.0, 0.1, -3.0]);
        physics.Add(3, Shape.Box(0.25, 0.25, 0.25), at: [0.25, 0.25, -6.5], mass: 1.5);
        physics.AddCharacter(10, at: [0.0, 0.91, 0.0]);
        var lines = Run("character", physics, [10, 3], 300, [30, 60, 90, 150, 210, 300], () =>
        {
            var left = physics.FootingOf(10)!.Velocity;
            physics.Drive(10, [0.1, left[1] - (9.81 / 60), -2.2]);
        }).ToList();
        var footing = physics.FootingOf(10)!;
        lines.Add(string.Join(
            ' ',
            "character footing",
            footing.Ground.ToString(CultureInfo.InvariantCulture),
            footing.Grounded ? "true" : "false",
            footing.Wall ? "true" : "false",
            footing.Ceiling ? "true" : "false",
            Bits(footing.Velocity),
            Bits(footing.Normal)));
        return lines;
    }

    public static IReadOnlyList<string> Chain()
    {
        using var physics = new Physics();
        physics.Add(20, Shape.Box(0.2, 0.1, 0.2), PhysicsMotion.Fixed, at: [0.0, 4.0, 0.0]);
        List<ulong> ids = [];
        for (var link = 0; link < 5; link++)
        {
            var id = (ulong)(21 + link);
            ids.Add(id);
            physics.Add(id, Shape.Box(0.05, 0.25, 0.05), at: [0.0, 3.75 - (link * 0.5), 0.0], mass: 1.0);
        }

        physics.Add(40, Shape.Box(0.3, 0.05, 0.3), at: [0.0, 1.45, 0.0], mass: 2.0);
        ids.Add(40);
        for (var link = 0; link < 4; link++)
        {
            physics.Join((ulong)(30 + link), Joint.Point(), (ulong)(20 + link), [0.0, 4.0 - (link * 0.5), 0.0], (ulong)(21 + link));
        }

        physics.Join(34, Joint.Hinge(new JointLimit(-0.8, 0.8)), 24, [0.0, 2.0, 0.0], 25);
        physics.Join(35, Joint.Fixed(), 25, [0.0, 1.5, 0.0], 40);
        physics.Push(25, [3.0, 0.0, 2.0]);
        return Run("chain", physics, ids, 360, [30, 60, 180, 360], null);
    }

    private static List<string> Run(string scene, Physics physics, List<ulong> ids, int steps, int[] marks, Action? before)
    {
        List<string> lines = [];
        for (var step = 1; step <= steps; step++)
        {
            before?.Invoke();
            physics.Step(Dt);
            if (Array.IndexOf(marks, step) >= 0)
            {
                lines.AddRange(ids.Select(id => $"{scene} {step} {id} T {Bits(physics.TransformOf(id)!)} V {Bits(physics.VelocityOf(id)!)}"));
            }
        }

        return lines;
    }

    private static string Bits(IEnumerable<float> words) =>
        string.Join(' ', words.Select(w => BitConverter.SingleToUInt32Bits(w).ToString("x8", CultureInfo.InvariantCulture)));

    private static string Bits(IEnumerable<double> words) =>
        string.Join(' ', words.Select(w => BitConverter.DoubleToUInt64Bits(w).ToString("x16", CultureInfo.InvariantCulture)));
}
