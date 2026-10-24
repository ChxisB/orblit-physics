using Orblit.Scene;
using Orblit.VectorMath;

namespace Orblit.Physics.Scene.Tests;

/// <summary>The scenes the tests share: a floor, a crate, and a loop that steps them.</summary>
internal static class SceneKit
{
    /// <summary>
    /// How far a resting body can be from touching exactly: the solver lets it in a quarter of its
    /// slop, five millimetres, so a contact is not lost and found again every step, and a little more
    /// for the bounce still settling.
    /// </summary>
    public const double Resting = 0.03;

    /// <summary>Ground through the origin, facing up.</summary>
    public static SceneEntity Floor(double height = 0) => new(
        "floor",
        "Floor",
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(new Vector3(0, height, 0)),
            [SceneComponents.Body] = new BodyComponent(BodyShape.Plane, motion: BodyMotion.Fixed),
        });

    /// <summary>A one-metre crate, dropped from <paramref name="at"/> unless it is told otherwise.</summary>
    // F2: six fields of one entity, called by name.
    public static SceneEntity Crate(
        string id = "crate",
        string? parent = null,
        Vector3? at = null,
        Vector3? rotation = null,
        Vector3? scale = null,
        BodyComponent? body = null) => new(
        id,
        id,
        parent,
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Transform] = new TransformComponent(at ?? new Vector3(0, 3, 0), rotation, scale),
            [SceneComponents.Body] = body ?? new BodyComponent(),
        });

    /// <summary>An entity with a body, at <paramref name="at"/> in the middle of the world unless told otherwise.</summary>
    public static SceneEntity Entity(string id, BodyComponent body, Vector3? at = null, Vector3? scale = null) => new(
        id,
        id,
        components: new Dictionary<string, SceneComponent>
        {
            [SceneComponents.Body] = body,
            [SceneComponents.Transform] = new TransformComponent(at, scale: scale),
        });

    public static SceneDocument SceneOf(params SceneEntity[] entities) => new("Scene", entities: entities);

    public static TransformComponent TransformOf(SceneDocument document, string id) =>
        (TransformComponent)document[id]![SceneComponents.Transform]!;

    /// <summary>
    /// Runs <paramref name="seconds"/> of simulation a sixtieth at a time, as a frame loop would, and
    /// answers with every diff it made. <paramref name="each"/> is called after every frame.
    /// </summary>
    public static List<SceneDiff> Run(ScenePhysics scene, double seconds, Action? each = null)
    {
        var diffs = new List<SceneDiff>();
        for (var i = 0; i < (int)Math.Round(seconds * 60, MidpointRounding.AwayFromZero); i++)
        {
            diffs.Add(scene.Advance(1.0 / 60));
            each?.Invoke();
        }

        return diffs;
    }

    /// <summary>A tolerance check that says what was measured, which xunit's own does not.</summary>
    public static void Near(double expected, double actual, double tolerance, string why) =>
        Assert.True(Math.Abs(expected - actual) <= tolerance, $"{why}: {actual} is not within {tolerance} of {expected}");
}
