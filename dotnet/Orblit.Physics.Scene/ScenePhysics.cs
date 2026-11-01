using Orblit.Physics.Vehicle;
using Orblit.Scene;

namespace Orblit.Physics.Scene;

// The namespace of the same name would win over an alias written outside this one.
using Vehicle = Orblit.Physics.Vehicle.Vehicle;

/// <summary>
/// The bodies in a scene document, simulated.
/// </summary>
/// <remarks>
/// <para>
/// Built from a document, it adds a body to a <see cref="Physics"/> world for every entity with a
/// <see cref="BodyComponent"/>, where the document puts that entity in the world. <see cref="Advance"/>
/// steps the world and answers with a <see cref="SceneDiff"/> that moves each entity to where its body
/// ended up: the same kind of diff an edit makes, so whatever already draws a document draws the
/// simulation with the <c>apply</c> it already has.
/// </para>
/// <para>
/// Edits go the other way through <see cref="Apply"/>: moving an entity, changing its body or deleting
/// it changes the world to match. An edited body is rebuilt where the document now says it is, which
/// forgets how it was moving, a teleport, which is what dragging a crate in the editor is. The diffs
/// <see cref="Advance"/> returns are already in <see cref="Document"/>, and handing them back to
/// <see cref="Apply"/> would rebuild every body that moved and stop it dead.
/// </para>
/// <para>
/// The document names entities with strings and the world names bodies with numbers, so this keeps the
/// one-to-one between them: <see cref="BodyOf"/> and <see cref="EntityOf"/>. An entity keeps its number
/// for as long as it has a body, and no number is ever given to a second entity, so an event that names
/// a body never names the wrong thing.
/// </para>
/// <para>
/// An entity with a <see cref="JointComponent"/> joins two bodies: the nearest body at or above it, to
/// the nearest body above that or to the world, as <see cref="JointComponent.EndsOf"/> says. It is made
/// where the document puts the joint entity, holding the bodies as they stand, and remade whenever an
/// edit rebuilds either body or touches the joint, remade as the bodies stand then, so a door edited
/// half open reads nought half open. <see cref="JointOf"/> and <see cref="EntityOfJoint"/> are the
/// one-to-one between joint entities and the world's joint numbers, which are counted apart from the
/// bodies'. A joint that breaks stays broken, and in <see cref="Broken"/>, until its own entity is
/// edited.
/// </para>
/// <para>
/// A body with <see cref="BodyComponent.Trigger"/> is a place: it reports what enters and leaves it, as
/// <see cref="PhysicsEventKind.Entered"/> and <see cref="PhysicsEventKind.Exited"/> with the trigger's
/// own number first. An entity with a <see cref="ZoneComponent"/> and a body is a trigger whose region
/// changes how the free bodies in it move. <see cref="BodyComponent.Surface"/> is a belt: what stands on
/// the body is carried.
/// </para>
/// <para>
/// <see cref="BodyComponent.Locks"/>, <see cref="BodyComponent.GravityScale"/>, the two speed caps, the
/// centre of mass and the inertia are how a free body moves, and are sent to the world with the body.
/// <see cref="Ignore"/> makes two entities pass through each other. It is a fact about the pair, so it is
/// kept here and made again when an edit rebuilds either body.
/// </para>
/// <para>
/// <see cref="Snapshot"/> copies the simulation and <see cref="Restore"/> goes back to it, for an undo or
/// a rollback that resimulates from a known frame. What the world holds of the last step, contacts and
/// counts, is <see cref="Physics.Contacts"/> and <see cref="Physics.Stats"/> on <see cref="Physics"/>,
/// and <see cref="EntityOf"/> says which entity a body is.
/// </para>
/// <para>
/// The world steps by a fixed <see cref="Step"/>, so a body only moves when a step ends. A render that
/// runs faster shows it standing still for some frames and jumping on others. With
/// <see cref="Smooth"/>, <see cref="Document"/> shows each body a step behind the world, at the blend of
/// its last two steps, and so moves on every <see cref="Advance"/>. An edit is a teleport and does not
/// blend. <see cref="ResetSmoothing"/> says the same of a body moved through <see cref="Physics"/>, and
/// <see cref="StopSmoothing"/> shows a subtree as the world has it.
/// </para>
/// <para>
/// <see cref="Physics"/> is the world itself, for everything a document cannot say: pushing, driving and
/// casting. What touched what is <see cref="Events"/>, gathered over every step a frame took. Numbers
/// from one upwards are this class's to hand out; a body added to the world directly wants a negative
/// one, and is simulated but never written back.
/// </para>
/// </remarks>
public sealed partial class ScenePhysics : IDisposable
{
    /// <summary>Metres a second squared, for the sag a wheel's spring has at rest.</summary>
    private readonly double _gravity;

    private SceneDocument _document;

    private readonly OrderedDictionary<string, ulong> _bodyOf = [];
    private readonly Dictionary<ulong, string> _entityOf = [];

    /// <summary>
    /// Where each body was when the document was last told. Without <see cref="Smooth"/> it is exactly as
    /// the world reported it. A body is written back when the world disagrees with this rather than with
    /// the document, so the rounding in a trip from a pose to three angles and back is never mistaken
    /// for movement.
    /// </summary>
    private readonly OrderedDictionary<ulong, float[]> _known = [];

    /// <summary>
    /// With <see cref="Smooth"/>, where the world had each body after the last two steps. A body is shown
    /// at <c>_latest</c> when the two are the same, so one at rest is never moved by the rounding in a
    /// blend.
    /// </summary>
    private readonly OrderedDictionary<ulong, float[]> _previous = [];
    private readonly OrderedDictionary<ulong, float[]> _latest = [];

    /// <summary>
    /// The entities <see cref="StopSmoothing"/> has switched off. Everything under one is shown as the
    /// world has it.
    /// </summary>
    private readonly HashSet<string> _sharp = [];

    private ulong _next = 1;

    /// <summary>
    /// The world's hull for each distinct set of corners at each size, so a hundred crates cut from one
    /// mesh share one. The key is the corners as the world is given them, scaled and joined.
    /// </summary>
    private readonly Dictionary<string, ulong> _hullIdOf = [];

    /// <summary>The key of the hull each body is made of, for the bodies that have one.</summary>
    private readonly Dictionary<ulong, string> _hullKeyOf = [];
    private readonly HashSet<ulong> _compoundBodies = [];
    private readonly HashSet<ulong> _meshBodies = [];
    private readonly Dictionary<ulong, HashSet<string>> _partHullKeys = [];

    /// <summary>
    /// Counted apart from the bodies', and never handed out twice, like them. A hull laid directly through
    /// <see cref="Physics"/> wants a negative number.
    /// </summary>
    private ulong _nextHull = 1;

    private readonly OrderedDictionary<string, ulong> _jointOf = [];
    private readonly Dictionary<ulong, string> _entityOfJoint = [];

    /// <summary>
    /// The two bodies each joint in the world holds, so rebuilding a body can find the joints the world
    /// dropped with it.
    /// </summary>
    private readonly OrderedDictionary<string, (ulong A, ulong B)> _held = [];
    private readonly HashSet<string> _broken = [];
    private ulong _nextJoint = 1;

    /// <summary>
    /// The entity pairs told to pass through each other, each with the smaller id first so a pair is one
    /// entry whichever way it was named.
    /// </summary>
    private readonly HashSet<(string A, string B)> _ignored = [];

    /// <summary>
    /// The characters that are crouching. Their body's shape is the crouched one, and their feet are
    /// where they were.
    /// </summary>
    private readonly HashSet<ulong> _crouching = [];

    /// <summary>
    /// What each character is asked to do, by entity. Not part of a snapshot: a walk is the game's, with
    /// a playhead the world cannot rewind.
    /// </summary>
    private readonly OrderedDictionary<string, ICharacterWalk> _walks = [];

    /// <summary>
    /// The cars, by their chassis entity, and what each is asked to do. Like a walk, the driver's input is
    /// the game's and not part of a snapshot, and the tyres start unbent after a restore.
    /// </summary>
    private readonly OrderedDictionary<string, Car> _cars = [];
    private readonly Dictionary<string, VehicleInput> _pilots = [];

    private double _owed;
    private IReadOnlyList<PhysicsEvent> _events = [];

    // F2: five parameters, the last four optional and called by name.
    public ScenePhysics(
        SceneDocument document,
        PhysicsSettings? settings = null,
        double step = 1.0 / 60,
        int maxSteps = 8,
        bool smooth = false)
    {
        Physics = new Physics(settings);
        Step = step;
        MaxSteps = maxSteps;
        Smooth = smooth;
        _gravity = Math.Abs(settings?.Gravity?[1] ?? -9.81);
        _document = document;
        Rebuild([.. document.Entities.Select(entity => entity.Id)]);
    }

    /// <summary>The world the bodies are in.</summary>
    public Physics Physics { get; }

    /// <summary>
    /// Seconds per step. The world always steps by exactly this much, however unevenly
    /// <see cref="Advance"/> is called, because a solver handed a wobbling delta produces a wobbling
    /// simulation.
    /// </summary>
    public double Step { get; }

    /// <summary>
    /// The most steps one <see cref="Advance"/> takes. A late frame, a breakpoint or a window being
    /// dragged, would otherwise owe the world a second of steps, take longer than a frame to pay them,
    /// and owe more the next time. Past this the simulation slows down rather than seizing up.
    /// </summary>
    public int MaxSteps { get; }

    /// <summary>
    /// Whether <see cref="Document"/> shows each body blended between its last two steps, rather than
    /// where the last step left it.
    /// </summary>
    /// <remarks>
    /// A body is then shown one step behind the world, part of the way from where it was to where it is,
    /// by the time owed to the next step. It moves on every <see cref="Advance"/>, including one that took
    /// no step. <see cref="Physics"/> keeps the truth, so a cast, a query, an event or a contact is about
    /// where the body is now, which is up to one step ahead of what the document shows. Fixed for the life
    /// of the scene. A snapshot restores only into a scene that smooths as the one it came from did.
    /// </remarks>
    public bool Smooth { get; }

    /// <summary>The document as the simulation has left it.</summary>
    public SceneDocument Document => _document;

    /// <summary>
    /// Everything that happened in the steps the last <see cref="Advance"/> took, in the order it
    /// happened. Empty when that advance took none.
    /// </summary>
    /// <remarks>
    /// Read this rather than the world's own, which keeps only its last step. A slow frame that owed three
    /// steps would lose what happened in the first two, and a fast one that owed none would hear the last
    /// frame's again.
    /// </remarks>
    public IReadOnlyList<PhysicsEvent> Events => _events;

    /// <summary>The joint entities whose joints have broken, and are not holding until they are edited.</summary>
    public IReadOnlySet<string> Broken => new HashSet<string>(_broken);

    /// <summary>The body an entity is simulated as, or null when it has none.</summary>
    public ulong? BodyOf(string entity) => _bodyOf.TryGetValue(entity, out var body) ? body : null;

    /// <summary>
    /// The entity a body stands for, or null when it is not one of the document's, including a body whose
    /// entity has since lost it.
    /// </summary>
    public string? EntityOf(ulong body) => _entityOf.GetValueOrDefault(body);

    /// <summary>
    /// The joint an entity's <see cref="JointComponent"/> is, or null when it has none. An entity keeps its
    /// number for as long as it has the component, whether or not the joint is holding anything:
    /// <see cref="Physics.JointStateOf"/> is null for one that has broken or has no body to hold.
    /// </summary>
    public ulong? JointOf(string entity) => _jointOf.TryGetValue(entity, out var joint) ? joint : null;

    /// <summary>
    /// The entity joint <paramref name="number"/> was made for, or null when it is not one of the
    /// document's. Kept after the joint breaks, so the event that says it broke can be read.
    /// </summary>
    public string? EntityOfJoint(ulong number) => _entityOfJoint.GetValueOrDefault(number);

    /// <summary>
    /// Makes entities <paramref name="a"/> and <paramref name="b"/> pass through each other, whatever their
    /// layers say: no contact and no touch event. Returns false, changing nothing, for an entity that has
    /// no body and for an entity against itself.
    /// </summary>
    /// <remarks>
    /// It holds through the edits that rebuild either body, and ends when either entity loses its body or
    /// goes. It replaces any rule the world had for the pair. A character reads it too, and walks through
    /// the other body. <see cref="Physics"/> rules for one pair are the way to say anything else about it.
    /// </remarks>
    public bool Ignore(string a, string b)
    {
        if (!_bodyOf.TryGetValue(a, out var first) || !_bodyOf.TryGetValue(b, out var second) || a == b)
        {
            return false;
        }

        _ignored.Add(PairOf(a, b));
        return Physics.SetRule(first, second, new PhysicsRule(ignore: true));
    }

    /// <summary>Lets <paramref name="a"/> and <paramref name="b"/> touch again. False if they were not ignoring each other.</summary>
    public bool Unignore(string a, string b)
    {
        if (!_ignored.Remove(PairOf(a, b)))
        {
            return false;
        }

        return Physics.RemoveRule(_bodyOf[a], _bodyOf[b]);
    }

    /// <summary>
    /// Shows <paramref name="entity"/> and everything under it as the world has it, with no blend, until
    /// <see cref="StartSmoothing"/>. For a body that must not trail a step behind what it is attached to,
    /// or one that <see cref="Physics"/> places each frame. A body switches at once, so one on screen
    /// jumps by what it had trailed.
    /// </summary>
    /// <remarks>
    /// Returns false, changing nothing, for an entity the document does not have. It does nothing to a
    /// scene that is not <see cref="Smooth"/>.
    /// </remarks>
    public bool StopSmoothing(string entity)
    {
        if (!_document.Contains(entity))
        {
            return false;
        }

        _sharp.Add(entity);
        Collapse(BodiesUnder(entity));
        return true;
    }

    /// <summary>
    /// Blends <paramref name="entity"/> again, after <see cref="StopSmoothing"/> for the same entity. The
    /// blend starts from where the body stands, so it does not jump. Returns false if the entity was not
    /// stopped. An entity under a stopped one stays stopped until that one starts.
    /// </summary>
    public bool StartSmoothing(string entity) => _sharp.Remove(entity);

    /// <summary>
    /// Forgets how <paramref name="entity"/> and everything under it were moving, so the next
    /// <see cref="Advance"/> shows them where the world has them and the blend starts again from there.
    /// </summary>
    /// <remarks>
    /// Call it after moving a body through <see cref="Physics"/>, such as with
    /// <see cref="Physics.Place"/>. An edit through <see cref="Apply"/> does it by itself. Without it, a
    /// body that was moved slides from where it was to where it went, across the next step. Returns false,
    /// changing nothing, for an entity the document does not have. It does nothing to a scene that is not
    /// <see cref="Smooth"/>.
    /// </remarks>
    public bool ResetSmoothing(string entity)
    {
        if (!_document.Contains(entity))
        {
            return false;
        }

        Collapse(BodiesUnder(entity));
        return true;
    }

    /// <summary>
    /// Moves the simulation on by <paramref name="seconds"/> and says what moved.
    /// </summary>
    /// <remarks>
    /// Takes as many whole <see cref="Step"/>s as are owed, none when less than a step has built up since
    /// the last call. Answers with a diff that takes every entity whose body moved, and every body under
    /// one that did, to where the world now has it. Empty when nothing moved. The diff is already in
    /// <see cref="Document"/>. With <see cref="Smooth"/> the bodies are shown blended, so a call that took
    /// no step still moves them.
    /// </remarks>
    public SceneDiff Advance(double seconds)
    {
        if (seconds > 0)
        {
            _owed += seconds;
        }

        var events = new List<PhysicsEvent>();
        var taken = TakeSteps(events);
        _events = events;
        NoteBroken(events);
        if (taken == 0 && !Smooth)
        {
            return SceneDiff.None;
        }

        var placed = WriteBack();
        var turned = taken == 0 ? SceneDiff.None : Seat();
        return turned.IsEmpty ? placed : new SceneDiff([.. placed.Operations, .. turned.Operations]);
    }

    /// <summary>
    /// Sets what <paramref name="entity"/> is asked to do, every step, or stops asking with a null
    /// <paramref name="walk"/>.
    /// </summary>
    /// <remarks>
    /// Only a character has a walk, and an entity that is not one, or that loses its body, loses it.
    /// Asking replaces what was there. A character with none stands where it is and does not fall, as
    /// <see cref="Physics.AddCharacter"/> describes.
    /// </remarks>
    public void Walk(string entity, ICharacterWalk? walk)
    {
        if (walk is null)
        {
            _walks.Remove(entity);
            return;
        }

        _walks[entity] = walk;
    }

    /// <summary>
    /// The car an entity's vehicle component makes, or null when it has none.
    /// </summary>
    /// <remarks>
    /// A car needs a vehicle component on a free body, at the size of one, with a wheel component on at
    /// least one of its children and a spec the vehicle refuses nothing in: a document that breaks one of
    /// these keeps the data and the body is just a body. Read the engine and the wheels from it. The scene
    /// updates it once a step, so leave that to the scene.
    /// </remarks>
    public Vehicle? VehicleOf(string entity) => _cars.TryGetValue(entity, out var car) ? car.Vehicle : null;

    /// <summary>
    /// Sets what the driver of <paramref name="entity"/> asks of its car, every step, or lets go of the
    /// controls with a null <paramref name="input"/>. A gear the car lacks is refused.
    /// </summary>
    /// <remarks>
    /// The scene has no input of its own: whatever the game reads, a stick or a key or a script, it passes
    /// on here. Asking replaces what was there. An entity that is not a car, or that stops being one, loses
    /// it. A car steps by <see cref="Vehicle.MaxStep"/> at most, so a scene with one cannot
    /// <see cref="Step"/> longer.
    /// </remarks>
    public void Pilot(string entity, VehicleInput? input)
    {
        if (input is null)
        {
            _pilots.Remove(entity);
        }
        else
        {
            _pilots[entity] = input;
        }

        if (_cars.TryGetValue(entity, out var car))
        {
            car.Vehicle.SetInput(input ?? new VehicleInput());
        }
    }

    /// <summary>
    /// Makes <paramref name="entity"/> crouch, or stand again, and answers whether it now is as asked.
    /// </summary>
    /// <remarks>
    /// A crouch takes the body's <c>CrouchHeight</c> as the capsule's height, from tip to tip, with the
    /// feet where they were. Standing needs the room, and a character under a beam stays crouched and says
    /// so with false. False too for an entity that is not a character, or has no <c>CrouchHeight</c>.
    /// Rebuilding a character, as any edit to it does, stands it.
    /// </remarks>
    public bool Crouch(string entity, bool crouching)
    {
        if (!_bodyOf.TryGetValue(entity, out var number) || _document[entity]?[SceneComponents.Body] is not BodyComponent body)
        {
            return false;
        }

        if (!IsCharacter(body) || !(body.CrouchHeight > 0))
        {
            return false;
        }

        if (_crouching.Contains(number) == crouching)
        {
            return true;
        }

        var scale = ScaleOf(entity);
        var shape = crouching ? Capsule(body.Radius, body.CrouchHeight, scale) : ShapeOf(number, body, scale);
        if (!Physics.SetStance(number, shape))
        {
            return false;
        }

        if (crouching)
        {
            _crouching.Add(number);
        }
        else
        {
            _crouching.Remove(number);
        }

        return true;
    }

    /// <summary>Whether <paramref name="entity"/> is crouching.</summary>
    public bool IsCrouching(string entity) => _bodyOf.TryGetValue(entity, out var number) && _crouching.Contains(number);

    /// <summary>
    /// Tells the world about an edit to <see cref="Document"/>.
    /// </summary>
    /// <remarks>
    /// Every entity the edit touches, and everything under one, has its body rebuilt where the document now
    /// puts it, a moved parent carries the bodies under it, as it carries what they draw, or taken out of
    /// the world when it no longer has one. A rebuilt body has lost its velocity, whatever the edit was:
    /// editing a document mid-simulation is a teleport. Every joint on a rebuilt body, and every joint
    /// entity touched, is made again as things now stand; one that had broken is made again only when the
    /// edit is to its own entity.
    /// </remarks>
    public void Apply(SceneDiff diff)
    {
        if (diff.IsEmpty)
        {
            return;
        }

        // Names, order, visibility and settings move nothing a body cares about.
        var touched = Unique(diff.Operations.Select(IdTouchedBy).OfType<string>());
        var before = _document;
        _document = diff.ApplyTo(before);
        _sharp.RemoveWhere(id => !_document.Contains(id));

        // Editing a broken joint is asking for it back. Moving what is above it is not, or dragging the
        // door would mend the hinge it tore off.
        _broken.ExceptWith(touched);

        // Under a touched entity as it was, for what the edit took away, and as it is, for what the edit
        // brought.
        var affected = Unique(touched.SelectMany(id => before.SubtreeOf(id).Concat(_document.SubtreeOf(id)).Select(entity => entity.Id)));
        Rebuild(affected);
    }

    /// <summary>
    /// Copies the simulation as it stands: the world, and the document with the bodies where the last
    /// <see cref="Advance"/> left them.
    /// </summary>
    /// <remarks>
    /// Given the same edits and the same <see cref="Advance"/> calls, a simulation restored from it reaches
    /// the same document, on the same build. The copy is native memory, so it is let go of with
    /// <see cref="ScenePhysicsSnapshot.Dispose"/>.
    /// </remarks>
    public ScenePhysicsSnapshot Snapshot() => new(Physics.Snapshot(), _document)
    {
        Smooth = Smooth,
        BodyOf = new(_bodyOf),
        EntityOf = new(_entityOf),
        Known = CopyOf(_known),
        Previous = CopyOf(_previous),
        Latest = CopyOf(_latest),
        Sharp = [.. _sharp],
        Next = _next,
        HullIdOf = new(_hullIdOf),
        HullKeyOf = new(_hullKeyOf),
        NextHull = _nextHull,
        CompoundBodies = [.. _compoundBodies],
        MeshBodies = [.. _meshBodies],
        PartHullKeys = _partHullKeys.ToDictionary(entry => entry.Key, entry => new HashSet<string>(entry.Value)),
        JointOf = new(_jointOf),
        EntityOfJoint = new(_entityOfJoint),
        Held = new(_held),
        Broken = [.. _broken],
        NextJoint = _nextJoint,
        Ignored = [.. _ignored],
        Crouching = [.. _crouching],
        Owed = _owed,
        Events = [.. _events],
    };

    /// <summary>
    /// Goes back to <paramref name="snapshot"/>: the world, the document, which entity is which body and
    /// joint, what has broken and what is ignored, the time owed to the next step and the events of the
    /// last <see cref="Advance"/>. Everything done since is gone, edits included.
    /// </summary>
    /// <remarks>
    /// <para>
    /// Answers with the diff that takes <see cref="Document"/> as it stood to the one restored, so
    /// whatever draws the document can follow. The restored document is already in <see cref="Document"/>.
    /// </para>
    /// <para>
    /// The numbers handed out since are taken back with the rest, so an entity added after the snapshot is
    /// given the same number again. That is what makes a replay the same replay. A number kept from the
    /// run that was left names nothing, or something else, in the run that follows.
    /// </para>
    /// <para>
    /// A snapshot keeps to the <see cref="Step"/> it was taken at. One restored into a simulation that
    /// steps by another does not replay. It also keeps to <see cref="Smooth"/>: one restored into a scene
    /// that smooths differently throws an <see cref="ArgumentOutOfRangeException"/>, because it has no
    /// blend to go back to or no use for one.
    /// </para>
    /// </remarks>
    public SceneDiff Restore(ScenePhysicsSnapshot snapshot)
    {
        if (snapshot.Smooth != Smooth)
        {
            throw new ArgumentOutOfRangeException(
                nameof(snapshot),
                snapshot,
                snapshot.Smooth ? "was taken by a scene that smooths" : "was taken by a scene that does not smooth");
        }

        var before = _document;

        // Before anything changes, because a snapshot that has been disposed throws here, and nothing is
        // left half changed.
        Physics.Restore(snapshot.World);

        _document = snapshot.Document;
        RestoreBodies(snapshot);
        RestoreHulls(snapshot);
        RestoreJoints(snapshot);
        _owed = snapshot.Owed;
        _events = [.. snapshot.Events];
        _cars.Clear();
        Refit([.. _document.Entities.Select(entity => entity.Id)]);

        return SceneDiff.Between(before, _document);
    }

    public void Dispose() => Physics.Dispose();

    private int TakeSteps(List<PhysicsEvent> events)
    {
        var taken = 0;
        var sharp = Smooth ? SharpBodies() : [];
        while (_owed >= Step && taken < MaxSteps)
        {
            Drive();
            Physics.Step(Step);
            events.AddRange(Physics.Events);
            if (Smooth)
            {
                Carry(sharp);
            }

            _owed -= Step;
            taken++;
        }

        if (_owed >= Step)
        {
            _owed = 0;
        }

        return taken;
    }

    /// <summary>The world has already taken a broken joint out, and says which by its number.</summary>
    private void NoteBroken(List<PhysicsEvent> events)
    {
        foreach (var even in events.Where(one => one.Kind == PhysicsEventKind.Broke))
        {
            if (!_entityOfJoint.TryGetValue(even.A, out var id))
            {
                continue;
            }

            _broken.Add(id);
            _held.Remove(id);
        }
    }

    private static string? IdTouchedBy(SceneOp operation) => operation switch
    {
        AddEntity add => add.Id,
        RemoveEntity remove => remove.Id,
        SetComponent set => set.Id,
        SetField field => field.Id,
        Reparent reparent => reparent.Id,
        _ => null,
    };

    private void RestoreBodies(ScenePhysicsSnapshot snapshot)
    {
        Reload(_bodyOf, snapshot.BodyOf);
        Reload(_entityOf, snapshot.EntityOf);

        // Copied again, because a step writes into these in place and the snapshot has to stay as it was
        // for the next restore.
        Reload(_known, CopyOf(snapshot.Known));
        Reload(_previous, CopyOf(snapshot.Previous));
        Reload(_latest, CopyOf(snapshot.Latest));
        Reload(_sharp, snapshot.Sharp);
        _next = snapshot.Next;
        Reload(_crouching, snapshot.Crouching);
    }

    private void RestoreHulls(ScenePhysicsSnapshot snapshot)
    {
        Reload(_meshBodies, snapshot.MeshBodies);
        Reload(_compoundBodies, snapshot.CompoundBodies);
        Reload(_partHullKeys, snapshot.PartHullKeys.Select(entry => KeyValuePair.Create(entry.Key, new HashSet<string>(entry.Value))));
        Reload(_hullIdOf, snapshot.HullIdOf);
        Reload(_hullKeyOf, snapshot.HullKeyOf);
        _nextHull = snapshot.NextHull;
    }

    private void RestoreJoints(ScenePhysicsSnapshot snapshot)
    {
        Reload(_jointOf, snapshot.JointOf);
        Reload(_entityOfJoint, snapshot.EntityOfJoint);
        Reload(_held, snapshot.Held);
        Reload(_broken, snapshot.Broken);
        _nextJoint = snapshot.NextJoint;
        Reload(_ignored, snapshot.Ignored);
    }

    /// <summary>
    /// A copy of <paramref name="poses"/> that shares nothing with it, because a step writes into these in
    /// place.
    /// </summary>
    private static OrderedDictionary<ulong, float[]> CopyOf(OrderedDictionary<ulong, float[]> poses)
    {
        var copy = new OrderedDictionary<ulong, float[]>();
        foreach (var (body, pose) in poses)
        {
            copy[body] = [.. pose];
        }

        return copy;
    }

    private static void Reload<T>(ICollection<T> into, IEnumerable<T> from)
    {
        // A copy first, because the two may share a source that is cleared with the target.
        var items = from.ToList();
        into.Clear();
        foreach (var item in items)
        {
            into.Add(item);
        }
    }

    /// <summary>
    /// <paramref name="ids"/> in the order first met, each once. A set that numbers bodies and joints as it
    /// is walked is an ordered one.
    /// </summary>
    private static List<string> Unique(IEnumerable<string> ids)
    {
        var seen = new HashSet<string>();
        return [.. ids.Where(seen.Add)];
    }

    private static (string A, string B) PairOf(string a, string b) =>
        string.CompareOrdinal(a, b) <= 0 ? (a, b) : (b, a);
}
