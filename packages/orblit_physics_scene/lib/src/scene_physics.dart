import 'dart:math' as math;
import 'dart:typed_data';

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:vector_math/vector_math_64.dart';

import 'pose.dart';

/// A [ScenePhysics] at one instant, to go back to with [ScenePhysics.restore].
///
/// It holds the world's own [PhysicsSnapshot], so like that one it is native
/// memory and is let go of with [dispose]. Beside the world it keeps what this
/// class knows and the world does not: the document, which entity is which
/// body and joint, the numbers still to hand out, and what has broken or been
/// told to pass through. In a scene that smooths, it also keeps how each body
/// was moving in the blend.
final class ScenePhysicsSnapshot {
  ScenePhysicsSnapshot._(ScenePhysics from)
    : _world = from.physics.snapshot(),
      _smooth = from.smooth,
      _document = from._document,
      _bodyOf = Map.of(from._bodyOf),
      _entityOf = Map.of(from._entityOf),
      _known = _copyOf(from._known),
      _previous = _copyOf(from._previous),
      _latest = _copyOf(from._latest),
      _sharp = Set.of(from._sharp),
      _next = from._next,
      _hullIdOf = Map.of(from._hullIdOf),
      _hullKeyOf = Map.of(from._hullKeyOf),
      _nextHull = from._nextHull,
      _jointOf = Map.of(from._jointOf),
      _entityOfJoint = Map.of(from._entityOfJoint),
      _held = Map.of(from._held),
      _broken = Set.of(from._broken),
      _nextJoint = from._nextJoint,
      _ignored = Set.of(from._ignored),
      _owed = from._owed,
      _events = List.of(from._events);

  final PhysicsSnapshot _world;
  final bool _smooth;
  final SceneDocument _document;
  final Map<String, int> _bodyOf;
  final Map<int, String> _entityOf;
  final Map<int, Float32List> _known;
  final Map<int, Float32List> _previous;
  final Map<int, Float32List> _latest;
  final Set<String> _sharp;
  final int _next;
  final Map<String, int> _hullIdOf;
  final Map<int, String> _hullKeyOf;
  final int _nextHull;
  final Map<String, int> _jointOf;
  final Map<int, String> _entityOfJoint;
  final Map<String, ({int a, int b})> _held;
  final Set<String> _broken;
  final int _nextJoint;
  final Set<({String a, String b})> _ignored;
  final double _owed;
  final List<PhysicsEvent> _events;

  bool get disposed => _world.disposed;

  void dispose() => _world.dispose();
}

/// A copy of [poses] that shares nothing with it, because a step writes into
/// these in place.
Map<int, Float32List> _copyOf(Map<int, Float32List> poses) => {
  for (final MapEntry(:key, :value) in poses.entries)
    key: Float32List.fromList(value),
};

/// The bodies in a scene document, simulated.
///
/// Built from a document, it adds a body to a [Physics] world for every entity
/// with a [BodyComponent], where the document puts that entity in the world.
/// [advance] steps the world and answers with a [SceneDiff] that moves each
/// entity to where its body ended up — the same kind of diff an edit makes, so
/// whatever already draws a document draws the simulation with the `apply` it
/// already has.
///
/// Edits go the other way through [apply]: moving an entity, changing its body
/// or deleting it changes the world to match. An edited body is rebuilt where
/// the document now says it is, which forgets how it was moving — a teleport,
/// which is what dragging a crate in the editor is. The diffs [advance] returns
/// are already in [document], and handing them back to [apply] would rebuild
/// every body that moved and stop it dead.
///
/// The document names entities with strings and the world names bodies with
/// numbers, so this keeps the one-to-one between them: [bodyOf] and
/// [entityOf]. An entity keeps its number for as long as it has a body, and no
/// number is ever given to a second entity, so an event that names a body
/// never names the wrong thing.
///
/// An entity with a [JointComponent] joins two bodies: the nearest body at or
/// above it, to the nearest body above that or to the world, as
/// [JointComponent.endsOf] says. It is made where the document puts the joint
/// entity, holding the bodies as they stand, and remade whenever an edit
/// rebuilds either body or touches the joint — remade as the bodies stand
/// then, so a door edited half open reads nought half open. [jointOf] and
/// [entityOfJoint] are the one-to-one between joint entities and the world's
/// joint numbers, which are counted apart from the bodies'. A joint that
/// breaks stays broken, and in [broken], until its own entity is edited.
///
/// A body with [BodyComponent.trigger] is a place: it reports what enters and
/// leaves it, as [PhysicsEventKind.entered] and [PhysicsEventKind.exited] with
/// the trigger's own number first. An entity with a [ZoneComponent] and a body
/// is a trigger whose region changes how the free bodies in it move.
/// [BodyComponent.surface] is a belt: what stands on the body is carried.
///
/// [BodyComponent.locks], [BodyComponent.gravityScale], the two speed caps, the
/// centre of mass and the inertia are how a free body moves, and are sent to
/// the world with the body. [ignore] makes two entities pass through each
/// other. It is a fact about the pair, so it is kept here and made again when
/// an edit rebuilds either body.
///
/// [snapshot] copies the simulation and [restore] goes back to it, for an undo
/// or a rollback that resimulates from a known frame. What the world holds of
/// the last step, contacts and counts, is [Physics.contacts] and
/// [Physics.stats] on [physics], and [entityOf] says which entity a body is.
///
/// The world steps by a fixed [step], so a body only moves when a step ends.
/// A render that runs faster shows it standing still for some frames and
/// jumping on others. With [smooth], [document] shows each body a step behind
/// the world, at the blend of its last two steps, and so moves on every
/// [advance]. An edit is a teleport and does not blend. [resetSmoothing] says
/// the same of a body moved through [physics], and [stopSmoothing] shows a
/// subtree as the world has it.
///
/// [physics] is the world itself, for everything a document cannot say:
/// pushing, driving and casting. What touched what is [events], gathered over
/// every step a frame took. Numbers from one upwards are this class's to hand
/// out; a body added to the world directly wants a negative one, and is
/// simulated but never written back.
class ScenePhysics {
  ScenePhysics(
    SceneDocument document, {
    PhysicsSettings settings = const PhysicsSettings(),
    this.step = 1 / 60,
    this.maxSteps = 8,
    this.smooth = false,
  }) : physics = Physics(settings: settings),
       _document = document {
    _rebuild([for (final entity in document.entities) entity.id]);
  }

  /// The world the bodies are in.
  final Physics physics;

  /// Seconds per step. The world always steps by exactly this much, however
  /// unevenly [advance] is called, because a solver handed a wobbling delta
  /// produces a wobbling simulation.
  final double step;

  /// The most steps one [advance] takes. A late frame — a breakpoint, a window
  /// being dragged — would otherwise owe the world a second of steps, take
  /// longer than a frame to pay them, and owe more the next time. Past this the
  /// simulation slows down rather than seizing up.
  final int maxSteps;

  /// Whether [document] shows each body blended between its last two steps,
  /// rather than where the last step left it.
  ///
  /// A body is then shown one step behind the world, part of the way from
  /// where it was to where it is, by the time owed to the next step. It moves
  /// on every [advance], including one that took no step. [physics] keeps the
  /// truth, so a cast, a query, an event or a contact is about where the body
  /// is now, which is up to one step ahead of what the document shows.
  ///
  /// Fixed for the life of the scene. A snapshot restores only into a scene
  /// that smooths as the one it came from did.
  final bool smooth;

  SceneDocument _document;
  final Map<String, int> _bodyOf = {};
  final Map<int, String> _entityOf = {};

  /// Where each body was when the document was last told. Without [smooth] it
  /// is exactly as the world reported it. A body is written back when the
  /// world disagrees with this rather than with the document, so the rounding
  /// in a trip from a pose to three angles and back is never mistaken for
  /// movement.
  final Map<int, Float32List> _known = {};

  /// With [smooth], where the world had each body after the last two steps. A
  /// body is shown at [_latest] when the two are the same, so one at rest is
  /// never moved by the rounding in a blend.
  final Map<int, Float32List> _previous = {};
  final Map<int, Float32List> _latest = {};

  /// The entities [stopSmoothing] has switched off. Everything under one is
  /// shown as the world has it.
  final Set<String> _sharp = {};

  int _next = 1;

  /// The world's hull for each distinct set of corners at each size, so a
  /// hundred crates cut from one mesh share one. The key is the corners as the
  /// world is given them, scaled and joined.
  final Map<String, int> _hullIdOf = {};

  /// The key of the hull each body is made of, for the bodies that have one.
  final Map<int, String> _hullKeyOf = {};

  /// Counted apart from the bodies', and never handed out twice, like them. A
  /// hull laid directly through [physics] wants a negative number.
  int _nextHull = 1;

  final Map<String, int> _jointOf = {};
  final Map<int, String> _entityOfJoint = {};

  /// The two bodies each joint in the world holds, so rebuilding a body can
  /// find the joints the world dropped with it.
  final Map<String, ({int a, int b})> _held = {};
  final Set<String> _broken = {};
  int _nextJoint = 1;

  /// The entity pairs told to pass through each other, each with the smaller
  /// id first so a pair is one entry whichever way it was named.
  final Set<({String a, String b})> _ignored = {};

  double _owed = 0;
  List<PhysicsEvent> _events = const [];

  /// The document as the simulation has left it.
  SceneDocument get document => _document;

  /// Everything that happened in the steps the last [advance] took, in the
  /// order it happened. Empty when that advance took none.
  ///
  /// Read this rather than the world's own, which keeps only its last step.
  /// A slow frame that owed three steps would lose what happened in the first
  /// two, and a fast one that owed none would hear the last frame's again.
  List<PhysicsEvent> get events => _events;

  /// The body an entity is simulated as, or null when it has none.
  int? bodyOf(String entity) => _bodyOf[entity];

  /// The entity a body stands for, or null when it is not one of the
  /// document's — including a body whose entity has since lost it.
  String? entityOf(int body) => _entityOf[body];

  /// The joint an entity's [JointComponent] is, or null when it has none. An
  /// entity keeps its number for as long as it has the component, whether or
  /// not the joint is holding anything: [Physics.jointStateOf] is null for one
  /// that has broken or has no body to hold.
  int? jointOf(String entity) => _jointOf[entity];

  /// The entity joint [number] was made for, or null when it is not one of
  /// the document's. Kept after the joint breaks, so the event that says it
  /// broke can be read.
  String? entityOfJoint(int number) => _entityOfJoint[number];

  /// The joint entities whose joints have broken, and are not holding until
  /// they are edited.
  Set<String> get broken => Set.unmodifiable(_broken);

  /// Makes entities [a] and [b] pass through each other, whatever their layers
  /// say: no contact and no touch event. Returns false, changing nothing, for
  /// an entity that has no body and for an entity against itself.
  ///
  /// It holds through the edits that rebuild either body, and ends when
  /// either entity loses its body or goes. It replaces any rule the world had
  /// for the pair. A character does not read it. [physics] rules for one pair
  /// are the way to say anything else about it.
  bool ignore(String a, String b) {
    final first = _bodyOf[a];
    final second = _bodyOf[b];
    if (first == null || second == null || a == b) return false;
    _ignored.add(_pairOf(a, b));
    return physics.setRule(first, second, const PhysicsRule(ignore: true));
  }

  /// Lets [a] and [b] touch again. Returns false if they were not ignoring
  /// each other.
  bool unignore(String a, String b) {
    if (!_ignored.remove(_pairOf(a, b))) return false;
    return physics.removeRule(_bodyOf[a]!, _bodyOf[b]!);
  }

  /// Shows [entity] and everything under it as the world has it, with no
  /// blend, until [startSmoothing]. For a body that must not trail a step
  /// behind what it is attached to, or one that [physics] places each frame.
  /// A body switches at once, so one on screen jumps by what it had trailed.
  ///
  /// Returns false, changing nothing, for an entity the document does not
  /// have. It does nothing to a scene that is not [smooth].
  bool stopSmoothing(String entity) {
    if (!_document.contains(entity)) return false;
    _sharp.add(entity);
    _collapse(_bodiesUnder(entity));
    return true;
  }

  /// Blends [entity] again, after [stopSmoothing] for the same entity. The
  /// blend starts from where the body stands, so it does not jump. Returns
  /// false if [entity] was not stopped. An entity under a stopped one stays
  /// stopped until that one starts.
  bool startSmoothing(String entity) => _sharp.remove(entity);

  /// Forgets how [entity] and everything under it were moving, so the next
  /// [advance] shows them where the world has them and the blend starts again
  /// from there.
  ///
  /// Call it after moving a body through [physics], such as with
  /// [Physics.place]. An edit through [apply] does it by itself. Without it, a
  /// body that was moved slides from where it was to where it went, across the
  /// next step.
  ///
  /// Returns false, changing nothing, for an entity the document does not
  /// have. It does nothing to a scene that is not [smooth].
  bool resetSmoothing(String entity) {
    if (!_document.contains(entity)) return false;
    _collapse(_bodiesUnder(entity));
    return true;
  }

  /// Moves the simulation on by [seconds] and says what moved.
  ///
  /// Takes as many whole [step]s as are owed, none when less than a step has
  /// built up since the last call. Answers with a diff that takes every entity
  /// whose body moved, and every body under one that did, to where the world
  /// now has it. Empty when nothing moved. The diff is already in [document].
  ///
  /// With [smooth] the bodies are shown blended, so a call that took no step
  /// still moves them.
  SceneDiff advance(double seconds) {
    if (seconds > 0) _owed += seconds;
    var taken = 0;
    final events = <PhysicsEvent>[];
    final sharp = smooth ? _sharpBodies() : const <int>{};
    while (_owed >= step && taken < maxSteps) {
      physics.step(step);
      events.addAll(physics.events);
      if (smooth) _carry(sharp);
      _owed -= step;
      taken++;
    }
    if (_owed >= step) _owed = 0;
    _events = events;

    // The world has already taken a broken joint out, and says which by its
    // number.
    for (final event in events) {
      if (event.kind != PhysicsEventKind.broke) continue;
      final id = _entityOfJoint[event.a];
      if (id == null) continue;
      _broken.add(id);
      _held.remove(id);
    }
    return taken == 0 && !smooth ? SceneDiff.none : _writeBack();
  }

  /// Tells the world about an edit to [document].
  ///
  /// Every entity the edit touches, and everything under one, has its body
  /// rebuilt where the document now puts it — a moved parent carries the
  /// bodies under it, as it carries what they draw — or taken out of the world
  /// when it no longer has one. A rebuilt body has lost its velocity, whatever
  /// the edit was: editing a document mid-simulation is a teleport. Every
  /// joint on a rebuilt body, and every joint entity touched, is made again
  /// as things now stand; one that had broken is made again only when the
  /// edit is to its own entity.
  void apply(SceneDiff diff) {
    if (diff.isEmpty) return;

    final touched = <String>{};
    for (final operation in diff.operations) {
      // Names, order, visibility and settings move nothing a body cares about.
      if (operation
          case AddEntity(:final id) ||
              RemoveEntity(:final id) ||
              SetComponent(:final id) ||
              SetField(:final id) ||
              Reparent(:final id)) {
        touched.add(id);
      }
    }

    final before = _document;
    _document = diff.applyTo(before);
    _sharp.removeWhere((id) => !_document.contains(id));

    // Editing a broken joint is asking for it back. Moving what is above it
    // is not, or dragging the door would mend the hinge it tore off.
    _broken.removeAll(touched);

    // Under a touched entity as it was, for what the edit took away, and as it
    // is, for what the edit brought.
    final affected = <String>{
      for (final id in touched) ...[
        for (final entity in before.subtreeOf(id)) entity.id,
        for (final entity in _document.subtreeOf(id)) entity.id,
      ],
    };
    _rebuild(affected);
  }

  /// Copies the simulation as it stands: the world, and the document with the
  /// bodies where the last [advance] left them.
  ///
  /// Given the same edits and the same [advance] calls, a simulation restored
  /// from it reaches the same document, on the same build. The copy is native
  /// memory, so it is let go of with [ScenePhysicsSnapshot.dispose].
  ScenePhysicsSnapshot snapshot() => ScenePhysicsSnapshot._(this);

  /// Goes back to [snapshot]: the world, the document, which entity is which
  /// body and joint, what has broken and what is ignored, the time owed to the
  /// next step and the events of the last [advance]. Everything done since is
  /// gone, edits included.
  ///
  /// Answers with the diff that takes [document] as it stood to the one
  /// restored, so whatever draws the document can follow. The restored
  /// document is already in [document].
  ///
  /// The numbers handed out since are taken back with the rest, so an entity
  /// added after the snapshot is given the same number again. That is what
  /// makes a replay the same replay. A number kept from the run that was left
  /// names nothing, or something else, in the run that follows.
  ///
  /// A snapshot keeps to the [step] it was taken at. One restored into a
  /// simulation that steps by another does not replay. It also keeps to
  /// [smooth]: one restored into a scene that smooths differently throws an
  /// [ArgumentError], because it has no blend to go back to or no use for one.
  SceneDiff restore(ScenePhysicsSnapshot snapshot) {
    if (snapshot._smooth != smooth) {
      throw ArgumentError.value(
        snapshot,
        'snapshot',
        snapshot._smooth
            ? 'was taken by a scene that smooths'
            : 'was taken by a scene that does not smooth',
      );
    }

    final before = _document;
    // Before anything changes, because a snapshot that has been disposed
    // throws here, and nothing is left half changed.
    physics.restore(snapshot._world);

    _document = snapshot._document;
    _bodyOf
      ..clear()
      ..addAll(snapshot._bodyOf);
    _entityOf
      ..clear()
      ..addAll(snapshot._entityOf);
    // Copied again, because a step writes into these in place and the
    // snapshot has to stay as it was for the next restore.
    _known
      ..clear()
      ..addAll(_copyOf(snapshot._known));
    _previous
      ..clear()
      ..addAll(_copyOf(snapshot._previous));
    _latest
      ..clear()
      ..addAll(_copyOf(snapshot._latest));
    _sharp
      ..clear()
      ..addAll(snapshot._sharp);
    _next = snapshot._next;
    _hullIdOf
      ..clear()
      ..addAll(snapshot._hullIdOf);
    _hullKeyOf
      ..clear()
      ..addAll(snapshot._hullKeyOf);
    _nextHull = snapshot._nextHull;
    _jointOf
      ..clear()
      ..addAll(snapshot._jointOf);
    _entityOfJoint
      ..clear()
      ..addAll(snapshot._entityOfJoint);
    _held
      ..clear()
      ..addAll(snapshot._held);
    _broken
      ..clear()
      ..addAll(snapshot._broken);
    _nextJoint = snapshot._nextJoint;
    _ignored
      ..clear()
      ..addAll(snapshot._ignored);
    _owed = snapshot._owed;
    _events = List.of(snapshot._events);

    return SceneDiff.between(before, _document);
  }

  void dispose() => physics.dispose();

  // --- document to world ----------------------------------------------------

  /// Makes the world agree with the document about [ids]: each one that has a
  /// body gets it, where the document now puts it, and each one that has lost
  /// its body — or has gone — loses it from the world.
  void _rebuild(Iterable<String> ids) {
    final added = <int>[];
    final gone = <int>{};
    for (final id in ids) {
      final number = _bodyOf[id];
      if (number != null) {
        physics.remove(number);
        gone.add(number);
        _hullKeyOf.remove(number);
      }

      final body = _document[id]?[SceneComponents.body];
      if (body is! BodyComponent) {
        if (number != null) {
          _bodyOf.remove(id);
          _entityOf.remove(number);
          _known.remove(number);
          _previous.remove(number);
          _latest.remove(number);
        }
        continue;
      }

      final kept = number ?? _next++;
      _bodyOf[id] = kept;
      _entityOf[kept] = id;
      final zone = _document[id]?[SceneComponents.zone];
      _add(
        kept,
        body,
        worldOf(_document, id),
        zone is ZoneComponent ? zone : null,
      );
      added.add(kept);
    }
    _rejoin(ids, gone);
    _reignore(gone);
    _dropIdleHulls();
    if (added.isEmpty) return;

    // Read back rather than remembered from what was sent: the world keeps
    // floats and tidies the rotation it is handed, and a body whose first
    // reading differed from what was sent would be written back on the first
    // step without having moved.
    final read = Float32List(added.length * 7);
    physics.readInto(added, read);
    for (var i = 0; i < added.length; i++) {
      final pose = read.sublist(i * 7, i * 7 + 7);
      _known[added[i]] = Float32List.fromList(pose);
      // Both ends of the blend are the new place, so a body that was put
      // somewhere does not slide there from where it was.
      if (smooth) {
        _previous[added[i]] = Float32List.fromList(pose);
        _latest[added[i]] = Float32List.fromList(pose);
      }
    }
  }

  void _add(
    int number,
    BodyComponent body,
    Matrix4 world,
    ZoneComponent? zone,
  ) {
    final rotation = Quaternion.identity();
    final scale = Vector3.zero();
    world.decompose(Vector3.zero(), rotation, scale);
    scale.absolute();
    final at = world.transform3(body.centre.clone());
    final plane = body.shape == BodyShape.plane;
    if (body.shape == BodyShape.hull) _layHull(number, body, scale);

    physics.add(
      number,
      shape: _shapeOf(number, body, scale),
      motion: plane ? PhysicsMotion.fixed : _motionOf(body.motion),
      at: [at.x, at.y, at.z],
      rotation: [rotation.x, rotation.y, rotation.z, rotation.w],
      mass: body.mass,
      friction: body.friction,
      restitution: body.restitution,
      linearDamping: body.linearDamping,
      angularDamping: body.angularDamping,
      layers: Layers(is_: body.layers, cares: body.cares),
      asleep: body.startsAsleep,
      // A zone is a region, and only a trigger is one.
      trigger: body.trigger || zone != null,
      stay: body.stay,
    );

    final surface = body.surface;
    if (surface.length2 > 0) {
      physics.setSurface(number, velocity: [surface.x, surface.y, surface.z]);
    }
    if (zone != null) physics.setZone(number, _zoneFor(zone));

    // Quiet, so a body added asleep stays asleep with its controls.
    final controls = _controlsFor(body, scale);
    if (controls != null) physics.setControls(number, controls, quiet: true);
  }

  /// How [body] says it moves, at the size [scale] makes it, or null when it
  /// says nothing and the world's own ways apply. The centre of mass grows
  /// with the entity, like the shape it is measured from. The inertia does not:
  /// it is a mass property the document gives outright.
  static PhysicsControls? _controlsFor(BodyComponent body, Vector3 scale) {
    final centre = body.centreOfMass.clone()..multiply(scale);
    final inertia = body.inertia;
    final plain =
        body.locks.isEmpty &&
        body.gravityScale == 1 &&
        body.maxSpeed == 0 &&
        body.maxSpin == 0 &&
        centre.length2 == 0 &&
        inertia.length2 == 0;
    if (plain) return null;

    // A number the world would refuse takes every other control with it, so a
    // hand-edited file's negative cap or inertia is read as none.
    return PhysicsControls(
      locks: {
        for (final lock in body.locks) PhysicsLock.values.byName(lock.name),
      },
      gravityScale: body.gravityScale,
      maxSpeed: math.max(body.maxSpeed, 0),
      maxSpin: math.max(body.maxSpin, 0),
      centre: [centre.x, centre.y, centre.z],
      inertia: inertia.length2 == 0
          ? null
          : [
              math.max(inertia.x, 0),
              math.max(inertia.y, 0),
              math.max(inertia.z, 0),
            ],
    );
  }

  static PhysicsZone _zoneFor(ZoneComponent zone) {
    final gravity = zone.gravity;
    return PhysicsZone(
      gravity: gravity == null ? null : [gravity.x, gravity.y, gravity.z],
      linearDamping: zone.linearDamping,
      angularDamping: zone.angularDamping,
      priority: zone.priority,
    );
  }

  /// Lays the hull [body] is made of, at the size [scale] makes it, unless the
  /// world already has it, and notes that body [number] is made of it.
  ///
  /// Corners that are not a whole number of points are not a hull, and neither
  /// are ones the world cannot cook: no hull is noted, and the body is not
  /// made.
  void _layHull(int number, BodyComponent body, Vector3 scale) {
    final corners = body.hull;
    if (corners.length % 3 != 0) return;

    final scaled = [
      for (var i = 0; i < corners.length; i++) corners[i] * scale[i % 3],
    ];
    final key = scaled.join(',');
    _hullKeyOf[number] = key;
    if (_hullIdOf.containsKey(key)) return;

    final id = _nextHull++;
    if (physics.layHull(id, points: scaled)) _hullIdOf[key] = id;
  }

  /// Takes out of the world the hulls no body is made of any more. A hull is
  /// cooked once and kept while any body names it, and it goes with the last.
  void _dropIdleHulls() {
    final used = _hullKeyOf.values.toSet();
    // A copy, because a hull that goes is taken out of the map being read.
    for (final MapEntry(:key, :value) in _hullIdOf.entries.toList()) {
      if (!used.contains(key) && physics.dropHull(value)) _hullIdOf.remove(key);
    }
  }

  /// Body [number]'s shape, made of [body] at the size [scale] makes it. A
  /// ball, a capsule and a cylinder cannot be stretched, only grown, so each
  /// takes the scale that makes it biggest in the directions it has: every one
  /// for a ball, the sideways ones for a radius and the upright one for a
  /// height. A hull is stretched along each axis.
  Shape _shapeOf(int number, BodyComponent body, Vector3 scale) {
    final size = body.size;
    switch (body.shape) {
      case BodyShape.box:
        return Shape.box(
          size.x.abs() * scale.x / 2,
          size.y.abs() * scale.y / 2,
          size.z.abs() * scale.z / 2,
        );
      case BodyShape.sphere:
        final largest = math.max(scale.x, math.max(scale.y, scale.z));
        return Shape.sphere(body.radius.abs() * largest);
      case BodyShape.capsule:
        final radius = body.radius.abs() * math.max(scale.x, scale.z);
        // Height is tip to tip; the solver wants the straight part, from the
        // middle. None left means all ends and no middle: a ball.
        final straight = body.height.abs() * scale.y / 2 - radius;
        return straight > 0
            ? Shape.capsule(radius, straight)
            : Shape.sphere(radius);
      case BodyShape.cylinder:
        return Shape.cylinder(
          body.radius.abs() * math.max(scale.x, scale.z),
          body.height.abs() * scale.y / 2,
        );
      case BodyShape.hull:
        // Zero names no hull, so corners that enclose nothing make no body.
        return Shape.hull(_hullIdOf[_hullKeyOf[number]] ?? 0);
      case BodyShape.plane:
        // The entity's own up, through the body's centre. The world turns and
        // places the normal by the body's pose, so it is given unturned here.
        return const Shape.plane(0, 1, 0);
    }
  }

  static PhysicsMotion _motionOf(BodyMotion motion) => switch (motion) {
    BodyMotion.fixed => PhysicsMotion.fixed,
    BodyMotion.driven => PhysicsMotion.driven,
    BodyMotion.free => PhysicsMotion.free,
  };

  /// Makes the world's joints agree with the document for every joint entity
  /// in [ids], and for every joint the world dropped when the bodies in
  /// [gone] were taken out to be rebuilt.
  void _rejoin(Iterable<String> ids, Set<int> gone) {
    final joints = <String>{
      for (final id in ids)
        if (_jointOf.containsKey(id) ||
            _document[id]?[SceneComponents.joint] is JointComponent)
          id,
      for (final MapEntry(key: id, value: held) in _held.entries)
        if (gone.contains(held.a) || gone.contains(held.b)) id,
    };
    if (joints.isEmpty) return;

    // Taken apart first, all of them, so none is made against a body another
    // is about to be moved off.
    for (final id in joints) {
      if (_held.remove(id) != null) physics.unjoin(_jointOf[id]!);
    }

    // Nearest the root first. The world solves joints in the order they were
    // made, and a chain solved from its fixed end outwards settles in fewer
    // passes than one solved from its tip.
    final depths = {for (final id in joints) id: _ancestorsOf(id).length};
    for (final id
        in joints.toList()..sort((a, b) => depths[a]!.compareTo(depths[b]!))) {
      _join(id);
    }
  }

  /// Makes the world's rules agree with the pairs told to be ignored: a pair
  /// whose entity has lost its body is forgotten, and one whose body was
  /// rebuilt in [gone] is made again, because the world drops a rule with the
  /// body it names.
  void _reignore(Set<int> gone) {
    _ignored.removeWhere(
      (pair) => !_bodyOf.containsKey(pair.a) || !_bodyOf.containsKey(pair.b),
    );
    for (final pair in _ignored) {
      final a = _bodyOf[pair.a]!;
      final b = _bodyOf[pair.b]!;
      if (gone.contains(a) || gone.contains(b)) {
        physics.setRule(a, b, const PhysicsRule(ignore: true));
      }
    }
  }

  static ({String a, String b}) _pairOf(String a, String b) =>
      a.compareTo(b) <= 0 ? (a: a, b: b) : (a: b, b: a);

  /// Makes [id]'s joint in the world, if it has one, it is not broken and
  /// there is a body for it to hold.
  void _join(String id) {
    final joint = _document[id]?[SceneComponents.joint];
    if (joint is! JointComponent) {
      final number = _jointOf.remove(id);
      if (number != null) _entityOfJoint.remove(number);
      _broken.remove(id);
      return;
    }

    final number = _jointOf[id] ??= _nextJoint++;
    _entityOfJoint[number] = id;
    if (_broken.contains(id)) return;

    final ends = JointComponent.endsOf(
      id,
      parentOf: (id) {
        final parent = _document[id]?.parent;
        return parent == id ? null : parent;
      },
      hasBody: _bodyOf.containsKey,
    );
    final body = ends.body;
    if (body == null) return;
    final b = _bodyOf[body]!;
    final a = ends.holder == null ? 0 : _bodyOf[ends.holder]!;

    // The joint entity's own place and turn, not its size: a hinge scaled
    // to two is the same hinge.
    final at = Vector3.zero();
    final rotation = Quaternion.identity();
    worldOf(_document, id).decompose(at, rotation, Vector3.zero());
    final middle = physics.transformOf(b)!;

    final made = physics.join(
      number,
      _jointFor(joint),
      a: a,
      b: b,
      at: [at.x, at.y, at.z],
      rotation: [rotation.x, rotation.y, rotation.z, rotation.w],
      to: [middle[0], middle[1], middle[2]],
      breakingForce: joint.breakingForce,
      breakingTorque: joint.breakingTorque,
      collide: joint.collide,
    );
    if (made) _held[id] = (a: a, b: b);
  }

  /// What [joint] holds, in the world's units: the document gives angles in
  /// degrees, as a transform's rotation is, and the world wants radians.
  static Joint _jointFor(JointComponent joint) {
    JointLimit? limitOf(JointAxis axis) {
      final range = joint.limits[axis];
      if (range == null) return null;
      return axis.turns
          ? JointLimit(radians(range.low), radians(range.high))
          : JointLimit(range.low, range.high);
    }

    return switch (joint.kind) {
      JointKind.fixed => const Joint.fixed(),
      JointKind.point => const Joint.point(),
      JointKind.hinge => Joint.hinge(
        limit: limitOf(JointAxis.aboutX),
        speed: radians(joint.speed),
        strength: joint.strength,
      ),
      JointKind.slider => Joint.slider(
        limit: limitOf(JointAxis.alongX),
        speed: joint.speed,
        strength: joint.strength,
      ),
      JointKind.distance => Joint.distance(limit: limitOf(JointAxis.alongX)),
      JointKind.cone => Joint.cone(
        swing: radians(joint.swing),
        twist: limitOf(JointAxis.aboutX),
      ),
      JointKind.sixAxis => Joint.sixAxis(
        alongX: limitOf(JointAxis.alongX),
        alongY: limitOf(JointAxis.alongY),
        alongZ: limitOf(JointAxis.alongZ),
        aboutX: limitOf(JointAxis.aboutX),
        aboutY: limitOf(JointAxis.aboutY),
        aboutZ: limitOf(JointAxis.aboutZ),
      ),
    };
  }

  // --- world to document ----------------------------------------------------

  SceneDiff _writeBack() {
    final bodies = _known.keys.toList(growable: false);
    final now = smooth ? _blended(bodies) : _read(bodies, _known);

    final moved = <String>{};
    for (var i = 0; i < bodies.length; i++) {
      final known = _known[bodies[i]]!;
      if (_same(known, now, i * 7)) continue;
      known.setAll(0, Float32List.sublistView(now, i * 7, i * 7 + 7));
      moved.add(_entityOf[bodies[i]]!);
    }
    if (moved.isEmpty) return SceneDiff.none;

    // A body under one that moved is written too, even though the world left
    // it where it was: its transform is relative to its parent, and what it is
    // relative to has gone. Parents first, so each child is measured against
    // where its parent went.
    final depths = <String, int>{};
    final writing = <String>[];
    for (final id in _bodyOf.keys) {
      final above = _ancestorsOf(id);
      if (moved.contains(id) || above.any(moved.contains)) {
        depths[id] = above.length;
        writing.add(id);
      }
    }
    writing.sort((a, b) => depths[a]!.compareTo(depths[b]!));

    final placed = <String, TransformComponent>{};
    final operations = <SceneOp>[];
    for (final id in writing) {
      final entity = _document[id]!;
      final body = entity[SceneComponents.body]! as BodyComponent;
      final had = entity[SceneComponents.transform];
      final was = had is TransformComponent ? had : null;

      final next = _transformFor(
        entity,
        body,
        _known[_bodyOf[id]!]!,
        was,
        placed,
      );
      if (next == null) continue;
      placed[id] = next;
      operations.add(
        SetComponent(
          id,
          SceneComponents.transform,
          from: was?.toJson(),
          to: next.toJson(),
        ),
      );
    }

    // Built once rather than an operation at a time: replacing one entity
    // copies the document, and a pile of crates settling is hundreds a step.
    _document = _document.copyWith(
      entities: [
        for (final entity in _document.entities)
          if (placed[entity.id] case final transform?)
            entity.withComponent(SceneComponents.transform, transform)
          else
            entity,
      ],
    );
    return SceneDiff(operations);
  }

  /// Where the world has [bodies], seven floats each. A body it has lost, taken
  /// out through [physics] directly, is read as [kept] has it, so it reads as
  /// not having moved.
  Float32List _read(List<int> bodies, Map<int, Float32List> kept) {
    final poses = Float32List(bodies.length * 7);
    for (var i = 0; i < bodies.length; i++) {
      poses.setAll(i * 7, kept[bodies[i]]!);
    }
    physics.readInto(bodies, poses);
    return poses;
  }

  /// Where [bodies] are shown: each one the time owed to the next step of the
  /// way from where it was after the last but one step to where it is after the
  /// last. Seven floats each.
  Float32List _blended(List<int> bodies) {
    final alpha = _owed / step;
    final poses = Float32List(bodies.length * 7);
    for (var i = 0; i < bodies.length; i++) {
      blendPose(
        _previous[bodies[i]]!,
        _latest[bodies[i]]!,
        alpha,
        Float32List.sublistView(poses, i * 7, i * 7 + 7),
      );
    }
    return poses;
  }

  /// Takes where the world has every body as the end of the blend, and the end
  /// it had as the start. A body in [sharp] has no start: it is shown as the
  /// world has it.
  void _carry(Set<int> sharp) {
    final bodies = _latest.keys.toList(growable: false);
    final now = _read(bodies, _latest);
    for (var i = 0; i < bodies.length; i++) {
      final latest = _latest[bodies[i]]!;
      final pose = Float32List.sublistView(now, i * 7, i * 7 + 7);
      final start = sharp.contains(bodies[i]) ? pose : latest;
      _previous[bodies[i]]!.setAll(0, start);
      latest.setAll(0, pose);
    }
  }

  /// Makes [bodies] stand still in the blend, where the world has them now.
  void _collapse(List<int> bodies) {
    if (!smooth) return;
    final now = _read(bodies, _latest);
    for (var i = 0; i < bodies.length; i++) {
      final pose = Float32List.sublistView(now, i * 7, i * 7 + 7);
      _previous[bodies[i]]!.setAll(0, pose);
      _latest[bodies[i]]!.setAll(0, pose);
    }
  }

  /// The bodies of every entity [stopSmoothing] has switched off, and of every
  /// entity under one.
  Set<int> _sharpBodies() => {
    for (final id in _sharp) ..._bodiesUnder(id),
  };

  /// The bodies of [entity] and of everything under it.
  List<int> _bodiesUnder(String entity) => [
    for (final under in _document.subtreeOf(entity))
      if (_bodyOf[under.id] case final body?) body,
  ];

  /// The transform that puts [entity]'s body at [pose] — its local position
  /// and rotation under its parent, keeping the scale it already had — or null
  /// when its parent has been squashed flat and no transform can.
  TransformComponent? _transformFor(
    SceneEntity entity,
    BodyComponent body,
    Float32List pose,
    TransformComponent? was,
    Map<String, TransformComponent> placed,
  ) {
    // Nothing here changes a size, so the entity is as big in the world as the
    // document already makes it.
    final scale = Vector3.zero();
    worldOf(
      _document,
      entity.id,
    ).decompose(Vector3.zero(), Quaternion.identity(), scale);

    // The world has the shape's middle; the entity is wherever the body's
    // centre is measured from.
    final rotation = Quaternion(pose[3], pose[4], pose[5], pose[6])
      ..normalize();
    // Through the matrix, as placing the body did: `Quaternion.rotated` turns
    // by the inverse, and the entity would jump sideways.
    final offset = rotation.asRotationMatrix().transformed(
      body.centre.clone()..multiply(scale),
    );
    final at = Vector3(pose[0], pose[1], pose[2])..sub(offset);
    final world = Matrix4.compose(at, rotation, scale);

    final parent = entity.parent == entity.id ? null : entity.parent;
    final local = worldOf(_document, parent, overriding: placed);
    if (local.invert() == 0) return null;
    local.multiply(world);

    final position = Vector3.zero();
    final turned = Quaternion.identity();
    local.decompose(position, turned, Vector3.zero());
    return TransformComponent(
      position: position,
      rotation: eulerOf(turned.asRotationMatrix(), near: was?.rotation),
      scale: was?.scale.clone(),
    );
  }

  /// [id]'s parent, its parent's parent, and so on up.
  List<String> _ancestorsOf(String id) {
    final above = <String>[];
    final seen = <String>{id};
    var at = _document[id]?.parent;
    while (at != null && seen.add(at) && _document.contains(at)) {
      above.add(at);
      at = _document[at]!.parent;
    }
    return above;
  }

  static bool _same(Float32List known, Float32List now, int from) {
    for (var i = 0; i < 7; i++) {
      if (known[i] != now[from + i]) return false;
    }
    return true;
  }
}
