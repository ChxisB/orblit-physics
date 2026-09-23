import 'dart:math' as math;
import 'dart:typed_data';

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:vector_math/vector_math_64.dart';

import 'pose.dart';

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

  SceneDocument _document;
  final Map<String, int> _bodyOf = {};
  final Map<int, String> _entityOf = {};

  /// Where each body was when the document was last told, exactly as the world
  /// reported it. A body is written back when the world disagrees with this
  /// rather than with the document, so the rounding in a trip from a pose to
  /// three angles and back is never mistaken for movement.
  final Map<int, Float32List> _known = {};

  int _next = 1;
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

  /// Moves the simulation on by [seconds] and says what moved.
  ///
  /// Takes as many whole [step]s as are owed — none, when less than a step has
  /// built up since the last call — and answers with a diff that takes every
  /// entity whose body moved, and every body under one that did, to where the
  /// world now has it. Empty when nothing moved. The diff is already in
  /// [document].
  SceneDiff advance(double seconds) {
    if (seconds > 0) _owed += seconds;
    var taken = 0;
    final events = <PhysicsEvent>[];
    while (_owed >= step && taken < maxSteps) {
      physics.step(step);
      events.addAll(physics.events);
      _owed -= step;
      taken++;
    }
    if (_owed >= step) _owed = 0;
    _events = events;
    return taken == 0 ? SceneDiff.none : _writeBack();
  }

  /// Tells the world about an edit to [document].
  ///
  /// Every entity the edit touches, and everything under one, has its body
  /// rebuilt where the document now puts it — a moved parent carries the
  /// bodies under it, as it carries what they draw — or taken out of the world
  /// when it no longer has one. A rebuilt body has lost its velocity, whatever
  /// the edit was: editing a document mid-simulation is a teleport.
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

  void dispose() => physics.dispose();

  // --- document to world ----------------------------------------------------

  /// Makes the world agree with the document about [ids]: each one that has a
  /// body gets it, where the document now puts it, and each one that has lost
  /// its body — or has gone — loses it from the world.
  void _rebuild(Iterable<String> ids) {
    final added = <int>[];
    for (final id in ids) {
      final number = _bodyOf[id];
      if (number != null) physics.remove(number);

      final body = _document[id]?[SceneComponents.body];
      if (body is! BodyComponent) {
        if (number != null) {
          _bodyOf.remove(id);
          _entityOf.remove(number);
          _known.remove(number);
        }
        continue;
      }

      final kept = number ?? _next++;
      _bodyOf[id] = kept;
      _entityOf[kept] = id;
      _add(kept, body, worldOf(_document, id));
      added.add(kept);
    }
    if (added.isEmpty) return;

    // Read back rather than remembered from what was sent: the world keeps
    // floats and tidies the rotation it is handed, and a body whose first
    // reading differed from what was sent would be written back on the first
    // step without having moved.
    final read = Float32List(added.length * 7);
    physics.readInto(added, read);
    for (var i = 0; i < added.length; i++) {
      _known[added[i]] = Float32List.fromList(read.sublist(i * 7, i * 7 + 7));
    }
  }

  void _add(int number, BodyComponent body, Matrix4 world) {
    final rotation = Quaternion.identity();
    final scale = Vector3.zero();
    world.decompose(Vector3.zero(), rotation, scale);
    scale.absolute();
    final at = world.transform3(body.centre.clone());
    final plane = body.shape == BodyShape.plane;

    physics.add(
      number,
      shape: _shapeOf(body, scale),
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
    );
  }

  /// [body]'s shape at the size [scale] makes it. A ball and a capsule cannot
  /// be stretched, only grown, so each takes the scale that makes it biggest
  /// in the directions it has: every one for a ball, the sideways ones for a
  /// capsule's radius and the upright one for its height.
  static Shape _shapeOf(BodyComponent body, Vector3 scale) {
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

  // --- world to document ----------------------------------------------------

  SceneDiff _writeBack() {
    final bodies = _known.keys.toList(growable: false);
    final now = Float32List(bodies.length * 7);
    // Filled with what is known first, so a body the world has lost — taken
    // out through [physics] directly — reads as not having moved.
    for (var i = 0; i < bodies.length; i++) {
      now.setAll(i * 7, _known[bodies[i]]!);
    }
    physics.readInto(bodies, now);

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
