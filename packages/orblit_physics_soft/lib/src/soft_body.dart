import 'dart:math' as math;
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import 'air.dart';
import 'anchor.dart';
import 'closed_mesh.dart';
import 'spec.dart';
import 'surface.dart';

/// Particles joined by links that hold a distance.
///
/// A cloth is a grid of particles with links along its rows and columns, along
/// both diagonals so it resists shear, and two apart so it resists folding. A
/// link two apart shortens only with the square of the bend, so a cloth drapes
/// like silk: it resists a sharp fold and gives to a gentle curve. A rope is a
/// chain. Each step moves the particles under gravity, then pulls
/// every link back to its length. The links are solved as position
/// constraints, so they stay stable at a stiffness a spring would blow up at.
///
/// A balloon is a closed mesh of particles that also holds the volume it began
/// with, so a body of it sags under gravity and squashes under a load without
/// going flat. A pillow is two sheets that close up round their rims.
///
/// A particle can be pinned, which holds it where the caller puts it.
///
/// Given a [SoftSurface], the particles stay off what it describes and grip
/// it by [SoftSpec.friction]. Each step looks once, ahead of every particle,
/// for the surface it will meet, and holds it to that plane in the sub-steps.
/// A particle is a point with a skin, so a surface thinner than the gap
/// between two particles lets one through, and the particles do not meet one
/// another. A body asleep does not look, so wake it when the surface moves.
final class SoftBody {
  SoftBody._({
    required this.spec,
    required Float64List x,
    required double mass,
    required _Links links,
    required this.surface,
    required this.air,
    MeshVolume? volume,
  }) : _x = x,
       _links = links,
       _volume = volume,
       _prev = Float64List.fromList(x),
       _v = Float64List(x.length),
       _inverseMass = 1 / mass,
       _w = Float64List(x.length ~/ 3)..fillRange(0, x.length ~/ 3, 1 / mass),
       _plane = Float64List(x.length ~/ 3 * 4),
       _touch = Uint8List(x.length ~/ 3);

  /// A sheet of [columns] by [rows] particles. [across] is the offset from one
  /// particle to the next along a row and [down] to the next row, so their
  /// lengths are the spacing and their angle is the shear. [particleMass] is
  /// the mass of one particle in kilograms.
  factory SoftBody.cloth({
    required int columns,
    required int rows,
    required Vector3 origin,
    required Vector3 across,
    required Vector3 down,
    double particleMass = 0.01,
    SoftSpec? spec,
    SoftSurface? surface,
    SoftAir? air,
  }) {
    if (columns < 2 || rows < 2) {
      throw ArgumentError('A cloth needs 2 columns and 2 rows at least');
    }
    _checkMass(particleMass);
    final soft = spec ?? SoftSpec();
    final x = Float64List(columns * rows * 3);
    for (var j = 0; j < rows; j++) {
      for (var i = 0; i < columns; i++) {
        final p = origin + across * i.toDouble() + down * j.toDouble();
        p.copyIntoArray(x, (j * columns + i) * 3);
      }
    }
    return SoftBody._(
      spec: soft,
      x: x,
      mass: particleMass,
      links: _clothLinks(columns, rows, x, soft),
      surface: surface,
      air: air,
    );
  }

  /// A closed surface of [triangles] over [points], three indices into
  /// [points] for each triangle, wound so the normals point out. Every edge is
  /// a link, and the volume the points enclose is held as a constraint. A
  /// [volumeCompliance] above zero lets it give under load.
  ///
  /// Throws [ArgumentError] for a surface with a hole in it or one wound
  /// inside out.
  ///
  /// shortcut: no link resists folding across an edge, so the shape is held by
  /// the volume alone and a balloon wrinkles when squeezed. Add bend links
  /// across each shared edge if a game needs it taut.
  factory SoftBody.balloon({
    required List<Vector3> points,
    required List<int> triangles,
    double particleMass = 0.01,
    double volumeCompliance = 0,
    SoftSpec? spec,
    SoftSurface? surface,
    SoftAir? air,
  }) {
    _checkMass(particleMass);
    if (!(volumeCompliance >= 0)) {
      throw ArgumentError.value(
        volumeCompliance,
        'volumeCompliance',
        'must not be negative',
      );
    }
    final soft = spec ?? SoftSpec();
    final x = Float64List(points.length * 3);
    for (var i = 0; i < points.length; i++) {
      points[i].copyIntoArray(x, i * 3);
    }
    final mesh = ClosedMesh(points.length, triangles);
    final links = _LinkBuilder(x);
    for (final (a, b) in mesh.edges) {
      links.stretch(a, b, soft.stretchCompliance);
    }
    return SoftBody._(
      spec: soft,
      x: x,
      mass: particleMass,
      links: links.build(),
      surface: surface,
      air: air,
      volume: MeshVolume(mesh, x, compliance: volumeCompliance),
    );
  }

  /// A chain through [points], each joined to the next. It resists folding
  /// across the particle between two links.
  factory SoftBody.rope({
    required List<Vector3> points,
    double particleMass = 0.01,
    SoftSpec? spec,
    SoftSurface? surface,
    SoftAir? air,
  }) {
    if (points.length < 2) {
      throw ArgumentError('A rope needs 2 particles at least');
    }
    _checkMass(particleMass);
    final soft = spec ?? SoftSpec();
    final x = Float64List(points.length * 3);
    for (var i = 0; i < points.length; i++) {
      points[i].copyIntoArray(x, i * 3);
    }
    final links = _LinkBuilder(x);
    for (var i = 0; i + 1 < points.length; i++) {
      links.stretch(i, i + 1, soft.stretchCompliance);
    }
    for (var i = 0; i + 2 < points.length; i++) {
      links.bend(i, i + 2, soft.bendCompliance);
    }
    return SoftBody._(
      spec: soft,
      x: x,
      mass: particleMass,
      links: links.build(),
      surface: surface,
      air: air,
    );
  }

  final SoftSpec spec;

  /// The closed mesh and its volume, for a balloon.
  final MeshVolume? _volume;

  /// What the particles stay off, or null for a body that falls through
  /// everything.
  final SoftSurface? surface;

  /// The wind the particles are dragged along by, or null for still air.
  final SoftAir? air;

  final Float64List _x;
  final Float64List _prev;
  final Float64List _v;
  final Float64List _w;
  final double _inverseMass;
  final _Links _links;

  /// For each particle that will meet a surface this step, the plane it must
  /// stay in front of: a normal and the offset along it.
  final Float64List _plane;
  final Uint8List _touch;

  final _anchors = <int, Anchor>{};

  /// The wind at the middle of the body, as of the last step.
  var _wind = Vector3.zero();

  /// Seconds the largest move has been under [sleepSpeed].
  double _still = 0;
  bool _asleep = false;

  /// A body slower than this, everywhere, for [sleepTime] seconds stops being
  /// stepped until something wakes it.
  static const sleepSpeed = 0.005;
  static const sleepTime = 1.0;

  /// How many particles there are. A cloth numbers them along its rows.
  int get count => _x.length ~/ 3;

  /// Whether the solver has stopped for lack of anything to do.
  bool get asleep => _asleep;

  /// The volume a balloon encloses now, in cubic metres, or null for a body
  /// that is not closed.
  double? get volume => _volume?.volumeOf(_x);

  Vector3 positionOf(int i) => Vector3.array(_x, i * 3);

  Vector3 velocityOf(int i) => Vector3.array(_v, i * 3);

  bool isPinned(int i) => _w[i] == 0;

  /// The fastest particle, in metres a second.
  double get maxSpeed {
    var most = 0.0;
    for (var i = 0; i < _v.length; i += 3) {
      most = math.max(
        most,
        _v[i] * _v[i] + _v[i + 1] * _v[i + 1] + _v[i + 2] * _v[i + 2],
      );
    }
    return math.sqrt(most);
  }

  /// The most any edge or diagonal is stretched or squeezed, as a fraction of
  /// its length. The links that resist folding are not counted.
  double get maxStretch {
    final links = _links;
    var most = 0.0;
    for (var l = 0; l < links.length; l++) {
      if (links.bends[l] != 0) continue;
      most = math.max(most, (_length(l) / links.rest[l] - 1).abs());
    }
    return most;
  }

  /// Holds particle [i] where it is. It stops falling and moves only by
  /// [place].
  void pin(int i) {
    _w[i] = 0;
    _v.fillRange(i * 3, i * 3 + 3, 0);
    wake();
  }

  /// Lets a pinned or attached particle go.
  void release(int i) {
    _anchors.remove(i);
    _w[i] = _inverseMass;
    wake();
  }

  /// Holds particle [i] to the point [where] answers, which is asked at the
  /// start of every step. The particle jumps to it, so a point that moves
  /// faster than the body can follow stretches the links.
  ///
  /// With a [limit], in newtons, the particle lets go when the body pulls on
  /// it harder than that, averaged over one step. [bodyPoint] gives the point
  /// for a place on a physics body. A bone is any closure of the caller's own.
  /// Placing an attached particle by hand is undone by the next step.
  void attach(int i, Vector3 Function() where, {double? limit}) {
    RangeError.checkValidIndex(i, _w, 'i', count);
    if (limit != null && !(limit > 0)) {
      throw ArgumentError.value(limit, 'limit', 'must be above zero');
    }
    final links = _links;
    final ending = [
      for (var l = 0; l < links.length; l++)
        if (links.a[l] == i || links.b[l] == i) l,
    ];
    pin(i);
    _anchors[i] = Anchor(i, where, limit, ending);
  }

  /// Lets an attached particle hang free. A particle pinned by hand stays.
  void detach(int i) {
    if (_anchors.remove(i) != null) release(i);
  }

  bool isAttached(int i) => _anchors.containsKey(i);

  /// The force, in newtons, the body puts on attached particle [i], averaged
  /// over the last step. It points the way the body pulls. It is the last
  /// value while the body sleeps, which is the load it is holding up.
  Vector3 pullOf(int i) {
    final anchor = _anchors[i];
    if (anchor == null) throw StateError('Particle $i is not attached');
    return anchor.pull.clone();
  }

  /// Moves a pinned particle. The links pull the rest after it.
  void place(int i, Vector3 at) {
    if (_w[i] != 0) {
      throw StateError('Particle $i is not pinned');
    }
    at.copyIntoArray(_x, i * 3);
    at.copyIntoArray(_prev, i * 3);
    wake();
  }

  void wake() {
    _asleep = false;
    _still = 0;
  }

  /// Moves the body on by [dt] seconds, in sub-steps. A span above
  /// [SoftSpec.maxStep] is cut to it.
  void step(double dt) {
    if (!(dt > 0)) {
      throw ArgumentError.value(dt, 'dt', 'must be above zero');
    }
    _follow();
    _blow();
    if (_asleep) return;
    final span = math.min(dt, SoftSpec.maxStep);
    final pieces = (span / spec.substepLength).ceil();
    final h = span / pieces;
    for (final anchor in _anchors.values) {
      anchor.pull.setZero();
    }
    _look(span);
    for (var s = 0; s < pieces; s++) {
      _integrate(h);
      _solve(h);
      _gather(h);
      _derive(h);
    }
    _tear(pieces);
    _rest(span);
  }

  /// Asks each anchor where its particle goes. A body whose anchors have not
  /// moved stays asleep.
  void _follow() {
    for (final anchor in _anchors.values) {
      final target = anchor.where();
      if (positionOf(anchor.particle).distanceToSquared(target) > 1e-18) {
        place(anchor.particle, target);
      }
    }
  }

  /// Samples the wind, and wakes the body if it has changed.
  void _blow() {
    final air = this.air;
    if (air == null) return;
    final wind = air.windAt(_centre());
    if (wind.distanceToSquared(_wind) > 1e-12) wake();
    _wind = wind;
  }

  Vector3 _centre() {
    final sum = Vector3.zero();
    for (var i = 0; i < count; i++) {
      sum.x += _x[i * 3];
      sum.y += _x[i * 3 + 1];
      sum.z += _x[i * 3 + 2];
    }
    return sum / count.toDouble();
  }

  /// How fast the air takes a particle up to its own speed, per second.
  double get _drag => air == null ? 0 : spec.windDrag;

  /// Adds the force each link put on its anchor in this sub-step. A link
  /// pushes its ends apart by `lambda` along its length, over the sub-step
  /// squared, so a stretched link, whose `lambda` is negative, pulls them
  /// together.
  void _gather(double h) {
    final links = _links;
    for (final anchor in _anchors.values) {
      for (final l in anchor.links) {
        final a = links.a[l] * 3;
        final b = links.b[l] * 3;
        final length = _length(l);
        if (length < 1e-12) continue;
        final sign = links.a[l] == anchor.particle ? 1.0 : -1.0;
        final scale = sign * links.lambda[l] / (length * h * h);
        anchor.pull.x += (_x[a] - _x[b]) * scale;
        anchor.pull.y += (_x[a + 1] - _x[b + 1]) * scale;
        anchor.pull.z += (_x[a + 2] - _x[b + 2]) * scale;
      }
    }
  }

  /// Turns the sums into means, and lets go of what was pulled too hard.
  void _tear(int pieces) {
    for (final anchor in _anchors.values.toList()) {
      anchor.pull.scale(1 / pieces);
      final limit = anchor.limit;
      if (limit != null && anchor.pull.length > limit) release(anchor.particle);
    }
  }

  void _integrate(double h) {
    final drag = _drag;
    final g = (spec.gravity + _wind * drag).storage;
    final keep = math.max(0.0, 1 - spec.damping * h) / (1 + drag * h);
    for (var i = 0; i < count; i++) {
      final k = i * 3;
      for (var c = 0; c < 3; c++) {
        _prev[k + c] = _x[k + c];
        if (_w[i] == 0) continue;
        _v[k + c] = (_v[k + c] + g[c] * h) * keep;
        _x[k + c] += _v[k + c] * h;
      }
    }
  }

  void _solve(double h) {
    final links = _links;
    links.lambda.fillRange(0, links.length, 0);
    _volume?.restart();
    final inverseSquare = 1 / (h * h);
    for (var pass = 0; pass < spec.iterations; pass++) {
      for (var l = 0; l < links.length; l++) {
        _project(l, inverseSquare);
      }
      _volume?.project(_x, _w, inverseSquare);
      _press();
    }
  }

  /// Finds the surface each free particle would meet this step if nothing
  /// turned it. The ball is a tenth wider than the skin, so a particle held
  /// exactly one skin off a surface still meets it, rather than grazing it and
  /// being let go every other step.
  ///
  /// A ball that overlaps a curved surface answers a point inside it, so a
  /// particle is held a tenth short of a full skin from one.
  void _look(double span) {
    final surface = this.surface;
    if (surface == null) return;
    final skin = spec.thickness;
    final fall = (spec.gravity + _wind * _drag) * (0.5 * span * span);
    for (var i = 0; i < count; i++) {
      _touch[i] = 0;
      if (_w[i] == 0) continue;
      final move = Vector3.array(_v, i * 3) * span + fall;
      final length = move.length;
      // A particle with nowhere to go still looks down, for a surface it is in.
      final dir = length > 1e-9 ? move / length : Vector3(0, -1, 0);
      final at = Vector3.array(_x, i * 3);
      final contact = surface.sweep(at, dir, length, skin * 1.1);
      if (contact != null) _hold(i, contact);
    }
  }

  void _hold(int i, SoftContact contact) {
    contact.normal.copyIntoArray(_plane, i * 4);
    _plane[i * 4 + 3] = contact.normal.dot(contact.at) + spec.thickness;
    _touch[i] = 1;
  }

  /// Moves every particle that has gone behind its plane back in front of it,
  /// and takes the slide it made on the way in. What a particle sank in this
  /// sub-step is undone whole, so a fast one is caught. A depth it began the
  /// sub-step with is undone by no more than one skin, and not kept as speed,
  /// so a surface that closes on the body moves it out over several sub-steps
  /// rather than throwing it.
  void _press() {
    if (surface == null) return;
    for (var i = 0; i < count; i++) {
      if (_touch[i] != 0) _push(i);
    }
  }

  void _push(int i) {
    final k = i * 3;
    final q = i * 4;
    final (nx, ny, nz) = (_plane[q], _plane[q + 1], _plane[q + 2]);
    final depth = _plane[q + 3] - nx * _x[k] - ny * _x[k + 1] - nz * _x[k + 2];
    if (depth <= 0) return;
    final sank = math.max(
      0.0,
      -((_x[k] - _prev[k]) * nx +
          (_x[k + 1] - _prev[k + 1]) * ny +
          (_x[k + 2] - _prev[k + 2]) * nz),
    );
    final carried = math.min(math.max(0.0, depth - sank), spec.thickness);
    final push = math.min(depth, sank) + carried;
    _x[k] += nx * push;
    _x[k + 1] += ny * push;
    _x[k + 2] += nz * push;
    _prev[k] += nx * carried;
    _prev[k + 1] += ny * carried;
    _prev[k + 2] += nz * carried;
    // The slide over the surface in this sub-step, less what the grip takes.
    final dx = _x[k] - _prev[k];
    final dy = _x[k + 1] - _prev[k + 1];
    final dz = _x[k + 2] - _prev[k + 2];
    final along = dx * nx + dy * ny + dz * nz;
    final (sx, sy, sz) = (dx - nx * along, dy - ny * along, dz - nz * along);
    final slide = math.sqrt(sx * sx + sy * sy + sz * sz);
    if (slide == 0) return;
    final grip = math.min(1.0, spec.friction * push / slide);
    _x[k] -= sx * grip;
    _x[k + 1] -= sy * grip;
    _x[k + 2] -= sz * grip;
  }

  /// Pulls the two ends of link [l] towards its length by an amount that
  /// follows its compliance. `lambda` is the push the link has made so far in
  /// this sub-step, so a second pass corrects the first rather than adding to it.
  void _project(int l, double inverseSquare) {
    final links = _links;
    final a = links.a[l] * 3;
    final b = links.b[l] * 3;
    final dx = _x[a] - _x[b];
    final dy = _x[a + 1] - _x[b + 1];
    final dz = _x[a + 2] - _x[b + 2];
    final length = math.sqrt(dx * dx + dy * dy + dz * dz);
    final wa = _w[a ~/ 3];
    final wb = _w[b ~/ 3];
    final alpha = links.alpha[l] * inverseSquare;
    final weight = wa + wb + alpha;
    // Coincident ends have no direction to push along, and two pinned ends
    // with no give cannot move.
    if (length < 1e-12 || weight == 0) return;
    final change =
        (-(length - links.rest[l]) - alpha * links.lambda[l]) / weight;
    links.lambda[l] += change;
    final s = change / length;
    _x[a] += wa * s * dx;
    _x[a + 1] += wa * s * dy;
    _x[a + 2] += wa * s * dz;
    _x[b] -= wb * s * dx;
    _x[b + 1] -= wb * s * dy;
    _x[b + 2] -= wb * s * dz;
  }

  void _derive(double h) {
    for (var i = 0; i < count; i++) {
      if (_w[i] == 0) continue;
      for (var c = i * 3; c < i * 3 + 3; c++) {
        _v[c] = (_x[c] - _prev[c]) / h;
      }
    }
  }

  void _rest(double span) {
    _still = maxSpeed < sleepSpeed ? _still + span : 0;
    _asleep = _still >= sleepTime;
  }

  double _length(int l) {
    final a = _links.a[l] * 3;
    final b = _links.b[l] * 3;
    final dx = _x[a] - _x[b];
    final dy = _x[a + 1] - _x[b + 1];
    final dz = _x[a + 2] - _x[b + 2];
    return math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  static void _checkMass(double mass) {
    if (!(mass > 0 && mass.isFinite)) {
      throw ArgumentError.value(mass, 'particleMass', 'must be above zero');
    }
  }
}

/// The distance links of a body: where each runs from and to, the length it
/// holds and how much it gives. A bend link resists folding and the rest
/// resist stretching. They run in the order they were added, which for a
/// cloth is from the top row down, so a load is carried down in one pass.
final class _Links {
  _Links(this.a, this.b, this.rest, this.alpha, this.bends)
    : lambda = Float64List(a.length);

  final Int32List a;
  final Int32List b;
  final Float64List rest;
  final Float64List alpha;
  final Uint8List bends;

  /// What each link has pushed in the sub-step being solved.
  final Float64List lambda;

  int get length => a.length;
}

/// Collects links with the length they have in the starting positions.
final class _LinkBuilder {
  _LinkBuilder(this._x);

  final Float64List _x;
  final _from = <int>[];
  final _to = <int>[];
  final _rest = <double>[];
  final _alpha = <double>[];
  final _bend = <int>[];

  void stretch(int a, int b, double alpha) => _add(a, b, alpha, 0);

  void bend(int a, int b, double alpha) => _add(a, b, alpha, 1);

  void _add(int a, int b, double alpha, int bend) {
    final dx = _x[a * 3] - _x[b * 3];
    final dy = _x[a * 3 + 1] - _x[b * 3 + 1];
    final dz = _x[a * 3 + 2] - _x[b * 3 + 2];
    _from.add(a);
    _to.add(b);
    _rest.add(math.sqrt(dx * dx + dy * dy + dz * dz));
    _alpha.add(alpha);
    _bend.add(bend);
  }

  _Links build() => _Links(
    Int32List.fromList(_from),
    Int32List.fromList(_to),
    Float64List.fromList(_rest),
    Float64List.fromList(_alpha),
    Uint8List.fromList(_bend),
  );
}

_Links _clothLinks(int columns, int rows, Float64List x, SoftSpec spec) {
  final links = _LinkBuilder(x);
  int at(int i, int j) => j * columns + i;
  final stretch = spec.stretchCompliance;
  final bend = spec.bendCompliance;
  for (var j = 0; j < rows; j++) {
    for (var i = 0; i < columns; i++) {
      final right = i + 1 < columns;
      final below = j + 1 < rows;
      if (right) links.stretch(at(i, j), at(i + 1, j), stretch);
      if (below) links.stretch(at(i, j), at(i, j + 1), stretch);
      if (right && below) links.stretch(at(i, j), at(i + 1, j + 1), stretch);
      if (i > 0 && below) links.stretch(at(i, j), at(i - 1, j + 1), stretch);
      if (i + 2 < columns) links.bend(at(i, j), at(i + 2, j), bend);
      if (j + 2 < rows) links.bend(at(i, j), at(i, j + 2), bend);
    }
  }
  return links.build();
}
