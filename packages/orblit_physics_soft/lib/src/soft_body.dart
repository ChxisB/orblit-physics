import 'dart:math' as math;
import 'dart:typed_data';

import 'package:vector_math/vector_math_64.dart';

import 'spec.dart';

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
/// A particle can be pinned, which holds it where the caller puts it. Nothing
/// here touches a physics world: a body that asks for collisions is given one.
final class SoftBody {
  SoftBody._(this.spec, this._x, double mass, this._links)
    : _prev = Float64List.fromList(_x),
      _v = Float64List(_x.length),
      _inverseMass = 1 / mass,
      _w = Float64List(_x.length ~/ 3)..fillRange(0, _x.length ~/ 3, 1 / mass);

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
      soft,
      x,
      particleMass,
      _clothLinks(columns, rows, x, soft),
    );
  }

  /// A chain through [points], each joined to the next. It resists folding
  /// across the particle between two links.
  factory SoftBody.rope({
    required List<Vector3> points,
    double particleMass = 0.01,
    SoftSpec? spec,
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
    return SoftBody._(soft, x, particleMass, links.build());
  }

  final SoftSpec spec;

  final Float64List _x;
  final Float64List _prev;
  final Float64List _v;
  final Float64List _w;
  final double _inverseMass;
  final _Links _links;

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

  /// Lets a pinned particle go.
  void release(int i) {
    _w[i] = _inverseMass;
    wake();
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
    if (_asleep) return;
    final span = math.min(dt, SoftSpec.maxStep);
    final pieces = (span / spec.substepLength).ceil();
    final h = span / pieces;
    for (var s = 0; s < pieces; s++) {
      _integrate(h);
      _solve(h);
      _derive(h);
    }
    _rest(span);
  }

  void _integrate(double h) {
    final g = spec.gravity.storage;
    final keep = math.max(0.0, 1 - spec.damping * h);
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
    final inverseSquare = 1 / (h * h);
    for (var pass = 0; pass < spec.iterations; pass++) {
      for (var l = 0; l < links.length; l++) {
        _project(l, inverseSquare);
      }
    }
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
