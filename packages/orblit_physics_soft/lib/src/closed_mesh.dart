import 'dart:typed_data';

/// A triangle mesh with no holes, and the volume it holds.
///
/// Every edge belongs to two triangles that run along it in opposite
/// directions, which is what a surface wound the same way all over does. The
/// volume is positive when the normals point out.
final class ClosedMesh {
  /// Throws [ArgumentError] unless [triangles], three indices into
  /// [particles] each, close up.
  ClosedMesh(int particles, List<int> triangles)
    : triangles = Int32List.fromList(triangles) {
    if (triangles.length < 12 || triangles.length % 3 != 0) {
      throw ArgumentError.value(
        triangles.length,
        'triangles',
        'must be three indices for each of 4 triangles at least',
      );
    }
    final seen = <int, int>{};
    for (var t = 0; t < triangles.length; t += 3) {
      for (var e = 0; e < 3; e++) {
        final a = triangles[t + e];
        final b = triangles[t + (e + 1) % 3];
        RangeError.checkValueInInterval(a, 0, particles - 1, 'triangles');
        RangeError.checkValueInInterval(b, 0, particles - 1, 'triangles');
        final key = a * particles + b;
        seen[key] = (seen[key] ?? 0) + 1;
      }
    }
    for (final key in seen.keys) {
      _checkPaired(seen, particles, key);
    }
    edges = [
      for (final key in seen.keys)
        if (key ~/ particles < key % particles)
          (key ~/ particles, key % particles),
    ];
  }

  /// Three indices for each triangle, wound so the normals point out.
  final Int32List triangles;

  /// Each edge once, as the lower index and then the higher.
  late final List<(int, int)> edges;

  /// Throws unless the edge at [key] is run once, and its reverse once.
  static void _checkPaired(Map<int, int> seen, int particles, int key) {
    final a = key ~/ particles;
    final b = key % particles;
    if (a == b || seen[key] != 1 || seen[b * particles + a] != 1) {
      throw ArgumentError.value(
        (a, b),
        'triangles',
        'must close up, each edge run once in each direction',
      );
    }
  }
}

/// Holds the volume of a [ClosedMesh] as a position constraint.
///
/// The volume is the sum, over the triangles, of the signed volume of the
/// tetrahedron each makes with the origin. A closed mesh gives the same sum
/// wherever it is. The gradient at a particle is the sum, over the triangles
/// it is in, of the cross product of the other two corners over six.
final class MeshVolume {
  MeshVolume(this._mesh, Float64List x, {required this.compliance})
    : _gradient = Float64List(x.length) {
    rest = volumeOf(x);
    if (!(rest > 0)) {
      throw ArgumentError('The triangles are wound inside out: volume $rest');
    }
  }

  final ClosedMesh _mesh;

  /// The volume to hold, in cubic metres: the one the mesh began with.
  late final double rest;

  /// How far the volume gives under load. Zero holds it exactly.
  final double compliance;

  final Float64List _gradient;

  /// What the constraint has pushed in the sub-step being solved.
  double _lambda = 0;

  /// The volume the particles in [x] enclose, in cubic metres. Reads [x] and
  /// changes nothing, so it can be asked at any time.
  double volumeOf(Float64List x) {
    final triangles = _mesh.triangles;
    var sum = 0.0;
    for (var t = 0; t < triangles.length; t += 3) {
      final a = triangles[t] * 3;
      final b = triangles[t + 1] * 3;
      final c = triangles[t + 2] * 3;
      sum +=
          x[a] * (x[b + 1] * x[c + 2] - x[b + 2] * x[c + 1]) +
          x[a + 1] * (x[b + 2] * x[c] - x[b] * x[c + 2]) +
          x[a + 2] * (x[b] * x[c + 1] - x[b + 1] * x[c]);
    }
    return sum / 6;
  }

  void restart() => _lambda = 0;

  /// Moves the particles, [weight] being the inverse mass of each, towards
  /// the volume to hold by an amount that follows the compliance.
  void project(Float64List x, Float64List weight, double inverseSquare) {
    final error = _gather(x) - rest;
    final alpha = compliance * inverseSquare;
    var stiffness = alpha;
    for (var i = 0; i < weight.length; i++) {
      final k = i * 3;
      stiffness +=
          weight[i] *
          (_gradient[k] * _gradient[k] +
              _gradient[k + 1] * _gradient[k + 1] +
              _gradient[k + 2] * _gradient[k + 2]);
    }
    if (stiffness == 0) return;
    final change = (-error - alpha * _lambda) / stiffness;
    _lambda += change;
    for (var i = 0; i < weight.length; i++) {
      for (var c = i * 3; c < i * 3 + 3; c++) {
        x[c] += weight[i] * change * _gradient[c];
      }
    }
  }

  /// Fills the gradient, and answers the volume. The volume is a polynomial
  /// of degree three in the coordinates, so a third of the gradient dotted
  /// with the position is the volume itself.
  double _gather(Float64List x) {
    final triangles = _mesh.triangles;
    _gradient.fillRange(0, _gradient.length, 0);
    for (var t = 0; t < triangles.length; t += 3) {
      final a = triangles[t] * 3;
      final b = triangles[t + 1] * 3;
      final c = triangles[t + 2] * 3;
      _accumulate(x, a, b, c);
      _accumulate(x, b, c, a);
      _accumulate(x, c, a, b);
    }
    var sum = 0.0;
    for (var k = 0; k < x.length; k++) {
      sum += x[k] * _gradient[k];
    }
    return sum / 3;
  }

  /// Adds a sixth of the cross product of particles [p] and [q] to the
  /// gradient of the particle at [target].
  void _accumulate(Float64List x, int target, int p, int q) {
    _gradient[target] += (x[p + 1] * x[q + 2] - x[p + 2] * x[q + 1]) / 6;
    _gradient[target + 1] += (x[p + 2] * x[q] - x[p] * x[q + 2]) / 6;
    _gradient[target + 2] += (x[p] * x[q + 1] - x[p + 1] * x[q]) / 6;
  }
}
