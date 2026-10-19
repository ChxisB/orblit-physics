import 'dart:math' as math;

import 'package:orblit_physics_soft/orblit_physics_soft.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

const _dt = 1 / 60;

/// The plane y = 0, for a body to land on.
final class _Floor implements SoftSurface {
  @override
  SoftContact? sweep(
    Vector3 from,
    Vector3 direction,
    double distance,
    double radius,
  ) {
    final lowest = from.y + math.min(0, direction.normalized().y * distance);
    if (lowest - radius > 0) return null;
    return SoftContact(
      at: Vector3(from.x, 0, from.z),
      normal: Vector3(0, 1, 0),
    );
  }
}

/// A sphere of [radius] about [centre], made from an octahedron cut into
/// four triangles twice over, wound with the normals out.
(List<Vector3>, List<int>) _sphere(double radius, Vector3 centre) {
  var points = [
    Vector3(1, 0, 0),
    Vector3(-1, 0, 0),
    Vector3(0, 1, 0),
    Vector3(0, -1, 0),
    Vector3(0, 0, 1),
    Vector3(0, 0, -1),
  ];
  var triangles = <int>[];
  for (final sx in [0, 1]) {
    for (final sy in [2, 3]) {
      for (final sz in [4, 5]) {
        // A face with an odd number of negative axes runs the other way round.
        final flip = (sx + (sy - 2) + (sz - 4)).isOdd;
        triangles.addAll(flip ? [sx, sz, sy] : [sx, sy, sz]);
      }
    }
  }
  for (var round = 0; round < 2; round++) {
    final middles = <(int, int), int>{};
    int middle(int a, int b) =>
        middles.putIfAbsent(a < b ? (a, b) : (b, a), () {
          points = [...points, ((points[a] + points[b]) / 2)..normalize()];
          return points.length - 1;
        });
    final cut = <int>[];
    for (var t = 0; t < triangles.length; t += 3) {
      final (a, b, c) = (triangles[t], triangles[t + 1], triangles[t + 2]);
      final (ab, bc, ca) = (middle(a, b), middle(b, c), middle(c, a));
      cut.addAll([a, ab, ca, ab, b, bc, ca, bc, c, ab, bc, ca]);
    }
    triangles = cut;
  }
  return ([for (final p in points) p * radius + centre], triangles);
}

/// A rubber balloon: its edges give, so only the volume holds it up. Edges as
/// stiff as the default make the mesh a rigid shell that needs no volume.
SoftBody _balloon({double give = 0, SoftSurface? surface}) {
  final (points, triangles) = _sphere(0.3, Vector3(0, 0.6, 0));
  return SoftBody.balloon(
    points: points,
    triangles: triangles,
    volumeCompliance: give,
    spec: SoftSpec(stretchCompliance: 1e-2),
    surface: surface,
  );
}

void _run(SoftBody body, double seconds) {
  for (var n = 0; n < (seconds / _dt).round(); n++) {
    body.step(_dt);
  }
}

double _lowest(SoftBody body) => [
  for (var i = 0; i < body.count; i++) body.positionOf(i).y,
].reduce(math.min);

double _highest(SoftBody body) => [
  for (var i = 0; i < body.count; i++) body.positionOf(i).y,
].reduce(math.max);

void main() {
  test('a closed mesh reads as the volume it encloses, wherever it is', () {
    final body = _balloon();
    final sphere = 4 / 3 * math.pi * 0.3 * 0.3 * 0.3;
    // Flat faces cut off a tenth of a sphere.
    expect(body.volume, closeTo(sphere, sphere * 0.1));

    final (points, triangles) = _sphere(0.3, Vector3(40, -25, 7));
    final far = SoftBody.balloon(points: points, triangles: triangles);
    expect(far.volume, closeTo(body.volume!, 1e-9));
  });

  test('a balloon on a floor keeps its volume and does not go flat', () {
    final body = _balloon(surface: _Floor());
    final before = body.volume!;
    _run(body, 3);
    expect(body.volume, closeTo(before, before * 0.02));
    expect(_highest(body) - _lowest(body), greaterThan(0.4));
  });

  test('a cloth encloses nothing', () {
    final sheet = SoftBody.cloth(
      columns: 2,
      rows: 2,
      origin: Vector3.zero(),
      across: Vector3(1, 0, 0),
      down: Vector3(0, 0, 1),
    );
    expect(sheet.volume, isNull);
  });

  test('a balloon with give is squeezed more than one without', () {
    final firm = _balloon(surface: _Floor());
    final soft = _balloon(give: 1e-4, surface: _Floor());
    _run(firm, 3);
    _run(soft, 3);
    expect(soft.volume!, lessThan(firm.volume! * 0.99));
  });

  test('a surface with a hole, or wound inside out, is refused', () {
    final (points, triangles) = _sphere(0.3, Vector3.zero());
    expect(
      () => SoftBody.balloon(points: points, triangles: triangles.sublist(3)),
      throwsArgumentError,
    );
    final inside = [
      for (var t = 0; t < triangles.length; t += 3) ...[
        triangles[t],
        triangles[t + 2],
        triangles[t + 1],
      ],
    ];
    expect(
      () => SoftBody.balloon(points: points, triangles: inside),
      throwsArgumentError,
    );
    expect(
      () => SoftBody.balloon(
        points: points,
        triangles: [...triangles, 0, 1, 999],
      ),
      throwsRangeError,
    );
  });
}
