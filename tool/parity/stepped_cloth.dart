// A square sheet hung from one corner that falls onto a floor. Written as
// the raw bits of every particle at three steps. The C# run is
// `SteppedClothScene` in `dotnet/Orblit.Physics.Soft.Tests`, and the pinned
// lines are `Reference/stepped_cloth.txt` there. Copy this file to
// `packages/orblit_physics_soft/test/` and run it with `dart test`, then
// delete the copy. See README.md.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:orblit_physics_soft/orblit_physics_soft.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

String _f64(List<double> l) {
  final d = ByteData(8 * l.length);
  for (var i = 0; i < l.length; i++) {
    d.setFloat64(8 * i, l[i], Endian.little);
  }
  return [
    for (var i = 0; i < l.length; i++)
      d.getUint32(8 * i + 4, Endian.little).toRadixString(16).padLeft(8, '0') +
          d.getUint32(8 * i, Endian.little).toRadixString(16).padLeft(8, '0'),
  ].join(' ');
}

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

void main() {
  test('cloth', () {
    final b = SoftBody.cloth(
      columns: 8,
      rows: 8,
      origin: Vector3(0, 0.5, 0),
      across: Vector3(0.15, 0, 0),
      down: Vector3(0, 0, 0.15),
      surface: _Floor(),
    );
    b.pin(0);
    for (var s = 1; s <= 180; s++) {
      b.step(1 / 60);
      if (s == 30 || s == 90 || s == 180) {
        for (var i = 0; i < b.count; i++) {
          final p = b.positionOf(i);
          final v = b.velocityOf(i);
          print(
            'cloth $s $i P ${_f64([p.x, p.y, p.z])} V ${_f64([v.x, v.y, v.z])}',
          );
        }
      }
    }
  });
}
