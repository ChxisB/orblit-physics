import 'dart:math' as math;

import 'package:vector_math/vector_math_64.dart';

/// How a joint measures the turn of its second body, as the solver does.
///
/// [turn] is the second body's frame seen from the first's. It is split into a
/// twist about x and then a swing that tips x over, the shorter way round, and
/// answered as the three numbers `Physics.jointStateOf` reads back: the twist,
/// then the swing's turn about y and about z, in radians.
///
/// A pose's angles have to be measured exactly as the joint measures them,
/// because they become the places the joint's springs pull to. A different
/// split is right for small turns and wrong for large ones.
List<double> turnAngles(Quaternion turn) {
  var x = turn.x;
  var y = turn.y;
  var z = turn.z;
  var w = turn.w;
  if (w < 0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }
  final twist = 2 * math.atan2(x, w);

  // With x tipped all the way over there is no twist to take out, and what
  // is left is all swing.
  final size = math.sqrt(x * x + w * w);
  final about = size > 1e-9
      ? Quaternion(-x / size, 0, 0, w / size)
      : Quaternion.identity();
  final swing = Quaternion(x, y, z, w) * about;

  final sine = math.sqrt(swing.y * swing.y + swing.z * swing.z);
  final swung = 2 * math.atan2(sine, swing.w);
  if (sine <= 1e-9) return [twist, 2 * swing.y, 2 * swing.z];
  return [twist, swing.y * swung / sine, swing.z * swung / sine];
}
