import 'package:vector_math/vector_math_64.dart';

/// A particle held to a point that moves, and how hard it may be pulled.
final class Anchor {
  Anchor(this.particle, this.where, this.limit, this.links);

  final int particle;

  /// Where the particle is held, asked once at the start of each step.
  final Vector3 Function() where;

  /// The force in newtons, averaged over a step, past which the particle lets
  /// go. Null holds it whatever the pull.
  final double? limit;

  /// The links that end at the particle.
  final List<int> links;

  /// The force the body puts on the anchor. While a step is under way it is
  /// the sum over the sub-steps so far, and after it the mean.
  final pull = Vector3.zero();
}
