import 'package:vector_math/vector_math_64.dart';

/// How a soft body is solved, and how stiff it is.
///
/// The solver steps in pieces no longer than [substepLength], so a body hangs
/// the same at 30 and at 60 frames a second. Each piece makes [iterations]
/// passes over the links. Stiffness is a compliance, the metres a link gives
/// under one newton.
///
/// A pass cannot carry a load down a long chain, so a body stretches more the
/// more of it hangs from few links, and less the shorter the sub-step. A 20 by
/// 20 sheet hung from its top edge stretches under 2 per cent at the default.
/// Hung from two corners it needs a sub-step of 1/960 s to stay under 5.
final class SoftSpec {
  SoftSpec({
    Vector3? gravity,
    this.damping = 1.0,
    this.stretchCompliance = 1e-6,
    this.bendCompliance = 1e-3,
    this.substepLength = 1 / 480,
    this.iterations = 1,
  }) : gravity = gravity?.clone() ?? Vector3(0, -9.81, 0) {
    if (!(damping >= 0)) {
      throw ArgumentError.value(damping, 'damping', 'must not be negative');
    }
    if (!(stretchCompliance >= 0)) {
      throw ArgumentError.value(
        stretchCompliance,
        'stretchCompliance',
        'must not be negative',
      );
    }
    if (!(bendCompliance >= 0)) {
      throw ArgumentError.value(
        bendCompliance,
        'bendCompliance',
        'must not be negative',
      );
    }
    if (!(substepLength > 0 && substepLength <= maxSubstepLength)) {
      throw ArgumentError.value(
        substepLength,
        'substepLength',
        'must be above zero and at most $maxSubstepLength s',
      );
    }
    if (iterations < 1) {
      throw ArgumentError.value(iterations, 'iterations', 'must be 1 or more');
    }
  }

  /// The longest sub-step. Past it a body hangs in rags, whatever it is made of.
  static const maxSubstepLength = 1 / 120;

  /// The longest span a single `step` simulates. A longer one is cut to this,
  /// so a hitch loses time rather than spending it.
  static const maxStep = 0.1;

  /// Metres a second squared. Do not change it after the spec is made.
  final Vector3 gravity;

  /// The fraction of a particle's speed shed each second, as air does.
  final double damping;

  /// Metres of give per newton in an edge. Zero is a rod.
  final double stretchCompliance;

  /// The same for the links that resist folding. Larger folds more easily.
  final double bendCompliance;

  /// The longest sub-step, in seconds.
  final double substepLength;

  /// Passes over the links in each sub-step.
  final int iterations;
}
