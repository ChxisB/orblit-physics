import 'package:orblit_physics/orblit_physics.dart';
import 'package:vector_math/vector_math_64.dart';

import 'air.dart';

/// Wind from the triggers of a physics world.
///
/// [zones] maps the id of a trigger body to the wind blowing inside it. Where
/// triggers overlap their winds add. A trigger that is not in the map is not
/// wind, so a trigger made for something else blows nothing.
final class PhysicsWind implements SoftAir {
  PhysicsWind(this.physics, this.zones);

  final Physics physics;
  final Map<int, Vector3> zones;

  @override
  Vector3 windAt(Vector3 point) {
    final wind = Vector3.zero();
    for (final id in physics.overlap(at: point.storage, triggers: true)) {
      final zone = zones[id];
      if (zone != null) wind.add(zone);
    }
    return wind;
  }
}
