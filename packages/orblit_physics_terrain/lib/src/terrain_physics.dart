import 'dart:math' as math;
import 'dart:typed_data';

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_terrain/orblit_terrain.dart';

/// A terrain as ground in a physics world.
///
/// Each region is laid as a height field of its own, one fixed body a region,
/// and only the regions near a point are laid at all: a world a hundred
/// regions across has the few around the camera in it, not millions of
/// triangles nobody is standing on. [sync] keeps it that way as the point
/// moves, and lays a region again when the terrain under it changes.
///
/// A region's field is its own texels and the first texel of the regions
/// after it, which is the ground the renderer draws for it, with a margin of
/// one texel all round that is never stood on. The margin is the neighbours'
/// heights, and it is what makes the fields one ground: a ridge running along
/// the line between two regions holds a ball up the way it would if the line
/// were not there. So a region is laid again when a neighbour changes, not
/// only when it does.
///
/// A hole is a hole, and so is ground whose region does not exist: a texel
/// with no height to give takes every triangle it is a corner of with it,
/// which is the rule `Terrain.heightAt` and the renderer keep.
///
/// The world is the caller's. This adds and removes its own bodies in it and
/// touches nothing else; they are numbered by [idOf] unless the constructor is
/// given another numbering, and [regionOf] turns an event's body back into the
/// region it hit.
class TerrainPhysics {
  TerrainPhysics(
    this.physics,
    this.terrain, {
    this.friction = 0.5,
    this.restitution = 0.0,
    this.layers = Layers.everything,
    int Function(RegionKey key) numbering = idOf,
  }) : _numbering = numbering;

  /// The world the ground is laid in.
  final Physics physics;

  /// The ground being laid. Its regions are read, never written.
  final Terrain terrain;

  /// As for any body, and the same for every region.
  final double friction;
  final double restitution;
  final Layers layers;

  final int Function(RegionKey key) _numbering;

  final Map<RegionKey, _Laid> _laid = {};
  final Map<int, RegionKey> _regionOf = {};

  /// The body region [key] is laid as, unless the constructor was given
  /// another numbering.
  ///
  /// Negative, so it never meets the numbers an entity store or a
  /// `ScenePhysics` hands out, and far below zero, so it never meets the
  /// small negative numbers a game gives bodies of its own, like a player's
  /// character. Packed from the key's two numbers, so no two regions within
  /// 2^27 of the origin share one.
  static int idOf(RegionKey key) =>
      _first + ((key.x & _field) << 28 | (key.z & _field));

  static const int _first = -(1 << 62);
  static const int _field = (1 << 28) - 1;

  /// The regions laid now.
  Iterable<RegionKey> get laid => _laid.keys;

  /// The body region [key] is laid as, or null while it is not laid.
  int? bodyOf(RegionKey key) => _laid[key]?.body;

  /// The region laid as [body], or null when it is not one of these.
  RegionKey? regionOf(int body) => _regionOf[body];

  /// Lays the regions within [radius] metres of world ([x], [z]), takes up
  /// the ones that have gone far, and answers how many it laid or laid again.
  ///
  /// A region is near when any of its square is within [radius]. It is taken
  /// up once it is a region's width further than that, so a camera wandering
  /// back and forth along the line between two regions does not lay and take
  /// up the same one every frame; and as soon as it is gone from the terrain.
  ///
  /// A region already laid is laid again when its heights or cover, or any
  /// neighbour's, have changed since. With [refresh] false it is left as it
  /// was, however it has changed, which is how an editor holds off relaying
  /// the ground under a stroke that is still being drawn: sync with it false
  /// while the stroke goes on, and true once it is done.
  int sync({
    required double x,
    required double z,
    required double radius,
    bool refresh = true,
  }) {
    if (!x.isFinite || !z.isFinite || !radius.isFinite || radius < 0) {
      throw ArgumentError('A point and a radius, finite and not negative.');
    }
    final width = terrain.regionSize * terrain.spacing;

    for (final key in _laid.keys.toList()) {
      if (terrain.regionAt(key) == null ||
          _distance(key, x, z, width) > radius + width) {
        _takeUp(key);
      }
    }

    var laid = 0;
    if (refresh) {
      for (final MapEntry(:key, :value) in _laid.entries.toList()) {
        final now = _revisionsAround(key);
        if (!_same(value.revisions, now) && _lay(key, now)) laid++;
      }
    }

    final x0 = ((x - radius) / width).floor();
    final x1 = ((x + radius) / width).floor();
    final z0 = ((z - radius) / width).floor();
    final z1 = ((z + radius) / width).floor();
    for (var kz = z0; kz <= z1; kz++) {
      for (var kx = x0; kx <= x1; kx++) {
        final key = RegionKey(kx, kz);
        if (_laid.containsKey(key) || terrain.regionAt(key) == null) continue;
        if (_distance(key, x, z, width) > radius) continue;
        if (_lay(key, _revisionsAround(key))) laid++;
      }
    }
    return laid;
  }

  /// Takes up every region laid.
  void clear() {
    for (final key in _laid.keys.toList()) {
      _takeUp(key);
    }
  }

  bool _lay(RegionKey key, List<int> revisions) {
    final size = terrain.regionSize;
    final samples = size + 3;
    final heights = Float32List(samples * samples)
      ..fillRange(0, samples * samples, double.nan);

    // Texel (i0, j0) of the whole terrain is sample (0, 0): a texel short of
    // the region's own corner, for the margin.
    final i0 = key.x * size - 1;
    final j0 = key.z * size - 1;
    for (var dz = -1; dz <= 1; dz++) {
      for (var dx = -1; dx <= 1; dx++) {
        final region = terrain.regionAt(RegionKey(key.x + dx, key.z + dz));
        if (region == null) continue;
        // Its texels that fall in the grid: one column or row of a neighbour
        // before, the whole of this one, two of a neighbour after.
        final ri = region.key.x * size;
        final rj = region.key.z * size;
        final iFrom = math.max(i0, ri);
        final iTo = math.min(i0 + samples, ri + size);
        final jFrom = math.max(j0, rj);
        final jTo = math.min(j0 + samples, rj + size);
        for (var j = jFrom; j < jTo; j++) {
          for (var i = iFrom; i < iTo; i++) {
            final at = (j - rj) * size + (i - ri);
            if (Cover(region.cover[at]).hole) continue;
            heights[(j - j0) * samples + (i - i0)] = region.heights[at];
          }
        }
      }
    }

    final body = _numbering(key);
    final spacing = terrain.spacing;
    final laid = physics.layGround(
      body,
      heights: heights,
      columns: samples,
      rows: samples,
      spacing: spacing,
      margin: true,
      at: [i0 * spacing, 0.0, j0 * spacing],
      friction: friction,
      restitution: restitution,
      layers: layers,
    );
    if (laid) {
      _laid[key] = _Laid(body, revisions);
      _regionOf[body] = key;
    }
    return laid;
  }

  void _takeUp(RegionKey key) {
    final laid = _laid.remove(key);
    if (laid == null) return;
    _regionOf.remove(laid.body);
    physics.remove(laid.body);
  }

  /// The revision of the region at [key] and of each of its eight
  /// neighbours, zero for one that is not there. Revisions are never reused,
  /// so a region taken out and a different one put back is a change.
  List<int> _revisionsAround(RegionKey key) => [
    for (var dz = -1; dz <= 1; dz++)
      for (var dx = -1; dx <= 1; dx++)
        terrain.regionAt(RegionKey(key.x + dx, key.z + dz))?.revision ?? 0,
  ];

  static bool _same(List<int> a, List<int> b) {
    for (var n = 0; n < a.length; n++) {
      if (a[n] != b[n]) return false;
    }
    return true;
  }

  /// How far world ([x], [z]) is from the nearest point of region [key]'s
  /// square, [width] metres a side.
  static double _distance(RegionKey key, double x, double z, double width) {
    final dx = math.max(
      0.0,
      math.max(key.x * width - x, x - (key.x + 1) * width),
    );
    final dz = math.max(
      0.0,
      math.max(key.z * width - z, z - (key.z + 1) * width),
    );
    return math.sqrt(dx * dx + dz * dz);
  }
}

/// A region as it was laid: the body it is, and the revisions around it then.
final class _Laid {
  const _Laid(this.body, this.revisions);

  final int body;
  final List<int> revisions;
}
