import 'dart:math' as math;

import 'package:orblit_physics/orblit_physics.dart';
import 'package:orblit_physics_terrain/orblit_physics_terrain.dart';
import 'package:orblit_terrain/orblit_terrain.dart';
import 'package:test/test.dart';

void main() {
  late Physics physics;

  setUp(() => physics = Physics());
  tearDown(() => physics.dispose());

  /// Runs `seconds` of simulation at a fixed sixtieth.
  void run(double seconds) {
    for (var i = 0; i < (seconds * 60).round(); i++) {
      physics.step(1 / 60);
    }
  }

  double heightOf(int id) => physics.transformOf(id)![1];

  /// Where a ray straight down through world (x, z) meets the ground laid,
  /// or null where it meets none.
  double? castAt(double x, double z) {
    final hit = physics.cast(
      from: [x, 50.0, z],
      direction: const [0.0, -1.0, 0.0],
      distance: 100.0,
    );
    return hit == null ? null : 50.0 - hit.distance;
  }

  test('the ground laid is the ground drawn, holes and all', () {
    // Small regions, a spacing that is not one, a region on each side of the
    // origin and bumps that are not symmetric in anything: every way a texel
    // could land a place away from where it belongs.
    final terrain = Terrain(regionSize: 8, spacing: 0.75);
    double bumps(double x, double z) =>
        math.sin(x * 0.9) + 0.6 * math.cos(z * 1.3 + 0.4) + 0.05 * x;
    for (final key in const [
      RegionKey(-1, -1),
      RegionKey(0, -1),
      RegionKey(-1, 0),
      RegionKey(0, 0),
      RegionKey(1, 0),
    ]) {
      terrain.fillHeights(key, bumps);
    }
    terrain
        .regionAt(const RegionKey(0, 0))!
        .setCover(3, 5, Cover.auto.withHole(true));

    final ground = TerrainPhysics(physics, terrain);
    expect(ground.sync(x: 0, z: 0, radius: 20), 5);

    final random = math.Random(7);
    var holes = 0;
    for (var n = 0; n < 600; n++) {
      final x = -7.0 + random.nextDouble() * 20.0;
      final z = -7.0 + random.nextDouble() * 14.0;
      final drawn = terrain.heightAt(x, z);
      final met = castAt(x, z);
      if (drawn == null) {
        holes++;
        expect(met, isNull, reason: 'nothing is drawn at ($x, $z)');
      } else {
        expect(met, closeTo(drawn, 1e-3), reason: 'at ($x, $z)');
      }
    }
    // The hole, the gap where region (1, -1) is missing, and the far edge of
    // region (1, 0), which has no region after it to finish its last cells.
    expect(holes, greaterThan(20));
    expect(castAt(3.4 * 0.75, 5.2 * 0.75), isNull, reason: 'the hole');
  });

  test('a crate in a valley along the line between regions sits in it', () {
    // The valley's floor is the line x = 8, the last texel of region 0 and
    // the first of region 1. Each region knows the other's slope only
    // through its margin; without it, neither would hold the crate up
    // against the other side.
    final terrain = Terrain(regionSize: 8);
    for (final key in const [RegionKey(0, 0), RegionKey(1, 0)]) {
      terrain.fillHeights(key, (x, z) => (x - 8).abs() * 0.5);
    }
    TerrainPhysics(physics, terrain).sync(x: 8, z: 4, radius: 1);

    physics.add(
      2,
      shape: const Shape.box(0.4, 0.4, 0.4),
      at: const [8.0, 2.0, 4.0],
      friction: 0.8,
    );
    run(3);

    final at = physics.transformOf(2)!;
    expect(at[0], closeTo(8.0, 0.02), reason: 'still in the middle');
    // Corners 0.4 either side of the floor, on slopes of one in two.
    expect(at[1], closeTo(0.6, 0.03));
    expect(physics.velocityOf(2)![0].abs(), lessThan(0.01));
  });

  test('sync lays what comes near and takes up what goes far', () {
    final terrain = Terrain(regionSize: 8);
    for (var x = 0; x < 6; x++) {
      terrain.addRegion(RegionKey(x, 0));
    }
    final ground = TerrainPhysics(physics, terrain);
    RegionKey key(int x) => RegionKey(x, 0);

    expect(ground.sync(x: 4, z: 4, radius: 1), 1);
    expect(ground.laid, [key(0)]);
    expect(physics.alive(TerrainPhysics.idOf(key(0))), isTrue);

    // Region 1 starts 4 metres off, region 3 is 4 past the end of region 2,
    // and region 0 ends 12 back: further than the radius but not by a
    // region's width, so it stays.
    expect(ground.sync(x: 20, z: 4, radius: 5), 3);
    expect(ground.laid.toSet(), {key(0), key(1), key(2), key(3)});

    // Nothing has changed, so nothing is laid again.
    expect(ground.sync(x: 20, z: 4, radius: 5), 0);

    // Region 0 now ends 28 back, region 2 has gone from the terrain, and
    // region 5 starts 4 metres on.
    terrain.removeRegion(key(2));
    ground.sync(x: 36, z: 4, radius: 5);
    expect(ground.laid.toSet(), {key(3), key(4), key(5)});
    expect(physics.alive(TerrainPhysics.idOf(key(0))), isFalse);
    expect(physics.alive(TerrainPhysics.idOf(key(2))), isFalse);
    expect(ground.bodyOf(key(3)), TerrainPhysics.idOf(key(3)));
    expect(ground.regionOf(TerrainPhysics.idOf(key(4))), key(4));
    expect(ground.regionOf(TerrainPhysics.idOf(key(0))), isNull);

    ground.clear();
    expect(ground.laid, isEmpty);
    expect(physics.count, 0);

    expect(
      () => ground.sync(x: double.nan, z: 0, radius: 1),
      throwsArgumentError,
    );
    expect(() => ground.sync(x: 0, z: 0, radius: -1), throwsArgumentError);
  });

  test('an edit lays the region again, and the neighbours that see it', () {
    final terrain = Terrain(regionSize: 8);
    final here = terrain.addRegion(const RegionKey(0, 0));
    final next = terrain.addRegion(const RegionKey(1, 0));
    terrain.addRegion(const RegionKey(3, 0));
    final ground = TerrainPhysics(physics, terrain);
    expect(ground.sync(x: 8, z: 4, radius: 1), 2);

    physics.add(2, shape: const Shape.box(0.3, 0.3, 0.3), at: const [4, 1, 4]);
    run(3);
    expect(physics.asleep(2), isTrue);

    // Raised under a sleeping crate: it is woken and pushed up with it.
    terrain.fillHeights(const RegionKey(0, 0), (x, z) => 1.0);
    expect(ground.sync(x: 8, z: 4, radius: 1), 2, reason: 'and region 1');
    run(1);
    expect(heightOf(2), closeTo(1.3, 0.03));

    // Region 3 is two away from region 1, so a change to it reaches no one.
    terrain.regionAt(const RegionKey(3, 0))!.setHeight(0, 0, 5);
    expect(ground.sync(x: 8, z: 4, radius: 1), 0);

    // Held off while a stroke is drawn, laid once it is done.
    next.setHeight(0, 4, 2.0);
    here.setHeight(4, 4, 1.5);
    expect(ground.sync(x: 8, z: 4, radius: 1, refresh: false), 0);
    // x = 8 is region 1's first texel, and region 1 has not been raised.
    expect(castAt(8.0, 4.0), closeTo(0.0, 1e-4), reason: 'not yet');
    expect(ground.sync(x: 8, z: 4, radius: 1), 2);
    expect(castAt(8.0, 4.0), closeTo(2.0, 1e-4));
  });

  test('bodies are numbered where no one else numbers them', () {
    final seen = <int>{};
    for (var x = -3; x <= 3; x++) {
      for (var z = -3; z <= 3; z++) {
        final id = TerrainPhysics.idOf(RegionKey(x, z));
        // Well clear of -1, which is where a game's player often is.
        expect(id, lessThan(-(1 << 61)));
        seen.add(id);
      }
    }
    expect(seen.length, 49);
    for (final corner in const [
      RegionKey(-(1 << 27), -(1 << 27)),
      RegionKey((1 << 27) - 1, (1 << 27) - 1),
    ]) {
      expect(TerrainPhysics.idOf(corner), lessThan(-(1 << 61)));
    }

    final terrain = Terrain(regionSize: 4)..addRegion(const RegionKey(0, 0));
    final ground = TerrainPhysics(
      physics,
      terrain,
      numbering: (key) => 100 + key.x,
    );
    ground.sync(x: 0, z: 0, radius: 1);
    expect(physics.alive(100), isTrue);
    expect(ground.regionOf(100), const RegionKey(0, 0));
  });
}
