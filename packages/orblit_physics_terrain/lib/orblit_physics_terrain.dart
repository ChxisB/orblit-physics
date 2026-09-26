/// An Orblit terrain as ground in a physics world.
///
/// [TerrainPhysics] lays each region of an `orblit_terrain` terrain near a
/// point as a height field in an `orblit_physics` world, and lays it again
/// when its heights change. It is how a barrel rolling down a hill finds the
/// hill; a foot looking for the ground under it wants `Terrain.heightAt`,
/// which needs no physics.
library;

export 'src/terrain_physics.dart' show TerrainPhysics;
