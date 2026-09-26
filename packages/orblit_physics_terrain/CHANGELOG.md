## 0.1.0

- First release. `TerrainPhysics` lays an `orblit_terrain` terrain in a
  physics world as ground: one height field per region, for the regions near a
  point, each with a margin of its neighbours' heights so the ground is one
  ground across the lines between them. Holes and missing regions are holes.
  `sync` lays what has come near, takes up what has gone far, and lays again
  any region whose heights, or whose neighbours' heights, have changed since
  it was laid — or, while a stroke is still being drawn, holds off until it
  is done.
