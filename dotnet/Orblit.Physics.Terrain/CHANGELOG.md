# Changelog

## 0.1.0

The version is the one the package had when the port began.

First release. `TerrainPhysics` lays a terrain in a physics world as ground:
one height field a region, for the regions near a point, each with a margin
of its neighbours' heights so the ground is one ground across the lines
between them. Holes and missing regions are holes.

`Sync` lays what has come near, takes up what has gone far, and lays again any
region whose heights, or whose neighbours' heights, have changed since it was
laid. While a stroke is still being drawn, it can hold off until the stroke
is done.
