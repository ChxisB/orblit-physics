// Ground as heights on a grid: the one shape here that is not convex.
//
// Every other shape is one convex thing, and the narrowphase answers for it
// in one go. Ground is thousands of triangles, and is answered for a few at a
// time: the cells under whatever is touching it, two triangles to a cell, each
// asked on its own and the answers pooled.
//
// Two rules make a pool of triangles behave like one surface.
//
// Everything below is solid. A triangle is not a sheet with two sides but the
// top of a column of ground, so something under it is pushed up and out,
// however deep it is, and never down through. That is what lets ground be
// raised under a crate that has gone to sleep on it.
//
// A seam between two triangles is not an edge. A ball rolling across flat
// ground crosses the line between two cells, and asked on its own the
// triangle it is leaving has an edge there that the ball is touching. Pushed
// out of that edge, the ball hops. So a push is only taken from a triangle if
// it points somewhere the triangle and its neighbour agree is open: out of the
// triangle's face, or over an edge that really does fall away. Anything else
// is left to the neighbour whose face it is.
//
// The outer edge of a grid is a seam too, not a cliff. Ground that a game
// streams in pieces is laid as one field per piece, side by side, and those
// must behave as one ground: the piece next door answers for what is past
// this one's edge. Better still, a field can be laid with a margin — one ring
// of its neighbours' samples round the outside, never stood on — and then its
// edges are like any other, because it knows what is across them.

#ifndef ORBLIT_PHYSICS_HEIGHTFIELD_H
#define ORBLIT_PHYSICS_HEIGHTFIELD_H

#include <cstdint>
#include <vector>

#include "maths.h"

namespace orblit {

/// What lies across one edge of a triangle.
enum class Across : uint8_t {
  /// The grid ends here, and a neighbouring field carries on, as far as
  /// anything here knows along the same slope. Nothing may be pushed out
  /// over it.
  seam,

  /// A hole. The edge is a rim, and anything may be pushed out over it.
  rim,

  /// Another triangle of this field, which may be pushed out over only as
  /// far as its own face.
  ground,
};

/// One triangle of a field, in the field's own frame, with what each of its
/// edges borders.
///
/// Edge `k` runs from `v[k]` to `v[(k + 1) % 3]`.
struct Facet {
  Vec3 v[3];

  /// Out of the ground. Always has some up in it: no triangle of a height
  /// field is a wall.
  Vec3 normal;

  /// Perpendicular to each edge in the triangle's plane, pointing out of the
  /// triangle. A push leaning along one of these is leaving over that edge.
  Vec3 side[3];

  /// Level, perpendicular to each edge and pointing out: the walls of the
  /// column under the triangle. Every point of the field's footprint is
  /// inside exactly one triangle's walls, which is what says whose ground a
  /// point below the surface is in.
  Vec3 wall[3];

  Across across[3];

  /// For an edge that borders ground: the neighbour's own `side` on that
  /// edge. A push that leans past it has gone beyond the neighbour's face.
  Vec3 beyond[3];

  /// Whether `push`, out of this triangle, points somewhere open.
  bool admits(const Vec3 &push) const;

  /// An admitted push as the ground gives it. One that leans over an edge
  /// past the face across it — by no more than rounding, or it would not
  /// have been admitted — is brought back onto that face.
  Vec3 straightened(const Vec3 &push) const;

  /// Whether `p` is straight out from the face, along the normal, rather than
  /// out past one of its edges. `slack` metres past an edge still counts.
  bool over(const Vec3 &p, float slack) const;

  /// The point of the triangle nearest `to`.
  Vec3 nearest(const Vec3 &to) const;

  /// The corner furthest along `direction`.
  Vec3 furthest(const Vec3 &direction) const;
};

class HeightField {
 public:
  /// Copies `heights`, `columns` by `rows` of them, row after row. With a
  /// margin, the outermost ring of them is the neighbours'.
  void lay(uint32_t columns, uint32_t rows, float spacing, const float *heights,
           bool margin);

  uint32_t columns() const { return columns_; }
  uint32_t rows() const { return rows_; }
  float spacing() const { return spacing_; }

  /// No sample is a number: nothing to stand on anywhere.
  bool empty() const { return !(lowest_ <= highest_); }

  /// The box it fills in its own frame, from its highest sample down as far
  /// as a float goes, because everything below the surface is ground. The
  /// margin is not in it.
  Bounds bounds() const;

  /// Sample (c, r), or a nan for a hole or anything off the grid.
  float height(int64_t c, int64_t r) const;

  /// The cells whose squares `area` covers, clamped to the field's own. False
  /// when it covers none.
  bool cellsUnder(const Bounds &area, int64_t &c0, int64_t &r0, int64_t &c1,
                  int64_t &r1) const;

  /// Triangle `which` of cell (c, r) — 0 on the side of the diagonal nearer
  /// +x, 1 nearer +z — with its neighbours. False for a hole.
  ///
  /// Cell (c, r) is the square from sample (c, r) to sample (c + 1, r + 1),
  /// and its diagonal runs between those two.
  bool facet(int64_t c, int64_t r, int which, Facet &out) const;

  /// The lowest and highest sample of a cell's four corners, for throwing a
  /// cell away before asking about its triangles. False if all four are holes.
  bool cellHeights(int64_t c, int64_t r, float &low, float &high) const;

 private:
  /// The three corners of a triangle, false if any is a hole or off the grid.
  bool corners(int64_t c, int64_t r, int which, Vec3 v[3]) const;

  uint32_t columns_ = 0;
  uint32_t rows_ = 0;
  float spacing_ = 1.0f;
  int64_t margin_ = 0;
  std::vector<float> heights_;
  float lowest_ = 0.0f;
  float highest_ = -1.0f;
};

} // namespace orblit

#endif // ORBLIT_PHYSICS_HEIGHTFIELD_H
