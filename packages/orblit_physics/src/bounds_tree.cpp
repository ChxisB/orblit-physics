#include "bounds_tree.h"

#include <algorithm>
#include <numeric>

namespace orblit {

BoundsTree::BoundsTree(const std::vector<Bounds> &bounds) : order_(bounds.size()) {
  std::iota(order_.begin(), order_.end(), 0u);
  if (!bounds.empty()) build(bounds, 0, bounds.size());
}

uint32_t BoundsTree::build(const std::vector<Bounds> &bounds, uint32_t first,
                         uint32_t count) {
  const uint32_t index = nodes_.size();
  Branch node;
  node.area = bounds[order_[first]];
  for (uint32_t i = 1; i < count; ++i) {
    const Bounds &next = bounds[order_[first + i]];
    node.area.low = minPerAxis(node.area.low, next.low);
    node.area.high = maxPerAxis(node.area.high, next.high);
  }
  nodes_.push_back(node);
  if (count <= 4) {
    nodes_[index].first = first;
    nodes_[index].count = count;
    return index;
  }
  const Vec3 span = node.area.high - node.area.low;
  int axis = span.y > span.x ? 1 : 0;
  if (span.z > span[axis]) axis = 2;
  const uint32_t half = count / 2;
  std::nth_element(order_.begin() + first, order_.begin() + first + half,
                   order_.begin() + first + count, [&](uint32_t a, uint32_t b) {
    const float ca = bounds[a].low[axis] + bounds[a].high[axis];
    const float cb = bounds[b].low[axis] + bounds[b].high[axis];
    return ca == cb ? a < b : ca < cb;
  });
  const uint32_t left = build(bounds, first, half);
  const uint32_t right = build(bounds, first + half, count - half);
  nodes_[index].left = left;
  nodes_[index].right = right;
  return index;
}

} // namespace orblit
