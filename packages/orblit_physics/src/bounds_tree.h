#pragma once

#include <cstdint>
#include <vector>

#include "maths.h"

namespace orblit {

/// Immutable spatial index. Leaves name the caller's bounds by their index.
class BoundsTree final {
 public:
  explicit BoundsTree(const std::vector<Bounds> &bounds);
  const Bounds &bounds() const { return nodes_[0].area; }

  template <typename Visit>
  void visit(const Bounds &area, Visit visit) const {
    if (!nodes_.empty()) visitFrom(0, area, visit);
  }

 private:
  struct Branch {
    Bounds area;
    uint32_t first = 0;
    uint32_t count = 0;
    uint32_t left = 0;
    uint32_t right = 0;
  };

  uint32_t build(const std::vector<Bounds> &bounds, uint32_t first, uint32_t count);

  template <typename Visit>
  void visitFrom(uint32_t index, const Bounds &area, Visit &visit) const {
    const Branch &node = nodes_[index];
    if (!node.area.overlaps(area)) return;
    if (node.count != 0) {
      for (uint32_t i = 0; i < node.count; ++i) visit(order_[node.first + i]);
      return;
    }
    visitFrom(node.left, area, visit);
    visitFrom(node.right, area, visit);
  }

  std::vector<Branch> nodes_;
  std::vector<uint32_t> order_;
};

} // namespace orblit
