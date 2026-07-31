#include "osr/routing/for_each_parking_edge.h"

#include <functional>

namespace osr {

void for_each_parking_edge(ways::routing const& r,
                           node_idx_t const node_idx,
                           std::function<void(parking_edge_idx_t)> const& f) {
	fmt::println("∀parking_edge( {} )", node_idx);
  if (r.has_parking_edges_.test(node_idx)) {
    auto it = std::lower_bound(
        begin(r.node_parking_edges_), end(r.node_parking_edges_), node_idx,
        [&](pair<node_idx_t, parking_edge_idx_t> const& p, node_idx_t const n) {
          return p.first < n;
        });
    for (; it != end(r.node_parking_edges_) && it->first == node_idx; ++it) {
			fmt::println("{}, ", it->second);
      f(it->second);
    }
		fmt::println("  DONE");
  }
}

}  // namespace osr
