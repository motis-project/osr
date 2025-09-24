#pragma once

#include <algorithm>
#include <random>

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct node_order final {
  enum method { kRandom, kRealImportance };

  static method string2method(std::string const& str) {
    if (str == "real") {
      return kRealImportance;
    }
    return kRandom;
  }

  cista::offset::vector_map<node_idx_t, node_idx_t> node2order_;
  cista::offset::vector_map<node_idx_t, node_idx_t> order2node_;

  void initialize(auto const num_nodes) {
    node2order_.reserve(num_nodes);
    order2node_.reserve(num_nodes);
    for (uint64_t i = 0; i < num_nodes; ++i) {
      node2order_.emplace_back(i);
      order2node_.emplace_back(i);
    }
  }

  void shuffle_random(uint_fast64_t const seed = 1337) {
    std::ranges::shuffle(order2node_, std::mt19937_64{seed});
    update_node2order();
  }

  void by_real_world_importance(ways::routing const& r) {
    std::ranges::sort(order2node_, {}, [&](auto const n) {
      return r.node_properties_[n].importance_;
    });
    update_node2order();
  }

  void update_node2order() {
    for (node_idx_t i{0}; i < node2order_.size(); ++i) {
      node2order_[order2node_[i]] = i;
    }
  }

  [[nodiscard]] bool constexpr lower_than(node_idx_t const node,
                                          node_idx_t const min_order) const {
    return node2order_[node] < min_order;
  }
};

}  // namespace osr
