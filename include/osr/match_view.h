#pragma once

#include <span>

#include "geo/latlng.h"

#include "osr/types.h"

namespace osr {

struct node_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  level_t lvl_{kNoLevel};
  direction way_dir_{direction::kForward};
  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
  cost_t cost_{0U};
  std::vector<geo::latlng> path_{};
};

struct raw_node_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
};

struct way_candidate {
  friend bool operator<(way_candidate const& a, way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  double dist_to_way_;
  way_idx_t way_{way_idx_t::invalid()};
  node_candidate left_{}, right_{};
};

struct raw_way_candidate {
  friend bool operator<(raw_way_candidate const& a,
                        raw_way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  double dist_to_way_;
  way_idx_t way_{way_idx_t::invalid()};
  raw_node_candidate left_{}, right_{};
};

using match_t = std::vector<way_candidate>;
using match_view_t = std::span<way_candidate const>;

}
