#pragma once

#include "utl/helpers/algorithm.h"

#include "osr/dial.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "utl/timer.h"

namespace osr {

using dist_t = std::uint16_t;

constexpr auto const kInfeasible = std::numeric_limits<dist_t>::max();

struct label {
  node_idx_t node_;
  dist_t dist_;
};

struct get_bucket {
  dist_t operator()(label const& l) { return l.dist_; }
};

struct dijkstra_state {
  void reset(dist_t const max_dist) {
    pq_.clear();
    pq_.n_buckets(max_dist);
    dist_.clear();
  }

  void add_start(node_idx_t const start, dist_t const start_dist) {
    dist_[start] = {node_idx_t::invalid(), start_dist};
    pq_.push(label{.node_ = start, .dist_ = start_dist});
  }

  struct entry {
    node_idx_t pred_{node_idx_t::invalid()};
    dist_t dist_{kInfeasible};
  };

  dist_t get_dist(node_idx_t const n) const {
    auto const it = dist_.find(n);
    return it == end(dist_) ? kInfeasible : it->second.dist_;
  }

  dial<label, get_bucket> pq_{get_bucket{}};
  hash_map<node_idx_t, entry> dist_;
};

struct pedestrian {
  dist_t operator()(std::uint16_t const dist) {
    return static_cast<dist_t>(std::round(dist * 1.5F));
  }
};

struct bike {
  dist_t operator()(std::uint16_t const dist) {
    return static_cast<dist_t>(std::round(dist * 1.5F));
  }
};

struct car {
  dist_t operator()(std::uint16_t const dist) {
    return static_cast<dist_t>(std::round(dist * 1.5F));
  }
};

template <typename EdgeWeightFn>
void dijkstra(ways const& w,
              dijkstra_state& s,
              dist_t const max_dist,
              EdgeWeightFn&& edge_weight_fn) {
  while (!s.pq_.empty()) {
    auto l = s.pq_.top();
    s.pq_.pop();

    if (s.dist_[l.node_].dist_ < l.dist_) {
      continue;
    }

    for (auto const [way, i] :
         utl::zip(w.node_ways_[l.node_], w.node_in_way_idx_[l.node_])) {
      auto const expand = [&](std::uint16_t const from,
                              std::uint16_t const to) {
        auto const dist_idx = std::min(from, to);
        auto const dist = w.way_node_dist_[way][dist_idx];
        auto const edge_weight = edge_weight_fn(dist);
        if (edge_weight == kInfeasible) {
          return;
        }
        auto const new_dist = l.dist_ + edge_weight;
        auto const target_node = w.way_nodes_[way][to];
        auto& entry = s.dist_[target_node];
        if (new_dist < entry.dist_ && new_dist < max_dist) {
          entry = {.pred_ = w.way_nodes_[way][from],
                   .dist_ = static_cast<dist_t>(new_dist)};
          s.pq_.push(label{.node_ = target_node,
                           .dist_ = static_cast<dist_t>(new_dist)});
        }
      };

      if (i != 0U) {
        expand(i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(i, i + 1);
      }
    }
  }
}

}  // namespace osr