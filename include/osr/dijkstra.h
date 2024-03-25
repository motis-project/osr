#pragma once

#include "fmt/core.h"

#include "utl/helpers/algorithm.h"

#include "osr/dial.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "utl/timer.h"

namespace osr {

using dist_t = std::uint16_t;

constexpr auto const kInfeasible = std::numeric_limits<dist_t>::max();

enum class direction : std::uint8_t {
  kForward,
  kBackward,
};

constexpr std::string_view to_str(direction const d) {
  switch (d) {
    case direction::kForward: return "FWD";
    case direction::kBackward: return "FWD";
    default: std::unreachable();
  }
}

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

struct foot {
  dist_t operator()(way_properties const& e,
                    direction,
                    std::uint16_t const dist) {
    if (e.is_walk_accessible()) {
      return static_cast<dist_t>(std::round(dist * 1.5F));
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_walk_accessible() ? 0U : kInfeasible;
  }
};

struct bike {
  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    if (e.is_bike_accessible() &&
        (dir == direction::kForward || !e.is_oneway_bike())) {
      return static_cast<dist_t>(std::round(dist * 3.5F));
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_bike_accessible() ? 0U : kInfeasible;
  }
};

struct car {
  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    if (e.is_bike_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return e.max_speed_m_per_s() * dist * 0.8;
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }
};

template <typename WeightFn>
void dijkstra(ways const& w,
              dijkstra_state& s,
              dist_t const max_dist,
              WeightFn&& weight_fn) {
  while (!s.pq_.empty()) {
    auto l = s.pq_.top();
    s.pq_.pop();

    if (s.dist_[l.node_].dist_ < l.dist_) {
      continue;
    }

    for (auto const [way, i] :
         utl::zip(w.node_ways_[l.node_], w.node_in_way_idx_[l.node_])) {
      auto const expand = [&](std::uint16_t const from, std::uint16_t const to,
                              direction const dir) {
        auto const dist_idx = std::min(from, to);
        // TODO check if it gets faster with checking for infeasibility first
        // with dist=0, only then lookup the actual distance (memory access =
        // costly)
        auto const dist = w.way_node_dist_[way][dist_idx];
        auto const edge_weight = weight_fn(w.way_properties_[way], dir, dist);
        if (edge_weight == kInfeasible) {
          fmt::println("way={} infeasible in dir={}: {}", w.way_osm_idx_[way],
                       to_str(dir), fmt::streamed(w.way_properties_[way]));
          return;
        }
        auto const target_node = w.way_nodes_[way][to];
        auto const node_weight = weight_fn(w.node_properties_[target_node]);
        if (node_weight == kInfeasible) {
          fmt::println("node={} infeasible: {}", w.node_to_osm_[target_node],
                       fmt::streamed(w.node_properties_[target_node]));
          return;
        }
        auto const new_dist = l.dist_ + edge_weight + node_weight;
        auto& entry = s.dist_[target_node];
        if (new_dist < entry.dist_ && new_dist < max_dist) {
          entry = {.pred_ = w.way_nodes_[way][from],
                   .dist_ = static_cast<dist_t>(new_dist)};
          s.pq_.push(label{.node_ = target_node,
                           .dist_ = static_cast<dist_t>(new_dist)});
        }
      };

      if (i != 0U) {
        expand(i, i - 1, direction::kBackward);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(i, i + 1, direction::kForward);
      }
    }
  }
}

}  // namespace osr