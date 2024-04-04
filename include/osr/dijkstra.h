#pragma once

#include <variant>

#include "fmt/core.h"

#include "utl/helpers/algorithm.h"

#include "osr/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

using dist_t = std::uint16_t;

constexpr auto const kInfeasible = std::numeric_limits<dist_t>::max();

enum class direction : std::uint8_t {
  kForward,
  kBackward,
};

constexpr direction opposite(direction const dir) {
  return dir == direction::kForward ? direction::kBackward
                                    : direction::kForward;
}

template <direction Dir>
constexpr direction flip(direction const dir) {
  return Dir == direction::kForward ? dir : opposite(dir);
}

constexpr std::string_view to_str(direction const d) {
  switch (d) {
    case direction::kForward: return "forward";
    case direction::kBackward: return "backward";
  }
  std::unreachable();
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
    auto& e = dist_[start];
    if (start_dist < e.dist_) {
      e.dist_ = start_dist;
      pq_.push(label{.node_ = start, .dist_ = start_dist});
    }
  }

  struct entry {
    node_idx_t pred_{node_idx_t::invalid()};
    dist_t dist_{kInfeasible};
  };

  dist_t get_dist(node_idx_t const n) const {
    auto const it = dist_.find(n);
    return it == end(dist_) ? kInfeasible : it->second.dist_;
  }

  std::optional<entry> get(node_idx_t const n) const {
    auto const it = dist_.find(n);
    return it == end(dist_) ? std::nullopt : std::optional{it->second};
  }

  struct hash {
    using is_avalanching = void;
    auto operator()(node_idx_t const& obj) const noexcept -> uint64_t {
      return ankerl::unordered_dense::detail::wyhash::hash(
          static_cast<uint64_t>(to_idx(obj)));
    }
  };

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<node_idx_t, entry, hash> dist_;
};

template <direction Dir, typename WeightFn>
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

    for (auto const [way, i] : utl::zip_unchecked(
             w.node_ways_[l.node_], w.node_in_way_idx_[l.node_])) {
      auto const expand = [&](std::uint16_t const from, std::uint16_t const to,
                              direction const dir) {
        if (weight_fn(w.way_properties_[way], dir, 0U) == kInfeasible) {
          return;
        }
        auto const dist_idx = std::min(from, to);
        auto const dist = w.way_node_dist_[way][dist_idx];
        auto const edge_weight = weight_fn(w.way_properties_[way], dir, dist);
        if (edge_weight == kInfeasible) {
          return;
        }
        auto const target_node = w.way_nodes_[way][to];
        auto const node_weight = weight_fn(w.node_properties_[target_node]);
        if (node_weight == kInfeasible) {
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
        expand(i, i - 1, flip<Dir>(direction::kBackward));
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(i, i + 1, flip<Dir>(direction::kForward));
      }
    }
  }
}

template <typename WeightFn>
void dijkstra(ways const& w,
              dijkstra_state& s,
              dist_t const max_dist,
              direction const dir,
              WeightFn&& weight_fn) {
  dir == direction::kForward
      ? dijkstra<direction::kForward>(w, s, max_dist, weight_fn)
      : dijkstra<direction::kBackward>(w, s, max_dist, weight_fn);
}

}  // namespace osr