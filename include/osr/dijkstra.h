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
  label(node_idx_t const n, dist_t const d, std::uint8_t const pred)
      : node_{n},
        dist_{d},
        pred_way_idx_{static_cast<std::uint8_t>(pred + 1U)} {}

  label(node_idx_t const n, dist_t const d) : node_{n}, dist_{d} {}

  dist_t dist() const noexcept { return dist_; }
  node_idx_t node() const noexcept { return node_; }
  bool is_restricted() const noexcept { return pred_way_idx_ != 0U; }
  way_pos_t pred_way_pos() const noexcept {
    assert(is_restricted());
    return pred_way_idx_ - 1U;
  }

private:
  node_idx_t node_;
  dist_t dist_;
  way_pos_t pred_way_idx_{0U};
};

struct get_bucket {
  dist_t operator()(label const& l) { return l.dist(); }
};

struct dijkstra_state {
  void reset(dist_t const max_dist) {
    pq_.clear();
    pq_.n_buckets(max_dist);
    dist_.clear();
  }

  void add_start(ways const& w,
                 way_idx_t const way,
                 node_idx_t const start,
                 dist_t const start_dist) {
    auto& e = dist_[start];
    if (start_dist < e.dist_) {
      e.dist_ = start_dist;

      if (!w.node_is_restricted_[start]) {
        pq_.push(label{start, start_dist});
      } else {
        pq_.push(label{start, start_dist, w.get_way_pos(start, way)});
      }
    }
  }

  struct entry {
    node_idx_t pred_{node_idx_t::invalid()};
    dist_t dist_{kInfeasible};
    way_pos_t pred_way_pos_{0U};
  };

  struct restricted_entry {
    restricted_entry() {
      utl::fill(pred_, node_idx_t::invalid());
      utl::fill(dist_, kInfeasible);
    }

    std::array<node_idx_t, 16> pred_;
    std::array<dist_t, 16> dist_;
    std::array<way_pos_t, 16> pred_way_pos_{};
  };

  dist_t get_dist(node_idx_t const n, way_pos_t const i) const {
    auto const e = get(n, i);
    return e.has_value() ? e->dist_ : kInfeasible;
  }

  std::optional<entry> get(node_idx_t const n, way_pos_t const way_pos) const {
    auto const it = dist_.find(n);
    if (it != end(dist_)) {
      return it->second;
    }

    auto const r_it = restricted_dist_.find(n);
    if (r_it != end(restricted_dist_)) {
      return entry{.pred_ = r_it->second.pred_[way_pos],
                   .dist_ = r_it->second.dist_[way_pos],
                   .pred_way_pos_ = r_it->second.pred_way_pos_[way_pos]};
    }

    return std::nullopt;
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
  ankerl::unordered_dense::map<node_idx_t, restricted_entry, hash>
      restricted_dist_;
};

template <direction Dir, typename WeightFn>
void dijkstra(ways const& w,
              dijkstra_state& s,
              dist_t const max_dist,
              WeightFn&& weight_fn) {
  while (!s.pq_.empty()) {
    auto l = s.pq_.top();
    s.pq_.pop();

    if (s.dist_[l.node()].dist_ < l.dist()) {
      continue;
    }

    auto to_way_pos = way_pos_t{0U};
    for (auto const [way, i] : utl::zip_unchecked(
             w.node_ways_[l.node()], w.node_in_way_idx_[l.node()])) {
      auto const expand = [&](std::uint8_t const from, std::uint8_t const to,
                              direction const dir) {
        if (weight_fn(w.way_properties_[way], dir, 0U) == kInfeasible) {
          return;
        }

        if (l.is_restricted() &&
            w.is_restricted(l.node(), l.pred_way_pos(), to_way_pos)) {
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
        auto const new_dist = l.dist() + edge_weight + node_weight;

        if (l.is_restricted()) {
          auto& entry = s.restricted_dist_[target_node];
          if (new_dist < entry.dist_[to_way_pos] && new_dist < max_dist) {
            entry.dist_[to_way_pos] = new_dist;
            entry.pred_[to_way_pos] = w.way_nodes_[way][from];
          }

          if (w.node_is_restricted_[target_node]) {
            s.pq_.push(
                label{target_node, static_cast<dist_t>(new_dist), to_way_pos});
          } else {
            s.pq_.push(label{target_node, static_cast<dist_t>(new_dist)});
          }
        } else {
          auto& entry = s.dist_[target_node];
          if (new_dist < entry.dist_ && new_dist < max_dist) {
            entry = {.pred_ = w.way_nodes_[way][from],
                     .dist_ = static_cast<dist_t>(new_dist)};

            if (w.node_is_restricted_[target_node]) {
              s.pq_.push(label{target_node, static_cast<dist_t>(new_dist),
                               to_way_pos});
            } else {
              s.pq_.push(label{target_node, static_cast<dist_t>(new_dist)});
            }
          }
        }
      };

      if (i != 0U) {
        expand(i, i - 1, flip<Dir>(direction::kBackward));
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(i, i + 1, flip<Dir>(direction::kForward));
      }

      ++to_way_pos;
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