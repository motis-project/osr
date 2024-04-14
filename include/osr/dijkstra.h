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
  way_pos_t way_pos_;
};

struct get_bucket {
  dist_t operator()(label const& l) { return l.dist_; }
};

struct dijkstra_state {
  void reset(dist_t const max_dist) {
    pq_.clear();
    pq_.n_buckets(max_dist + 1U);
    dist_.clear();
    restricted_dist_.clear();
  }

  void add_start(ways const& w,
                 way_idx_t const way,
                 node_idx_t const start,
                 dist_t const start_dist) {
    auto is_updated =
        update_dist(start, w.get_way_pos(start, way), node_idx_t::invalid(), 0,
                    start_dist, w.node_is_restricted_[start]);
    if (is_updated) {
      push(w, {start, start_dist, w.get_way_pos(start, way)});
    }
  }

  struct entry {
    node_idx_t pred_{node_idx_t::invalid()};
    dist_t dist_{kInfeasible};
    way_pos_t pred_way_pos_{0U};
  };

  struct restricted_entry {
    restricted_entry() { utl::fill(dist_, kInfeasible); }
    std::array<node_idx_t, 16> pred_;
    std::array<dist_t, 16> dist_;
    std::array<way_pos_t, 16> pred_way_pos_{};
  };

  struct entry_ref {
    node_idx_t& pred_;
    dist_t& dist_;
    way_pos_t& pred_way_pos_;
  };

  dist_t get_dist(node_idx_t const n, way_pos_t const i) const {
    auto const e = get(n, i);
    return e.has_value() ? e->dist_ : kInfeasible;
  }

  entry_ref at(node_idx_t const n,
               way_pos_t const way_pos,
               bool const restricted) {
    if (restricted) {
      auto& x = restricted_dist_.at(n);
      return {.pred_ = x.pred_[way_pos],
              .dist_ = x.dist_[way_pos],
              .pred_way_pos_ = x.pred_way_pos_[way_pos]};
    } else {
      auto& x = dist_.at(n);
      return {
          .pred_ = x.pred_, .dist_ = x.dist_, .pred_way_pos_ = x.pred_way_pos_};
    }
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

  bool update_dist(node_idx_t const to,
                   way_pos_t const to_way_pos,
                   node_idx_t const from,
                   way_pos_t const from_way_pos,
                   dist_t const dist,
                   bool const is_restricted) {
    if (is_restricted) {
      auto& e = restricted_dist_[to];
      if (e.dist_[to_way_pos] <= dist) {
        return false;
      }
      e.pred_[to_way_pos] = from;
      e.dist_[to_way_pos] = dist;
      e.pred_way_pos_[to_way_pos] = from_way_pos;
      return true;
    } else {
      auto& e = dist_[to];
      if (e.dist_ <= dist) {
        return false;
      }
      e.pred_ = from;
      e.dist_ = dist;
      e.pred_way_pos_ = from_way_pos;
      return true;
    }
  }

  void push(ways const& w, label const& l) {
    trace("    => INSERT (node={}, dist={}, restricted={}, way_pos={}, way={})",
          w.node_to_osm_[l.node_], l.dist_, w.node_is_restricted_[l.node_],
          l.way_pos_, w.way_osm_idx_[w.node_ways_[l.node_][l.way_pos_]]);
    CISTA_UNUSED_PARAM(w);
    pq_.push(l);
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

    auto const n_restricted = w.node_is_restricted_[l.node_];
    auto const e = s.at(l.node_, l.way_pos_, n_restricted);
    if (e.dist_ < l.dist_) {
      continue;
    }

    trace("EXTRACT (node={}, dist={}, restricted={}, way_pos={}, way={})",
          w.node_to_osm_[l.node_], l.dist_, n_restricted, l.way_pos_,
          w.way_osm_idx_[w.node_ways_[l.node_][l.way_pos_]]);

    auto way_pos = way_pos_t{0U};
    for (auto const [way, i] : utl::zip_unchecked(
             w.node_ways_[l.node_], w.node_in_way_idx_[l.node_])) {
      auto const expand = [&](std::uint16_t const from, std::uint16_t const to,
                              direction const dir) {
        trace("  way={}: from={} -> to={} (node={} -> node={})",
              w.way_osm_idx_[way], from, to,
              w.node_to_osm_[w.way_nodes_[way][from]],
              w.node_to_osm_[w.way_nodes_[way][to]]);

        if (weight_fn(w.way_properties_[way], dir, 0U) == kInfeasible) {
          trace("    => INFEASIBLE WAY");
          return;
        }

        if (n_restricted && w.is_restricted(l.node_, l.way_pos_, way_pos)) {
          trace("    => RESTRICTED");
          return;
        }

        auto const dist_idx = std::min(from, to);
        auto const dist = w.way_node_dist_[way][dist_idx];
        auto const edge_weight = weight_fn(w.way_properties_[way], dir, dist);
        if (edge_weight == kInfeasible) {
          trace("    => INFEASIBLE WAY");
          return;
        }

        auto const target_node = w.way_nodes_[way][to];
        auto const node_weight = weight_fn(w.node_properties_[target_node]);
        if (node_weight == kInfeasible) {
          trace("    => INFEASIBLE TARGET NODE");
          return;
        }

        auto const new_dist = l.dist_ + edge_weight + node_weight;
        if (new_dist >= max_dist) {
          trace("    => MAX DIST REACHED");
          return;
        }

        auto const target_node_way_pos = w.get_way_pos(target_node, way);
        auto const is_updated = s.update_dist(
            target_node, target_node_way_pos, l.node_, l.way_pos_,
            static_cast<dist_t>(new_dist), w.node_is_restricted_[target_node]);
        if (is_updated) {
          s.push(w, {.node_ = target_node,
                     .dist_ = static_cast<dist_t>(new_dist),
                     .way_pos_ = target_node_way_pos});
        } else {
          trace("    => NO UPDATE");
        }
      };

      if (i != 0U) {
        expand(i, i - 1, flip<Dir>(direction::kBackward));
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(i, i + 1, flip<Dir>(direction::kForward));
      }

      ++way_pos;
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