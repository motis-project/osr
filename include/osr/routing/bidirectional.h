#pragma once

#include "geo/constants.h"
#include "geo/latlng.h"
#include "osr/elevation_storage.h"
#include "osr/routing/a_star.h"
#include "osr/types.h"

namespace osr {

struct sharing_data;

template <typename Profile>
struct bidirectional {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using node_h = typename a_star<Profile>::node_h;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(node_h const& n) { return n.cost_ + n.heuristic_; }
  };

  void add(ways const& w,
           label const l,
           location const& loc,
           cost_map& cost_map,
           dial<node_h, get_bucket>& d) {
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      auto const heur = heuristic(w, l, loc);
      if (l.cost() + heur < d.n_buckets() - 1) {
        d.push(node_h{l, l.cost(), static_cast<cost_t>(std::max(0.0, heur))});
      }
    }
  }

  void add_start(ways const& w, label const l) {
    std::cout << "starting" << std::endl;
    add(w, l, end_loc_, cost1_, pq1_);
  }

  void add_end(ways const& w, label const l) {
    std::cout << "ending" << std::endl;
    add(w, l, start_loc_, cost2_, pq2_);
  }

  void clear_mp() { meet_point_ = meet_point_.invalid(); }

  void reset(cost_t max, location const& start_loc, location const& end_loc) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);

    meet_point_ = meet_point_.invalid();
    cost1_.clear();
    cost2_.clear();
    expanded_start_.clear();
    expanded_end_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;

    auto const start_coord = geo::latlng_to_merc(start_loc.pos_);
    auto const end_coord = geo::latlng_to_merc(end_loc.pos_);

    auto const dx = start_coord.x_ - end_coord.x_;
    auto const dy = start_coord.y_ - end_coord.y_;
    auto const dist = std::sqrt(dx * dx + dy * dy);

    std::cout << "init dist" << dist << "\n";
    auto const pi = Profile::heuristic(dist) / 2;
    PI_ = pi < max ? static_cast<cost_t>(pi) : max;
  }

  double distapprox(geo::latlng const& p1, geo::latlng const& p2) {
    auto constexpr per_lat = geo::kEarthRadiusMeters * geo::kPI / 180;

    auto const x = std::abs(p1.lat() - p2.lat()) * per_lat;
    auto const y = std::abs(p1.lng() - p2.lng()) * 80000.0;
    return std::max(std::max(x, y), (x + y) / 1.42);
  }

  double heuristic(ways const& w, label const l, location const& loc) {

    auto const p = w.get_node_pos(l.n_).as_latlng();
    auto const dist = distapprox(p, loc.pos_);

    auto const other_loc = loc == start_loc_ ? end_loc_ : start_loc_;
    auto const other_dist = distapprox(p, other_loc.pos_);
    return 0.5 * Profile::heuristic(dist) -
           Profile::heuristic(other_dist);  // deleted + PI*2
  }

  cost_t get_cost_from_start(node const n) const {
    auto const it = cost1_.find(n.get_key());
    return it == cost1_.end() ? kInfeasible : it->second.cost(n);
  }

  cost_t get_cost_from_end(node const n) const {
    auto const it = cost2_.find(n.get_key());
    return it == cost2_.end() ? kInfeasible : it->second.cost(n);
  }

  cost_t get_cost_to_mp(node const n) const {
    auto cost1 = get_cost_from_start(n);
    auto cost2 = get_cost_from_end(n);
    /*if (cost1 == kInfeasible || cost2 == kInfeasible) {
      auto rev_node = Profile::get_reverse(n);
      cost1 = get_cost_from_start(rev_node);
      cost2 = get_cost_from_end(rev_node);
    }*/
    if constexpr (kDebug) {
      std::cout << "\n Costs: " << cost1 << " + " << cost2 << std::endl;
    }
    if (cost1 == kInfeasible || cost2 == kInfeasible) {
      return kInfeasible;
    }
    return cost1 + cost2;
  }

  template <direction SearchDir, bool WithBlocked, typename fn>
  std::optional<node> run(ways const& w,
                          ways::routing const& r,
                          cost_t const max,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          dial<node_h, get_bucket>& d,
                          cost_map& cost_map,
                          fn get_cost,
                          location& loc) {

    auto curr_node_h = d.pop();

    auto l = curr_node_h.l;
    auto curr_node = l.get_node();

    if (get_cost(curr_node) < l.cost()) {
      return std::nullopt;
    }

    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr_node, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const) {
          if (l.cost() >= max / 2 - cost) {
            // std::cout << "Overflow " << std::endl;
            return;
          }
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          auto const total = l.cost() + cost;
          if (total < max / 2 &&
              cost_map[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr_node)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            auto const heur = heuristic(w, next, loc);
            if (heur > max / 2) return;
            node_h next_h = node_h{next, next.cost_,
                                   static_cast<cost_t>(std::max(0.0, heur))};
            if (next_h.cost_ + next_h.heuristic_ < max) {
              d.push(std::move(next_h));
              if constexpr (kDebug) {
                std::cout << " -> PUSH\n";
              }
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        });
    return curr_node;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    auto best_cost = kInfeasible - PI_;

    // next_item is are top heap values (forward and reverse)
    cost_t top_f;  // cost + heuristic forward top
    cost_t top_r;  // cost + heuristic reverse top

    while (!pq1_.empty() && !pq2_.empty()) {
      top_f = pq1_.buckets_[pq1_.get_next_bucket()].back().priority();
      top_r = pq2_.buckets_[pq2_.get_next_bucket()].back().priority();
      if (top_f + top_r >= best_cost + PI_ * 2) {
        std::cout << top_f << " " << top_r << " " << best_cost << " " << PI_
                  << "what\n";
        break;
      }
      auto curr1 = run<SearchDir, WithBlocked>(
          w, r, max, blocked, sharing, elevations, pq1_, cost1_,
          [this](auto curr) { return get_cost_from_start(curr); }, end_loc_);
      auto curr2 = run<opposite(SearchDir), WithBlocked>(
          w, r, max, blocked, sharing, elevations, pq2_, cost2_,
          [this](auto curr) { return get_cost_from_end(curr); }, start_loc_);
      // A meeting point is identified, when a node is already expanded by the
      // other search
      if (curr1 != std::nullopt) {
        if (expanded_end_.contains(curr1.value().n_)) {
          if constexpr (kDebug) {
            std::cout << "  potential MEETPOINT found by start ";
            curr1.value().print(std::cout, w);
          }
          if (cost2_.find(curr1.value().get_key()) != cost2_.end()) {
            // This node has been expanded from both directions and has entries
            // in both cost maps
            if (get_cost_to_mp(curr1.value()) < best_cost) {
              meet_point_ = curr1.value();
              best_cost = get_cost_to_mp(curr1.value());

              // if constexpr (kDebug) {
              std::cout << " with cost " << best_cost << " -> ACCEPTED\n";
              //}
              // break;
            } else if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          } else if constexpr (kDebug) {
            std::cout << " -> DOMINATED\n";
          }
        }
        expanded_start_.emplace(curr1.value().n_);
      }
      if (curr2 != std::nullopt) {
        if (expanded_start_.contains(curr2.value().n_)) {
          if constexpr (kDebug) {
            std::cout << "  potential MEETPOINT found by end ";
            curr2.value().print(std::cout, w);
          }
          if (cost1_.find(curr2.value().get_key()) != cost1_.end()) {
            if (get_cost_to_mp(curr2.value()) < best_cost) {
              meet_point_ = curr2.value();
              best_cost = get_cost_to_mp(curr2.value());
              // if constexpr (kDebug) {
              std::cout << " with cost " << best_cost << " -> ACCEPTED\n";
              //}
              // break;
            } else if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          } else if constexpr (kDebug) {
            std::cout << " -> DOMINATED\n";
          }
        }
        expanded_end_.emplace(curr2.value().n_);
      }
    }
  }

  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (blocked == nullptr) {
      dir == direction::kForward ? run<direction::kForward, false>(
                                       w, r, max, blocked, sharing, elevations)
                                 : run<direction::kBackward, false>(
                                       w, r, max, blocked, sharing, elevations);
    } else {
      dir == direction::kForward ? run<direction::kForward, true>(
                                       w, r, max, blocked, sharing, elevations)
                                 : run<direction::kBackward, true>(
                                       w, r, max, blocked, sharing, elevations);
    }
  }

  dial<node_h, get_bucket> pq1_{get_bucket{}};
  dial<node_h, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  hash_set<node_idx_t> expanded_start_;
  hash_set<node_idx_t> expanded_end_;
  node meet_point_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  cost_t PI_;
};

}  // namespace osr
