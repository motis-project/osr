#pragma once

#include <limits>

#include "storage.h"
#include "utl/verify.h"

#include "geo/constants.h"
#include "geo/latlng.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr::ch {
struct shortcut_data;
struct shortcut_storage;

struct sharing_data;

template <typename Profile>
struct bidi_dijkstra_ch {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;
  constexpr static auto const kDistanceLatDegrees =
      geo::kEarthRadiusMeters * geo::kPI / 180;
  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = meet_point_1_.invalid();
    meet_point_2_ = meet_point_2_.invalid();
    best_cost_ = kInfeasible;
  }

  void reset(cost_t const max) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    clear_mp();
    max_reached_1_ = false;
    max_reached_2_ = false;
  }

  void add(ways const&,
           label const l,
           direction const,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           osr::sharing_data const*) {
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      auto const total = static_cast<cost_t>(l.cost());
      d.push(label{l.get_node(), total});
    }
  }

  void add_start(ways const& w, label const l, osr::sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "starting" << l.get_node().n_ << std::endl;
    }
    add(w, l, direction::kForward, cost1_, pq1_, sharing);
  }

  void add_end(ways const& w, label const l, osr::sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    add(w, l, direction::kBackward, cost2_, pq2_, sharing);
  }

  template <direction SearchDir>
  cost_t get_cost(node const n) const {
    if (SearchDir == direction::kForward) {
      auto const it = cost1_.find(n.get_key());
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(n.get_key());
      return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
    }
  }

  cost_t get_cost_to_mp(node const n1, node const n2) const {
    auto const f_cost = get_cost<direction::kForward>(n1);
    auto const b_cost = get_cost<direction::kBackward>(n2);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    if (meet_point_has_uturn == true) {
      return f_cost + b_cost + car::kUturnPenalty;
    }
    return f_cost + b_cost;
  }

  static cost_t get_possible_shortcut_cost(ways::routing const& r, car::node from, cost_t from_cost, car::node to, cost_t to_cost,shortcut_storage const* shortcut_storage) {

    way_and_dir from_way_and_dir = shortcut_storage->resolve_last_way_and_dir(r.node_ways_[from.n_][from.way_], from.dir_);
    way_and_dir to_way_and_dir = shortcut_storage->resolve_first_way_and_dir(r.node_ways_[to.n_][to.way_], to.dir_);
    if (from_way_and_dir.way == to_way_and_dir.way && from_way_and_dir.dir == opposite(to_way_and_dir.dir)) {
      return from_cost + to_cost + car::kUturnPenalty;
    }
    return from_cost + to_cost;
  }
  template <direction SearchDir, bool WithBlocked>
  bool run_single(ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  osr::sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs,
                  ch::shortcut_storage const* shortcuts) {
    auto const adjusted_max = (max);

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<SearchDir>(curr);

    if (curr_cost < l.cost()) {
      return true;
    }
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }
    auto const order = shortcuts->node_order_->get_order(curr.get_node());
    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations, shortcuts,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {
          if (order > shortcuts->node_order_->get_order(neighbor.get_node())) {
            return;
          }
          if (order > shortcuts->node_order_->get_order(neighbor.get_node()) && SearchDir == direction::kBackward) {
            return;
          }

          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }

          auto const total = curr_cost + cost;

          if (total >= adjusted_max) {
            if (SearchDir == direction::kForward) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
          }

          if (total < max &&
              costs[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node(), track);
            pq.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        });

    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost, bool has_uturn,
                                        node meetpoint1, node meetpoint2) {
      if constexpr (kDebug) {
        std::cout << "  potential MEETPOINT found by start ";
        meetpoint1.print(std::cout, w);
      }
      auto const tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        meet_point_has_uturn = has_uturn;
        //assert(tentative == get_cost_to_mp(meet_point_1_, meet_point_2_));
        best_cost_ = static_cast<cost_t>(tentative);

        if constexpr (kDebug) {
          std::cout << " with cost " << best_cost_ << " (" << cost << "," << other_cost <<") -> ACCEPTED\n";
        }
      } else if constexpr (kDebug) {
        std::cout << " with cost " << tentative << " (" << cost << "," << other_cost << ") -> DOMINATED\n";
      }
    };

    auto const handle_meetpoint = [&]() {
      auto const opposite_cost_map =
          opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
      auto const opposite_candidate = opposite_cost_map->find(curr.get_key());

      if (opposite_candidate != end(*opposite_cost_map)) {
        auto min_cost = std::numeric_limits<cost_t>::max();
        auto node_size = w.r_->node_ways_[curr.get_key()].size();
        car::node min_node;
        auto min_node_has_uturn = false;
        for (auto p = way_pos_t{0U}; p < node_size; ++p) {
          car::node n = {curr.get_key(),p,direction::kForward};
          car::node n2 = {curr.get_key(),p,direction::kBackward};
          auto ncost = opposite_candidate->second.cost(n);
          auto nncost = opposite_candidate->second.cost(n2);
          if (ncost < min_cost) {
            min_cost = ncost;
            min_node = n;
          }
          if (nncost < min_cost) {
            min_cost = nncost;
            min_node = n2;
          }
        }
        if (min_cost != kInfeasible) {
          if (w.r_->is_restricted(
                    curr.get_key(), curr.way_, min_node.way_, SearchDir)) {
            return;
          }
          auto forward_way = SearchDir == direction::kForward ? w.r_->node_ways_[curr.get_key()][curr.way_] : w.r_->node_ways_[curr.get_key()][min_node.way_];
          auto forward_dir = SearchDir == direction::kForward ? curr.dir_ : min_node.dir_;
          auto backward_way = SearchDir == direction::kForward ? w.r_->node_ways_[curr.get_key()][min_node.way_] : w.r_->node_ways_[curr.get_key()][curr.way_];
          auto backward_dir = SearchDir == direction::kForward ? min_node.dir_ : curr.dir_;

          way_and_dir backward_resolved_way_and_dir = shortcuts->resolve_first_way_and_dir(backward_way, backward_dir);
          way_and_dir forward_resolved_way_and_dir = shortcuts->resolve_last_way_and_dir(forward_way, forward_dir);

          if (forward_resolved_way_and_dir.way == backward_resolved_way_and_dir.way && forward_resolved_way_and_dir.dir == opposite(backward_resolved_way_and_dir.dir)) {
            min_cost += car::kUturnPenalty;
            min_node_has_uturn = true;
          }
          evaluate_meetpoint(
              curr_cost, min_cost, min_node_has_uturn,
              SearchDir == direction::kForward ? curr : min_node,
              SearchDir == direction::kForward ? min_node : curr);
        }
      }
    };

    handle_meetpoint();
    return true;
  }

  bool abort_on_success(dial<label, get_bucket> q) {
    if (q.empty()) {
      return true;
    }
    if (best_cost_ != kInfeasible) {
      auto const top = q.buckets_[q.get_next_bucket()].back().cost();
      if (top >= best_cost_) {
        return true;
      }
    }
    return false;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           osr::sharing_data const* sharing,
           elevation_storage const* elevations,
           ch::shortcut_storage const* shortcuts) {

    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty() && !abort_on_success(pq1_)) {
        run_single<SearchDir, WithBlocked>(
              w, r, max, blocked, sharing, elevations, pq1_, cost1_, shortcuts);
      }
      if (!pq2_.empty() && !abort_on_success(pq2_)) {
        run_single<opposite(SearchDir), WithBlocked>(
              w, r, max, blocked, sharing, elevations, pq2_, cost2_, shortcuts);
      }
      if (abort_on_success(pq2_) && abort_on_success(pq1_)) {
        break;
      }
    }
    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }
    return true;
  }

  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           osr::sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir,
           ch::shortcut_storage const* shortcuts) {
    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(w, r, max, blocked, sharing,
                                                   elevations, shortcuts)
                 : run<direction::kBackward, false>(w, r, max, blocked, sharing,
                                                    elevations, shortcuts);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(w, r, max, blocked, sharing,
                                                  elevations, shortcuts)
                 : run<direction::kBackward, true>(w, r, max, blocked, sharing,
                                                   elevations, shortcuts);
    }
  }

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  node meet_point_1_;
  node meet_point_2_;
  bool meet_point_has_uturn;
  cost_t best_cost_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  bool max_reached_1_;
  bool max_reached_2_;
};

}  // namespace osr
