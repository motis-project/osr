#pragma once

#include "geo/constants.h"
#include "geo/latlng.h"
#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "utl/verify.h"

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
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() { meet_point_ = meet_point_.invalid(); }

  void reset(cost_t const max,
             location const& start_loc,
             location const& end_loc) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    auto const pi =
        Profile::heuristic(distapprox(start_loc_.pos_, end_loc_.pos_)) * 0.5;
    PI_ = pi < max ? static_cast<cost_t>(pi) : max;
  }

  void add(ways const& w,
           label const l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d) {
    auto const heur = heuristic(w, l.n_, dir) - PI_;
    auto const total =
        static_cast<cost_t>(std::clamp(l.cost() + heur, 0.0, 65535.0));
    std::cout << "total" << total << std::endl;
    if (l.cost() + heur < d.n_buckets() - 1 &&
        cost_map[l.get_node().get_key()].update(l, l.get_node(), total,
                                                node::invalid())) {

      d.push(label{l.get_node(), total});  // TODO
    }
  }

  void add_start(ways const& w, label const l) {
    std::cout << "starting" << l.get_node().n_ << std::endl;
    add(w, l, direction::kForward, cost1_, pq1_);
  }

  void add_end(ways const& w, label const l) {
    std::cout << "ending" << l.get_node().n_ << std::endl;
    add(w, l, direction::kBackward, cost2_, pq2_);
  }

  cost_t get_cost(node const n, direction const dir) const {
    auto const cost = dir == direction::kForward ? &cost1_ : &cost2_;  // TODO
    auto const it = cost->find(n.get_key());
    return it != end(*cost) ? it->second.cost(n) : kInfeasible;
  }

  double distapprox(geo::latlng const& p1, geo::latlng const& p2) {
    auto constexpr per_lat = geo::kEarthRadiusMeters * geo::kPI / 180;

    auto const x = std::abs(p1.lat() - p2.lat()) * per_lat;
    auto const y = std::abs(p1.lng() - p2.lng()) * 80000.0;  // TODO
    return std::max(std::max(x, y), (x + y) / 1.42);
  }

  double heuristic(ways const& w, node_idx_t idx, direction const dir) {
    auto const p = w.get_node_pos(idx).as_latlng();
    auto const dist = distapprox(p, end_loc_.pos_);
    auto const other_dist = distapprox(p, start_loc_.pos_);
    return 0.5 * (Profile::heuristic(dist) - Profile::heuristic(other_dist)) *
           (dir == direction::kForward ? 1 : -1);
  }

  cost_t get_cost_from_x(node const n, cost_map const& cost) const {
    auto const it = cost.find(n.get_key());
    return it == cost.end() ? kInfeasible : it->second.cost(n) + PI_;
  }

  cost_t get_cost_from_start(node const n) const {
    return get_cost_from_x(n, cost1_);
  }

  cost_t get_cost_from_end(node const n) const {
    return get_cost_from_x(n, cost2_);
  }

  cost_t get_cost_to_mp(node const n) const {
    auto const f_cost = get_cost(n, direction::kForward);
    auto const b_cost = get_cost(n, direction::kBackward);
    return f_cost + PI_ + b_cost + PI_;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run_single(ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& cost_map,
                  direction const dir) {
    auto const adjusted_max = max;  // TODO;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    // std::cout << "woo" << pq << " "<< &pq1_ << " "<< &pq2_ << " "<<
    // get_cost(curr, dir) << " " << l.cost() << " " << l.get_node().n_ <<
    // "fw:" << (dir == direction::kForward) << std::endl;
    if (get_cost(curr, dir) < l.cost()) {
      return true;
    }
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }

          auto const heur =
              heuristic(w, neighbor.n_, dir) - heuristic(w, l.n_, dir);
          auto const total =
              static_cast<double>(l.cost()) + static_cast<double>(cost) + heur;

          // std::cout << "aaah" << total << "fw:" << (dir ==
          // direction::kForward)
          //          << "n:" << neighbor.get_node() << std::endl;
          if (total < adjusted_max &&
              cost_map[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            /*if
            (w.r_->way_nodes_[w.r_->node_ways_[neighbor.n_][neighbor.way_]].size()
            == 2) { (w.r_->node_in_way_idx_[neighbor.n_][neighbor.way_])^1

            }*/
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
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
    auto const other_cost = get_cost(curr, opposite(dir));
    if (other_cost != kInfeasible) {
      // if constexpr (kDebug) {
      std::cout << "  potential MEETPOINT found by start ";
      curr.print(std::cout, w);
      //}
      auto const tentative = l.cost() + PI_ + other_cost + PI_;
      if (tentative < best_cost_) {  // TODO overflow
        meet_point_ = curr;
        utl::verify(tentative == get_cost_to_mp(meet_point_),
                    "weird {} {}  a {} {} b {} {} {} {} n {} {}", tentative,
                    get_cost_to_mp(meet_point_), get_cost(meet_point_, dir),
                    l.cost(), get_cost(meet_point_, opposite(dir)), other_cost,
                    PI_, dir == direction::kForward, meet_point_.n_,
                    l.get_node().n_);
        best_cost_ = get_cost_to_mp(meet_point_);
        meet_point_.print(std::cout, w);

        // if constexpr (kDebug) {
        std::cout << " with cost " << best_cost_ << " -> 1ACCEPTED\n";
        //}
        // break;
      } else if constexpr (kDebug) {
        std::cout << " -> DOMINATED\n";
      }
    }

    if (best_cost_ != kInfeasible - PI_) {
      auto const top_f = pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r = pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (top_f + top_r >= best_cost_ - PI_) {
        std::cout << top_f << " " << top_r << " " << best_cost_ << " " << PI_
                  << "what\n";
        return false;
      }
    }
    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    best_cost_ = kInfeasible - PI_;  // TODO clear
    auto pq = &pq1_;
    auto dir = direction::kForward;

    while (!pq->empty()) {
      auto const cont =
          dir == direction::kForward
              ? run_single<direction::kForward, WithBlocked>(
                    w, r, max, blocked, sharing, elevations, *pq, cost1_, dir)
              : run_single<direction::kBackward, WithBlocked>(
                    w, r, max, blocked, sharing, elevations, *pq, cost2_, dir);
      if (!cont) {
        break;
      }
      pq = dir == direction::kForward ? &pq2_ : &pq1_;
      dir = opposite(dir);
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

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  node meet_point_;
  cost_t best_cost_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  cost_t PI_;
};

}  // namespace osr
