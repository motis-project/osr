#pragma once

#include "geo/webmercator.h"
#include "osr/lookup.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <typename Profile>
struct a_star{
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;

  constexpr static auto const kDebug = false;

  void add_start(ways const& w, label const l) {
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      pq_.push(node_h{l, 0, heuristic(l, w)});
    }
  }

  void add_end(ways const& w, label const l) {
    end_node_label = l;
    end_loc_ = w.get_node_pos(l.n_);
  }

  void reset(cost_t const max) {
    pq_.clear();
    pq_.n_buckets(max + 1U);
    cost_.clear();
  }

  struct node_h{
    cost_t priority() const {
      return static_cast<std::uint16_t>(cost + heuristic);
    }
    bool operator<(const node_h& other) const {
      return this->priority() < other.priority();
    }
    bool operator>(const node_h& other) const {
      return this->priority() > other.priority();
    }
    Profile::label l;
    cost_t cost;
    cost_t heuristic;
  };

  cost_t heuristic(label const l, ways const& w) {
    auto const start_coord = geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const end_coord = geo::latlng_to_merc(end_loc_.as_latlng());
    auto const dx = end_coord.x_ - start_coord.x_;
    auto const dy = end_coord.y_ - start_coord.y_;
    auto const dist = std::sqrt(dx * dx + dy * dy);

    return Profile::heuristic(dist);
  }

  struct get_bucket {
    cost_t operator()(node_h const& n) {
      return n.cost + n.heuristic;
    }
  };

  cost_t get_cost(node const n) const {
    auto const it = cost_.find(n.get_key());
    return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing) {
        while (!pq_.empty()) {
          auto curr_node_h = pq_.pop();
          auto l = curr_node_h.l;

          if (get_cost(l.get_node()) < l.cost()) {
            continue;
          }

          if constexpr (kDebug) {
            std::cout << "EXTRACT ";
            l.get_node().print(std::cout, w);
            std::cout << "\n";
          }

          auto const curr = l.get_node();

          Profile::template adjacent<SearchDir, WithBlocked>(
              r, curr, blocked, sharing,
              [&](node const neighbor, std::uint32_t const cost, distance_t,
                  way_idx_t const way, std::uint16_t, std::uint16_t) {
                if constexpr (kDebug) {
                  std::cout << "  NEIGHBOR ";
                  neighbor.print(std::cout, w);
                }

                auto const total = l.cost() + cost;
                if (total <= max &&
                    cost_[neighbor.get_key()].update(
                        l, neighbor, static_cast<cost_t>(total), curr)) {
                  auto next = label{neighbor, static_cast<cost_t>(total)};
                  next.track(l, r, way, neighbor.get_node());
                  node_h next_h = node_h{next, next.cost_, heuristic(next, w)};
                  if(next_h.cost  + next_h.heuristic < max) {
                    pq_.push(next_h);
                  }
                }
              });
        }
  }

  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           direction const dir) {
    if (blocked == nullptr) {
      dir == direction::kForward
          ? run<direction::kForward, false>(w, r, max, blocked, sharing)
          : run<direction::kBackward, false>(w, r, max, blocked, sharing);
    } else {
      dir == direction::kForward
          ? run<direction::kForward, true>(w, r, max, blocked, sharing)
          : run<direction::kBackward, true>(w, r, max, blocked, sharing);
    }
  }

  dial<node_h, get_bucket> pq_{get_bucket{}};
  std::optional<label> end_node_label;
  point end_loc_;
  ankerl::unordered_dense::map<key, entry, hash> cost_;
};

}

