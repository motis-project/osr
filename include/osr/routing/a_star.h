#pragma once

#include "geo/webmercator.h"
#include "osr/lookup.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

constexpr auto const kDebug = false;

template <typename Profile>
struct a_star{
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;


  void add_start(ways const& w, label const l) {
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      pq_.push(l);
    }
  }

  void add_end(ways const& w, label const l) {
    end_node_label = l;
    end_loc_ = w.get_node_pos(l.n_);
  }

  // run? reset?

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
    auto const dx = start_coord.x_ - end_node_label.x_;
    auto const dy = start_coord.y_ - end_node_label.y_;
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

  dial<node_h, get_bucket> pq_{get_bucket{}};
  label end_node_label;
  location end_loc_;
  ankerl::unordered_dense::map<key, entry, hash> cost_;
};

}

