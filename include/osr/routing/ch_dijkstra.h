#pragma once

#include "osr/elevation_storage.h"
#include "osr/preprocessing/contraction_hierarchy/shortcut_storage.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <Profile P>
struct ch_dijkstra {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using cost_map = ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(cost_t const max) {
    pq_s_.clear();
    pq_t_.clear();
    pq_s_.n_buckets(max + 1U);
    pq_t_.n_buckets(max + 1U);
    cost_s_.clear();
    cost_t_.clear();
    tentative_cost_ = kInfeasible;
    tentative_mp_ = {node::invalid(), node::invalid()};
    settled_s_.clear();
    settled_t_.clear();
    max_reached_s_ = false;
    max_reached_t_ = false;
  }

  void reset_partially() {
    pq_s_.clear();
    pq_t_.clear();
    cost_t_.clear();
    max_reached_t_ = false;
    settled_t_.clear();
  }

  bool add(label const l, auto& c_map, auto& pq) {
    if (c_map[l.get_node().get_key()].update(
            l, l.get_node(), l.cost(), node::invalid(), way_idx_t::invalid())) {
      pq.push(l);
      return true;
    }
    return false;
  }

  void add_start(ways const& w, label const l) {
    if (add(l, cost_s_, pq_s_) && kDebug) {
      std::cout << "ADD START ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }
  }

  void add_end(ways const& w, label const l) {
    if (add(l, cost_t_, pq_t_) && kDebug) {
      std::cout << "ADD END ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }
  }

  template <direction SearchDir>
  cost_t get_cost(node const n) {
    if constexpr (SearchDir == direction::kForward) {
      auto const it = cost_s_.find(n.get_key());
      return it != end(cost_s_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost_t_.find(n.get_key());
      return it != end(cost_t_) ? it->second.cost(n) : kInfeasible;
    }
  }

  template <direction SearchDir>
  void constexpr evaluate_meetpoint(cost_t const cost,
                                    cost_t const other_cost,
                                    node const meetpoint1,
                                    node const meetpoint2,
                                    ways const& w) {
    if constexpr (kDebug) {
      std::cout << "  potential MEETPOINT found by start ";
      meetpoint1.print(std::cout, w);
    }
    if (uint32_t const tentative =
            static_cast<uint32_t>(cost) + other_cost +
            P::template turning_cost<SearchDir>(*w.r_, meetpoint1, meetpoint2);
        tentative < tentative_cost_) {
      tentative_mp_ = {meetpoint1, meetpoint2};
      tentative_cost_ = static_cast<cost_t>(tentative);

      if constexpr (kDebug) {
        std::cout << " with cost " << tentative_cost_ << " -> ACCEPTED\n";
      }
    } else if constexpr (kDebug) {
      std::cout << " -> DOMINATED\n";
    }
  }

  template <direction SearchDir, adj_conf Config>
  node run_single(P::parameters const& params,
                  ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  shortcut_storage<node> const& sc_stor,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    auto l = pq.pop();
    auto const curr = l.get_node();
    auto const it = costs.find(curr.get_key());
    auto const curr_cost =
        it != end(costs) ? it->second.cost(curr) : kInfeasible;
    if (curr_cost < l.cost()) {
      return node::invalid();
    }
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    P::template adjacent<SearchDir, adj_conf::kUseCH | Config>(
        params, r, curr, nullptr, sharing, elevations, &sc_stor,
        sc_stor.order_.node2order_[curr.get_node()],
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool const track, node, cost_t) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }

          auto const total = l.cost() + cost;
          if (total >= max) {
            if (SearchDir == direction::kForward) {
              max_reached_s_ = true;
            } else {
              max_reached_t_ = true;
            }
            return;
          }
          if (total < max &&
              costs[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr, way)) {
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
    return curr;
  }

  template <direction SearchDir, adj_conf Config>
  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           shortcut_storage<node> const& sc_stor) {
    settled_s_.resize(w.r_->node_ways_.size());
    settled_t_.resize(w.r_->node_ways_.size());
    while (!pq_s_.empty() || !pq_t_.empty()) {
      auto const min_pq_s =
          pq_s_.empty() ? kInfeasible : pq_s_.get_next_bucket();
      auto const min_pq_t =
          pq_t_.empty() ? kInfeasible : pq_t_.get_next_bucket();
      bool const use_pq_s = min_pq_s <= min_pq_t;
      node curr;
      if (use_pq_s) {
        curr = run_single<SearchDir, Config>(
            params, w, r, max, sharing, elevations, sc_stor, pq_s_, cost_s_);
      } else {
        curr = run_single<opposite(SearchDir), Config>(
            params, w, r, max, sharing, elevations, sc_stor, pq_t_, cost_t_);
      }
      if (curr == node::invalid()) {
        continue;
      }
      auto& cur_pq = use_pq_s ? pq_s_ : pq_t_;
      auto& cur_cost = use_pq_s ? cost_s_ : cost_t_;
      auto const& other_cost = use_pq_s ? cost_t_ : cost_s_;
      auto& cur_settled = use_pq_s ? settled_s_ : settled_t_;
      auto const& other_settled = use_pq_s ? settled_t_ : settled_s_;

      cur_settled[curr.get_node().v_] = true;
      if (other_settled[curr.get_node().v_]) {
        other_cost.at(curr.get_key())
            .for_each(curr.get_key(), [&](node const node, cost_t const cost) {
              if (use_pq_s) {
                evaluate_meetpoint<SearchDir>(
                    cur_cost[curr.get_key()].cost(curr), cost, curr, node, w);
              } else {
                evaluate_meetpoint<SearchDir>(
                    cost, cur_cost[curr.get_key()].cost(curr), node, curr, w);
              }
            });
      }

      if (tentative_cost_ != kInfeasible && !cur_pq.empty() &&
          cur_pq.get_next_bucket() > tentative_cost_) {
        if (kDebug) {
          std::cout << "stopping criterion met " << cur_pq.get_next_bucket()
                    << " " << tentative_cost_ << std::endl;
        }
        cur_pq.clear();
      }
    }
    if (tentative_cost_ != kInfeasible && tentative_cost_ >= max) {
      tentative_cost_ = kInfeasible;
      tentative_mp_ = {node::invalid(), node::invalid()};
      return false;
    }
    return !max_reached_s_ || !max_reached_t_;
  }

  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           shortcut_storage<node> const& sc_stor,
           direction const dir) {
    return dir == direction::kForward
               ? run<direction::kForward, adj_conf::kNone>(
                     params, w, r, max, sharing, elevations, sc_stor)
               : run<direction::kBackward, adj_conf::kNone>(
                     params, w, r, max, sharing, elevations, sc_stor);
  }

  dial<label, get_bucket> pq_s_{get_bucket{}};
  dial<label, get_bucket> pq_t_{get_bucket{}};
  cost_map cost_s_;
  cost_map cost_t_;
  std::vector<bool> settled_s_;
  std::vector<bool> settled_t_;
  cost_t tentative_cost_{kInfeasible};
  std::pair<node, node> tentative_mp_;
  bool max_reached_s_ = false;
  bool max_reached_t_ = false;
};

}  // namespace osr
