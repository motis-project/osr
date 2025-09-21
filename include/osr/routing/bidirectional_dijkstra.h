#pragma once
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/preprocessing/ch_preprocessing.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <Profile P>
struct bidirectional_dijkstra {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;
  using ch_preprocessor_t = ch_preprocessor<P>;

  constexpr static auto const kDebug = false;

  bool use_ch_preprocessing_ = false;
  ch_preprocessor_t const* ch_pre_ = nullptr;

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
    radius_ = max;
    max_reached_1_ = false;
    max_reached_2_ = false;
  }

  void add_start(ways const& w, label const l, sharing_data const*) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << " starting " << l.get_node().n_ << std::endl;
    }
    if (l.cost() < pq1_.n_buckets() - 1 &&
        cost1_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      pq1_.push(l);
    }
  }

  void add_end(ways const& w, label const l, sharing_data const*) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << " ending " << l.get_node().n_ << std::endl;
    }
    if (l.cost() < pq2_.n_buckets() - 1 &&
        cost2_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      pq2_.push(l);
    }
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
    return f_cost + b_cost;
  }

  template <direction SearchDir, bool WithBlocked, direction PathDir>
  bool run_single(typename P::parameters const& params,
                  ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    if (pq.empty()) {
      return false;
    }

    auto const is_fwd = PathDir == direction::kForward;
    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);

    if (curr_cost < l.cost()) {
      return true;
    }

    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      curr.print(std::cout, w);
      std::cout << " cost: " << l.cost() << "\n";
    }

    if (l.cost() >= max) {
      if (is_fwd) {
        max_reached_1_ = true;
      } else {
        max_reached_2_ = true;
      }
      return false;
    }

    auto const opposite_cost_map = is_fwd ? &cost2_ : &cost1_;
    auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
    if (opposite_candidate != end(*opposite_cost_map)) {
      cost_t best_other = kInfeasible;
      node best_other_state = node::invalid();
      P::resolve_all(r, curr.get_node(), level_t{0.0F}, [&](node const alt) {
        auto const oc = opposite_candidate->second.cost(alt);
        if (oc != kInfeasible && oc < best_other) {
          best_other = oc;
          best_other_state = alt;
        }
      });
      if (best_other != kInfeasible) {
        auto const tentative_cost = curr_cost + best_other;
        if (tentative_cost < best_cost_) {
          if (is_fwd) {
            meet_point_1_ = curr;
            meet_point_2_ = best_other_state;
          } else {
            meet_point_1_ = best_other_state;
            meet_point_2_ = curr;
          }
          best_cost_ = static_cast<cost_t>(tentative_cost);

          if constexpr (kDebug) {
            std::cout << "  MEETING POINT FOUND! Cost: " << best_cost_ << "\n";
          }
        }
      }
    }

    if (use_ch_preprocessing_ && ch_pre_ != nullptr) {
      auto neighbors = ch_pre_->get_neighbors(
          params, r, curr, blocked, sharing, elevations,
          SearchDir == direction::kForward ? direction::kForward
                                           : direction::kBackward);
      for (auto const& neighbor : neighbors) {
        cost_t edge_cost = ch_pre_->get_cost(params, curr, neighbor, r, blocked,
                                             sharing, elevations);
        if (edge_cost >= kInfeasible) continue;

        auto const total = curr_cost + edge_cost;
        if (total >= max) {
          if (is_fwd) {
            max_reached_1_ = true;
          } else {
            max_reached_2_ = true;
          }
          continue;
        }

        if (total < max && costs[neighbor.get_key()].update(
                               l, neighbor, static_cast<cost_t>(total), curr)) {
          auto next = label{neighbor, static_cast<cost_t>(total)};
          next.track(l, r, way_idx_t{}, neighbor.get_node(), false);
          pq.push(std::move(next));

          if constexpr (kDebug) {
            std::cout << " -> PUSH (CH) neighbor cost: " << total << "\n";
          }
        }
      }
    } else {
      P::template adjacent<SearchDir, WithBlocked>(
          params, r, curr, blocked, sharing, elevations,
          [&](node const neighbor, std::uint32_t const edge_cost, distance_t,
              way_idx_t const way, std::uint16_t, std::uint16_t,
              elevation_storage::elevation const, bool const track) {
            if constexpr (kDebug) {
              std::cout << "  NEIGHBOR ";
              neighbor.print(std::cout, w);
            }

            auto const total = curr_cost + edge_cost;
            if (total >= max) {
              if (is_fwd) {
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
                std::cout << " -> PUSH cost: " << total << "\n";
              }
            } else {
              if constexpr (kDebug) {
                std::cout << " -> DOMINATED\n";
              }
            }
          });
    }

    if (best_cost_ != kInfeasible) {
      auto const top_f =
          pq1_.empty() ? kInfeasible
          : pq1_.buckets_[pq1_.get_next_bucket()].empty()
              ? kInfeasible
              : pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r =
          pq2_.empty() ? kInfeasible
          : pq2_.buckets_[pq2_.get_next_bucket()].empty()
              ? kInfeasible
              : pq2_.buckets_[pq2_.get_next_bucket()].back().cost();

      if (top_f + top_r >= best_cost_) {
        return false;
      }
    }

    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(typename P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    if (radius_ == max) { /* normal case - radius_==max always */
    }
    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty() &&
          !run_single<SearchDir, WithBlocked, direction::kForward>(
              params, w, r, max, blocked, sharing, elevations, pq1_, cost1_)) {
        break;
      }
      if (!pq2_.empty() &&
          !run_single<opposite(SearchDir), WithBlocked, direction::kBackward>(
              params, w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
        break;
      }
    }
    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }
    return !max_reached_1_ || !max_reached_2_;
  }

  bool run(typename P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (use_ch_preprocessing_ && ch_pre_ != nullptr) {
      if (blocked == nullptr) {
        return dir == direction::kForward
                   ? run<direction::kForward, false>(params, w, r, max, blocked,
                                                     sharing, elevations)
                   : run<direction::kBackward, false>(
                         params, w, r, max, blocked, sharing, elevations);
      } else {
        return dir == direction::kForward
                   ? run<direction::kForward, true>(params, w, r, max, blocked,
                                                    sharing, elevations)
                   : run<direction::kBackward, true>(params, w, r, max, blocked,
                                                     sharing, elevations);
      }
    }
    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(params, w, r, max, blocked,
                                                   sharing, elevations)
                 : run<direction::kBackward, false>(params, w, r, max, blocked,
                                                    sharing, elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(params, w, r, max, blocked,
                                                  sharing, elevations)
                 : run<direction::kBackward, true>(params, w, r, max, blocked,
                                                   sharing, elevations);
    }
  }

  explicit bidirectional_dijkstra(bool use_ch_preprocessing = false,
                                  ch_preprocessor_t const* ch_pre = nullptr)
      : use_ch_preprocessing_{use_ch_preprocessing}, ch_pre_{ch_pre} {}

  void set_ch_preprocessor(ch_preprocessor_t const* ch_pre) {
    ch_pre_ = ch_pre;
  }

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_{kInfeasible};
  cost_map cost1_;
  cost_map cost2_;
  cost_t radius_{};
  bool max_reached_1_{false};
  bool max_reached_2_{false};
};

}  // namespace osr
