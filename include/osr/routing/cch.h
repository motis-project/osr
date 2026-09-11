#pragma once

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <vector>

#include "fmt/core.h"

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <Profile P, bool EarlyTermination = false>
struct cch {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;

  struct settled_hash {
    using is_avalanching = void;

    auto operator()(node const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;

      auto h = hash{}(n.get_key());
      if constexpr (requires { n.way_; }) {
        h = wyhash::mix(h,
                        wyhash::hash(static_cast<std::uint64_t>(n.way_)));
      }
      if constexpr (requires { n.dir_; }) {
        h = wyhash::mix(
            h, wyhash::hash(n.dir_ == direction::kForward ? 0ULL : 1ULL));
      }
      return h;
    }
  };

  using settled_set = ankerl::unordered_dense::set<node, settled_hash>;

  static constexpr auto const kDebug = false;
  static constexpr auto const kQueryDebugOutput = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(cost_t const max) {
    pqForward_.clear();
    pqForward_.n_buckets(max + 1U);
    pqBackward_.clear();
    pqBackward_.n_buckets(max + 1U);
    costForward_.clear();
    costBackward_.clear();
    settledForward_.clear();
    settledBackward_.clear();
    mu_ = kInfeasible;
    meet_forward_ = node::invalid();
    meet_backward_ = node::invalid();
    max_reached_ = false;
  }

  void add_start(ways const& w, label const l) {
    if (costForward_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                    node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pqForward_.n_buckets(),
                  "cch::add_start: label cost exceeds max: {} >= {}", l.cost(),
                  pqForward_.n_buckets());
      pqForward_.push(l);
    }
  }

  void add_destination(ways const& w, label const l) {
    if (costBackward_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                     node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "DESTINATION ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pqBackward_.n_buckets(),
                  "cch::add_destination: label cost exceeds max: {} >= {}",
                  l.cost(), pqBackward_.n_buckets());
      pqBackward_.push(l);
    }
  }

  template <direction Dir>
  cost_t get_cost(node const n) const {
    if constexpr (Dir == direction::kForward) {
      auto const it = costForward_.find(n.get_key());
      return it != end(costForward_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = costBackward_.find(n.get_key());
      return it != end(costBackward_) ? it->second.cost(n) : kInfeasible;
    }
  }

  cost_t meeting_turn_cost(P::parameters const& params,
                           ways::routing const& r,
                           node const incoming,
                           node const outgoing) const {
    if constexpr (!uses_customized_cost_overlay() ||
                  !requires { incoming.way_; incoming.dir_; outgoing.way_;
                               outgoing.dir_; }) {
      return 0U;
    } else {
      if (incoming.get_node() != outgoing.get_node()) {
        return kInfeasible;
      }

      auto const backward_it = costBackward_.find(outgoing.get_key());
      if (backward_it == end(costBackward_)) {
        return kInfeasible;
      }

      // In the backward predecessor chain, pred(outgoing) is the next graph
      // state after the meeting node in the final forward route. If there is no
      // predecessor, the meeting node is the destination seed and there is no
      // outgoing turn to validate.
      if (!backward_it->second.pred(outgoing).has_value()) {
        return 0U;
      }

      // The backward meeting state stores the outgoing way context at the shared
      // graph node. Validate the stitch from the forward arrival context into
      // that outgoing context before accepting this meeting pair.
      if (r.template is_restricted<direction::kForward, is_bus_profile()>(
              incoming.get_node(), incoming.way_, outgoing.way_)) {
        return kInfeasible;
      }

      auto const is_u_turn = incoming.way_ == outgoing.way_ &&
                             outgoing.dir_ == opposite(incoming.dir_);
      return is_u_turn ? params.uturn_penalty_
                       : P::turn_cost(
                             params, r.get_turn_angle(
                                         incoming.get_node(), incoming.way_,
                                         incoming.dir_, outgoing.way_,
                                         outgoing.dir_));
    }
  }

  void select_meet_by_node(P::parameters const& params,
                           ways::routing const& r) {
    if constexpr (!uses_customized_cost_overlay()) {
      return;
    } else {
      auto best = kInfeasible;
      auto best_forward = node::invalid();
      auto best_backward = node::invalid();

      for (auto const& [node_id, forward_entry] : costForward_) {
        auto const backward_it = costBackward_.find(node_id);
        if (backward_it == end(costBackward_)) {
          continue;
        }

        P::resolve_all(r, node_id, kNoLevel, [&](auto const forward_state) {
          auto const f = forward_entry.cost(forward_state);
          if (f == kInfeasible) {
            return;
          }

          P::resolve_all(r, node_id, kNoLevel, [&](auto const backward_state) {
            auto const b = backward_it->second.cost(backward_state);
            if (b == kInfeasible) {
              return;
            }

            auto const stitch_cost =
                meeting_turn_cost(params, r, forward_state, backward_state);
            if (stitch_cost == kInfeasible) {
              return;
            }

            auto const candidate = clamp_cost(
                static_cast<std::uint64_t>(f) + static_cast<std::uint64_t>(b) +
                static_cast<std::uint64_t>(stitch_cost));
            if (candidate < best) {
              best = candidate;
              best_forward = forward_state;
              best_backward = backward_state;
            }
          });
        });
      }

      if (best != kInfeasible) {
        mu_ = best;
        meet_forward_ = best_forward;
        meet_backward_ = best_backward;
        if constexpr (kQueryDebugOutput) {
          if constexpr (requires { best_forward.way_; best_forward.dir_;
                                    best_backward.way_; best_backward.dir_; }) {
            fmt::println(
                "cch meet | node={} cost={} forward_state=({}, {}) "
                "backward_state=({}, {})",
                to_idx(best_forward.get_node()), mu_, best_forward.way_,
                to_str(best_forward.dir_), best_backward.way_,
                to_str(best_backward.dir_));
          } else {
            fmt::println("cch meet | node={} cost={}",
                         to_idx(best_forward.get_node()), mu_);
          }
        }
      }
    }
  }

  template <direction Dir>
  bool settle(node const n) {
    if constexpr (Dir == direction::kForward) {
      if (!settledForward_.insert(n).second) {
        return false;
      }
    } else {
      if (!settledBackward_.insert(n).second) {
        return false;
      }
    }
    return true;
  }

  template <direction Dir>
  bool is_stale(label const& l) const {
    return get_cost<Dir>(l.get_node()) < l.cost();
  }

  template <direction Dir>
  void discard_stale_top(dial<label, get_bucket>& pq) {
    while (!pq.empty()) {
      auto const& candidate = pq.buckets_[pq.get_next_bucket()].back();
      if (!is_stale<Dir>(candidate)) {
        return;
      }
      pq.pop();
    }
  }

  bool is_upward(ways::routing const& r, node const from, node const to) const {
    return r.node_importance_[to.get_node()] >
           r.node_importance_[from.get_node()];
  }

  static constexpr bool uses_customized_cost_overlay() {
    if constexpr (requires { P::node::get_mode(); }) {
      return P::node::get_mode() == mode::kCar;
    } else {
      return false;
    }
  }

  static constexpr bool is_bus_profile() {
    if constexpr (requires { P::kIsBus; }) {
      return P::kIsBus;
    } else {
      return false;
    }
  }

  static auto const& customized_edges(ways::routing const& r) {
    if constexpr (is_bus_profile()) {
      return r.cch_bus_edge_weights_;
    } else {
      return r.cch_car_edge_weights_;
    }
  }

  static cch_edge const* customized_edge(ways::routing const& r,
                                         node_idx_t const from,
                                         node_idx_t const to) {
    for (auto const& e : customized_edges(r)[from]) {
      if (e.to_ == to) {
        return &e;
      }
    }
    return nullptr;
  }

  static cch_edge_weight const* best_weight(cch_edge const& e,
                                            bool const up,
                                            node const from,
                                            node const to) {
    auto best = static_cast<cch_edge_weight const*>(nullptr);
    for (auto const& w : e.weights_) {
      if (w.up_ != up || w.cost_ == kInfeasible) {
        continue;
      }
      if constexpr (requires { from.way_; from.dir_; to.way_; to.dir_; }) {
        if (w.from_way_ != from.way_ || w.from_dir_ != from.dir_ ||
            w.to_way_ != to.way_ || w.to_dir_ != to.dir_) {
          continue;
        }
      }
      if (best == nullptr || w.cost_ < best->cost_ ||
          (w.cost_ == best->cost_ && w.distance_ < best->distance_)) {
        best = &w;
      }
    }
    return best;
  }

  // Profiles without customized edge weights derive their shortcut cost from
  // distance.
  static cost_t shortcut_cost(P::parameters const& params,
                              distance_t const distance) {
    if constexpr (requires { params.speed_meters_per_second_; }) {
      return static_cast<cost_t>(
          std::round(distance / params.speed_meters_per_second_));
    } else {
      return static_cast<cost_t>(distance);
    }
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    while (!pqForward_.empty() || !pqBackward_.empty()) {
      discard_stale_top<direction::kForward>(pqForward_);
      discard_stale_top<direction::kBackward>(pqBackward_);
      if (pqForward_.empty() && pqBackward_.empty()) {
        break;
      }

      auto const forward =
          pqBackward_.empty() ||
          (!pqForward_.empty() &&
           pqForward_.get_next_bucket() <= pqBackward_.get_next_bucket());

      auto l = forward ? pqForward_.pop() : pqBackward_.pop();
      auto const curr = l.get_node();

      if (forward) {
        if (is_stale<direction::kForward>(l)) {
          continue;
        }
        if (!settle<direction::kForward>(curr)) {
          continue;
        }
      } else {
        if (is_stale<direction::kBackward>(l)) {
          continue;
        }
        if (!settle<direction::kBackward>(curr)) {
          continue;
        }
      }

      if constexpr (kDebug) {
        std::cout << "EXTRACT ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }

      auto relax_neighbor = [&](node const neighbor, std::uint32_t const cost,
                                distance_t, way_idx_t const way, std::uint16_t,
                                std::uint16_t, elevation_storage::elevation,
                                bool const track) {
        if constexpr (kDebug) {
          std::cout << "  NEIGHBOR ";
          neighbor.print(std::cout, w);
        }

        // Compact CCH queries relax upward edges and state-changing self-loops.
        auto const is_state_loop =
            uses_customized_cost_overlay() &&
            curr.get_node() == neighbor.get_node() && curr != neighbor;
        if (!is_upward(r, curr, neighbor) && !is_state_loop) {
          return;
        }

        auto const total = static_cast<std::uint64_t>(l.cost()) + cost;
        if (total >= max) {
          max_reached_ = true;
          return;
        }
        auto const debug_meet_node =
            kQueryDebugOutput &&
            w.node_to_osm_[neighbor.get_node()] == osm_node_idx_t{1800775440U};
        auto const dump_debug_meet_node_states = [&]() {
          if (!debug_meet_node) {
            return;
          }
          fmt::println("  cch meet probe states at node/{}",
                       to_idx(w.node_to_osm_[neighbor.get_node()]));
          P::resolve_all(r, neighbor.get_node(), kNoLevel, [&](auto const state) {
            auto const f = get_cost<direction::kForward>(state);
            auto const b = get_cost<direction::kBackward>(state);
            if (f == kInfeasible && b == kInfeasible) {
              return;
            }
            if constexpr (requires { state.way_; state.dir_; }) {
              fmt::println("    state=({}, {}) forward={} backward={}",
                           state.way_, to_str(state.dir_), f, b);
            } else {
              fmt::println("    state forward={} backward={}", f, b);
            }
          });
        };
        if (forward) {
          auto const total_cost = static_cast<cost_t>(total);
          auto const improved =
              costForward_[neighbor.get_key()].update(l, neighbor, total_cost,
                                                      curr);
          if (debug_meet_node) {
            if constexpr (requires { curr.way_; curr.dir_; neighbor.way_;
                                      neighbor.dir_; }) {
              fmt::println(
                  "cch meet probe | search=forward curr={} rank={} state=({}, "
                  "{}) neighbor={} rank={} state=({}, {}) edge_cost={} "
                  "candidate_total={} improved={} exact_forward={} "
                  "exact_backward={} mu={}",
                  to_idx(w.node_to_osm_[curr.get_node()]),
                  r.node_importance_[curr.get_node()], curr.way_,
                  to_str(curr.dir_),
                  to_idx(w.node_to_osm_[neighbor.get_node()]),
                  r.node_importance_[neighbor.get_node()], neighbor.way_,
                  to_str(neighbor.dir_), cost, total_cost, improved,
                  get_cost<direction::kForward>(neighbor),
                  get_cost<direction::kBackward>(neighbor), mu_);
            } else {
              fmt::println(
                  "cch meet probe | search=forward curr={} rank={} "
                  "neighbor={} rank={} edge_cost={} candidate_total={} "
                  "improved={} exact_forward={} exact_backward={} mu={}",
                  to_idx(w.node_to_osm_[curr.get_node()]),
                  r.node_importance_[curr.get_node()],
                  to_idx(w.node_to_osm_[neighbor.get_node()]),
                  r.node_importance_[neighbor.get_node()], cost, total_cost,
                  improved, get_cost<direction::kForward>(neighbor),
                  get_cost<direction::kBackward>(neighbor), mu_);
            }
            dump_debug_meet_node_states();
          }
          if (improved) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node(), track);
            pqForward_.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        } else {
          auto const total_cost = static_cast<cost_t>(total);
          auto const improved =
              costBackward_[neighbor.get_key()].update(l, neighbor, total_cost,
                                                       curr);
          if (debug_meet_node) {
            if constexpr (requires { curr.way_; curr.dir_; neighbor.way_;
                                      neighbor.dir_; }) {
              fmt::println(
                  "cch meet probe | search=backward curr={} rank={} state=({}, "
                  "{}) neighbor={} rank={} state=({}, {}) edge_cost={} "
                  "candidate_total={} improved={} exact_forward={} "
                  "exact_backward={} mu={}",
                  to_idx(w.node_to_osm_[curr.get_node()]),
                  r.node_importance_[curr.get_node()], curr.way_,
                  to_str(curr.dir_),
                  to_idx(w.node_to_osm_[neighbor.get_node()]),
                  r.node_importance_[neighbor.get_node()], neighbor.way_,
                  to_str(neighbor.dir_), cost, total_cost, improved,
                  get_cost<direction::kForward>(neighbor),
                  get_cost<direction::kBackward>(neighbor), mu_);
            } else {
              fmt::println(
                  "cch meet probe | search=backward curr={} rank={} "
                  "neighbor={} rank={} edge_cost={} candidate_total={} "
                  "improved={} exact_forward={} exact_backward={} mu={}",
                  to_idx(w.node_to_osm_[curr.get_node()]),
                  r.node_importance_[curr.get_node()],
                  to_idx(w.node_to_osm_[neighbor.get_node()]),
                  r.node_importance_[neighbor.get_node()], cost, total_cost,
                  improved, get_cost<direction::kForward>(neighbor),
                  get_cost<direction::kBackward>(neighbor), mu_);
            }
            dump_debug_meet_node_states();
          }
          if (improved) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node(), track);
            pqBackward_.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        }
      };

      auto relax_shortcut = [&](shortcut const& s) {
        // Shortcuts are stored outside the profile graph, so add them as extra
        // CCH edges after normal profile adjacency has been expanded.
        auto const neighbor =
            P::create_node(s.to_, kNoLevel, way_pos_t{0U}, SearchDir);
        if (!is_upward(r, curr, neighbor)) {
          return;
        }
        if constexpr (WithBlocked) {
          if (blocked->test(s.to_)) {
            return;
          }
        }
        relax_neighbor(neighbor, shortcut_cost(params, s.distance_),
                       s.distance_, way_idx_t::invalid(), 0U, 0U,
                       elevation_storage::elevation{}, false);
      };

      if constexpr (uses_customized_cost_overlay()) {
        for (auto const& e : customized_edges(r)[curr.get_node()]) {
          for (auto const& weight : e.weights_) {
            auto const debug_node =
                kQueryDebugOutput &&
                w.node_to_osm_[curr.get_node()] == osm_node_idx_t{1866422978U};
            auto const debug_edge = kQueryDebugOutput &&
                                    curr.get_node() == node_idx_t{1985U} &&
                                    e.to_ == node_idx_t{1537U};
            if (debug_node) {
              fmt::println(
                  "cch adjacent probe | search={} curr={} rank={} state=({}, "
                  "{}) edge_to={} edge_rank={} weight_up={} base_cost={} "
                  "dist={} from=({}, {}) to=({}, {}) via={} via_in=({}, {}) "
                  "via_out=({}, {})",
                  forward ? "forward" : "backward",
                  to_idx(w.node_to_osm_[curr.get_node()]),
                  r.node_importance_[curr.get_node()], curr.way_,
                  to_str(curr.dir_), to_idx(w.node_to_osm_[e.to_]),
                  r.node_importance_[e.to_], weight.up_, weight.cost_,
                  weight.distance_, weight.from_way_,
                  to_str(weight.from_dir_), weight.to_way_,
                  to_str(weight.to_dir_),
                  weight.via_ == node_idx_t::invalid()
                      ? 0U
                      : to_idx(w.node_to_osm_[weight.via_]),
                  weight.via_in_way_, to_str(weight.via_in_dir_),
                  weight.via_out_way_, to_str(weight.via_out_dir_));
            }
            if (debug_edge) {
              fmt::println(
                  "cch edge probe | search={} curr={} rank={} cost_so_far={} "
                  "state=({}, {}) edge_to={} edge_rank={} weight_up={} "
                  "base_cost={} dist={} from=({}, {}) to=({}, {}) via={} "
                  "via_in=({}, {}) via_out=({}, {})",
                  forward ? "forward" : "backward",
                  to_idx(w.node_to_osm_[curr.get_node()]),
                  r.node_importance_[curr.get_node()], l.cost(), curr.way_,
                  to_str(curr.dir_), to_idx(w.node_to_osm_[e.to_]),
                  r.node_importance_[e.to_], weight.up_, weight.cost_,
                  weight.distance_, weight.from_way_, to_str(weight.from_dir_),
                  weight.to_way_, to_str(weight.to_dir_),
                  weight.via_ == node_idx_t::invalid()
                      ? 0U
                      : to_idx(w.node_to_osm_[weight.via_]),
                  weight.via_in_way_, to_str(weight.via_in_dir_),
                  weight.via_out_way_, to_str(weight.via_out_dir_));
            }
            if (weight.up_ != forward || weight.cost_ == kInfeasible) {
              if (debug_node || debug_edge) {
                fmt::println("  -> skip: direction/infeasible");
              }
              continue;
            }
            auto edge_cost = weight.cost_;
            auto const edge_source_way =
                forward ? weight.from_way_ : weight.to_way_;
            auto const edge_source_dir =
                forward ? weight.from_dir_ : weight.to_dir_;
            auto const is_u_turn =
                forward ? (curr.way_ == edge_source_way &&
                           edge_source_dir == opposite(curr.dir_))
                        : (curr.way_ == edge_source_way &&
                           curr.dir_ == opposite(edge_source_dir));
            if (forward) {
              if (r.template is_restricted<direction::kForward,
                                           is_bus_profile()>(
                      curr.get_node(), curr.way_, edge_source_way)) {
                if (debug_node || debug_edge) {
                  fmt::println("  -> skip: restricted");
                }
                continue;
              }
            } else {
              if (r.template is_restricted<direction::kForward,
                                           is_bus_profile()>(
                      curr.get_node(), edge_source_way, curr.way_)) {
                if (debug_node || debug_edge) {
                  fmt::println("  -> skip: restricted");
                }
                continue;
              }
            }
            auto const turn_cost =
                is_u_turn
                    ? params.uturn_penalty_
                    : P::turn_cost(params, forward
                                               ? r.get_turn_angle(
                                                     curr.get_node(), curr.way_,
                                                     curr.dir_, edge_source_way,
                                                     edge_source_dir)
                                               : r.get_turn_angle(
                                                     curr.get_node(),
                                                     edge_source_way,
                                                     edge_source_dir, curr.way_,
                                                     curr.dir_));
            edge_cost = clamp_cost(static_cast<std::uint64_t>(edge_cost) +
                                   static_cast<std::uint64_t>(turn_cost));
            auto const neighbor =
                P::create_node(e.to_, kNoLevel,
                               forward ? weight.to_way_ : weight.from_way_,
                               forward ? weight.to_dir_ : weight.from_dir_);
            if constexpr (WithBlocked) {
              if (blocked->test(e.to_)) {
                if (debug_node || debug_edge) {
                  fmt::println("  -> skip: blocked");
                }
                continue;
              }
            }
            if (debug_node || debug_edge) {
              fmt::println("  -> relax neighbor={} state=({}, {}) total_edge_cost={}",
                           to_idx(w.node_to_osm_[neighbor.get_node()]),
                           neighbor.way_, to_str(neighbor.dir_), edge_cost);
            }
            relax_neighbor(neighbor, edge_cost, weight.distance_,
                           way_idx_t::invalid(), 0U, 0U,
                           elevation_storage::elevation{}, false);
          }
        }
      } else {
        if (forward) {
          P::template adjacent<SearchDir, WithBlocked>(
              params, r, curr, blocked, sharing, elevations, relax_neighbor);
        } else {
          P::template adjacent<opposite(SearchDir), WithBlocked>(
              params, r, curr, blocked, sharing, elevations, relax_neighbor);
        }
        for (auto const& s : r.shortcuts_[curr.get_node()]) {
          relax_shortcut(s);
        }
      }
    }
    select_meet_by_node(params, r);
    return !max_reached_ && mu_ == kInfeasible;
  }

  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
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

  dial<label, get_bucket> pqForward_{get_bucket{}};
  dial<label, get_bucket> pqBackward_{get_bucket{}};

  ankerl::unordered_dense::map<key, entry, hash> costForward_;
  ankerl::unordered_dense::map<key, entry, hash> costBackward_;
  bool max_reached_{};

  settled_set settledForward_;
  settled_set settledBackward_;
  cost_t mu_{kInfeasible};
  node meet_forward_{node::invalid()};
  node meet_backward_{node::invalid()};

  // for early termination
  std::vector<node> destinations_;
  std::size_t remaining_destinations_{0U};
  cost_t early_termination_max_cost_{kInfeasible};
  bool terminated_early_max_cost_{false};
};

}  // namespace osr
