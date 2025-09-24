#pragma once

#include <chrono>
#include <algorithm>
#include <ranges>
#include <vector>

#include "fmt/std.h"
#include "utl/enumerate.h"

#include "osr/preprocessing/contraction_hierarchy/node_order.h"
#include "osr/preprocessing/contraction_hierarchy/shortcut_storage.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <typename Node>
struct incoming_sc_candidate {
  Node via_;
  Node in_;
  way_idx_t way_;
  cost_t way_cost_;
};

template <typename Node>
struct outgoing_sc_candidate {
  Node out_;
  Node via_;
  way_idx_t way_;
  cost_t way_cost_;
  cost_t cost_;
  cost_t turning_cost_;
};

template <typename Node>
struct found_shortcut {
  shortcut_storage<Node>::shortcut shortcut_;
  shortcut_storage<Node>::shortcut_intern shortcut_intern_;
  cost_t cost_;
};

template <Profile P>
std::vector<incoming_sc_candidate<typename P::node>> find_incoming(
    typename P::parameters const& params,
    ways::routing const& r,
    shortcut_storage<typename P::node> const& sc_stor,
    node_idx_t const via_idx) {
  auto incoming = std::vector<incoming_sc_candidate<typename P::node>>{};
  P::resolve_all(r, via_idx, kNoLevel, [&](typename P::node const u) {
    P::template adjacent<direction::kBackward, adj_conf::kUseCH>(
        params, r, u, nullptr, nullptr, nullptr, &sc_stor,
        sc_stor.order_.node2order_[via_idx],
        [&](typename P::node const neighbor, std::uint32_t const cost,
            distance_t, way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool, typename P::node const via_out,
            cost_t const turning_cost) {
          cost_t const way_cost =
              cost - P::node_cost(r.node_properties_[neighbor.get_node()]) -
              turning_cost;
          for (auto& in : incoming) {
            if (in.in_ == neighbor && in.way_ == way) {
              if (in.way_cost_ <= way_cost) {
                return;
              }
              in.via_ = via_out;
              in.in_ = neighbor;
              in.way_cost_ = way_cost;
              return;
            }
          }
          incoming.push_back(incoming_sc_candidate<typename P::node>{
              via_out, neighbor, way, way_cost});
        });
  });
  return incoming;
}

template <Profile P>
std::vector<outgoing_sc_candidate<typename P::node>> find_outgoing(
    typename P::parameters const& params,
    ways::routing const& r,
    shortcut_storage<typename P::node> const& sc_stor,
    typename P::node const via_node,
    cost_t& max_cost,
    cost_t& max_turning_cost) {
  auto outgoing = std::vector<outgoing_sc_candidate<typename P::node>>{};
  P::template adjacent<direction::kForward,
                       adj_conf::kUseCH | adj_conf::kNoOmitRestricted>(
      params, r, via_node, nullptr, nullptr, nullptr, &sc_stor,
      sc_stor.order_.node2order_[via_node.get_node()],
      [&](typename P::node const neighbor, std::uint32_t const cost, distance_t,
          way_idx_t const way, std::uint16_t, std::uint16_t,
          elevation_storage::elevation, bool, typename P::node const via_out,
          cost_t const turning_cost) {
        cost_t const out_cost =
            static_cast<cost_t>(cost) -
            P::node_cost(r.node_properties_[neighbor.get_node()]);
        outgoing.push_back(outgoing_sc_candidate<typename P::node>{
            neighbor, via_out, way,
            static_cast<cost_t>(
                cost - P::node_cost(r.node_properties_[neighbor.get_node()])),
            out_cost, turning_cost});
        if (cost > max_cost) {
          max_cost = static_cast<cost_t>(cost);
        }
        max_turning_cost = std::max(max_turning_cost, turning_cost);
      });
  return outgoing;
}

template <Profile P>
void apply_outgoing_turn_replacement_difference(
    typename P::parameters const& params,
    ways::routing const& r,
    shortcut_storage<typename P::node> const& sc_stor,
    node_idx_t const via_order,
    typename P::node const in_node,
    cost_t const cost_limit,
    dijkstra<P>& d) {
  P::template adjacent<direction::kForward,
                       adj_conf::kUseCH | adj_conf::kNoOmitRestricted>(
      params, r, in_node, nullptr, nullptr, nullptr, &sc_stor, via_order,
      [&](typename P::node const neighbor, std::uint32_t const cost, distance_t,
          way_idx_t const way, std::uint16_t, std::uint16_t,
          elevation_storage::elevation, bool, typename P::node const start_node,
          cost_t) {
        int32_t r_max = 0;
        P::template adjacent<direction::kBackward, adj_conf::kNoOmitRestricted>(
            params, r, start_node, nullptr, nullptr, nullptr, nullptr,
            node_idx_t::invalid(),
            [&](typename P::node, std::uint32_t, distance_t, way_idx_t,
                std::uint16_t, std::uint16_t, elevation_storage::elevation,
                bool, typename P::node const from_node,
                cost_t const turning_cost) {
              r_max = std::max(
                  r_max, static_cast<int32_t>(turning_cost) -
                             P::template turning_cost<direction::kForward>(
                                 r, from_node, in_node));
            });
        r_max = std::min(r_max, static_cast<int32_t>(cost_limit));
        cost_t const total = r_max + cost;
        if (total >= cost_limit) {
          return;
        }
        d.cost_[neighbor.get_key()].update({neighbor, total}, neighbor, total,
                                           P::node::invalid(), way);
        d.pq_.push(typename P::label{neighbor, total});
      });
}

template <Profile P>
bool check_incoming_turn_replacement_difference(
    typename P::parameters const& params,
    ways::routing const& r,
    outgoing_sc_candidate<typename P::node> const& out,
    dijkstra<P> const& d,
    cost_t const sc_cost) {
  bool add_sc = true;
  P::template adjacent<direction::kBackward, adj_conf::kNoOmitRestricted>(
      params, r, out.out_, nullptr, nullptr, nullptr, nullptr,
      node_idx_t::invalid(),
      [&](typename P::node, std::uint32_t, distance_t, way_idx_t, std::uint16_t,
          std::uint16_t, elevation_storage::elevation, bool,
          typename P::node const dest_node, cost_t) {
        if (!add_sc) {
          return;
        }
        int32_t r_max = 0;
        P::template adjacent<direction::kForward, adj_conf::kNoOmitRestricted>(
            params, r, dest_node, nullptr, nullptr, nullptr, nullptr,
            node_idx_t::invalid(),
            [&](typename P::node, std::uint32_t, distance_t, way_idx_t,
                std::uint16_t, std::uint16_t, elevation_storage::elevation,
                bool, typename P::node const to_node,
                cost_t const turning_cost) {
              if (!add_sc) {
                return;
              }
              r_max = std::max(
                  r_max, static_cast<int32_t>(turning_cost) -
                             P::template turning_cost<direction::kForward>(
                                 r, out.out_, to_node));
            });
        add_sc = r_max + d.get_cost(dest_node) >= sc_cost;
      });
  return add_sc;
}

template <Profile P>
bool check_loop_avoidance(typename P::parameters const& params,
                          ways::routing const& r,
                          incoming_sc_candidate<typename P::node> const& in,
                          outgoing_sc_candidate<typename P::node> const& out,
                          cost_t const sc_cost) {
  bool add_sc = false;
  P::template adjacent<direction::kBackward, adj_conf::kNoOmitRestricted>(
      params, r, in.in_, nullptr, nullptr, nullptr, nullptr,
      node_idx_t::invalid(),
      [&](typename P::node, std::uint32_t, distance_t, way_idx_t, std::uint16_t,
          std::uint16_t, elevation_storage::elevation, bool,
          typename P::node const from_node, cost_t const ct_from) {
        if (add_sc) {
          return;
        }
        P::template adjacent<direction::kForward, adj_conf::kNoOmitRestricted>(
            params, r, out.out_, nullptr, nullptr, nullptr, nullptr,
            node_idx_t::invalid(),
            [&](typename P::node, std::uint32_t, distance_t, way_idx_t,
                std::uint16_t, std::uint16_t, elevation_storage::elevation,
                bool, typename P::node const to_node, cost_t const ct_to) {
              if (add_sc) {
                return;
              }
              add_sc = ct_from + sc_cost + ct_to <
                       P::template turning_cost<direction::kForward>(
                           r, from_node, to_node);
            });
      });
  return add_sc;
}

template <Profile P>
bool check_path(incoming_sc_candidate<typename P::node> const& in,
                outgoing_sc_candidate<typename P::node>& out,
                dijkstra<P> const& d) {
  auto node = out.out_;
  auto path = std::vector<path_label<typename P::node>>{};
  while (true) {
    auto const& e = d.cost_.at(node.get_key());
    auto const pred = e.pred(node);
    auto const cost = e.cost(node);
    auto const way = e.way(node);
    path.emplace_back(node, cost, way);
    if (!pred.has_value()) {
      break;
    }
    node = *pred;
  }
  if (path.size() < 2 || path.back() != path_label<typename P::node>{
                                            in.via_, in.way_cost_, in.way_}) {
    return false;
  }
  if (path.size() > 2) {
    auto const& l_begin = path.back();
    auto const& l_end = path[1];
    if (l_begin.node_.get_node() != l_end.node_.get_node()) {
      return false;
    }
    out.turning_cost_ = l_end.cost_ - l_begin.cost_;
  } else {
    out.turning_cost_ = 0;
  }
  return true;
}

template <Profile P>
void handle_outgoing_edge(
    typename P::parameters const& params,
    ways::routing const& r,
    cost_t const via_cost,
    incoming_sc_candidate<typename P::node> const& in,
    outgoing_sc_candidate<typename P::node>& out,
    dijkstra<P> const& d,
    std::vector<found_shortcut<typename P::node>>& found_SCs) {
  auto const sc_cost = d.get_cost(out.out_);
  if (sc_cost == kInfeasible ||
      d.cost_.at(out.out_.get_key()).way(out.out_) != out.way_ ||
      out.turning_cost_ == 0 &&
          sc_cost < in.way_cost_ + via_cost + out.cost_ +
                        P::node_cost(r.node_properties_[out.out_.get_node()]) ||
      !check_incoming_turn_replacement_difference(params, r, out, d, sc_cost) ||
      in.in_.get_node() == out.out_.get_node() &&
          !check_loop_avoidance<P>(params, r, in, out, sc_cost) ||
      !check_path(in, out, d)) {
    return;
  }
  found_SCs.push_back(found_shortcut<typename P::node>{
      {in.in_, out.out_},
      {in.via_, out.via_, in.way_, out.way_, in.way_cost_, out.way_cost_,
       out.turning_cost_},
      static_cast<cost_t>(
          sc_cost - P::node_cost(r.node_properties_[out.out_.get_node()]))});
}

template <Profile P>
std::vector<found_shortcut<typename P::node>> handle_incoming_edge(
    typename P::parameters const& params,
    ways const& w,
    ways::routing const& r,
    shortcut_storage<typename P::node> const& sc_stor,
    node_idx_t const via_order,
    cost_t const via_cost,
    incoming_sc_candidate<typename P::node>& in,
    cost_t const detour_limit) {
  cost_t max_cost = 0;
  cost_t max_turning_cost = 0;
  auto outgoing =
      find_outgoing<P>(params, r, sc_stor, in.via_, max_cost, max_turning_cost);
  if (max_turning_cost == kInfeasible) {
    max_turning_cost = detour_limit;
  }

  dijkstra<P>& d = get_dijkstra<P>();
  cost_t const cost_limit =
      in.way_cost_ + via_cost + max_cost + max_turning_cost + 1;
  d.reset(cost_limit);

  apply_outgoing_turn_replacement_difference(params, r, sc_stor, via_order,
                                             in.in_, cost_limit, d);
  d.template run<direction::kForward, adj_conf::kUseCH>(
      params, w, r, cost_limit, nullptr, nullptr, nullptr, &sc_stor, via_order);

  auto found_SCs = std::vector<found_shortcut<typename P::node>>{};
  for (auto& out : outgoing) {
    handle_outgoing_edge(params, r, via_cost, in, out, d, found_SCs);
  }
  return found_SCs;
}

template <Profile P>
void contract_node(typename P::parameters const& params,
                   ways const& w,
                   ways::routing const& r,
                   shortcut_storage<typename P::node>& sc_stor,
                   node_idx_t const via_idx,
                   cost_t const detour_limit) {
  node_properties const via_props = r.node_properties_[via_idx];
  node_idx_t const via_order = sc_stor.order_.node2order_[via_idx];
  cost_t const via_cost = P::node_cost(via_props);
  if (via_cost == kInfeasible) {
    return;
  }
  for (auto& in : find_incoming<P>(params, r, sc_stor, via_idx)) {
    auto new_idx = way_idx_t{w.n_ways() + sc_stor.shortcuts_.size()};
    for (auto const& [sc, sci, cost] : handle_incoming_edge<P>(
             params, w, r, sc_stor, via_order, via_cost, in, detour_limit)) {
      auto const i = sc.from_.get_node();
      auto const o = sc.to_.get_node();
      sc_stor.neighbors_fwd_[i].push_back({o, new_idx});
      sc_stor.neighbors_bwd_[o].push_back({i, new_idx});
      sc_stor.shortcuts_.emplace_back(sc);
      sc_stor.shortcuts_intern_.emplace_back(sci);
      sc_stor.costs_.emplace_back(cost);
      ++new_idx;
    }
  }
}

template <Profile P>
void preprocess_ch(std::filesystem::path const& directory,
                   search_profile const profile,
                   typename P::parameters const& params,
                   auto const seed,
                   cost_t const detour_limit,
                   node_order::method const ordering_method) {
  static constexpr auto kProgressUpdateInterval =
      std::chrono::milliseconds(500);
  auto const start = std::chrono::steady_clock::now();
  auto pt = utl::get_active_progress_tracker_or_activate("osr-preprocessing");

  auto sc_stor = shortcut_storage<typename P::node>{};
  auto& ordering = sc_stor.order_;
  auto const w = ways{directory, cista::mmap::protection::READ};
  auto const& r = *w.r_;

  auto const num_nodes = w.n_nodes();
  auto num_edges = way_idx_t::value_t{0};
  auto const num_restricted = r.node_is_restricted_.count();
  for (auto way : r.way_nodes_) {
    if (way.empty()) continue;
    num_edges += way.size() - 1;
  }
  fmt::println(
      "#nodes (#restricted) | #edges:\n"
      "{} ({}) | {}\n",
      num_nodes, num_restricted, num_edges);

  sc_stor.shortcuts_.reserve(2 * num_edges);
  sc_stor.shortcuts_intern_.reserve(2 * num_edges);
  sc_stor.costs_.reserve(2 * num_edges);
  sc_stor.neighbors_fwd_.resize(num_nodes);
  sc_stor.neighbors_bwd_.resize(num_nodes);

  fmt::print("ordering nodes...");
  ordering.initialize(num_nodes);
  ordering.shuffle_random(seed);
  if (ordering_method == node_order::method::kRealImportance) {
    ordering.by_real_world_importance(r);
  }
  fmt::println(" finished");

  fmt::println("contracting nodes...");
  auto constexpr kOffsetFactor = 4U;  // treat (kOffset*10) % as 0 %
  auto update_progress = [&](size_t const val) {
    pt->status(std::to_string(val) + "/" + std::to_string(num_nodes));
    auto const offset = kOffsetFactor * num_nodes / 10;
    pt->update(val > offset ? val - offset : 0);
  };
  pt->in_high((10 - kOffsetFactor) * num_nodes / 10);
  update_progress(0);
  auto timer = std::chrono::steady_clock::now();
  for (auto const [order, via_idx] : utl::enumerate(ordering.order2node_)) {
    contract_node<P>(params, w, r, sc_stor, via_idx, detour_limit);
    if (std::chrono::steady_clock::now() - timer >= kProgressUpdateInterval) {
      timer = std::chrono::steady_clock::now();
      update_progress(order);
    }
  }
  update_progress(num_nodes);

  auto const stop = std::chrono::steady_clock::now();
  fmt::println("\nfinished with {} #shortcuts", sc_stor.shortcuts_.size());
  fmt::println("preprocessing took {:.0%-M:%S} (mm:ss)", stop - start);

  sc_stor.write(directory, profile);
}

}  // namespace osr
