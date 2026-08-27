#include "osr/ways.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "utl/pairwise.h"
#include "utl/parallel_for.h"

#include "cista/io.h"

#include "osr/routing/profiles/car.h"

// uncomment the following line to enable IFC
#define USE_INERTIAL_FLOW_CUT
#ifdef USE_INERTIAL_FLOW_CUT
#include "inertialflowcutter/run.h"
#endif

namespace osr {

namespace {

template <typename Polyline>
quantized_angle_t get_prev_bearing(Polyline const& polyline,
                                   std::size_t const idx) {
  if (idx == 0U) {
    return 0U;
  }

  auto const origin = polyline[idx].as_latlng();
  for (auto i = idx; i != 0U; --i) {
    auto const candidate = polyline[i - 1U].as_latlng();
    if (origin != candidate) {
      return quantize_angle(geo::bearing(origin, candidate));
    }
  }

  return 0U;
}

template <typename Polyline>
quantized_angle_t get_next_bearing(Polyline const& polyline,
                                   std::size_t const idx) {
  if (idx + 1U >= polyline.size()) {
    return 0U;
  }

  auto const origin = polyline[idx].as_latlng();
  for (auto i = static_cast<std::size_t>(idx) + 1U; i < polyline.size(); ++i) {
    auto const candidate = polyline[i].as_latlng();
    if (origin != candidate) {
      return quantize_angle(geo::bearing(origin, candidate));
    }
  }

  return 0U;
}

}  // namespace

ways::ways(std::filesystem::path p, cista::mmap::protection const mode)
    : p_{std::move(p)},
      mode_{mode},
      r_{mode == cista::mmap::protection::READ
             ? routing::read(p_)
             : cista::wrapped<routing>{cista::raw::make_unique<routing>()}},
      node_to_osm_{mm("node_to_osm.bin")},
      way_osm_idx_{mm("way_osm_idx.bin")},
      way_polylines_{mm_vec<point>{mm("way_polylines_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_polylines_index.bin")}},
      way_osm_nodes_{mm_vec<osm_node_idx_t>{mm("way_osm_nodes_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_osm_nodes_index.bin")}},
      strings_{mm_vec<char>(mm("strings_data.bin")),
               mm_vec<std::uint64_t>(mm("strings_idx.bin"))},
      way_names_{mm("way_names.bin")},
      way_has_conditional_access_no_{
          mm_vec<std::uint64_t>(mm("way_has_conditional_access_no"))},
      way_conditional_access_no_{mm("way_conditional_access_no")} {}

void ways::build_components_and_importance() {
  r_->node_importance_.resize(n_nodes());
  auto q = hash_set<way_idx_t>{};
  auto flood_fill = [&](way_idx_t const way_idx, component_idx_t const c) {
    assert(q.empty());
    q.insert(way_idx);
    while (!q.empty()) {
      auto const next = *q.begin();
      q.erase(q.begin());
      for (auto const n : r_->way_nodes_[next]) {
        r_->node_importance_[n] =
            std::max(r_->node_importance_[n],
                     static_cast<std::uint32_t>(r_->way_importance_[next]));
        for (auto const w : r_->node_ways_[n]) {
          auto& wc = r_->way_component_[w];
          if (wc == component_idx_t::invalid()) {
            wc = c;
            q.insert(w);
          }
        }
      }
    }
  };

  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Build components and importance")
      .in_high(n_ways())
      .out_bounds(75, 90);

  auto next_component_idx = component_idx_t{0U};
  r_->way_component_.resize(n_ways(), component_idx_t::invalid());
  for (auto i = 0U; i != n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto& c = r_->way_component_[way_idx];
    if (c != component_idx_t::invalid()) {
      continue;
    }
    c = next_component_idx++;
    flood_fill(way_idx, c);
    pt->increment();
  }
  r_->way_importance_.clear();

#ifdef USE_INERTIAL_FLOW_CUT
  pt->status("Run inertial flow cutter").in_high(n_ways()).out_bounds(90, 91);

  auto v_tail = std::vector<unsigned>{};
  auto v_head = std::vector<unsigned>{};

  for (auto const nodes : r_->way_nodes_) {
    for (auto const [a, b] : utl::pairwise(nodes)) {
      v_tail.push_back(a.v_);
      v_head.push_back(b.v_);
    }
  }
  auto const thread_count = 4;
  ifc::run_inertial_flow_cutter(
      thread_count, static_cast<int>(n_nodes()), v_head, v_tail,
      [&](int i) {
        auto const p = get_node_pos(node_idx_t{i});
        return std::pair<double, double>{p.lng(), p.lat()};
      },
      [&](int node_idx, int level) {
        r_->node_importance_[node_idx_t{node_idx}] =
            static_cast<std::uint32_t>(level);
      });
#endif
}

void ways::add_restriction(std::vector<resolved_restriction>& rs) {
  using it_t = std::vector<resolved_restriction>::iterator;
  utl::sort(rs, [](auto&& a, auto&& b) { return a.via_ < b.via_; });
  utl::equal_ranges_linear(
      begin(rs), end(rs), [](auto&& a, auto&& b) { return a.via_ == b.via_; },
      [&](it_t const& lb, it_t const& ub) {
        auto const range = std::span{lb, ub};
        r_->node_restrictions_.resize(to_idx(range.front().via_) + 1U);
        r_->node_is_restricted_.set(range.front().via_, true);

        for (auto const& x : range) {
          if (x.type_ == resolved_restriction::type::kNo) {
            r_->node_restrictions_[x.via_].push_back(
                restriction{r_->get_way_pos(x.via_, x.from_),
                            r_->get_way_pos(x.via_, x.to_), x.applies_to_bus_});
          } else /* kOnly */ {
            for (auto const [i, from] :
                 utl::enumerate(r_->node_ways_[x.via_])) {
              for (auto const [j, to] :
                   utl::enumerate(r_->node_ways_[x.via_])) {
                if (x.from_ == from && x.to_ != to) {
                  r_->node_restrictions_[x.via_].push_back(restriction{
                      static_cast<way_pos_t>(i), static_cast<way_pos_t>(j),
                      x.applies_to_bus_});
                }
              }
            }
          }
        }
      });
  r_->node_restrictions_.resize(node_to_osm_.size());
}

void ways::compute_big_street_neighbors() {
  struct state {
    hash_set<way_idx_t> done_;
  };

  auto pt = utl::get_active_progress_tracker();

  auto is_orig_big_street = std::vector<bool>(n_ways());
  for (auto const [i, p] : utl::enumerate(r_->way_properties_)) {
    is_orig_big_street[i] = p.is_big_street();
  }

  utl::parallel_for_run_threadlocal<state>(
      n_ways(), [&](state& s, std::size_t const i) {
        auto const way = way_idx_t{i};

        if (is_orig_big_street[to_idx(way)]) {
          pt->update_monotonic(i);
          return;
        }

        s.done_.clear();

        auto const expand = [&](way_idx_t const x, bool const go_further,
                                auto&& recurse) {
          for (auto const& n : r_->way_nodes_[x]) {
            for (auto const& w : r_->node_ways_[n]) {
              if (is_orig_big_street[to_idx(w)]) {
                r_->way_properties_[way].is_big_street_ = true;
                return true;
              }

              if (s.done_.emplace(w).second && go_further) {
                if (recurse(x, false, recurse)) {
                  return true;
                }
              }
            }
          }
          return false;
        };

        s.done_.emplace(way);
        expand(way, true, expand);
        pt->update_monotonic(i);
      });
}

void ways::add_shortcuts() {
  struct neighbor {
    node_idx_t node_{};
    distance_t distance_{};
  };

  // A profile contributes a base edge to the undirected CCH topology when
  // both endpoint nodes are usable and at least one travel direction is
  // feasible. Instantiate this predicate for multiple profiles to build a
  // shared topology from their union.
  auto profile_base_edge_accessible = [&]<typename Profile>(
                                          node_idx_t const from,
                                          node_idx_t const to,
                                          way_idx_t const way,
                                          std::uint16_t const from_idx,
                                          std::uint16_t const to_idx,
                                          distance_t const distance) {
    auto const params = typename Profile::parameters{};
    if (Profile::node_cost(params, r_->node_properties_[from]) == kInfeasible ||
        Profile::node_cost(params, r_->node_properties_[to]) == kInfeasible) {
      return false;
    }

    auto const dir =
        from_idx < to_idx ? direction::kForward : direction::kBackward;
    auto const& properties = r_->way_properties_[way];
    return Profile::way_cost(params, properties, dir, distance) != kInfeasible ||
           Profile::way_cost(params, properties, opposite(dir), distance) !=
               kInfeasible;
  };

  auto topology_base_edge_accessible = [&]<typename... Profiles>(
                                           node_idx_t const from,
                                           node_idx_t const to,
                                           way_idx_t const way,
                                           std::uint16_t const from_idx,
                                           std::uint16_t const to_idx,
                                           distance_t const distance) {
    return (profile_base_edge_accessible.template operator()<Profiles>(
                from, to, way, from_idx, to_idx, distance) ||
            ...);
  };

  // Visit only original graph neighbors which belong to the selected profile
  // topology. Returning true from fn stops the traversal early.
  auto for_each_topology_neighbor = [&]<typename... Profiles>(
                                        node_idx_t const from, auto&& fn) {
    for (auto const [way, from_idx] :
         utl::zip(r_->node_ways_[from], r_->node_in_way_idx_[from])) {
      auto const nodes = r_->way_nodes_[way];
      auto visit = [&](std::uint16_t const to_idx,
                       std::uint16_t const distance_idx) {
        auto const to = nodes[to_idx];
        auto const distance = r_->get_way_node_distance(way, distance_idx);
        return topology_base_edge_accessible.template operator()<Profiles...>(
                   from, to, way, from_idx, to_idx, distance) &&
               fn(to, distance);
      };
      if (from_idx != 0U && visit(from_idx - 1U, from_idx - 1U)) {
        return true;
      }
      if (from_idx + 1U < nodes.size() && visit(from_idx + 1U, from_idx)) {
        return true;
      }
    }
    return false;
  };

  // Construct one shared topology from the union of both road profiles.
  auto for_each_selected_topology_neighbor = [&](node_idx_t const from,
                                                 auto&& fn) {
    return for_each_topology_neighbor.template operator()<car, bus>(
        from, std::forward<decltype(fn)>(fn));
  };

  // Existing Car graph edges do not need an additional shortcut entry.
  auto direct_edge_exists = [&](node_idx_t const from,
                                node_idx_t const to) -> bool {
    return for_each_selected_topology_neighbor(
        from, [&](node_idx_t const neighbor, distance_t const) {
          return neighbor == to;
        });
  };

  // Keep one shortcut per ordered node pair and retain the shortest distance.
  auto add_or_update_shortcut = [&](node_idx_t const from, node_idx_t const to,
                                    node_idx_t const via,
                                    distance_t const distance) {
    for (auto& s : r_->shortcuts_[from]) {
      if (s.to_ == to) {
        if (distance < s.distance_) {
          s.via_ = via;
          s.distance_ = distance;
        }
        return;
      }
    }
    r_->shortcuts_[from].push_back(
        shortcut{.to_ = to, .via_ = via, .distance_ = distance});
  };

  r_->shortcuts_.clear();
  r_->shortcuts_.resize(n_nodes());

  // Build the contraction order from the unique ranks assigned during
  // preprocessing. CCH ranks must be a permutation of [0, n_nodes).
  auto rank_to_node = std::vector<node_idx_t>(n_nodes(), node_idx_t::invalid());
  for (auto i = node_idx_t{0U}; i != n_nodes(); ++i) {
    auto const rank = r_->node_importance_[i];
    utl::verify(rank < n_nodes(), "CCH rank out of bounds: {} >= {}", rank,
                n_nodes());
    utl::verify(rank_to_node[rank] == node_idx_t::invalid(),
                "duplicate CCH rank: {}", rank);
    rank_to_node[rank] = i;
  }

  for (auto rank = std::uint32_t{0U}; rank != n_nodes(); ++rank) {
    auto const i = rank_to_node[rank];
    utl::verify(i != node_idx_t::invalid(), "missing CCH rank: {}", rank);

    auto higher_neighbors = std::vector<neighbor>{};
    // Candidate endpoints are only nodes with a higher rank than i.
    auto add_neighbor = [&](node_idx_t const node, distance_t const distance) {
      if (node == i || r_->node_importance_[node] <= rank) {
        return;
      }
      for (auto& n : higher_neighbors) {
        if (n.node_ == node) {
          n.distance_ = std::min(n.distance_, distance);
          return;
        }
      }
      higher_neighbors.push_back(neighbor{node, distance});
    };

    for_each_selected_topology_neighbor(
        i, [&](node_idx_t const node, distance_t const distance) {
          add_neighbor(node, distance);
          return false;
        });

    // Previously created shortcuts may also make higher-ranked neighbors.
    for (auto const& s : r_->shortcuts_[i]) {
      add_neighbor(s.to_, s.distance_);
    }

    // Connect every pair of higher-ranked neighbors through the contracted
    // node i unless a direct original graph edge already exists.
    for (auto from = std::size_t{0U}; from < higher_neighbors.size(); ++from) {
      for (auto to = from + 1U; to < higher_neighbors.size(); ++to) {
        auto const a = higher_neighbors[from];
        auto const b = higher_neighbors[to];
        if (direct_edge_exists(a.node_, b.node_)) {
          continue;
        }

        auto const sum = static_cast<std::uint64_t>(a.distance_) +
                         static_cast<std::uint64_t>(b.distance_);
        auto const distance = static_cast<distance_t>(
            std::min(sum, static_cast<std::uint64_t>(
                              std::numeric_limits<distance_t>::max())));
        add_or_update_shortcut(a.node_, b.node_, i, distance);
        add_or_update_shortcut(b.node_, a.node_, i, distance);
      }
    }
  }
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("created shortcuts");
  auto shortcut_count = std::size_t{0U};
  for (auto const shortcuts : r_->shortcuts_) {
    shortcut_count += shortcuts.size();
  }
  fmt::println("shortcuts: {}", shortcut_count);

  // Basic Customization POC
  auto customize = [&]<bool IsBus>(auto& cch_edge_weights) {
    using profile = generic_car<IsBus>;

    struct customization_edge {
      node_idx_t to_{};
      std::vector<cch_edge_weight> weights_{};
    };

    auto const params = typename profile::parameters{};

    auto lower_rank = [&](node_idx_t const a, node_idx_t const b) {
      return r_->node_importance_[a] < r_->node_importance_[b] ? a : b;
    };

    auto higher_rank = [&](node_idx_t const a, node_idx_t const b) {
      return r_->node_importance_[a] < r_->node_importance_[b] ? b : a;
    };

    auto find_cch_edge = [&](node_idx_t const low,
                             node_idx_t const high) -> cch_edge* {
      for (auto& e : cch_edge_weights[low]) {
        if (e.to_ == high) {
          return &e;
        }
      }
      return nullptr;
    };

    auto ensure_cch_edge = [&](node_idx_t const low,
                               node_idx_t const high) -> cch_edge& {
      if (auto* e = find_cch_edge(low, high); e != nullptr) {
        return *e;
      }
      cch_edge_weights[low].push_back(cch_edge{.to_ = high});
      return cch_edge_weights[low].back();
    };

    auto same_boundary = [](cch_edge_weight const& a,
                            cch_edge_weight const& b) {
      return a.up_ == b.up_ && a.from_way_ == b.from_way_ &&
             a.from_dir_ == b.from_dir_ && a.to_way_ == b.to_way_ &&
             a.to_dir_ == b.to_dir_;
    };

    auto relax_value = [&](cch_edge& e, cch_edge_weight const candidate) {
      for (auto& w : e.weights_) {
        if (!same_boundary(w, candidate)) {
          continue;
        }
        if (candidate.cost_ < w.cost_ ||
            (candidate.cost_ == w.cost_ && candidate.distance_ < w.distance_)) {
          w = candidate;
        }
        return;
      }
      e.weights_.push_back(candidate);
    };

    auto add_base_edge = [&](node_idx_t const from,
                             node_idx_t const to,
                             way_idx_t const way,
                             std::uint16_t const from_idx,
                             std::uint16_t const to_idx) {
      auto const distance =
          r_->get_way_node_distance(way, std::min(from_idx, to_idx));
      if (!profile_base_edge_accessible.template operator()<profile>(
              from, to, way, from_idx, to_idx, distance)) {
        return;
      }
      auto& e = ensure_cch_edge(lower_rank(from, to), higher_rank(from, to));

      auto const target_node_cost =
          profile::node_cost(params, r_->node_properties_[to]);
      if (target_node_cost == kInfeasible) {
        return;
      }
      auto const way_dir =
          from_idx < to_idx ? direction::kForward : direction::kBackward;
      auto const way_cost =
          profile::way_cost(params, r_->way_properties_[way], way_dir, distance);
      if (way_cost == kInfeasible) {
        return;
      }

      auto const cost =
          clamp_cost(static_cast<std::uint64_t>(way_cost) + target_node_cost);
      auto const from_way = r_->get_way_pos(from, way, from_idx);
      auto const to_way = r_->get_way_pos(to, way, to_idx);
      if (r_->node_importance_[from] < r_->node_importance_[to]) {
        relax_value(e, cch_edge_weight{.via_ = node_idx_t::invalid(),
                                       .distance_ = distance,
                                       .cost_ = cost,
                                       .from_way_ = from_way,
                                       .to_way_ = to_way,
                                       .from_dir_ = way_dir,
                                       .to_dir_ = way_dir,
                                       .up_ = true});
      } else {
        relax_value(e, cch_edge_weight{.via_ = node_idx_t::invalid(),
                                       .distance_ = distance,
                                       .cost_ = cost,
                                       .from_way_ = from_way,
                                       .to_way_ = to_way,
                                       .from_dir_ = way_dir,
                                       .to_dir_ = way_dir,
                                       .up_ = false});
      }
    };

    cch_edge_weights.clear();
    cch_edge_weights.resize(n_nodes());

    for (auto from = node_idx_t{0U}; from != n_nodes(); ++from) {
      for (auto const [way, node_in_way_idx] :
           utl::zip(r_->node_ways_[from], r_->node_in_way_idx_[from])) {
        auto const nodes = r_->way_nodes_[way];
        if (node_in_way_idx != 0U) {
          add_base_edge(from, nodes[node_in_way_idx - 1U], way, node_in_way_idx,
                        node_in_way_idx - 1U);
        }
        if (node_in_way_idx + 1U < nodes.size()) {
          add_base_edge(from, nodes[node_in_way_idx + 1U], way, node_in_way_idx,
                        node_in_way_idx + 1U);
        }
      }

      for (auto const& s : r_->shortcuts_[from]) {
        ensure_cch_edge(lower_rank(from, s.to_), higher_rank(from, s.to_));
      }
    }

    auto collect_upward_edges = [&](node_idx_t const from) {
      auto edges = std::vector<customization_edge>{};
      for (auto const& e : cch_edge_weights[from]) {
        if (e.weights_.empty()) {
          continue;
        }
        auto& edge = edges.emplace_back();
        edge.to_ = e.to_;
        edge.weights_.assign(begin(e.weights_), end(e.weights_));
      }
      std::sort(begin(edges), end(edges), [&](auto const& a, auto const& b) {
        return r_->node_importance_[a.to_] < r_->node_importance_[b.to_];
      });
      return edges;
    };

    auto combine_distance = [](distance_t const a, distance_t const b) {
      auto const sum = static_cast<std::uint64_t>(a) + static_cast<std::uint64_t>(b);
      return static_cast<distance_t>(
          std::min(sum, static_cast<std::uint64_t>(
                            std::numeric_limits<distance_t>::max())));
    };

    auto get_turn_cost = [&](node_idx_t const via,
                             way_pos_t const from_way,
                             direction const from_dir,
                             way_pos_t const to_way,
                             direction const to_dir) {
      if (r_->is_restricted<direction::kForward, IsBus>(via, from_way, to_way)) {
        return kInfeasible;
      }
      auto const is_u_turn = from_way == to_way && to_dir == opposite(from_dir);
      auto cost =
          is_u_turn ? cost_t{0U}
                    : profile::turn_cost(params, r_->get_turn_angle(
                                                     via, from_way, from_dir,
                                                     to_way, to_dir));
      if (is_u_turn) {
        cost = clamp_cost(static_cast<std::uint64_t>(cost) +
                          params.uturn_penalty_);
      }
      return cost;
    };

    auto customize_edge = [&](node_idx_t const u,
                              customization_edge const& v_edge,
                              customization_edge const& w_edge) {
      auto* vw = find_cch_edge(v_edge.to_, w_edge.to_);
      if (vw == nullptr) {
        return;
      }

      for (auto const& vu : v_edge.weights_) {
        if (vu.up_) {
          continue;
        }
        for (auto const& uw : w_edge.weights_) {
          if (!uw.up_) {
            continue;
          }
          auto const turn_cost = get_turn_cost(
              u, vu.to_way_, vu.to_dir_, uw.from_way_, uw.from_dir_);
          if (turn_cost == kInfeasible) {
            continue;
          }
          relax_value(
              *vw, cch_edge_weight{
                       .via_ = u,
                       .distance_ = combine_distance(vu.distance_, uw.distance_),
                       .cost_ = clamp_cost(static_cast<std::uint64_t>(vu.cost_) +
                                           static_cast<std::uint64_t>(turn_cost) +
                                           static_cast<std::uint64_t>(uw.cost_)),
                       .from_way_ = vu.from_way_,
                       .to_way_ = uw.to_way_,
                       .via_in_way_ = vu.to_way_,
                       .via_out_way_ = uw.from_way_,
                       .from_dir_ = vu.from_dir_,
                       .to_dir_ = uw.to_dir_,
                       .via_in_dir_ = vu.to_dir_,
                       .via_out_dir_ = uw.from_dir_,
                       .up_ = true});
        }
      }
      for (auto const& wu : w_edge.weights_) {
        if (wu.up_) {
          continue;
        }
        for (auto const& uv : v_edge.weights_) {
          if (!uv.up_) {
            continue;
          }
          auto const turn_cost = get_turn_cost(
              u, wu.to_way_, wu.to_dir_, uv.from_way_, uv.from_dir_);
          if (turn_cost == kInfeasible) {
            continue;
          }
          relax_value(
              *vw, cch_edge_weight{
                       .via_ = u,
                       .distance_ = combine_distance(wu.distance_, uv.distance_),
                       .cost_ = clamp_cost(static_cast<std::uint64_t>(wu.cost_) +
                                           static_cast<std::uint64_t>(turn_cost) +
                                           static_cast<std::uint64_t>(uv.cost_)),
                       .from_way_ = wu.from_way_,
                       .to_way_ = uv.to_way_,
                       .via_in_way_ = wu.to_way_,
                       .via_out_way_ = uv.from_way_,
                       .from_dir_ = wu.from_dir_,
                       .to_dir_ = uv.to_dir_,
                       .via_in_dir_ = wu.to_dir_,
                       .via_out_dir_ = uv.from_dir_,
                       .up_ = false});
        }
      }
    };

    for (auto rank = std::uint32_t{0U}; rank != n_nodes(); ++rank) {
      auto const x = rank_to_node[rank];
      utl::verify(x != node_idx_t::invalid(), "missing CCH rank: {}", rank);

      auto const upward_edges = collect_upward_edges(x);
      for (auto v = std::size_t{0U}; v < upward_edges.size(); ++v) {
        for (auto w = v + 1U; w < upward_edges.size(); ++w) {
          customize_edge(x, upward_edges[v], upward_edges[w]);
        }
      }
    }
    auto cch_edge_count = std::size_t{0U};
    auto cch_weight_count = std::size_t{0U};
    for (auto const& edges : cch_edge_weights) {
      cch_edge_count += edges.size();
      for (auto const& edge : edges) {
        cch_weight_count += edge.weights_.size();
      }
    }
    fmt::println("{} cch edges: {}, weights: {}", IsBus ? "bus" : "car",
                 cch_edge_count, cch_weight_count);
  };

  customize.template operator()<false>(r_->cch_car_edge_weights_);
  customize.template operator()<true>(r_->cch_bus_edge_weights_);
}

void ways::connect_ways() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  {  // Assign graph node ids to every node with >1 way.
    pt->status("Create graph nodes")
        .in_high(node_way_counter_.size())
        .out_bounds(40, 50);

    auto node_idx = node_idx_t{0U};
    node_way_counter_.multi_.for_each_set_bit([&](std::uint64_t const b_idx) {
      auto const i = osm_node_idx_t{b_idx};
      node_to_osm_.push_back(i);
      ++node_idx;
      pt->update(b_idx);
    });
    r_->node_is_restricted_.resize(to_idx(node_idx));
  }

  // Build edges.
  {
    pt->status("Connect ways")
        .in_high(way_osm_nodes_.size())
        .out_bounds(50, 75);
    auto node_ways = mm_paged_vecvec<node_idx_t, way_idx_t>{
        cista::paged<mm_vec32<way_idx_t>>{
            mm_vec32<way_idx_t>{mm("tmp_node_ways_data.bin")}},
        mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
            mm("tmp_node_ways_index.bin")}};
    auto node_in_way_idx = mm_paged_vecvec<node_idx_t, std::uint16_t>{
        cista::paged<mm_vec32<std::uint16_t>>{
            mm_vec32<std::uint16_t>{mm("tmp_node_in_way_idx_data.bin")}},
        mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
            mm("tmp_node_in_way_idx_index.bin")}};
    node_ways.resize(node_to_osm_.size());
    node_in_way_idx.resize(node_to_osm_.size());
    for (auto const [osm_way_idx, osm_nodes, polyline] :
         utl::zip(way_osm_idx_, way_osm_nodes_, way_polylines_)) {
      auto pred_pos = std::make_optional<point>();
      auto from = node_idx_t::invalid();
      auto distance = 0.0;
      auto i = std::uint16_t{0U};
      auto way_idx = way_idx_t{r_->way_nodes_.size()};
      auto dists = r_->way_node_dist_.add_back_sized(0U);
      auto nodes = r_->way_nodes_.add_back_sized(0U);
      for (auto const [osm_node_idx, pos] : utl::zip(osm_nodes, polyline)) {
        if (pred_pos.has_value()) {
          distance += geo::distance(pos, *pred_pos);
        }

        if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
          auto const to = get_node_idx(osm_node_idx);
          node_ways[to].push_back(way_idx);
          node_in_way_idx[to].push_back(i);
          nodes.push_back(to);

          if (from != node_idx_t::invalid()) {
            auto const dist = static_cast<distance_t>(std::round(distance));
            if (dist < std::numeric_limits<std::uint16_t>::max()) {
              dists.push_back(static_cast<std::uint16_t>(dist));
            } else {
              r_->long_way_node_dist_.push_back(routing::long_distance{
                  .way_ = way_idx,
                  .node_ = static_cast<std::uint16_t>(i - 1U),
                  .distance_ = dist});
              dists.push_back(std::numeric_limits<std::uint16_t>::max());
            }
          }

          distance = 0.0;
          from = to;

          if (i == std::numeric_limits<std::uint16_t>::max()) {
            fmt::println("error: way with {} nodes", osm_way_idx);
          }

          ++i;
        }

        pred_pos = pos;
      }
      pt->increment();
    }

    std::sort(begin(r_->long_way_node_dist_), end(r_->long_way_node_dist_));

    for (auto const x : node_ways) {
      r_->node_ways_.emplace_back(x);
    }
    for (auto const x : node_in_way_idx) {
      r_->node_in_way_idx_.emplace_back(x);
    }
  }

  compute_turn_bearings();

  auto e = std::error_code{};
  std::filesystem::remove(p_ / "tmp_node_ways_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_ways_index.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_index.bin", e);
}

std::size_t ways::get_polyline_node_idx(
    way_idx_t const way, std::uint16_t const target_routing_idx) const {
  auto const& routing_nodes = r_->way_nodes_[way];
  auto const& polyline_osm_nodes = way_osm_nodes_[way];

  auto current_routing_idx = 0U;
  for (auto const [poly_idx, osm_node] : utl::enumerate(polyline_osm_nodes)) {
    auto const expected_osm_node =
        node_to_osm_[routing_nodes[current_routing_idx]];
    if (osm_node == expected_osm_node) {
      if (current_routing_idx == target_routing_idx) {
        return poly_idx;
      }
      ++current_routing_idx;
    }
  }

  throw utl::fail("polyline node index not found: way={} idx={}", to_idx(way),
                  target_routing_idx);
}

void ways::compute_turn_bearings() {
  for (auto i = node_idx_t{0U}; i != n_nodes(); ++i) {
    auto bearings = r_->node_turn_bearings_.add_back_sized(0U);
    for (auto const [way, node_in_way_idx] :
         utl::zip(r_->node_ways_[i], r_->node_in_way_idx_[i])) {
      auto const polyline = way_polylines_[way];
      auto const polyline_idx = get_polyline_node_idx(way, node_in_way_idx);
      bearings.push_back(
          turn_bearing{.to_prev_ = get_prev_bearing(polyline, polyline_idx),
                       .to_next_ = get_next_bearing(polyline, polyline_idx)});
    }
  }
}

void ways::sync() {
  node_to_osm_.mmap_.sync();
  way_osm_idx_.mmap_.sync();
  way_polylines_.data_.mmap_.sync();
  way_polylines_.bucket_starts_.mmap_.sync();
  way_osm_nodes_.data_.mmap_.sync();
  way_osm_nodes_.bucket_starts_.mmap_.sync();
  strings_.data_.mmap_.sync();
  strings_.bucket_starts_.mmap_.sync();
  way_names_.mmap_.sync();
}

std::optional<std::string_view> ways::get_access_restriction(
    way_idx_t const way) const {
  if (!way_has_conditional_access_no_.test(way)) {
    return std::nullopt;
  }
  auto const it = std::lower_bound(
      begin(way_conditional_access_no_), end(way_conditional_access_no_), way,
      [](auto&& a, auto&& b) { return a.first < b; });
  utl::verify(
      it != end(way_conditional_access_no_) && it->first == way,
      "access restriction for way with access restriction not found way={}",
      way_osm_idx_[way]);
  return strings_[it->second].view();
}

cista::wrapped<ways::routing> ways::routing::read(
    std::filesystem::path const& p) {
  return cista::read<ways::routing>(p / "routing.bin");
}

void ways::routing::write(std::filesystem::path const& p) const {
  return cista::write(p / "routing.bin", *this);
}

}  // namespace osr
