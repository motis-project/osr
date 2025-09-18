#include "osr/ways.h"

#include <random>
#include <algorithm>
#include <vector>
#include <numeric>

#include "boost/geometry/algorithms/detail/overlay/get_turn_info.hpp"
#include "utl/parallel_for.h"

#include "cista/io.h"
#include "osr/routing/sharing_data.h"
#include "osr/elevation_storage.h"
#include "osr/routing/contraction_hierarchies.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/parameters.h"
#include "osr/types.h"

namespace osr {

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
  auto q = hash_set<way_idx_t>{};
  auto flood_fill = [&](way_idx_t const way_idx, component_idx_t const c) {
    assert(q.empty());
    q.insert(way_idx);
    while (!q.empty()) {
      auto const next = *q.begin();
      q.erase(q.begin());
      for (auto const n : r_->way_nodes_[next]) {
        r_->node_properties_[n].importance_ =
            std::max(r_->node_properties_[n].importance_,
                     r_->way_properties_[next].importance_);
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
                            r_->get_way_pos(x.via_, x.to_)});
          } else /* kOnly */ {
            for (auto const [i, from] :
                 utl::enumerate(r_->node_ways_[x.via_])) {
              for (auto const [j, to] :
                   utl::enumerate(r_->node_ways_[x.via_])) {
                if (x.from_ == from && x.to_ != to) {
                  r_->node_restrictions_[x.via_].push_back(restriction{
                      static_cast<way_pos_t>(i), static_cast<way_pos_t>(j)});
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
            dists.push_back(static_cast<std::uint16_t>(std::round(distance)));
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

    for (auto const x : node_ways) {
      r_->node_ways_.emplace_back(x);
    }
    for (auto const x : node_in_way_idx) {
      r_->node_in_way_idx_.emplace_back(x);
    }
  }

  auto e = std::error_code{};
  std::filesystem::remove(p_ / "tmp_node_ways_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_ways_index.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_index.bin", e);
}

static std::vector<ways::routing::node_identifier> outgoing_neighbors;
void contract_node(ways::routing& r_,
                   std::vector<std::vector<ways::routing::edge_idx_t>>& outgoing_edges,
                   std::vector<std::vector<ways::routing::edge_idx_t>>& incoming_edges,
                   const std::vector<ways::routing::node_identifier>& node_to_level,
                   std::vector<std::pair<cost_t, std::uint64_t>>& index_handler,
                   ways::routing::node_identifier to_contract,
                   std::uint64_t* search_ID
  ) {
  auto const& out_edges = outgoing_edges[to_contract];
  auto const own_level = node_to_level[to_contract];

  if constexpr (ch_config::kUseHeuristic) {
    if (static_cast<double>(own_level) > ch_config::kHeuristicThresholdFactor * static_cast<double>(outgoing_edges.size())) {
      local_1_hop_ch(to_contract, outgoing_edges, incoming_edges, r_);
      return;
    }
  }

  // remove edges adjacent to to_contract
  for (auto outgoing_edge_idx : out_edges) {
    auto const edge = r_.contracted_edges_[outgoing_edge_idx];
    [[maybe_unused]] auto const tmp = std::erase(incoming_edges[edge.to_], outgoing_edge_idx);
    assert(tmp == 1);
  }
  for (auto incoming_edge_idx : incoming_edges[to_contract]) {
    auto const edge = r_.contracted_edges_[incoming_edge_idx];
    [[maybe_unused]] auto const tmp =std::erase(outgoing_edges[edge.from_], incoming_edge_idx);
    assert(tmp == 1);
  }

  outgoing_neighbors.clear();
  cost_t max_outgoing_cost = 0U;

  for (auto const outgoing_edge_idx : out_edges) {
    auto outgoing_edge = r_.contracted_edges_[outgoing_edge_idx];
    auto neighbor_ID = outgoing_edge.to_;
    assert(node_to_level[neighbor_ID] > own_level);
    outgoing_neighbors.push_back(neighbor_ID);
    max_outgoing_cost = std::max(max_outgoing_cost, outgoing_edge.cost_);
  }
  if (outgoing_neighbors.empty()) {
    return;
  }

  for (auto incoming_edge_idx : incoming_edges[to_contract]) {
    auto incoming_edge = r_.contracted_edges_[incoming_edge_idx];
    auto in_neighbor_ID = incoming_edge.from_;
    assert(node_to_level[in_neighbor_ID] > own_level);
    cost_t cutoff = (max_outgoing_cost < kInfeasible - incoming_edge.cost_) ?
                    max_outgoing_cost + incoming_edge.cost_ : kInfeasible - 1; // catch overflows
    dijkstra_ch(in_neighbor_ID, outgoing_neighbors, r_,
      outgoing_edges, node_to_level, cutoff,
      *search_ID, index_handler, own_level);
    // check for obsolete edges
    {
      std::erase_if(outgoing_edges[in_neighbor_ID], [&](auto edge_idx) {
        auto edge = r_.contracted_edges_[edge_idx];
        auto const el = index_handler[edge.to_];
        if (el.second == *search_ID && el.first < edge.cost_) {
          [[maybe_unused]] auto const tmp = std::erase(incoming_edges[edge.to_], edge_idx);
          assert(tmp == 1);
          return true;
        }
        return false;
      });
    }

    for (auto const outgoing_edge_idx : out_edges) {
      auto const outgoing_edge = r_.contracted_edges_[outgoing_edge_idx];
      auto const out_neighbor_ID = outgoing_edge.to_;
      if (incoming_edge.cost_ >= kInfeasible - outgoing_edge.cost_) {
        continue; // catch overflows
      }
      auto const cost = static_cast<cost_t>(incoming_edge.cost_ + outgoing_edge.cost_);
      // determine if a shortcut is necessary
      auto el = index_handler[out_neighbor_ID];
      if (el.second != *search_ID || el.first > cost) {
        // if an edge already exists, replace it
        bool found = false;
        for (auto edge_idx : outgoing_edges[in_neighbor_ID]) {
          if (auto& edge = r_.contracted_edges_[edge_idx];
              edge.to_ == out_neighbor_ID) {
            // assert(edge.cost > cost); // only valid if heuristic is disabled
            if (edge.cost_ > cost) {
              r_.contracted_edges_[edge_idx].cost_ = cost;
              r_.contracted_edges_[edge_idx].contracted_child1_ = incoming_edge_idx;
              r_.contracted_edges_[edge_idx].contracted_child2_ = outgoing_edge_idx;
            }
            found = true;
            break;
          }
        }
        if (!found) {
          auto new_edge = ways::routing::contracted_edge{
            incoming_edge_idx,
            outgoing_edge_idx,
            in_neighbor_ID,
            out_neighbor_ID,
            cost
          };
          r_.contracted_edges_.emplace_back(new_edge);
          auto new_edge_idx = r_.contracted_edges_.size() - 1;
          outgoing_edges[in_neighbor_ID].push_back(new_edge_idx);
          incoming_edges[out_neighbor_ID].push_back(new_edge_idx);
        }
      }
    }
    ++*search_ID;
  }
}

std::uint8_t get_importance(const cista::wrapped<ways::routing>& r_, ways::routing::node_identifier const n) {
  return r_->node_properties_[r_->identifier_to_node_[n].n_].importance_;
}

void ways::build_CH() {
  auto params = std::get<car::parameters>(get_parameters(search_profile::kCar));
  auto const number_of_nodes = r_->node_ways_.data_.size() * 2;
  std::vector<std::vector<routing::edge_idx_t>> outgoing_edges(number_of_nodes), incoming_edges(number_of_nodes);
  std::map<car::node, routing::node_identifier> node_to_id;
  std::size_t current_index = 0;

  for (auto node_idx = node_idx_t{0U}; node_idx < node_to_osm_.size(); ++node_idx) {
    r_->node_idx_to_identifier_.emplace_back(current_index);
    for (way_pos_t way_pos = 0; way_pos < r_->node_ways_[node_idx].size(); ++way_pos) {
      for (auto const dir : {direction::kForward, direction::kBackward}) {
        node_to_id.emplace(car::node{node_idx, way_pos, dir}, current_index);
        r_->identifier_to_node_.emplace_back(routing::node{node_idx, way_pos, dir, current_index});
        current_index++;
      }
    }
  }

  // add existing edges to outgoing_edges and incoming_edges
  for (auto node_idx = node_idx_t{0U}; node_idx < node_to_osm_.size(); ++node_idx) {
    for (way_pos_t way_pos = 0; way_pos < r_->node_ways_[node_idx].size(); ++way_pos) {
      for (auto const dir : {direction::kForward, direction::kBackward}) {
        car::node head{node_idx, way_pos, dir};

        auto const add_existing_edge = [&] (car::node const tail,
                                            std::uint32_t const cost,
                                            distance_t,
                                            way_idx_t,
                                            std::uint16_t,
                                            std::uint16_t,
                                            elevation_storage::elevation,
                                            bool) {
          if (cost == kInfeasible || head == tail) {
            return;
          }
          auto edge = routing::contracted_edge{
            0U,
            0U,
            node_to_id[head],
            node_to_id[tail],
            static_cast<cost_t>(cost)
          };
          r_->contracted_edges_.emplace_back(edge);
          auto const edge_index = r_->contracted_edges_.size()-1;
          outgoing_edges[node_to_id[head]].push_back(edge_index);
          incoming_edges[node_to_id[tail]].push_back(edge_index);
        };
        car::adjacent<direction::kForward, false>(params, *r_, head, nullptr, nullptr, nullptr, add_existing_edge);
      }
    }
  }
  // add additional edges to avoid problems with nodes at end of way
  for (auto [node, id] : node_to_id) {
    if (node.is_end_of_way(*r_)) {
      for (auto const out_edge_idx : outgoing_edges[id]) {
        auto out_edge = r_->contracted_edges_[out_edge_idx];
        if (out_edge.is_contracted()) {
          continue;
        }
        for (auto const in_edge_idx : incoming_edges[id]) {
          auto in_edge = r_->contracted_edges_[in_edge_idx];
          if (in_edge.is_contracted() || in_edge.from_ == out_edge.to_) {
            continue;
          }
          // if an edge already exists, shorten it if necessary
          auto const total_cost = static_cast<cost_t>(in_edge.cost_ + out_edge.cost_);
          bool found = false;
          for (auto const candidate_edge_idx : outgoing_edges[in_edge.from_]) {
            auto candidate_edge = r_->contracted_edges_[candidate_edge_idx];
            if (candidate_edge.to_ == out_edge.to_) {
              found = true;

              if (candidate_edge.cost_ > total_cost) {
                r_->contracted_edges_[candidate_edge_idx].cost_ = total_cost;
                r_->contracted_edges_[candidate_edge_idx].contracted_child1_ = in_edge_idx;
                r_->contracted_edges_[candidate_edge_idx].contracted_child2_ = out_edge_idx;
              }
              break;
            }
          }
          if (found) {
            continue;
          }
          auto edge = routing::contracted_edge{
            in_edge_idx,
            out_edge_idx,
            in_edge.from_,
            out_edge.to_,
            total_cost
          };
          r_->contracted_edges_.emplace_back(edge);
          auto edge_index = r_->contracted_edges_.size() - 1;
          outgoing_edges[in_edge.from_].push_back(edge_index);
          incoming_edges[out_edge.to_].push_back(edge_index);
        }
      }
    }
  }

  std::vector<routing::node_identifier> node_to_level(number_of_nodes);
  std::vector<routing::node_identifier> level_to_node(number_of_nodes);
  std::iota(level_to_node.begin(), level_to_node.end(), routing::node_identifier{0});

  std::ranges::shuffle(level_to_node, std::mt19937_64{});
  if (ch_config::kUseNodeImportance) {
    std::ranges::stable_sort(level_to_node, [&](auto n1, auto n2) {
      return get_importance(r_, n1) < get_importance(r_, n2);
    });
  }
  for (routing::node_identifier level = 0; level < number_of_nodes; ++level) {
    node_to_level[level_to_node[level]] = level;
  }

  std::vector<std::pair<cost_t, std::uint64_t>> index_handler(number_of_nodes);
  std::ranges::fill(index_handler, std::make_pair(kInfeasible, std::numeric_limits<std::uint64_t>::max()));
  std::uint64_t search_ID = 0;

  for (auto const identifier : level_to_node) {
    contract_node(*r_, outgoing_edges, incoming_edges, node_to_level,
                  index_handler, identifier, &search_ID);
  }

  std::size_t const total_upwards_edges = std::accumulate(
    outgoing_edges.begin(), outgoing_edges.end(), std::size_t{0U},
    [](auto sum, auto const& list){ return sum + list.size(); });
  std::size_t const total_downwards_edges = std::accumulate(
    outgoing_edges.begin(), outgoing_edges.end(), std::size_t{0U},
    [](auto sum, auto const& list){ return sum + list.size(); });
  r_->upwards_edges_outgoing_.bucket_starts_.resize(number_of_nodes + 1);
  r_->downwards_edges_incoming_.bucket_starts_.resize(number_of_nodes + 1);
  r_->upwards_edges_outgoing_.data_.resize(static_cast<unsigned int>(total_upwards_edges));
  r_->downwards_edges_incoming_.data_.resize(static_cast<unsigned int>(total_downwards_edges));
  long upwards_size = 0, downwards_size = 0;
  for (const auto index : node_to_id | std::views::values) {
    r_->upwards_edges_outgoing_.bucket_starts_[index] = upwards_size;
    for (auto const edge_idx : outgoing_edges[index]) {
      r_->upwards_edges_outgoing_.data_[upwards_size] = edge_idx;
      ++upwards_size;
    }
    r_->downwards_edges_incoming_.bucket_starts_[index] = downwards_size;
    for (auto const edge_idx : incoming_edges[index]) {
      r_->downwards_edges_incoming_.data_[downwards_size] = edge_idx;
      ++downwards_size;
    }
  }
  r_->upwards_edges_outgoing_.bucket_starts_[number_of_nodes] = upwards_size;
  r_->downwards_edges_incoming_.bucket_starts_[number_of_nodes] = downwards_size;
  r_->upwards_edges_outgoing_.data_.resize(upwards_size);
  r_->downwards_edges_incoming_.data_.resize(downwards_size);
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