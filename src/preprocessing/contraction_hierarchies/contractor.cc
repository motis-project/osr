#include "osr/preprocessing/contraction_hierarchies/contractor.h"

#include "osr/routing/route.h"

namespace osr::ch {


void contractor::calculate_neighbors(ways const& w) {
  auto pt = utl::get_active_progress_tracker_or_activate("ch-contraction");
  pt->status("calculate neighbours").in_high(w.n_nodes()).out_bounds(0,100);
  for (node_idx_t node{0U}; node < w.n_nodes(); ++node) {
    pt->update(node.v_);
    for (auto const [way, i] : utl::zip_unchecked(w.r_->node_ways_[node], w.r_->node_in_way_idx_[node])) {
      const auto way_pos = w.r_->get_way_pos(node, way);
      for (const auto dir : {direction::kForward, direction::kBackward}) {
        const car::node n{.n_ = node, .way_ = way_pos, .dir_ = dir};
        car::adjacent<direction::kForward, false>(
              *w.r_, n, nullptr, nullptr, nullptr,
              [&](car::node const neighbor, std::uint32_t const cost,
                  distance_t distance, way_idx_t const n_way, std::uint16_t,
                  std::uint16_t, elevation_storage::elevation const, bool) {
                if (neighbor.get_node() == node) {
                  return;
                }
                auto& existing_neighbors = outgoing_neighbors_[node];
                auto const it = std::ranges::find_if(existing_neighbors, [&](auto const& e) {
                  return e.node_idx == neighbor.get_node();
                });
                if (it == existing_neighbors.end() || cost < it->cost) {
                  if (it != existing_neighbors.end()) {
                    *it = neighbor_data{
                        .node_idx = neighbor.get_node(),
                        .cost = cost,
                        .way = n_way,
                        .distance = distance,
                        .node = neighbor};
                  } else {
                    existing_neighbors.push_back(
                        neighbor_data{.node_idx = neighbor.get_node(),
                                      .cost = cost,
                                      .way = n_way,
                                      .distance = distance,
                                      .node = neighbor}
                    );
                  }
                }
              });
      }
    }
  }
  for (node_idx_t node{0U}; node < outgoing_neighbors_.size(); ++node) {
    auto const outgoing_neighbours = outgoing_neighbors_[node];
    for (auto const& outgoing_neighbour : outgoing_neighbours) {
      incoming_neighbors_[outgoing_neighbour.node_idx].push_back(
            neighbor_data{
                .node_idx = node,
                .cost = outgoing_neighbour.cost,
                .way = outgoing_neighbour.way,
                .distance = outgoing_neighbour.distance,
                .node = car::node{
                    node, w.r_->get_way_pos(node, outgoing_neighbour.way),
                    outgoing_neighbour.node.dir_}});
    }
  }
}

std::unordered_map<node_idx_t, std::vector<way_idx_t>> node_selfloops;

bool node_has_sefloop(node_idx_t node) {
  return node_selfloops.contains(node);
}

void contractor::contract_node(ways& w, ways const&, bitvec<node_idx_t>* blocked, shortcut_storage* shortcuts, node_idx_t node) {
  blocked->set(node, true);
  auto& existing_neighbors = outgoing_neighbors_[node];

  for (std::size_t i{0U}; i < incoming_neighbors_[node].size(); ++i) {
    auto const& from = incoming_neighbors_[node][i];
    if (blocked->test(from.node_idx)) {
      continue;
    }
    if (shortcuts->shortcut_is_selfloop(from.way)) {
      continue;
    }

    auto max_direct_cost = 0U;
    for (auto const& to : existing_neighbors) {
      if (from.node_idx == to.node_idx && to.way == from.way && !shortcuts->is_shortcut(to.way)) {
        continue;
      }
      if (shortcuts->shortcut_is_selfloop(from.way) || shortcuts->shortcut_is_selfloop(to.way)) {
        continue;
      }
      if (blocked->test(to.node_idx)) {
        continue;
      }
      auto is_restricted = w.r_->is_restricted(
            node, w.r_->get_way_pos(node, from.way),
            w.r_->get_way_pos(node, to.way),
            direction::kForward);
      if (is_restricted) {
        continue;
      }
      auto const direct_cost = get_possible_shortcut_cost(from, to, shortcuts);
      if (direct_cost > max_direct_cost) {
        max_direct_cost = direct_cost;
      }
    }

    auto& dijkstra = get_dijkstra<car>();
    dijkstra.reset(max_direct_cost + 1);
    dijkstra.add_start(
          w,
          {car::node{from.node_idx,
                     w.r_->get_way_pos(
                         from.node_idx,
                         from.way),
                          from.node.dir_},
           0});

    dijkstra.run(w, *w.r_, max_direct_cost + 1, blocked, nullptr,
                   nullptr, direction::kForward); // TODO SHORTCUTS

    for (std::size_t j{0U}; j < outgoing_neighbors_[node].size(); ++j) {
      auto const& to = outgoing_neighbors_[node][j];
      if (from.node_idx == to.node_idx && to.way == from.way && !shortcuts->is_shortcut(to.way)) {
        continue;
      }
      if (shortcuts->shortcut_is_selfloop(from.way) || shortcuts->shortcut_is_selfloop(to.way)) {
        continue;
      }
      if (blocked->test(to.node_idx)) {
        continue;
      }

      auto const is_uturn = has_uturn(from.way, from.node.dir_,to.way, to.node.dir_,shortcuts);

      auto is_restricted = w.r_->is_restricted(
                  node, w.r_->get_way_pos(node, from.way),
                  w.r_->get_way_pos(node, to.way),
                  direction::kForward);
      auto direct_cost = get_possible_shortcut_cost(from, to, shortcuts);

      way_idx_t selfloop_way_idx = way_idx_t::invalid();
      bool shortest_possible_selfloop_found = false;
      if ((is_uturn || is_restricted) && node_has_sefloop(node)) {
        way_idx_t shortest_possible_selfloop = way_idx_t::invalid();
        cost_t shortest_possible_selfloop_cost = kInfeasible;
        auto selfloops = node_selfloops[node];
        for (auto const& selfloop : selfloops) {
          if (!shortcuts->shortcut_is_selfloop(selfloop)) {
            continue;
          }
          if (has_uturn(from.way, from.node.dir_, selfloop,direction::kForward,shortcuts)) {
            continue;
          }
          if (has_uturn(selfloop, direction::kForward, to.way, to.node.dir_, shortcuts)) {
            continue;
          }
          way_and_dir const selfloop_first = shortcuts->resolve_first_way_and_dir(selfloop, direction::kForward);
          way_and_dir const selfloop_last = shortcuts->resolve_last_way_and_dir(selfloop, direction::kForward);
          auto is_restricted_from = w.r_->is_restricted(
              node, w.r_->get_way_pos(node, from.way),
              w.r_->get_way_pos(node, selfloop_first.way),
              direction::kForward);
          auto is_restricted_to = w.r_->is_restricted(
              node, w.r_->get_way_pos(node, selfloop_last.way),
              w.r_->get_way_pos(node, to.way),
              direction::kForward);
          if (is_restricted_from || is_restricted_to) {
            continue;
          }
          shortcut_data const* selfloop_data =
                shortcuts->get_shortcut(selfloop);
          if (selfloop_data->cost > car::kUturnPenalty && !is_restricted) { // wenn der selfloop also teurer als ein uturn wäre und dieser auch erlaubt ist
            if (car::kUturnPenalty < shortest_possible_selfloop_cost) {
              shortest_possible_selfloop = way_idx_t::invalid();
              shortest_possible_selfloop_found = true;
              shortest_possible_selfloop_cost = car::kUturnPenalty;
            }
          } else {
            if (selfloop_data->cost < shortest_possible_selfloop_cost) {
              shortest_possible_selfloop = selfloop;
              shortest_possible_selfloop_found = true;
              shortest_possible_selfloop_cost = selfloop_data->cost;
            }
          }
        }
        if (shortest_possible_selfloop_found) {
          if (cista::to_idx(shortest_possible_selfloop) != std::numeric_limits<std::uint32_t>::max()) {
            selfloop_way_idx = shortest_possible_selfloop;
            //fmt::println("contraction of {} -> {} -> {} has found a selfloop at {} way {} with {} cost", from.node_idx, node, to.node_idx, node, selfloop_way_idx, shortest_possible_selfloop_cost);
          }
          direct_cost = from.cost + to.cost + shortest_possible_selfloop_cost;
        }
      } else if (is_uturn) {
        direct_cost = from.cost + to.cost + car::kUturnPenalty;
      }
      if (is_restricted && !shortest_possible_selfloop_found) {
        continue;
      }

      auto min_cost = kInfeasible;
      car::resolve_all(*w.r_, to.node_idx, level_t{},
          [&](car::node const nodee) {
          if (from.node_idx == to.node_idx && from.way == w.r_->node_ways_[to.node_idx][nodee.way_] && from.node.dir_ == nodee.dir_) {
           return;
          }
          if (!is_restriction_subset_at_node(w, to.node_idx, to.node.way_, nodee.way_)) {
           return;
          }
          auto cost = dijkstra.get_cost(nodee);
          if (cost < min_cost) {
            min_cost = cost;
          }
      });
      cost_t const path_cost = min_cost;
      if (path_cost > direct_cost) {
        way_idx_t shortcut_way = shortcuts->add_shortcut(
              w, shortcut_data{.from = from.node_idx,
                                  .to = to.node_idx,
                                  .cost = direct_cost,
                                  .distance = static_cast<unsigned short>(from.distance + to.distance),
                                  .via_node = node,
                                  .upward_way = from.way,
                                  .upward_dir = from.node.dir_,
                                  .upward_distance = from.distance,
                                  .upward_cost = from.cost,
                                  .downward_dir = to.node.dir_,
                                  .downward_way = to.way,
                                  .downward_distance = to.distance,
                                  .downward_cost = to.cost,
                                  .selfloop_way_idx = selfloop_way_idx});
        if (from.node_idx == to.node_idx) {
          node_selfloops[from.node_idx].push_back(shortcut_way);
          //fmt::println("contracting {} adding a recursive edge from {} to {} with cost {} and ways {} {}", node, from.node_idx, to.node_idx, direct_cost, from.way, to.way);
        }
        // ADD NEIGHBOUR
        outgoing_neighbors_[from.node_idx].push_back(neighbor_data{
            .node_idx = to.node_idx,
            .cost = direct_cost,
            .way = shortcut_way,
            .distance = static_cast<unsigned short>(from.distance + to.distance),
            .node = car::node{to.node_idx,
                              w.r_->get_way_pos(to.node_idx, shortcut_way),
                              direction::kForward}});
        incoming_neighbors_[to.node_idx].push_back(neighbor_data{
              .node_idx = from.node_idx,
              .cost = direct_cost,
              .way = shortcut_way,
              .distance = static_cast<unsigned short>(from.distance + to.distance),
              .node = car::node{from.node_idx,
                                w.r_->get_way_pos(from.node_idx, shortcut_way),
                                direction::kForward}});
        if (w.r_->node_is_restricted_[from.node_idx]) {
          auto shortcut_pos = w.r_->get_way_pos(from.node_idx, shortcut_way);
          for (auto const& r : w.r_->node_restrictions_[from.node_idx]) {
            if (r.to_ == w.r_->get_way_pos(from.node_idx, from.way) &&
                  r.to_ != r.from_) {
              w.r_->node_restrictions_[from.node_idx].push_back(
                    restriction{r.from_, shortcut_pos});
            }
          }
        }
        if (w.r_->node_is_restricted_[to.node_idx]) {
          auto shortcut_pos = w.r_->get_way_pos(to.node_idx, shortcut_way);
          for (auto const& r : w.r_->node_restrictions_[to.node_idx]) {
            if (r.from_ == w.r_->get_way_pos(to.node_idx, to.way) &&
                  r.to_ != r.from_) {
              w.r_->node_restrictions_[to.node_idx].push_back(
                    restriction{shortcut_pos, r.to_});
            }
          }
        }
      }
    }
  }
}

} // namespace osr::ch