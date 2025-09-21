#include "osr/preprocessing/contraction_hierarchies/contractor.h"

#include "osr/routing/route.h"

namespace osr::ch {


void contractor::calculate_neighbors(ways const& w) {
  auto const pt = utl::get_active_progress_tracker_or_activate("ch-contraction");
  pt->status("calculate neighbours").in_high(w.n_nodes()).out_bounds(0,100);
  outgoing_neighbors_.resize(w.n_nodes());
  incoming_neighbors_.resize(w.n_nodes());
  for (node_idx_t node{0U}; node < w.n_nodes(); ++node) {
    pt->update(node.v_);

    for (auto const [way, i] : utl::zip_unchecked(w.r_->node_ways_[node], w.r_->node_in_way_idx_[node])) {
      const auto way_pos = w.r_->get_way_pos(node, way);

      for (const auto dir : {direction::kForward, direction::kBackward}) {

        const car::node n{.n_ = node, .way_ = way_pos, .dir_ = dir};

        car::adjacent<direction::kForward, false>(
            car::parameters{}, *w.r_, n, nullptr, nullptr, nullptr, nullptr,
            [&](car::node const neighbor, std::uint32_t const cost,
                distance_t const distance, way_idx_t const n_way, std::uint16_t,
                std::uint16_t, elevation_storage::elevation const, bool) {
              if (neighbor.get_node() == node) {
                return;
              }
              auto& existing_neighbors = outgoing_neighbors_[to_idx(node)];
              auto const it =
                  std::ranges::find_if(existing_neighbors, [&](auto const& e) {
                    return e.node_idx == neighbor.get_node();
                  });
              if (it == existing_neighbors.end() || cost < it->cost) {
                if (it != existing_neighbors.end()) {
                  *it = neighbor_data{.node_idx = neighbor.get_node(),
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
                                    .node = neighbor});
                }
              }
            });
      }
    }
  }
  for (node_idx_t node{0U}; node < outgoing_neighbors_.size(); ++node) {
    for (auto const outgoing_neighbours = outgoing_neighbors_[to_idx(node)];
         const auto& [neighbor_node_idx, neighbor_cost, neighbor_way, neighbor_distance, neighbor_node] : outgoing_neighbours) {
      incoming_neighbors_[to_idx(neighbor_node_idx)].push_back(
            neighbor_data{
                .node_idx = node,
                .cost = neighbor_cost,
                .way = neighbor_way,
                .distance = neighbor_distance,
                .node = car::node{
                    node, w.r_->get_way_pos(node, neighbor_way),
                    neighbor_node.dir_}});
    }
  }
}

std::unordered_map<node_idx_t, std::vector<way_idx_t>> node_selfloops;

bool node_has_sefloop(node_idx_t const node) {
  return node_selfloops.contains(node);
}

std::vector<contractor::bypass_path> find_subloops(contractor::bypass_path const& bypass) {
  std::vector<contractor::bypass_path> loops;
  if (bypass.nodes.size() < 4) {
    return loops;
  }
  for (size_t i = 0; i < bypass.nodes.size() - 3; ++i) {
    for (size_t j = bypass.nodes.size() - 1; j > i + 2; --j) {
      if (bypass.nodes[i].n_ == bypass.nodes[j].n_ &&
          bypass.nodes[i + 1].n_ == bypass.nodes[j - 1].n_) {
        contractor::bypass_path loop;

        for (size_t k = i; k <= j; ++k) {
          loop.nodes.push_back(bypass.nodes[k]);
        }

        for (size_t k = i; k < j; ++k) {
          for (auto const& seg : bypass.segments) {
            if (seg.from == bypass.nodes[k].n_ &&
                seg.to == bypass.nodes[k + 1].n_) {
              loop.segments.push_back(seg);
              loop.total_cost += seg.cost;
              loop.total_distance += seg.distance;
              break;
            }
          }
        }
        loops.push_back(std::move(loop));
      }
    }
  }
  return loops;
}

std::vector<contractor::bypass_path> contractor::find_restriction_bypasses(
    ways const& w,
    node_idx_t const node,
    node_idx_t const from_node,
    way_idx_t const from_way,
    direction const from_dir,
    shortcut_storage const* shortcut_storage) const {
  std::vector<bypass_path> bypasses;

  if (!w.r_->node_is_restricted_[node] ||
    (shortcut_storage && shortcut_storage->is_shortcut(from_way))) {
    return bypasses;
  }

  std::vector<way_pos_t> to_restrictions = get_restriction_to_ways(w, node, from_way, shortcut_storage);
  if (to_restrictions.empty()) {
    return bypasses;
  }
  auto& dijkstra = get_dijkstra<car>();
  dijkstra.reset(241);
  dijkstra.add_start(
        w,
        {car::node{from_node,
                   w.r_->get_way_pos(from_node, from_way),
                   from_dir},
         0});
  dijkstra.run(car::parameters{},w, *w.r_, 241, nullptr, nullptr,
                nullptr, direction::kForward, shortcut_storage);
  for (auto const& to : outgoing_neighbors_[to_idx(node)]) {
    if(shortcut_storage->is_shortcut(to.way)) {
      continue;
    }
    if (std::ranges::find(
            to_restrictions,
                       w.r_->get_way_pos(node, w.r_->node_ways_[to.node_idx][to.node.way_])) != to_restrictions.end()) {
      if (auto const cost = dijkstra.get_cost(to.node); cost != kInfeasible) {
        bypass_path path;
        auto node_it = to.node;
        while (true) {

          path.nodes.push_back(node_it);
          auto pred = dijkstra.cost_.at(node_it.get_key()).pred(node_it);

          if (!pred.has_value()) {
            break;
          }

          node_it = *pred;
        }
        std::ranges::reverse(path.nodes);
        if (path.nodes.size() > 2) {
          path.total_cost = dijkstra.get_cost(path.nodes[path.nodes.size()-2]) -
                      dijkstra.get_cost(path.nodes[1]);

          path.nodes.erase(path.nodes.begin());
          path.nodes.pop_back();
          if (!(path.nodes[0].n_ == node && path.nodes[path.nodes.size()-1].n_ == node)) {
            continue;
          }
          path.total_distance = 0;
          for (size_t i = 0; i < path.nodes.size() - 1; ++i) {
            auto const current = path.nodes[i];
            auto const next = path.nodes[i + 1];
            cost_t expected_cost = dijkstra.get_cost(path.nodes[i + 1]) -
                      dijkstra.get_cost(path.nodes[i]);
            car::adjacent<direction::kForward, false>(
                car::parameters{}, *w.r_, current, nullptr, nullptr, nullptr, nullptr,
                [&](car::node const target, std::uint32_t const cost_n,
                    distance_t const dist, way_idx_t const way,
                    std::uint16_t const, std::uint16_t const,
                    elevation_storage::elevation const, bool) {
                  if (target == next && cost_n == expected_cost) {
                    if (shortcut_storage->is_shortcut(way)) {
                      auto segments =
                          shortcut_storage->get_shortcut_segments(way);
                      path.segments.insert(path.segments.end(),
                                           segments.begin(), segments.end());
                    } else {
                      path.segments.push_back(ShortcutSegment{
                          .w = way,
                          .from = current.n_,
                          .to = next.n_,
                          .distance = dist,
                          .cost = cost_n,
                      });
                    }
                    path.total_distance += dist;
                  }
                });
          }
          bool is_distinct = true;
          for (const auto& existing : bypasses) {
            if (!path.is_distinct_from(existing)) {
              is_distinct = false;
              break;
            }
          }
          if (is_distinct) {
            bypasses.push_back(std::move(path));
          }
        }
      }
    }
  }
  return bypasses;
}
void contractor::add_loop(
  ways& w,
  shortcut_storage* shortcuts,
  node_idx_t const node,
  bypass_path const& bypass
) {
  way_idx_t const shortcut_way = shortcuts->add_shortcut(
              w, shortcut_data{.from = node,
                                  .to = node,
                                  .cost = bypass.total_cost,
                                  .distance = static_cast<unsigned short>(bypass.total_distance),
                                  .via_node = node,
                                  .upward_way = w.r_->node_ways_[bypass.nodes[0].n_][bypass.nodes[0].way_],
                                  .upward_dir = bypass.nodes[0].dir_,
                                  .upward_distance = static_cast<distance_t>(0), // these are not needed because the path will be entirly recontructed with .loop_segments data
                                  .upward_cost = static_cast<cost_t>(0),
                                  .downward_dir = bypass.nodes[bypass.nodes.size()-1].dir_,
                                  .downward_way = w.r_->node_ways_[bypass.nodes[bypass.nodes.size()-1].n_][bypass.nodes[bypass.nodes.size()-1].way_],
                                  .downward_distance = static_cast<distance_t>(0),
                                  .downward_cost = static_cast<cost_t>(0),
                                  .selfloop_way_idx = way_idx_t::invalid(),
                                  .loop_segments = bypass.segments,
                                  });
  node_selfloops[node].push_back(shortcut_way);

  auto const from_shortcut_pos = w.r_->get_way_pos(node, shortcut_way);
  auto const to_shortcut_pos = way_pos_t{
    static_cast<unsigned short int>(from_shortcut_pos + 1)};
  for (const auto& [from_, to_] : w.r_->node_restrictions_[node]) {
    if (to_ == bypass.nodes[0].way_ &&
          to_ != from_) {
      w.r_->node_restrictions_[node].push_back(
            restriction{from_, from_shortcut_pos});
          }
    if (from_ == bypass.nodes[bypass.nodes.size() - 1].way_ &&
          to_ != from_) {
      w.r_->node_restrictions_[node].push_back(
            restriction{to_shortcut_pos, to_});
          }
  }
}
void contractor::contract_node(ways& w, ways const& w_without, bitvec<node_idx_t>* blocked, shortcut_storage* shortcuts, node_idx_t node) {
  blocked->set(node, true);
  auto& existing_neighbors = outgoing_neighbors_[to_idx(node)];
  for (std::size_t i{0U}; i < incoming_neighbors_[to_idx(node)].size(); ++i) {
    auto const& from = incoming_neighbors_[to_idx(node)][i];
    if (w.r_->node_is_restricted_[node] && !shortcuts->is_shortcut(from.way)) {

      auto it = std::ranges::find_if(outgoing_neighbors_[to_idx(from.node_idx)], [&](const auto& n) {
            return n.node_idx == node;
        });
      if (it != outgoing_neighbors_[to_idx(from.node_idx)].end()) {
        for (auto bypasses = find_restriction_bypasses(
                 w_without, node, from.node_idx, from.way,
                 from.node.dir_, shortcuts);
             auto const& bypass : bypasses) {
          add_loop(w,shortcuts,node, bypass);

          for (std::vector<bypass_path> sub_loops = find_subloops(bypass);
               auto const& sub_loop : sub_loops) {
            node_idx_t sub_loop_node = sub_loop.nodes[0].n_;
            if (sub_loop_node == node){ continue; }
            add_loop(w,shortcuts, sub_loop_node, sub_loop);
          }
        }
      }

    }
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

    dijkstra.run(car::parameters{}, w, *w.r_, max_direct_cost + 1, blocked, nullptr,
                   nullptr, direction::kForward, shortcuts);

    for (std::size_t j{0U}; j < outgoing_neighbors_[to_idx(node)].size(); ++j) {
      auto const& to = outgoing_neighbors_[to_idx(node)][j];
      if (from.node_idx == to.node_idx && to.way == from.way && !shortcuts->is_shortcut(to.way)) {
        continue;
      }
      if (shortcuts->shortcut_is_selfloop(from.way) || shortcuts->shortcut_is_selfloop(to.way)) {
        continue;
      }
      if (blocked->test(to.node_idx)) {
        continue;
      }
      /*auto const from_res = shortcuts->resolve_first_way_and_dir(from.way,from.node.dir_);
      auto const to_res = shortcuts->resolve_last_way_and_dir(to.way,to.node.dir_);
      if (has_shortcut(from.node_idx, to.node_idx, from_res.way, to_res.way, from_res.dir, to_res.dir, shortcuts)) {
        //fmt::println("Skip because shortcut already exists from {} -{}-> to -{}-> {}", from.node_idx, from_res.way, to_res.way, to.node_idx);
        continue;
      }*/
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
      auto min_node = to.node;
      car::resolve_all(*w.r_, to.node_idx, level_t{},
          [&](car::node const r_node) {
          if (from.node_idx == to.node_idx && from.way == w.r_->node_ways_[to.node_idx][r_node.way_] && from.node.dir_ == r_node.dir_) {
           return;
          }
          if (!is_restriction_subset_at_node(w, to.node_idx, to.node.way_, r_node.way_)) {
           return;
          }
          auto cost = dijkstra.get_cost(r_node);
          if (cost < min_cost) {
            min_cost = cost;
            min_node = r_node;
          }
      });
      cost_t const path_cost = min_cost;
      auto add_shortcut = path_cost > direct_cost;

      if (min_cost != kInfeasible && min_cost < kInfeasible - car::kUturnPenalty && from.node_idx != to.node_idx) {
        std::vector<car::node> path = {};
        auto node_it = min_node;
        auto has_restriction = shortcuts->shortcut_has_restricted_node(w.r_->node_ways_[min_node.n_][min_node.way_]);
        while (true) {

          path.push_back(node_it);
          auto pred = dijkstra.cost_.at(node_it.get_key()).pred(node_it);

          if (!pred.has_value()) {
            break;
          }
          way_idx_t pred_way = w.r_->node_ways_[pred->n_][pred->way_];
          has_restriction = has_restriction || shortcuts->shortcut_has_restricted_node(pred_way) || w.r_->node_is_restricted_[pred->n_];
          node_it = *pred;
        }

        std::ranges::reverse(path);
        if (path.size() >= 2) {
          way_idx_t first_way_idx = w.r_->node_ways_[path[1].n_][path[1].way_];

          way_and_dir first_way_resolved = shortcuts->resolve_first_way_and_dir(first_way_idx, path.front().dir_);

          if (way_and_dir from_way_resolved =
                  shortcuts->resolve_first_way_and_dir(from.way,
                                                       from.node.dir_);
              first_way_resolved.way != from_way_resolved.way || (first_way_resolved.way == from_way_resolved.way && first_way_resolved.dir == from_way_resolved.dir)) {
            if (has_restriction) {
              add_shortcut = path_cost + 120 > direct_cost;
            }
          }
        }
      }

      if (add_shortcut) {
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
        outgoing_neighbors_[to_idx(from.node_idx)].push_back(neighbor_data{
            .node_idx = to.node_idx,
            .cost = direct_cost,
            .way = shortcut_way,
            .distance = static_cast<unsigned short>(from.distance + to.distance),
            .node = car::node{to.node_idx,
                              w.r_->get_way_pos(to.node_idx, shortcut_way),
                              direction::kForward}});
        incoming_neighbors_[to_idx(to.node_idx)].push_back(neighbor_data{
              .node_idx = from.node_idx,
              .cost = direct_cost,
              .way = shortcut_way,
              .distance = static_cast<unsigned short>(from.distance + to.distance),
              .node = car::node{from.node_idx,
                                w.r_->get_way_pos(from.node_idx, shortcut_way),
                                direction::kForward}});
        if (w.r_->node_is_restricted_[from.node_idx]) {
          auto shortcut_pos = w.r_->get_way_pos(from.node_idx, shortcut_way);
          for (const auto& [from_, to_] : w.r_->node_restrictions_[from.node_idx]) {
            if (to_ == w.r_->get_way_pos(from.node_idx, from.way) &&
                  to_ != from_) {
              w.r_->node_restrictions_[from.node_idx].push_back(
                    restriction{from_, shortcut_pos});
            }
          }
        }
        if (w.r_->node_is_restricted_[to.node_idx]) {
          auto shortcut_pos = from.node_idx != to.node_idx
                ? w.r_->get_way_pos(to.node_idx, shortcut_way)
                : way_pos_t{static_cast<unsigned short int>(w.r_->get_way_pos(to.node_idx, shortcut_way) + 1)};
          for (const auto& [from_, to_] : w.r_->node_restrictions_[to.node_idx]) {
            if (from_ == w.r_->get_way_pos(to.node_idx, to.way) &&
                  to_ != from_) {
              w.r_->node_restrictions_[to.node_idx].push_back(
                    restriction{shortcut_pos, to_});
            }
          }
        }
      }
    }
  }
}

} // namespace osr::ch