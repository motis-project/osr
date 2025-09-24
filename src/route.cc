#include "osr/routing/route.h"

#include <cassert>
#include <cstdint>
#include <algorithm>
#include <list>
#include <mutex>
#include <optional>

#include "boost/thread/tss.hpp"

#include "utl/concat.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/lookup.h"
#include "osr/preprocessing/contraction_hierarchy/shortcut_storage.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/ch_dijkstra.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/with_profile.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

constexpr auto const kMaxMatchingDistanceSquaredRatio = 9.0;
constexpr auto const kBottomKDefinitelyConsidered = 5;

template <Profile P>
bidirectional<P>& get_bidirectional() {
  static auto s = boost::thread_specific_ptr<bidirectional<P>>{};
  if (s.get() == nullptr) {
    s.reset(new bidirectional<P>{});
  }
  return *s.get();
}

template <Profile P>
dijkstra<P>& get_dijkstra() {
  static auto s = boost::thread_specific_ptr<dijkstra<P>>{};
  if (s.get() == nullptr) {
    s.reset(new dijkstra<P>{});
  }
  return *s.get();
}

template <Profile P>
ch_dijkstra<P>& get_ch_dijkstra() {
  static auto s = boost::thread_specific_ptr<ch_dijkstra<P>>{};
  if (s.get() == nullptr) {
    s.reset(new ch_dijkstra<P>{});
  }
  return *s.get();
}

struct connecting_way {
  constexpr bool valid() const { return way_ != way_idx_t::invalid(); }

  way_idx_t way_{way_idx_t::invalid()};
  std::uint16_t from_{}, to_{};
  bool is_loop_{};
  std::uint16_t distance_{};
  elevation_storage::elevation elevation_;
};

routing_algorithm to_algorithm(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("dijkstra"): return routing_algorithm::kDijkstra;
    case cista::hash("bidirectional"): return routing_algorithm::kAStarBi;
    case cista::hash("chdijkstra"): return routing_algorithm::kCHDijkstra;
  }
  throw utl::fail("unknown routing algorithm: {}", s);
}

template <direction SearchDir, adj_conf Config, Profile P>
connecting_way find_connecting_way(typename P::parameters const& params,
                                   ways const& w,
                                   ways::routing const& r,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations,
                                   typename P::node const from,
                                   typename P::node const to,
                                   cost_t const expected_cost) {
  auto conn = std::optional<connecting_way>{};
  P::template adjacent<SearchDir, Config>(
      params, r, from, blocked, sharing, elevations, nullptr,
      node_idx_t::invalid(),
      [&](typename P::node const target, std::uint32_t const cost,
          distance_t const dist, way_idx_t const way, std::uint16_t const a_idx,
          std::uint16_t const b_idx,
          elevation_storage::elevation const elevation, bool, typename P::node,
          cost_t) {
        if (target == to && cost == expected_cost) {
          auto const is_loop = way != way_idx_t::invalid() && r.is_loop(way) &&
                               static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
                                   r.way_nodes_[way].size() - 2U;
          conn = {way, a_idx, b_idx, is_loop, dist, elevation};
        }
      });
  utl::verify(
      conn.has_value(), "no connecting way node/{} -> node/{} found {} {}",
      (sharing == nullptr || from.get_node() < sharing->additional_node_offset_)
          ? to_idx(w.node_to_osm_[from.get_node()])
          : 0,
      (sharing == nullptr || to.get_node() < sharing->additional_node_offset_)
          ? to_idx(w.node_to_osm_[to.get_node()])
          : 0,
      expected_cost, expected_cost);
  return *conn;
}

template <Profile P>
connecting_way find_connecting_way(typename P::parameters const& params,
                                   ways const& w,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations,
                                   typename P::node const from,
                                   typename P::node const to,
                                   cost_t const expected_cost,
                                   direction const dir) {
  auto const call = [&]<adj_conf Config>() {
    if (dir == direction::kForward) {
      return find_connecting_way<direction::kForward, Config, P>(
          params, w, *w.r_, blocked, sharing, elevations, from, to,
          expected_cost);
    } else {
      return find_connecting_way<direction::kBackward, Config, P>(
          params, w, *w.r_, blocked, sharing, elevations, from, to,
          expected_cost);
    }
  };

  if (blocked == nullptr) {
    return call.template operator()<adj_conf::kNone>();
  } else {
    return call.template operator()<adj_conf::kBlocked>();
  }
}

template <Profile P>
double add_path(typename P::parameters const& params,
                ways const& w,
                ways::routing const& r,
                bitvec<node_idx_t> const* blocked,
                sharing_data const* sharing,
                elevation_storage const* elevations,
                typename P::node const from,
                typename P::node const to,
                cost_t const expected_cost,
                std::vector<path::segment>& path,
                direction const dir) {
  auto const& [way, from_idx, to_idx, is_loop, distance, elevation] =
      find_connecting_way<P>(params, w, blocked, sharing, elevations, from, to,
                             expected_cost, dir);

  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.dist_ = distance;
  segment.cost_ = expected_cost;
  segment.elevation_ = elevation;
  segment.mode_ = to.get_mode();

  if (way != way_idx_t::invalid()) {
    auto const start_idx = dir == direction::kBackward ? to_idx : from_idx;
    auto const end_idx = dir == direction::kBackward ? from_idx : to_idx;
    auto const is_reverse = (start_idx > end_idx) ^ is_loop;

    if (is_reverse) {
      segment.from_level_ = r.way_properties_[way].to_level();
      segment.to_level_ = r.way_properties_[way].from_level();
    } else {
      segment.from_level_ = r.way_properties_[way].from_level();
      segment.to_level_ = r.way_properties_[way].to_level();
    }
    segment.from_ = r.way_nodes_[way][start_idx];
    segment.to_ = r.way_nodes_[way][end_idx];

    for (auto const [osm_idx, coord] : infinite(
             reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                     is_reverse),
             is_loop)) {
      utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U,
                  "infinite loop");
      if (!active && w.node_to_osm_[r.way_nodes_[way][start_idx]] == osm_idx) {
        active = true;
      }
      if (active) {
        if (w.node_to_osm_[r.way_nodes_[way][start_idx]] == osm_idx) {
          // Again "from" node, then it's shorter to start from here.
          segment.polyline_.clear();
        }

        segment.polyline_.emplace_back(coord);
        if (w.node_to_osm_[r.way_nodes_[way][end_idx]] == osm_idx) {
          break;
        }
      }
    }
  } else {
    auto const get_node_pos = [&](node_idx_t const n) -> geo::latlng {
      if (n == node_idx_t::invalid()) {
        return {};
      } else if (w.is_additional_node(n)) {
        return sharing->get_additional_node_coordinates(n);
      } else {
        return w.get_node_pos(n).as_latlng();
      }
    };

    segment.from_level_ = level_t{0.0F};
    segment.to_level_ = level_t{0.0F};
    segment.from_ =
        dir == direction::kBackward ? to.get_node() : from.get_node();
    segment.to_ = dir == direction::kBackward ? from.get_node() : to.get_node();
    segment.polyline_ = {get_node_pos(segment.from_),
                         get_node_pos(segment.to_)};
  }

  return distance;
}

template <Profile P>
path reconstruct_bi(typename P::parameters const& params,
                    ways const& w,
                    lookup const& l,
                    bitvec<node_idx_t> const* blocked,
                    sharing_data const* sharing,
                    elevation_storage const* elevations,
                    bidirectional<P> const& b,
                    location const& from,
                    location const& to,
                    way_candidate const& start,
                    way_candidate const& dest,
                    cost_t const cost,
                    direction const dir) {
  auto forward_n = b.meet_point_1_;

  // TODO subtract meetpoint node cost?

  auto forward_segments = std::vector<path::segment>{};
  auto forward_dist = 0.0;

  while (true) {
    auto const& e = b.cost1_.at(forward_n.get_key());
    auto const pred = e.pred(forward_n);
    if (pred.has_value()) {
      auto const expected_cost = static_cast<cost_t>(
          e.cost(forward_n) - b.template get_cost<direction::kForward>(*pred));
      forward_dist +=
          add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                      forward_n, expected_cost, forward_segments, dir);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start_node_candidate =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;

  forward_segments.push_back(
      {.polyline_ =
           l.get_node_candidate_path(start, start_node_candidate, false, from),
       .from_level_ = start_node_candidate.lvl_,
       .to_level_ = start_node_candidate.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),

       .way_ = way_idx_t::invalid(),
       .cost_ = start_node_candidate.cost_,
       .dist_ = static_cast<distance_t>(start_node_candidate.dist_to_node_),
       .mode_ = forward_n.get_mode()});

  auto backward_segments = std::vector<path::segment>{};
  auto backward_n = b.meet_point_2_;
  auto backward_dist = 0.0;

  while (true) {
    auto const& e = b.cost2_.at(backward_n.get_key());
    auto const pred = e.pred(backward_n);
    if (pred.has_value()) {

      auto const expected_cost =
          static_cast<cost_t>(e.cost(backward_n) -
                              b.template get_cost<direction::kBackward>(*pred));
      backward_dist += add_path<P>(params, w, *w.r_, blocked, sharing,
                                   elevations, *pred, backward_n, expected_cost,
                                   backward_segments, opposite(dir));
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const& dest_node_candidate =
      backward_n.get_node() == dest.left_.node_ ? dest.left_ : dest.right_;

  backward_segments.push_back(
      {.polyline_ =
           l.get_node_candidate_path(dest, dest_node_candidate, true, to),
       .from_level_ = dest_node_candidate.lvl_,
       .to_level_ = dest_node_candidate.lvl_,
       .from_ = dir == direction::kForward ? backward_n.get_node()
                                           : node_idx_t::invalid(),
       .to_ = dir == direction::kBackward ? backward_n.get_node()
                                          : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_node_candidate.cost_,
       .dist_ = static_cast<distance_t>(dest_node_candidate.dist_to_node_),
       .mode_ = backward_n.get_mode()});

  if (dir == direction::kForward) {
    std::reverse(forward_segments.begin(), forward_segments.end());
  } else {
    std::reverse(backward_segments.begin(), backward_segments.end());
  }
  forward_segments.insert(forward_segments.end(), backward_segments.begin(),
                          backward_segments.end());

  auto total_dist = start_node_candidate.dist_to_node_ + forward_dist +
                    backward_dist + dest_node_candidate.dist_to_node_;

  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : forward_segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = cost,
                .dist_ = total_dist,
                .elevation_ = path_elevation,
                .segments_ = forward_segments};

  b.cost2_.at(backward_n.get_key()).write(backward_n, p);
  return p;
}

template <direction SearchDir, direction PathDir, Profile P>
auto reconstruct_ch_partially(typename P::parameters const& params,
                              ways const& w,
                              lookup const& l,
                              sharing_data const* sharing,
                              elevation_storage const* elevations,
                              shortcut_storage<typename P::node> const& sc_stor,
                              auto const& costs,
                              location const& loc,
                              way_candidate const& wc,
                              typename P::node& mp_node,
                              auto& path_segments) {
  auto retrieve_path = [&](typename P::node& node,
                           std::list<path_label<typename P::node>>& path) {
    while (true) {
      auto const& e = costs.at(node.get_key());
      auto const pred = e.pred(node);
      auto const way = e.way(node);
      if (!pred.has_value()) {
        break;
      }
      auto const expected_cost = static_cast<cost_t>(
          e.cost(node) - costs.at(pred->get_key()).cost(*pred));
      path.emplace_front(node, expected_cost, way);
      node = *pred;
    }
  };

  auto find_loop = [&](typename P::node const from_node,
                       typename P::node const to_node, cost_t const max) {
    assert(from_node.get_node() == to_node.get_node() &&
           "loop requires start and end node to have the same node_idx_t");
    auto const order_limit = sc_stor.order_.node2order_[from_node.get_node()];
    auto& normal_d = get_dijkstra<P>();
    normal_d.reset(max + 1);
    auto const start_node =
        flip<SearchDir>(PathDir) == direction::kForward ? from_node : to_node;
    normal_d.pq_.push(typename P::label{start_node, 0});
    auto p = std::list<path_label<typename P::node>>{};
    normal_d.template run<flip<SearchDir>(PathDir), adj_conf::kUseCH>(
        params, w, *w.r_, max + 1, nullptr, sharing, elevations, &sc_stor,
        order_limit);
    auto const dest_node =
        flip<SearchDir>(PathDir) == direction::kForward ? to_node : from_node;
    auto node = dest_node;
    normal_d.cost_.at(dest_node.get_key())
        .for_each(dest_node.get_key(),
                  [&](typename P::node const n, cost_t const c) {
                    if (static_cast<int32_t>(c) +
                            P::template turning_cost<flip<SearchDir>(PathDir)>(
                                *w.r_, n, dest_node) ==
                        max) {
                      node = n;
                    }
                  });
    while (true) {
      auto const& e = normal_d.cost_.at(node.get_key());
      auto const pred = *e.pred(node);
      auto const way = e.way(node);
      if (pred == start_node) {
        p.emplace_front(node, e.cost(node), way);
        break;
      }
      auto const expected_cost = e.cost(node) - normal_d.get_cost(pred);
      p.emplace_front(node, expected_cost, way);
      node = pred;
    }
    return p;
  };

  auto resolve_shortcuts = [&](std::list<path_label<typename P::node>>& path) {
    for (auto it = begin(path), next_it = it;
         (next_it = std::next(it)) != end(path);) {
      auto& label = *next_it;
      int64_t const vrt_idx = static_cast<int64_t>(label.way_.v_) - w.n_ways();
      if (vrt_idx < 0) {
        ++it;
        continue;
      }
      auto const& s = sc_stor.shortcuts_[way_idx_t{vrt_idx}];
      auto const& si = sc_stor.shortcuts_intern_[way_idx_t{vrt_idx}];
      if (si.loop_cost_ > 0) {
        auto res = find_loop(si.via_in_, si.via_out_, si.loop_cost_);
        path.insert(next_it, begin(res), end(res));
        next_it = std::next(it);
      }
      if constexpr (flip<SearchDir>(PathDir) == direction::kForward) {
        cost_t const via_cost =
            P::template turning_cost<flip<SearchDir>(PathDir)>(
                *w.r_, (*it).node_, s.from_) +
            si.from_way_cost_ +
            P::node_cost(w.r_->node_properties_[si.via_in_.get_node()]);
        cost_t const to_cost =
            (si.loop_cost_ > 0
                 ? 0
                 : P::template turning_cost<flip<SearchDir>(PathDir)>(
                       *w.r_, si.via_in_, si.via_out_)) +
            si.to_way_cost_ +
            P::node_cost(w.r_->node_properties_[label.node_.get_node()]);
        path.insert(next_it, {si.via_in_, via_cost, si.from_way_});
        label = {s.to_, to_cost, si.to_way_};
      } else {
        cost_t const via_cost =
            P::template turning_cost<flip<SearchDir>(PathDir)>(
                *w.r_, (*it).node_, s.to_) +
            si.to_way_cost_ +
            P::node_cost(w.r_->node_properties_[si.via_out_.get_node()]);
        cost_t const from_cost =
            (si.loop_cost_ > 0
                 ? 0
                 : P::template turning_cost<flip<SearchDir>(PathDir)>(
                       *w.r_, si.via_out_, si.via_in_)) +
            si.from_way_cost_ +
            P::node_cost(w.r_->node_properties_[label.node_.get_node()]);
        path.insert(next_it, {si.via_out_, via_cost, si.to_way_});
        label = {s.from_, from_cost, si.from_way_};
      }
    }
  };

  auto add_segments = [&](std::list<path_label<typename P::node>> const& path,
                          auto& segments, auto& dist) {
    for (auto label = begin(path), next_label = label;
         (next_label = std::next(label)) != end(path);) {
      dist += add_path<P>(params, w, *w.r_, nullptr, sharing, elevations,
                          label->node_, next_label->node_, next_label->cost_,
                          segments, flip<PathDir>(SearchDir));
      label = next_label;
    }
  };

  auto handle_endpoint = [&](typename P::node& node,
                             std::vector<path::segment>& segments) {
    auto const& candidate =
        node.get_node() == wc.left_.node_ ? wc.left_ : wc.right_;
    segments.front() = {
        .polyline_ = l.get_node_candidate_path(
            wc, candidate, PathDir == direction::kBackward, loc),
        .from_level_ = candidate.lvl_,
        .to_level_ = candidate.lvl_,
        .from_ = SearchDir == flip<PathDir>(direction::kBackward)
                     ? node.get_node()
                     : node_idx_t::invalid(),
        .to_ = SearchDir == flip<PathDir>(direction::kForward)
                   ? node.get_node()
                   : node_idx_t::invalid(),

        .way_ = way_idx_t::invalid(),
        .cost_ = candidate.cost_,
        .dist_ = static_cast<distance_t>(candidate.dist_to_node_),
        .mode_ = node.get_mode()};
    return candidate.dist_to_node_;
  };

  auto label_path = std::list<path_label<typename P::node>>{};
  auto dist = 0.0;

  retrieve_path(mp_node, label_path);
  label_path.emplace_front(mp_node, kInfeasible, way_idx_t::invalid());
  resolve_shortcuts(label_path);
  add_segments(label_path, path_segments, dist);
  return dist + handle_endpoint(mp_node, path_segments);
}

template <Profile P>
path reconstruct_ch(typename P::parameters const& params,
                    ways const& w,
                    lookup const& l,
                    sharing_data const* sharing,
                    elevation_storage const* elevations,
                    ch_dijkstra<P>& d,
                    shortcut_storage<typename P::node> const& sc_stor,
                    location const& from,
                    location const& to,
                    way_candidate const& start,
                    way_candidate const& dest,
                    cost_t const cost,
                    direction const dir) {
  auto fwd_node = d.tentative_mp_.first;
  auto bwd_node = d.tentative_mp_.second;
  auto fwd_segments = std::vector{path::segment{}};
  auto bwd_segments = std::vector{path::segment{}};
  auto fwd_dist = 0.0;
  auto bwd_dist = 0.0;

  if (dir == direction::kForward) {
    fwd_dist =
        reconstruct_ch_partially<direction::kForward, direction::kForward, P>(
            params, w, l, sharing, elevations, sc_stor, d.cost_s_, from, start,
            fwd_node, fwd_segments);
    bwd_dist =
        reconstruct_ch_partially<direction::kForward, direction::kBackward, P>(
            params, w, l, sharing, elevations, sc_stor, d.cost_t_, to, dest,
            bwd_node, bwd_segments);
  } else {
    fwd_dist =
        reconstruct_ch_partially<direction::kBackward, direction::kForward, P>(
            params, w, l, sharing, elevations, sc_stor, d.cost_s_, from, start,
            fwd_node, fwd_segments);
    bwd_dist =
        reconstruct_ch_partially<direction::kBackward, direction::kBackward, P>(
            params, w, l, sharing, elevations, sc_stor, d.cost_t_, to, dest,
            bwd_node, bwd_segments);
  }

  bwd_segments.back().cost_ -=
      P::node_cost(w.r_->node_properties_[d.tentative_mp_.second.get_node()]);
  std::ranges::reverse(dir == direction::kForward ? bwd_segments
                                                  : fwd_segments);
  // TODO replace with .append_range as soon as Apple Clang version is updated
  fwd_segments.insert(end(fwd_segments), begin(bwd_segments),
                      end(bwd_segments));

  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : fwd_segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = cost,
                .dist_ = fwd_dist + bwd_dist,
                .elevation_ = path_elevation,
                .segments_ = fwd_segments};

  d.cost_t_.at(bwd_node.get_key()).write(bwd_node, p);
  return p;
}

template <Profile P>
path reconstruct(typename P::parameters const& params,
                 ways const& w,
                 lookup const& l,
                 bitvec<node_idx_t> const* blocked,
                 sharing_data const* sharing,
                 elevation_storage const* elevations,
                 dijkstra<P> const& d,
                 location const& from,
                 location const& to,
                 way_candidate const& start,
                 way_candidate const& dest,
                 node_candidate const& dest_nc,
                 typename P::node const dest_node,
                 cost_t const cost,
                 direction const dir) {

  auto n = dest_node;
  auto segments = std::vector<path::segment>{
      {.polyline_ = l.get_node_candidate_path(dest, dest_nc,
                                              dir == direction::kForward, to),
       .from_level_ = dest_nc.lvl_,
       .to_level_ = dest_nc.lvl_,
       .from_ =
           dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .to_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc.cost_,
       .dist_ = static_cast<distance_t>(dest_nc.dist_to_node_),
       .mode_ = dest_node.get_mode()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = d.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(n) - d.get_cost(*pred));
      dist += add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                          n, expected_cost, segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_nc =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back(
      {.polyline_ = l.get_node_candidate_path(
           start, start_nc, dir == direction::kBackward, from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
       .dist_ = static_cast<distance_t>(start_nc.dist_to_node_),
       .mode_ = n.get_mode()});
  if (dir == direction::kForward) {
    std::reverse(begin(segments), end(segments));
  }
  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = cost,
                .dist_ = start_nc.dist_to_node_ + dist + dest_nc.dist_to_node_,
                .elevation_ = path_elevation,
                .segments_ = segments};
  d.cost_.at(dest_node.get_key()).write(dest_node, p);
  return p;
}

bool component_seen(ways const& w,
                    match_view_t matches,
                    size_t match_idx,
                    unsigned times = 1) {
  auto this_component = w.r_->way_component_[matches[match_idx].way_];
  for (auto j = 0U; j < match_idx; ++j) {
    if (w.r_->way_component_[matches[j].way_] == this_component) {
      if (--times == 0) {
        return true;
      }
    }
  }
  return false;
}

template <Profile P>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename P::node,
                         path>>
best_candidate(typename P::parameters const& params,
               ways const& w,
               dijkstra<P>& d,
               level_t const lvl,
               match_view_t m,
               cost_t const max,
               direction const dir,
               bool should_continue,
               way_candidate const& start,
               double const limit_squared_max_matching_distance) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = P::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    P::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!P::is_dest_reachable(params, *w.r_, node, dest.way_,
                                flip(opposite(dir), x->way_dir_), dir)) {
        return;
      }

      auto const target_cost = d.get_cost(node);
      if (target_cost == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + x->cost_;
      if (total_cost < best_cost.cost_) {
        best_node = node;
        best_cost.cost_ = static_cast<cost_t>(total_cost);
      }
    });
    return std::pair{best_node, best_cost};
  };

  for (auto const [j, dest] : utl::enumerate(m)) {
    if (w.r_->way_component_[start.way_] != w.r_->way_component_[dest.way_]) {
      continue;
    }
    if (!should_continue && component_seen(w, m, j, 10)) {
      continue;
    }
    if (std::pow(dest.dist_to_way_, 2) > limit_squared_max_matching_distance &&
        j > kBottomKDefinitelyConsidered) {
      break;
    }
    auto best_node = P::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid()) {
        auto const [x_node, x_cost] = get_best(dest, x);
        if (x_cost.cost_ < best_cost.cost_) {
          best = x;
          best_node = x_node;
          best_cost = x_cost;
        }
      }
    }

    if (best != nullptr) {
      return best_cost.cost_ < max
                 ? std::optional{std::tuple{best, &dest, best_node, best_cost}}
                 : std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<path> try_direct(osr::location const& from,
                               osr::location const& to) {
  auto const dist = geo::distance(from.pos_, to.pos_);
  if (dist < 8.0) {
    return std::optional{path{
        .cost_ = 60U,
        .dist_ = dist,
        .segments_ = {path::segment{.polyline_ = {from.pos_, to.pos_},
                                    .from_level_ = from.lvl_,
                                    .to_level_ = to.lvl_,
                                    .from_ = node_idx_t::invalid(),
                                    .to_ = node_idx_t::invalid(),
                                    .way_ = way_idx_t::invalid(),
                                    .cost_ = 60U,
                                    .dist_ = static_cast<distance_t>(dist)}},
        .uses_elevator_ = false}};
  } else {
    return std::nullopt;
  }
}

template <Profile P>
std::optional<path> route_bidirectional(typename P::parameters const& params,
                                        ways const& w,
                                        lookup const& l,
                                        bidirectional<P>& b,
                                        location const& from,
                                        location const& to,
                                        match_view_t from_match,
                                        match_view_t to_match,
                                        cost_t const max,
                                        direction const dir,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  b.reset(params, max, from, to);
  if (b.radius_ == max) {
    return std::nullopt;
  }

  auto const limit_squared_max_matching_distance =
      geo::approx_squared_distance(from.pos_, to.pos_,
                                   b.distance_lon_degrees_) /
      kMaxMatchingDistanceSquaredRatio;

  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (b.max_reached_1_ && component_seen(w, from_match, i)) {
      continue;
    }
    auto const start_way = start.way_;
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename P::label{node, nc->cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              b.add_start(params, w, label, sharing);
            });
      }
    }
    if (b.pq1_.empty()) {
      continue;
    }
    for (auto const [j, end] : utl::enumerate(to_match)) {
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }
      if (b.max_reached_2_ && component_seen(w, to_match, j)) {
        continue;
      }
      if (std::pow(end.dist_to_way_, 2) > limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }
      auto const end_way = end.way_;
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(
              *w.r_, end_way, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) {
                auto label = typename P::label{node, nc->cost_};
                label.track(label, *w.r_, end_way, node.get_node(), false);
                b.add_end(params, w, label, sharing);
              });
        }
      }
      if (b.pq2_.empty()) {
        continue;
      }
      auto const should_continue =
          b.run(params, w, *w.r_, max, blocked, sharing, elevations, dir);

      if (b.meet_point_1_.get_node() == node_idx_t::invalid()) {
        if (should_continue) {
          continue;
        }
        return std::nullopt;
      }

      auto const cost = b.get_cost_to_mp(b.meet_point_1_, b.meet_point_2_);

      return reconstruct_bi(params, w, l, blocked, sharing, elevations, b, from,
                            to, start, end, cost, dir);
    }
    b.pq1_.clear();
    b.pq2_.clear();
    b.cost2_.clear();
    b.max_reached_2_ = false;
  }
  return std::nullopt;
}

template <Profile P>
std::optional<path> route_dijkstra(typename P::parameters const& params,
                                   ways const& w,
                                   lookup const& l,
                                   dijkstra<P>& d,
                                   location const& from,
                                   location const& to,
                                   match_view_t from_match,
                                   match_view_t to_match,
                                   cost_t const max,
                                   direction const dir,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  auto const limit_squared_max_matching_distance =
      std::pow(geo::distance(from.pos_, to.pos_), 2) /
      kMaxMatchingDistanceSquaredRatio;

  d.reset(max);
  auto should_continue = true;
  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    if (utl::none_of(to_match, [&](way_candidate const& end) {
          return w.r_->way_component_[start.way_] ==
                 w.r_->way_component_[end.way_];
        })) {
      continue;
    }

    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir,
            [&](auto const node) { d.add_start(w, {node, nc->cost_}); });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    should_continue =
        d.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
        should_continue;

    auto const c = best_candidate(params, w, d, to.lvl_, to_match, max, dir,
                                  should_continue, start,
                                  limit_squared_max_matching_distance);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from,
                            to, start, *wc, *nc, node, p.cost_, dir);
    }
  }

  return std::nullopt;
}

template <Profile P>
std::optional<path> route_ch_dijkstra(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    ch_dijkstra<P>& d,
    location const& from,
    location const& to,
    match_view_t from_match,
    match_view_t to_match,
    cost_t const max,
    direction const dir,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    shortcut_storage<typename P::node> const& sc_stor) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  auto const limit_squared_max_matching_distance =
      std::pow(geo::distance(from.pos_, to.pos_), 2) /
      kMaxMatchingDistanceSquaredRatio;

  d.reset(max);
  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (d.max_reached_s_ && component_seen(w, from_match, i)) {
      continue;
    }

    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir,
            [&](auto const node) { d.add_start(w, {node, nc->cost_}); });
      }
    }

    if (d.pq_s_.empty()) {
      continue;
    }
    for (auto const [j, end] : utl::enumerate(to_match)) {
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }
      if (d.max_reached_t_ && component_seen(w, to_match, j)) {
        continue;
      }
      if (std::pow(end.dist_to_way_, 2) > limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }
      auto const end_way = end.way_;
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(
              *w.r_, end_way, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) {
                auto label = typename P::label{node, nc->cost_};
                label.track(label, *w.r_, end_way, node.get_node(), false);
                d.add_end(w, label);
              });
        }
      }
      if (d.pq_t_.empty()) {
        continue;
      }

      auto const should_continue =
          d.run(params, w, *w.r_, max, sharing, elevations, sc_stor, dir);

      if (d.tentative_mp_.first.get_node() == node_idx_t::invalid()) {
        if (should_continue) {
          continue;
        }
        return std::nullopt;
      }

      return reconstruct_ch<P>(params, w, l, sharing, elevations, d, sc_stor,
                               from, to, start, end, d.tentative_cost_, dir);
    }
    d.reset_partially();
  }
  return std::nullopt;
}

template <Profile P>
std::vector<std::optional<path>> route(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    dijkstra<P>& d,
    location const& from,
    std::vector<location> const& to,
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const dir,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct) {
  auto result = std::vector<std::optional<path>>{};
  result.resize(to_match.size());

  if (from_match.empty()) {
    return result;
  }

  auto const distance_lng_degrees = geo::approx_distance_lng_degrees(from.pos_);

  d.reset(max);
  auto should_continue = true;
  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    auto const start_way = start.way_;
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename P::label{node, nc->cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              d.add_start(w, label);
            });
      }
    }

    should_continue =
        d.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
        should_continue;

    auto found = 0U;
    for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
      if (r.has_value()) {
        ++found;
      } else if (auto const direct = try_direct(from, t); direct.has_value()) {
        r = direct;
      } else {
        auto const limit_squared_max_matching_distance =
            geo::approx_squared_distance(from.pos_, t.pos_,
                                         distance_lng_degrees) /
            kMaxMatchingDistanceSquaredRatio;

        auto const c =
            best_candidate(params, w, d, t.lvl_, m, max, dir, should_continue,
                           start, limit_squared_max_matching_distance);
        if (c.has_value()) {
          auto [nc, wc, n, p] = *c;
          d.cost_.at(n.get_key()).write(n, p);
          if (do_reconstruct(p)) {
            p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                               from, t, start, *wc, *nc, n, p.cost_, dir);
            p.uses_elevator_ = true;
          }
          r = std::make_optional(p);
          ++found;
        }
      }
    }

    if (found == result.size()) {
      return result;
    }
  }

  return result;
}

std::optional<path> route_bidirectional(profile_parameters const& params,
                                        ways const& w,
                                        lookup const& l,
                                        search_profile const profile,
                                        location const& from,
                                        location const& to,
                                        cost_t const max,
                                        direction const dir,
                                        double const max_match_distance,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations) {
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto const from_match =
        l.match<P>(pp, from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_bidirectional(pp, w, l, get_bidirectional<P>(), from, to,
                               from_match, to_match, max, dir, blocked, sharing,
                               elevations);
  });
}

std::vector<std::optional<path>> route(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct) {
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::vector<std::optional<path>> {
        auto const& pp = std::get<typename P::parameters>(params);
        auto const from_match =
            l.match<P>(pp, from, false, dir, max_match_distance, blocked);
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto const to_match = utl::to_vec(to, [&](auto&& x) {
          return l.match<P>(pp, x, true, dir, max_match_distance, blocked);
        });
        return route(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                     to_match, max, dir, blocked, sharing, elevations,
                     do_reconstruct);
      });
}

std::optional<path> route_dijkstra(profile_parameters const& params,
                                   ways const& w,
                                   lookup const& l,
                                   search_profile const profile,
                                   location const& from,
                                   location const& to,
                                   cost_t const max,
                                   direction const dir,
                                   double const max_match_distance,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations) {
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto const from_match =
        l.match<P>(pp, from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_dijkstra(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                          to_match, max, dir, blocked, sharing, elevations);
  });
}

std::optional<path> route_ch_dijkstra(profile_parameters const& params,
                                      ways const& w,
                                      lookup const& l,
                                      search_profile const profile,
                                      location const& from,
                                      location const& to,
                                      cost_t const max,
                                      direction const dir,
                                      double const max_match_distance,
                                      sharing_data const* sharing,
                                      elevation_storage const* elevations) {
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto const& wrp_sc_stor = get_shortcut_storage<typename P::node>();
    if (!wrp_sc_stor) {
      return std::nullopt;
    }
    auto const from_match =
        l.match<P>(pp, from, false, dir, max_match_distance, nullptr);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, nullptr);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }
    return route_ch_dijkstra(pp, w, l, get_ch_dijkstra<P>(), from, to,
                             from_match, to_match, max, dir, sharing,
                             elevations, *wrp_sc_stor);
  });
}

std::vector<std::optional<path>> route(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const dir,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct) {
  if (from_match.empty()) {
    return std::vector<std::optional<path>>(to.size());
  }
  return with_profile(profile, [&]<Profile P>(P&&) {
    return route(std::get<typename P::parameters>(params), w, l,
                 get_dijkstra<P>(), from, to, from_match, to_match, max, dir,
                 blocked, sharing, elevations, do_reconstruct);
  });
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          routing_algorithm algo) {
  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }

  switch (algo) {
    case routing_algorithm::kDijkstra:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_dijkstra(std::get<typename P::parameters>(params), w, l,
                              get_dijkstra<P>(), from, to, from_match, to_match,
                              max, dir, blocked, sharing, elevations);
      });
    case routing_algorithm::kAStarBi:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_bidirectional(std::get<typename P::parameters>(params), w,
                                   l, get_bidirectional<P>(), from, to,
                                   from_match, to_match, max, dir, blocked,
                                   sharing, elevations);
      });
    case routing_algorithm::kCHDijkstra:
      return with_profile(profile, [&]<Profile P>(P&&) {
        auto const& wrp_sc_stor = get_shortcut_storage<typename P::node>();
        if (!wrp_sc_stor) {
          return std::optional<path>{};
        }
        return route_ch_dijkstra(std::get<typename P::parameters>(params), w, l,
                                 get_ch_dijkstra<P>(), from, to, from_match,
                                 to_match, max, dir, sharing, elevations,
                                 *wrp_sc_stor);
      });
  }
  throw utl::fail("not implemented");
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          routing_algorithm algo) {
  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing ||
      profile == search_profile::kCarParkingWheelchair ||
      profile == search_profile::kCarParking) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }
  switch (algo) {
    case routing_algorithm::kDijkstra:
      return route_dijkstra(params, w, l, profile, from, to, max, dir,
                            max_match_distance, blocked, sharing, elevations);
    case routing_algorithm::kAStarBi:
      return route_bidirectional(params, w, l, profile, from, to, max, dir,
                                 max_match_distance, blocked, sharing,
                                 elevations);
    case routing_algorithm::kCHDijkstra:
      return route_ch_dijkstra(params, w, l, profile, from, to, max, dir,
                               max_match_distance, sharing, elevations);
  }
  throw utl::fail("not implemented");
}

}  // namespace osr
