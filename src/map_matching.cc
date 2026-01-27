#include "osr/routing/map_matching.h"

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <vector>

#include "utl/enumerate.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/routing/dijkstra.h"
#include "osr/routing/map_matching_data.h"
#include "osr/routing/map_matching_debug.h"
#include "osr/routing/path_reconstruction.h"
#include "osr/routing/route.h"
#include "osr/routing/with_profile.h"

#include "osr/geojson.h"

namespace osr {

template <Profile P>
std::vector<matched_way<P>> match_input_point(
    ways const& w,
    lookup const& l,
    typename P::parameters const& params,
    bitvec<node_idx_t> const* blocked,
    location const& loc) {
  auto matched_ways = std::vector<matched_way<P>>{};
  auto const approx_distance_lng_degrees =
      geo::approx_distance_lng_degrees(loc.pos_);
  auto max_match_distance = 25.0;
  auto i = 0U;

  auto const find_matches = [&]() {
    auto const squared_max_dist = std::pow(max_match_distance, 2);
    l.find(geo::box{loc.pos_, max_match_distance}, [&](way_idx_t const way) {
      auto const [squared_dist, best, segment_idx] =
          geo::approx_squared_distance_to_polyline<
              std::tuple<double, geo::latlng, size_t>>(
              loc.pos_, w.way_polylines_[way], approx_distance_lng_degrees);
      if (squared_dist < squared_max_dist) {
        auto const way_prop = w.r_->way_properties_[way];
        auto mw = matched_way<P>{
            .way_ = way,
            .dist_to_way_ = std::sqrt(squared_dist),
            .projected_point_ = best,
            .segment_idx_ = static_cast<unsigned>(segment_idx),
            .oneway_ = P::way_cost(params, way_prop, direction::kBackward,
                                   0U) == kInfeasible};
        auto const wc = way_candidate{.dist_to_way_ = mw.dist_to_way_,
                                      .way_ = way,
                                      .closest_point_on_way_ = best,
                                      .segment_idx_ = mw.segment_idx_};

        mw.fwd_out_ =
            l.find_next_node<P>(params, wc, loc, direction::kForward, loc.lvl_,
                                false, direction::kForward, blocked,
                                approx_distance_lng_degrees, best, segment_idx);
        mw.fwd_in_ =
            l.find_next_node<P>(params, wc, loc, direction::kBackward, loc.lvl_,
                                true, direction::kForward, blocked,
                                approx_distance_lng_degrees, best, segment_idx);
        mw.bwd_out_ =
            l.find_next_node<P>(params, wc, loc, direction::kBackward, loc.lvl_,
                                false, direction::kForward, blocked,
                                approx_distance_lng_degrees, best, segment_idx);
        mw.bwd_in_ =
            l.find_next_node<P>(params, wc, loc, direction::kForward, loc.lvl_,
                                false, direction::kBackward, blocked,
                                approx_distance_lng_degrees, best, segment_idx);
        if (!mw.fwd_out_.valid() && !mw.fwd_in_.valid() &&
            !mw.bwd_out_.valid() && !mw.bwd_in_.valid()) {
          return;
        }
        matched_ways.push_back(std::move(mw));
      }
    });
  };

  find_matches();

  while (matched_ways.empty() && i++ < 4U) {
    max_match_distance *= 2.0;
    find_matches();
  }

  return matched_ways;
}

template <Profile P>
matched_route map_match(
    ways const& w,
    lookup const& l,
    typename P::parameters const& params,
    std::vector<location> const& points,
    cost_t const max_segment_cost,
    bitvec<node_idx_t> const* blocked,
    elevation_storage const* elevations,
    std::function<std::optional<std::filesystem::path>(matched_route const&)>
        debug_path_fn) {
  utl::verify(points.size() >= 2, "map_match requires at least 2 points");

  auto const n_route_segments = points.size() - 1U;
  auto segments = std::vector<segment_data<P>>{};
  segments.resize(n_route_segments);

  auto const additional_node_offset = w.n_nodes();
  auto next_additional_node = node_idx_t{additional_node_offset};
  auto additional_node_coordinates = std::vector<geo::latlng>{};

  auto const get_node_pos = [&](node_idx_t const n) -> geo::latlng {
    if (n == node_idx_t::invalid()) {
      return {};
    } else if (w.is_additional_node(n)) {
      return additional_node_coordinates.at(to_idx(n) - additional_node_offset);
    } else {
      return w.get_node_pos(n).as_latlng();
    }
  };

  auto pds = utl::to_vec(points, [&](auto const& mp) {
    return point_data<P>{
        .loc_ = mp,
        .matched_ways_ = match_input_point<P>(w, l, params, blocked, mp)};
  });

  for (auto& pd : pds) {
    for (auto& mw : pd.matched_ways_) {
      additional_node_coordinates.emplace_back(mw.projected_point_);
      mw.additional_node_idx_ = next_additional_node++;
      mw.fwd_node_ = P::create_node(mw.additional_node_idx_, kNoLevel,
                                    way_pos_t{0U}, direction::kForward);
      if (!mw.oneway_) {
        mw.bwd_node_ = P::create_node(mw.additional_node_idx_, kNoLevel,
                                      way_pos_t{0U}, direction::kBackward);
      }
    }
  }

  for (auto seg_idx = 0U; seg_idx < n_route_segments; ++seg_idx) {
    auto& from_pd = pds[seg_idx];
    auto& to_pd = pds[seg_idx + 1U];
    auto& seg = segments[seg_idx];
    segment_data<P> const* prev_seg =
        seg_idx != 0U ? &segments[seg_idx - 1U] : nullptr;

    auto add_additional_edge = [&](matched_way<P> const& mw,
                                   node_candidate const& nc, bool reverse,
                                   bool outgoing) {
      if (nc.valid()) {
        auto const from_node = outgoing ? mw.additional_node_idx_ : nc.node_;
        auto const to_node = outgoing ? nc.node_ : mw.additional_node_idx_;
        auto const& path = nc.path_;
        seg.additional_edges_[from_node].push_back(additional_edge{
            .node_ = to_node,
            .distance_ =
                static_cast<distance_t>(nc.dist_to_node_ + mw.dist_to_way_ * 2),
            .underlying_way_ = mw.way_,
            .reverse_ = reverse,
            .polyline_ = path});
      }
    };

    for (auto const& mw : from_pd.matched_ways_) {
      add_additional_edge(mw, mw.fwd_out_, false, true);
      add_additional_edge(mw, mw.bwd_out_, true, true);
    }
    for (auto const& mw : to_pd.matched_ways_) {
      add_additional_edge(mw, mw.fwd_in_, false, false);
      add_additional_edge(mw, mw.bwd_in_, true, false);
    }

    auto dijkstra_max_cost = std::min(
        max_segment_cost,
        static_cast<cost_t>(
            P::heuristic(params,
                         geo::distance(from_pd.loc_.pos_, to_pd.loc_.pos_)) *
                5 +
            120));
    if (prev_seg != nullptr) {
      dijkstra_max_cost += prev_seg->max_cost_ - prev_seg->min_cost_;
    }

    seg.d_.reset(dijkstra_max_cost);

    auto const cost_offset =
        prev_seg != nullptr ? prev_seg->min_cost_ : cost_t{0U};

    for (auto const& from_mw : from_pd.matched_ways_) {
      // TODO: match penalty is often too small / equal for points that
      //   are close to each other
      auto const match_penalty =
          static_cast<cost_t>(P::heuristic(params, from_mw.dist_to_way_));
      auto const add_start = [&](typename P::node node, cost_t node_cost) {
        if (node == P::node::invalid()) {
          return;
        }
        auto cost = match_penalty;
        if (prev_seg != nullptr) {
          if (node_cost == kInfeasible) {
            return;
          }
          cost += node_cost - cost_offset;
        }
        seg.d_.add_start(w, typename P::label{node, cost});
      };

      add_start(from_mw.fwd_node_, from_mw.fwd_cost_);
      add_start(from_mw.bwd_node_, from_mw.bwd_cost_);
    }

    seg.sharing_ = std::make_unique<sharing_data>(sharing_data{
        .additional_node_offset_ = additional_node_offset,
        .additional_node_coordinates_ = additional_node_coordinates,
        .additional_edges_ = seg.additional_edges_});

    seg.d_.run(params, w, *w.r_, dijkstra_max_cost, blocked, seg.sharing_.get(),
               elevations, direction::kForward);

    auto reached = 0U;
    for (auto& to_mw : to_pd.matched_ways_) {
      if (to_mw.fwd_node_ != P::node::invalid()) {
        auto const cost = seg.d_.get_cost(to_mw.fwd_node_);
        if (cost != kInfeasible) {
          ++reached;
          to_mw.fwd_cost_ = cost;
        }
      }
      if (to_mw.bwd_node_ != P::node::invalid()) {
        auto const cost = seg.d_.get_cost(to_mw.bwd_node_);
        if (cost != kInfeasible) {
          ++reached;
          to_mw.bwd_cost_ = cost;
        }
      }
    }

    if (reached == 0U) {
      // calculate beelines
      seg.all_beelined_ = true;
      auto const prev_seg_routed =
          prev_seg != nullptr && !from_pd.matched_ways_.empty();
      for (auto& to_mw : to_pd.matched_ways_) {
        if (prev_seg_routed) {
          for (auto const [i, from_mw] :
               utl::enumerate(from_pd.matched_ways_)) {
            auto const min_from_cost =
                std::min(from_mw.fwd_cost_, from_mw.bwd_cost_);
            if (min_from_cost == kInfeasible) {
              continue;
            }

            auto const bee_dist =
                geo::distance(from_mw.projected_point_, to_mw.projected_point_);
            auto const bee_cost =
                static_cast<cost_t>(P::heuristic(params, bee_dist));
            auto const cost = static_cast<cost_t>(
                std::min(static_cast<std::uint32_t>(kInfeasible) - 1U,
                         static_cast<std::uint32_t>(min_from_cost) + bee_cost));
            if (cost < to_mw.fwd_cost_) {
              to_mw.fwd_cost_ = cost;
              to_mw.bwd_cost_ = cost;
              to_mw.beeline_from_ = i;
              to_mw.beeline_dist_ = static_cast<distance_t>(bee_dist);
            }
          }
        } else {
          auto const bee_dist =
              geo::distance(from_pd.loc_.pos_, to_mw.projected_point_);
          auto const cost = static_cast<cost_t>(P::heuristic(params, bee_dist));
          to_mw.fwd_cost_ = cost;
          to_mw.bwd_cost_ = cost;
          to_mw.beeline_dist_ = static_cast<distance_t>(bee_dist);
        }
      }
      if (to_pd.matched_ways_.empty()) {
        if (prev_seg_routed) {
          for (auto const [i, from_mw] :
               utl::enumerate(from_pd.matched_ways_)) {
            auto const min_from_cost =
                std::min(from_mw.fwd_cost_, from_mw.bwd_cost_);
            if (min_from_cost == kInfeasible) {
              continue;
            }

            auto const bee_dist =
                geo::distance(from_mw.projected_point_, to_pd.loc_.pos_);
            auto const bee_cost =
                static_cast<cost_t>(P::heuristic(params, bee_dist));
            auto const cost = static_cast<cost_t>(
                std::min(static_cast<std::uint32_t>(kInfeasible) - 1U,
                         static_cast<std::uint32_t>(min_from_cost) + bee_cost));
            if (cost < seg.min_cost_) {
              seg.min_cost_ = cost;
              seg.max_cost_ = cost;
              seg.beeline_from_ = i;
              seg.beeline_dist_ = static_cast<distance_t>(bee_dist);
            }
          }
        } else {
          auto const bee_dist =
              geo::distance(from_pd.loc_.pos_, to_pd.loc_.pos_);
          auto const cost = static_cast<cost_t>(P::heuristic(params, bee_dist));
          seg.min_cost_ = cost;
          seg.max_cost_ = cost;
          seg.beeline_dist_ = static_cast<distance_t>(bee_dist);
        }
      }
    }

    for (auto& to_mw : to_pd.matched_ways_) {
      if (to_mw.fwd_cost_ != kInfeasible) {
        seg.min_cost_ = std::min(seg.min_cost_, to_mw.fwd_cost_);
        seg.max_cost_ = std::max(seg.max_cost_, to_mw.fwd_cost_);
      }
      if (to_mw.bwd_cost_ != kInfeasible) {
        seg.min_cost_ = std::min(seg.min_cost_, to_mw.bwd_cost_);
        seg.max_cost_ = std::max(seg.max_cost_, to_mw.bwd_cost_);
      }
    }
  }

  auto result = matched_route{};
  result.path_.cost_ = 0;
  /*
   Reconstruction starts with the last segment and goes backwards.
   For each segment the destination may be known (from reconstruction of the
   next segment) or unknown (for the last segment or beelines).
   Reconstruction cases:

   1. segment routed via dijkstra, at least one destination reached
      (from matched, to matched, >=1 reached)
      reconstruct dijkstra path to
      - start from next segment (if available)
      - destination with lowest costs otherwise
   2. segment routed via dijkstra, no destination reached
      (from matched, to matched, 0 reached)
      - shortest beeline to next segment's start (if available)
      - beeline with lowest cost (all combinations from->to) otherwise
   3. no matches at all
      (from not matched, to not matched)
      - beeline between projected points
   4. destination not matched
      (from matched, to not matched)
      - shortest beeline (match -> projected point)
   5. start not matched
      (from not matched, to matched)
      - beeline to next segment's start (if available)
      - shortest beeline (projected point -> match)
   */

  auto next_dest = std::optional<node_ref>{};

  for (auto seg_idx = static_cast<int>(segments.size()) - 1; seg_idx >= 0;
       --seg_idx) {
    auto& seg = segments[static_cast<std::size_t>(seg_idx)];
    auto const& from_pd = pds[static_cast<std::size_t>(seg_idx)];
    auto const& to_pd = pds[static_cast<std::size_t>(seg_idx) + 1U];
    auto selected_dest = next_dest;

    if (!selected_dest && !to_pd.matched_ways_.empty()) {
      // select cheapest destination
      auto best_cost = std::numeric_limits<cost_t>::max();
      for (auto const [mw_idx, to_mw] : utl::enumerate(to_pd.matched_ways_)) {
        auto const to_cost = std::min(to_mw.fwd_cost_, to_mw.bwd_cost_);
        if (to_cost < best_cost) {
          best_cost = to_cost;
          selected_dest = node_ref{.matched_way_idx_ = mw_idx,
                                   .dir_ = to_mw.fwd_cost_ <= to_mw.bwd_cost_
                                               ? direction::kForward
                                               : direction::kBackward};
        }
      }
    }
    utl::verify(selected_dest.has_value() || to_pd.matched_ways_.empty(),
                "[map_match] no selected destination for seg_idx={}", seg_idx);

    auto const add_beeline = [&](geo::latlng const& from_pos,
                                 geo::latlng const& to_pos,
                                 distance_t const dist, cost_t const cost) {
      result.path_.segments_.emplace_back(path::segment{
          .polyline_ = {from_pos, to_pos},
          .cost_ = cost,
          .dist_ = dist,
      });
      seg.path_segments_ += 1U;
    };

    if (seg.all_beelined_) {
      ++result.n_beelined_;
      if (from_pd.matched_ways_.empty() && to_pd.matched_ways_.empty()) {
        // case 3
        add_beeline(from_pd.loc_.pos_, to_pd.loc_.pos_, seg.beeline_dist_,
                    seg.min_cost_);
        next_dest = {};
      } else if (!from_pd.matched_ways_.empty() &&
                 to_pd.matched_ways_.empty()) {
        // case 4
        auto const from_mw = from_pd.matched_ways_[seg.beeline_from_];
        add_beeline(from_mw.projected_point_, to_pd.loc_.pos_,
                    seg.beeline_dist_, seg.min_cost_);
        next_dest = node_ref{.matched_way_idx_ = seg.beeline_from_,
                             .dir_ = from_mw.fwd_cost_ <= from_mw.bwd_cost_
                                         ? direction::kForward
                                         : direction::kBackward};
        utl::verify(next_dest->matched_way_idx_ < from_pd.matched_ways_.size(),
                    "[map_match] [case 4] matched_way_idx_={} "
                    "from_pd.matched_ways_.size()={}",
                    next_dest->matched_way_idx_, from_pd.matched_ways_.size());
      } else if (from_pd.matched_ways_.empty() &&
                 !to_pd.matched_ways_.empty()) {
        // case 5
        assert(selected_dest.has_value());
        utl::verify(selected_dest.has_value(), "[case 5] no selected dest");
        auto const to_mw =
            to_pd.matched_ways_.at(selected_dest->matched_way_idx_);
        add_beeline(from_pd.loc_.pos_, to_mw.projected_point_,
                    to_mw.beeline_dist_, to_mw.get_cost(selected_dest->dir_));
        next_dest = {};
      } else {
        // case 2
        assert(selected_dest.has_value());
        utl::verify(selected_dest.has_value(), "[case 2] no selected dest");
        if (selected_dest->matched_way_idx_ >= to_pd.matched_ways_.size()) {
          std::cerr << "[map_match] [case 2] matched_way_idx_="
                    << selected_dest->matched_way_idx_
                    << " to_pd.matched_ways_.size()="
                    << to_pd.matched_ways_.size() << "\n";
        }
        auto const to_mw =
            to_pd.matched_ways_.at(selected_dest->matched_way_idx_);
        auto const from_mw = from_pd.matched_ways_.at(to_mw.beeline_from_);
        add_beeline(from_mw.projected_point_, to_mw.projected_point_,
                    to_mw.beeline_dist_, to_mw.get_cost(selected_dest->dir_));
        next_dest = node_ref{.matched_way_idx_ = to_mw.beeline_from_,
                             .dir_ = from_mw.fwd_cost_ <= from_mw.bwd_cost_
                                         ? direction::kForward
                                         : direction::kBackward};
        utl::verify(next_dest->matched_way_idx_ < from_pd.matched_ways_.size(),
                    "[map_match] [case 2] matched_way_idx_={} "
                    "from_pd.matched_ways_.size()={}",
                    next_dest->matched_way_idx_, from_pd.matched_ways_.size());
      }
    } else {
      // case 1
      assert(selected_dest.has_value());
      utl::verify(selected_dest.has_value(),
                  "[map_match] [case 1] no selected dest");
      ++result.n_routed_;
      auto const segments_before = result.path_.segments_.size();
      auto const& d = seg.d_;
      auto const to_mw =
          to_pd.matched_ways_.at(selected_dest->matched_way_idx_);
      auto n = std::optional{to_mw.get_node(selected_dest->dir_)};
      utl::verify(d.get_cost(*n) != kInfeasible,
                  "[map_match] [case 1] selected dest not reached");
      next_dest = {};
      while (n) {
        auto const& entry = d.cost_.at(n->get_key());
        auto const pred = entry.pred(*n);
        auto const cost = entry.cost(*n);

        if (pred) {
          auto const expected_cost =
              static_cast<cost_t>(cost - d.get_cost(*pred));

          auto const pred_node_idx = pred->get_node();
          auto const curr_node_idx = n->get_node();
          auto const pred_is_additional =
              seg.sharing_->is_additional_node(pred_node_idx);
          auto const curr_is_additional =
              seg.sharing_->is_additional_node(curr_node_idx);

          if (pred_is_additional || curr_is_additional) {
            auto polyline = std::vector<geo::latlng>{};
            auto edge_dist = distance_t{0};
            auto edge_way = way_idx_t::invalid();

            auto const edges_it = seg.additional_edges_.find(pred_node_idx);
            if (edges_it != seg.additional_edges_.end()) {
              for (auto const& ae : edges_it->second) {
                if (ae.node_ == curr_node_idx) {
                  polyline = ae.polyline_;
                  edge_dist = ae.distance_;
                  edge_way = ae.underlying_way_;
                  break;
                }
              }
            }

            if (polyline.empty()) {
              polyline = {get_node_pos(pred_node_idx),
                          get_node_pos(curr_node_idx)};
            }

            result.path_.segments_.emplace_back(path::segment{
                .polyline_ = std::move(polyline),
                .from_ =
                    pred_is_additional ? node_idx_t::invalid() : pred_node_idx,
                .to_ =
                    curr_is_additional ? node_idx_t::invalid() : curr_node_idx,
                .way_ = edge_way,
                .cost_ = expected_cost,
                .dist_ = edge_dist,
                .mode_ = n->get_mode()});
          } else {
            add_path<P>(params, w, *w.r_, blocked, seg.sharing_.get(),
                        elevations, *pred, *n, expected_cost,
                        result.path_.segments_, direction::kForward);
          }
        } else {
          for (auto const [i, from_mw] :
               utl::enumerate(from_pd.matched_ways_)) {
            if (from_mw.fwd_node_ == n || from_mw.bwd_node_ == n) {
              next_dest = node_ref{.matched_way_idx_ = i,
                                   .dir_ = from_mw.fwd_node_ == n
                                               ? direction::kForward
                                               : direction::kBackward};
              break;
            }
          }
        }

        n = pred;
      }
      seg.path_segments_ = result.path_.segments_.size() - segments_before;
      utl::verify(
          seg.path_segments_ > 0U,
          "[map_match] [case 1] no path segments reconstructed for seg_idx={}",
          seg_idx);
      utl::verify(next_dest.has_value(),
                  "[map_match] [case 1] did not find next_dest for seg_idx={}",
                  seg_idx);
    }
  }

  std::reverse(begin(result.path_.segments_), end(result.path_.segments_));

  auto offset = std::size_t{0U};
  for (auto const& seg : segments) {
    result.segment_offsets_.emplace_back(offset);
    offset += seg.path_segments_;
  }
  result.segment_offsets_.emplace_back(offset);

  if (debug_path_fn) {
    if (auto const debug_path = debug_path_fn(result); debug_path.has_value()) {
      write_map_match_debug<P>(w, l, params, points, pds, segments, result,
                               get_node_pos, *debug_path);
    }
  }

  return result;
}

matched_route map_match(
    ways const& w,
    lookup const& l,
    search_profile profile,
    profile_parameters const& params,
    std::vector<location> const& points,
    cost_t const max_segment_cost,
    bitvec<node_idx_t> const* blocked,
    elevation_storage const* elevations,
    std::function<std::optional<std::filesystem::path>(matched_route const&)>
        debug_path_fn) {
  return with_profile(profile, [&]<Profile P>(P&&) {
    return map_match<P>(w, l, std::get<typename P::parameters>(params), points,
                        max_segment_cost, blocked, elevations,
                        std::move(debug_path_fn));
  });
}

}  // namespace osr
