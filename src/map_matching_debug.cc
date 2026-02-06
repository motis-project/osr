#include "osr/routing/map_matching_debug.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>

#include "boost/iostreams/device/file.hpp"
#include "boost/iostreams/filter/gzip.hpp"
#include "boost/iostreams/filtering_stream.hpp"

#include "utl/enumerate.h"

#include "osr/lookup.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/bus.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/ferry.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/railway.h"

namespace osr {

namespace {

boost::json::array to_json_coord(geo::latlng const& p) {
  return boost::json::array{p.lng(), p.lat()};
}

boost::json::value to_json_opt_size_t(std::optional<std::size_t> const& opt) {
  return opt ? boost::json::value{static_cast<std::int64_t>(*opt)}
             : boost::json::value{nullptr};
}

char const* dir_to_str(direction d) {
  return d == direction::kForward ? "fwd" : "bwd";
}

boost::json::value to_json_node_ref(std::optional<debug_node_ref> const& ref) {
  if (!ref) {
    return nullptr;
  }
  return boost::json::object{
      {"nodeIdx", static_cast<std::int64_t>(ref->node_array_idx_)},
      {"wayPos", static_cast<std::int64_t>(ref->way_pos_)},
      {"dir", dir_to_str(ref->dir_)}};
}

boost::json::value to_json_mps(debug_match_point_segment const& mps) {
  if (!mps.valid_) {
    return nullptr;
  }
  return boost::json::object{
      {"nodeIdx", static_cast<std::int64_t>(mps.node_array_idx_)},
      {"distToNode", mps.dist_to_node_},
      {"cost", static_cast<std::int64_t>(mps.cost_)}};
}

boost::json::object to_json_route_result(debug_route_result const& rr) {
  auto obj = boost::json::object{
      {"reached", rr.reached_},
      {"cost", rr.cost_ == kInfeasible
                   ? boost::json::value{nullptr}
                   : boost::json::value{static_cast<std::int64_t>(rr.cost_)}}};
  if (rr.reached_) {
    auto path = boost::json::array{};
    for (auto const& step : rr.path_) {
      path.emplace_back(boost::json::object{
          {"nodeIdx", static_cast<std::int64_t>(step.node_array_idx_)},
          {"wayPos", static_cast<std::int64_t>(step.way_pos_)},
          {"dir", dir_to_str(step.dir_)},
          {"cost", static_cast<std::int64_t>(step.cost_)},
          {"predNodeIdx", to_json_opt_size_t(step.pred_node_array_idx_)},
          {"wayIdx", to_json_opt_size_t(step.way_array_idx_)}});
    }
    obj["path"] = std::move(path);

    auto geom = boost::json::array{};
    for (auto const& pt : rr.geometry_) {
      geom.emplace_back(to_json_coord(pt));
    }
    obj["geometry"] = std::move(geom);
  }
  return obj;
}

boost::json::object to_json_match(debug_match const& dm,
                                  bool include_start_label,
                                  bool include_route_results) {
  auto obj = boost::json::object{
      {"matchIdx", static_cast<std::int64_t>(dm.match_idx_)},
      {"wayIdx", static_cast<std::int64_t>(dm.way_array_idx_)},
      {"projectedPoint", to_json_coord(dm.projected_point_)},
      {"distToWay", dm.dist_to_way_},
      {"waySegmentIdx", dm.way_segment_idx_},
      {"oneway", dm.oneway_},
      {"additionalNodeIdx",
       static_cast<std::int64_t>(dm.additional_node_array_idx_)},
      {"fwdNode", to_json_node_ref(dm.fwd_node_)},
      {"bwdNode", to_json_node_ref(dm.bwd_node_)},
      {"fwdOut", to_json_mps(dm.fwd_out_)},
      {"fwdIn", to_json_mps(dm.fwd_in_)},
      {"bwdOut", to_json_mps(dm.bwd_out_)},
      {"bwdIn", to_json_mps(dm.bwd_in_)}};

  if (include_start_label && dm.start_label_) {
    obj["startLabel"] = boost::json::object{
        {"matchPenalty",
         static_cast<std::int64_t>(dm.start_label_->match_penalty_)},
        {"prevSegmentCost",
         static_cast<std::int64_t>(dm.start_label_->prev_segment_cost_)},
        {"costOffset",
         static_cast<std::int64_t>(dm.start_label_->cost_offset_)},
        {"totalStartCost",
         static_cast<std::int64_t>(dm.start_label_->total_start_cost_)}};
  }

  if (include_route_results) {
    obj["fwdResult"] = to_json_route_result(dm.fwd_result_);
    obj["bwdResult"] = to_json_route_result(dm.bwd_result_);
    if (dm.beeline_from_match_idx_) {
      obj["beelineFromMatchIdx"] =
          static_cast<std::int64_t>(*dm.beeline_from_match_idx_);
    }
    if (dm.beeline_dist_) {
      obj["beelineDist"] = static_cast<std::int64_t>(*dm.beeline_dist_);
    }
  }

  return obj;
}

}  // namespace

template <Profile P>
void write_map_match_debug(
    ways const& w,
    lookup const& l,
    typename P::parameters const& params,
    std::vector<location> const& points,
    std::vector<point_data<P>> const& pds,
    std::vector<segment_data<P>> const& segments,
    matched_route const& result,
    std::function<geo::latlng(node_idx_t)> const& get_node_pos,
    std::filesystem::path const& debug_path) {
  auto dbg_ways = std::vector<boost::json::object>{};
  auto dbg_nodes = std::vector<boost::json::object>{};
  auto dbg_route_segments = std::vector<debug_route_segment>{};
  auto way_to_array_idx = hash_map<way_idx_t, std::size_t>{};
  auto node_to_array_idx = hash_map<node_idx_t, std::size_t>{};

  std::function<std::size_t(way_idx_t, bool, std::optional<way_idx_t>,
                            std::vector<geo::latlng> const*)>
      register_way =
          [&](way_idx_t const way_idx, bool is_additional,
              std::optional<way_idx_t> underlying,
              std::vector<geo::latlng> const* add_geom) -> std::size_t {
    if (way_idx == way_idx_t::invalid()) {
      return std::numeric_limits<std::size_t>::max();
    }
    if (auto const it = way_to_array_idx.find(way_idx);
        it != way_to_array_idx.end()) {
      return it->second;
    }
    auto const idx = dbg_ways.size();
    way_to_array_idx[way_idx] = idx;

    auto way_obj = boost::json::object{};
    way_obj["idx"] = static_cast<std::int64_t>(idx);
    way_obj["internalId"] = static_cast<std::int64_t>(to_idx(way_idx));
    way_obj["isAdditionalEdge"] = is_additional;

    if (is_additional && underlying) {
      way_obj["underlyingWayIdx"] = static_cast<std::int64_t>(
          register_way(*underlying, false, std::nullopt, nullptr));
    }

    if (!is_additional) {
      way_obj["osmId"] =
          static_cast<std::int64_t>(to_idx(w.way_osm_idx_[way_idx]));

      // Geometry
      auto geom = boost::json::array{};
      for (auto const& pt : w.way_polylines_[way_idx]) {
        geom.emplace_back(
            boost::json::array{pt.as_latlng().lng(), pt.as_latlng().lat()});
      }
      way_obj["geometry"] = std::move(geom);

      // Node indices
      auto const& way_nodes = w.r_->way_nodes_[way_idx];
      if (!way_nodes.empty()) {
        way_obj["fromNodeId"] =
            static_cast<std::int64_t>(to_idx(way_nodes.front()));
        way_obj["toNodeId"] =
            static_cast<std::int64_t>(to_idx(way_nodes.back()));
      }

      // Node distances
      auto dists = boost::json::array{};
      for (auto i = 0U; i < w.r_->way_node_dist_[way_idx].size(); ++i) {
        dists.emplace_back(
            static_cast<std::int64_t>(w.r_->get_way_node_distance(
                way_idx, static_cast<std::uint16_t>(i))));
      }
      way_obj["nodeDistances"] = std::move(dists);

      // Properties
      auto const& p = w.r_->way_properties_[way_idx];
      way_obj["properties"] = boost::json::object{
          {"car", p.is_car_accessible()},
          {"bike", p.is_bike_accessible()},
          {"foot", p.is_foot_accessible()},
          {"bus", p.is_bus_accessible()},
          {"busWithPenalty", p.is_bus_accessible_with_penalty()},
          {"railway", p.is_railway_accessible()},
          {"isBigStreet", p.is_big_street()},
          {"isDestination", p.is_destination()},
          {"onewayCar", p.is_oneway_car()},
          {"onewayBike", p.is_oneway_bike()},
          {"onewayPsv", p.is_oneway_psv()},
          {"maxSpeedKmh", p.max_speed_km_per_h()},
          {"speedLimit", p.speed_limit_},
          {"fromLevel", p.from_level().to_float()},
          {"toLevel", p.to_level().to_float()},
          {"isElevator", p.is_elevator()},
          {"isSidewalkSeparate", p.is_sidewalk_separate()},
          {"isSteps", p.is_steps()},
          {"isParking", p.is_parking()},
          {"isRamp", p.is_ramp()},
          {"inRoute", p.in_route()},
          {"component",
           static_cast<std::int64_t>(to_idx(w.r_->way_component_[way_idx]))}};
    } else if (add_geom) {
      auto geom = boost::json::array{};
      for (auto const& pt : *add_geom) {
        geom.emplace_back(boost::json::array{pt.lng(), pt.lat()});
      }
      way_obj["geometry"] = std::move(geom);
    }

    dbg_ways.emplace_back(std::move(way_obj));
    return idx;
  };

  auto const register_node =
      [&](node_idx_t const node_idx, bool is_additional = false,
          geo::latlng const* add_pos = nullptr) -> std::size_t {
    if (node_idx == node_idx_t::invalid()) {
      return std::numeric_limits<std::size_t>::max();
    }
    if (auto const it = node_to_array_idx.find(node_idx);
        it != node_to_array_idx.end()) {
      return it->second;
    }
    auto const idx = dbg_nodes.size();
    node_to_array_idx[node_idx] = idx;

    auto node_obj = boost::json::object{};
    node_obj["idx"] = static_cast<std::int64_t>(idx);
    node_obj["internalId"] = static_cast<std::int64_t>(to_idx(node_idx));
    node_obj["isAdditionalNode"] = is_additional;

    if (!is_additional) {
      node_obj["osmId"] =
          static_cast<std::int64_t>(to_idx(w.node_to_osm_[node_idx]));
      auto const pos = w.get_node_pos(node_idx).as_latlng();
      node_obj["pos"] = boost::json::array{pos.lng(), pos.lat()};

      auto const& p = w.r_->node_properties_[node_idx];
      node_obj["properties"] = boost::json::object{
          {"car", p.is_car_accessible()},
          {"bike", p.is_bike_accessible()},
          {"foot", p.is_walk_accessible()},
          {"bus", p.is_bus_accessible()},
          {"busWithPenalty", p.is_bus_accessible_with_penalty()},
          {"isRestricted", w.r_->node_is_restricted_[node_idx]},
          {"isEntrance", p.is_entrance()},
          {"isElevator", p.is_elevator()},
          {"isParking", p.is_parking()},
          {"isMultiLevel", p.is_multi_level()},
          {"fromLevel", p.from_level().to_float()},
          {"toLevel", p.to_level().to_float()}};

      auto way_indices = boost::json::array{};
      for (auto const& wi : w.r_->node_ways_[node_idx]) {
        way_indices.emplace_back(static_cast<std::int64_t>(
            register_way(wi, false, std::nullopt, nullptr)));
      }
      node_obj["wayIndices"] = std::move(way_indices);
    } else if (add_pos) {
      node_obj["pos"] = boost::json::array{add_pos->lng(), add_pos->lat()};
    }

    dbg_nodes.emplace_back(std::move(node_obj));
    return idx;
  };

  auto const n_route_segments = points.size() - 1U;

  auto max_segment_cost = cost_t{0};

  // Collect debug data for each segment
  for (auto seg_idx = 0U; seg_idx < n_route_segments; ++seg_idx) {
    auto const& from_pd = pds[seg_idx];
    auto const& to_pd = pds[seg_idx + 1U];
    auto const& seg = segments[seg_idx];
    segment_data<P> const* prev_seg =
        seg_idx != 0U ? &segments[seg_idx - 1U] : nullptr;

    auto dbg_seg = debug_route_segment{};
    dbg_seg.segment_idx_ = seg_idx;
    dbg_seg.from_point_idx_ = seg_idx;
    dbg_seg.to_point_idx_ = seg_idx + 1U;
    dbg_seg.all_beelined_ = seg.all_beelined_;
    dbg_seg.min_cost_ = seg.min_cost_;
    dbg_seg.max_cost_ = seg.max_cost_;
    dbg_seg.dijkstra_cost_limit_ = static_cast<cost_t>(
        seg.d_.pq_.n_buckets() > 0 ? seg.d_.pq_.n_buckets() - 1 : 0);
    dbg_seg.max_reached_in_dijkstra_ = seg.d_.max_reached_;
    dbg_seg.dijkstra_early_termination_max_cost_ =
        seg.d_.early_termination_max_cost_;
    dbg_seg.dijkstra_terminated_early_max_cost_ =
        seg.d_.terminated_early_max_cost_;
    dbg_seg.dijkstra_remaining_destinations_ = seg.d_.remaining_destinations_;
    dbg_seg.d_dijkstra_ = seg.d_dijkstra_;

    max_segment_cost = std::max(max_segment_cost, seg.min_cost_);

    auto const to_debug_mps =
        [&](node_candidate const& nc) -> debug_match_point_segment {
      if (!nc.valid()) {
        return {};
      }
      auto const node_arr_idx =
          register_node(nc.node_, w.is_additional_node(nc.node_));
      return {.valid_ = true,
              .node_array_idx_ = node_arr_idx,
              .dist_to_node_ = nc.dist_to_node_,
              .cost_ = nc.cost_};
    };

    // Collect start matches
    for (auto const [mi, mw] : utl::enumerate(from_pd.matched_ways_)) {
      auto dm = debug_match{};
      dm.match_idx_ = mi;
      dm.way_array_idx_ = register_way(mw.way_, false, std::nullopt, nullptr);
      dm.projected_point_ = mw.projected_point_;
      dm.dist_to_way_ = mw.dist_to_way_;
      dm.way_segment_idx_ = mw.segment_idx_;
      dm.oneway_ = mw.oneway_;
      dm.additional_node_array_idx_ =
          register_node(mw.additional_node_idx_, true, &mw.projected_point_);

      if (mw.fwd_node_ != P::node::invalid()) {
        dm.fwd_node_ = debug_node_ref{
            .node_array_idx_ = node_to_array_idx[mw.additional_node_idx_],
            .way_pos_ = way_pos_t{0U},
            .dir_ = direction::kForward};
      }
      if (mw.bwd_node_ != P::node::invalid()) {
        dm.bwd_node_ = debug_node_ref{
            .node_array_idx_ = node_to_array_idx[mw.additional_node_idx_],
            .way_pos_ = way_pos_t{0U},
            .dir_ = direction::kBackward};
      }

      dm.fwd_out_ = to_debug_mps(mw.fwd_out_);
      dm.fwd_in_ = to_debug_mps(mw.fwd_in_);
      dm.bwd_out_ = to_debug_mps(mw.bwd_out_);
      dm.bwd_in_ = to_debug_mps(mw.bwd_in_);

      // Start label costs
      auto const match_penalty =
          static_cast<cost_t>(P::heuristic(params, mw.dist_to_way_));
      auto const prev_seg_cost = prev_seg != nullptr
                                     ? std::min(mw.fwd_cost_, mw.bwd_cost_)
                                     : cost_t{0U};
      auto const cost_offset =
          prev_seg != nullptr ? prev_seg->min_cost_ : cost_t{0U};
      dm.start_label_ = debug_start_label{
          .match_penalty_ = match_penalty,
          .prev_segment_cost_ = prev_seg_cost,
          .cost_offset_ = cost_offset,
          .total_start_cost_ =
              prev_seg != nullptr && prev_seg_cost != kInfeasible
                  ? static_cast<cost_t>(match_penalty + prev_seg_cost -
                                        cost_offset)
                  : match_penalty};

      dbg_seg.start_matches_.push_back(std::move(dm));
    }

    // Collect dest matches with route reconstruction
    for (auto const [mi, mw] : utl::enumerate(to_pd.matched_ways_)) {
      auto dm = debug_match{};
      dm.match_idx_ = mi;
      dm.way_array_idx_ = register_way(mw.way_, false, std::nullopt, nullptr);
      dm.projected_point_ = mw.projected_point_;
      dm.dist_to_way_ = mw.dist_to_way_;
      dm.way_segment_idx_ = mw.segment_idx_;
      dm.oneway_ = mw.oneway_;
      dm.additional_node_array_idx_ =
          register_node(mw.additional_node_idx_, true, &mw.projected_point_);

      if (mw.fwd_node_ != P::node::invalid()) {
        dm.fwd_node_ = debug_node_ref{
            .node_array_idx_ = node_to_array_idx[mw.additional_node_idx_],
            .way_pos_ = way_pos_t{0U},
            .dir_ = direction::kForward};
      }
      if (mw.bwd_node_ != P::node::invalid()) {
        dm.bwd_node_ = debug_node_ref{
            .node_array_idx_ = node_to_array_idx[mw.additional_node_idx_],
            .way_pos_ = way_pos_t{0U},
            .dir_ = direction::kBackward};
      }

      dm.fwd_out_ = to_debug_mps(mw.fwd_out_);
      dm.fwd_in_ = to_debug_mps(mw.fwd_in_);
      dm.bwd_out_ = to_debug_mps(mw.bwd_out_);
      dm.bwd_in_ = to_debug_mps(mw.bwd_in_);

      // Route results from dijkstra
      auto const reconstruct_route =
          [&](typename P::node const& dest_node,
              cost_t dest_cost) -> debug_route_result {
        if (dest_cost == kInfeasible) {
          return {.reached_ = false, .cost_ = kInfeasible};
        }
        // Check if destination is actually in the cost map
        auto const dest_it = seg.d_.cost_.find(dest_node.get_key());
        if (dest_it == seg.d_.cost_.end()) {
          return {.reached_ = false, .cost_ = kInfeasible};
        }
        auto dbg_result =
            debug_route_result{.reached_ = true, .cost_ = dest_cost};
        auto n = std::optional{dest_node};
        while (n) {
          auto const it = seg.d_.cost_.find(n->get_key());
          if (it == seg.d_.cost_.end()) {
            break;  // Node not in cost map, stop reconstruction
          }
          auto const& entry = it->second;
          auto const pred = entry.pred(*n);
          auto const cost = entry.cost(*n);
          auto const node_idx = n->get_node();
          auto const is_add =
              w.is_additional_node(node_idx) ||
              (seg.sharing_ && seg.sharing_->is_additional_node(node_idx));
          auto const node_arr_idx = register_node(node_idx, is_add);
          auto step = debug_route_step{
              .node_array_idx_ = node_arr_idx,
              .way_pos_ = way_pos_t{0U},  // simplified
              .dir_ = n->get_direction().value_or(direction::kForward),
              .cost_ = cost};
          if (pred) {
            auto const pred_node_idx = pred->get_node();
            auto const pred_is_add =
                w.is_additional_node(pred_node_idx) ||
                (seg.sharing_ &&
                 seg.sharing_->is_additional_node(pred_node_idx));
            step.pred_node_array_idx_ =
                register_node(pred_node_idx, pred_is_add);
          }
          dbg_result.path_.push_back(std::move(step));
          dbg_result.geometry_.push_back(get_node_pos(node_idx));
          n = pred;
        }
        std::reverse(dbg_result.path_.begin(), dbg_result.path_.end());
        std::reverse(dbg_result.geometry_.begin(), dbg_result.geometry_.end());
        return dbg_result;
      };

      if (mw.fwd_node_ != P::node::invalid()) {
        dm.fwd_result_ = reconstruct_route(mw.fwd_node_, mw.fwd_cost_);
      }
      if (mw.bwd_node_ != P::node::invalid()) {
        dm.bwd_result_ = reconstruct_route(mw.bwd_node_, mw.bwd_cost_);
      }

      // Beeline info
      if (seg.all_beelined_ && mw.beeline_dist_ > 0) {
        dm.beeline_from_match_idx_ = mw.beeline_from_;
        dm.beeline_dist_ = mw.beeline_dist_;
      }

      dbg_seg.dest_matches_.push_back(std::move(dm));
    }

    // Segment-level beeline (when no dest matches)
    if (seg.all_beelined_) {
      auto bee = debug_beeline{};
      if (from_pd.matched_ways_.empty() && to_pd.matched_ways_.empty()) {
        bee.reason_ = "no_matches";
        bee.from_point_ = from_pd.loc_.pos_;
        bee.to_point_ = to_pd.loc_.pos_;
      } else if (to_pd.matched_ways_.empty()) {
        bee.reason_ = "no_dest_matches";
        bee.from_match_idx_ = seg.beeline_from_;
        bee.from_point_ =
            from_pd.matched_ways_[seg.beeline_from_].projected_point_;
        bee.to_point_ = to_pd.loc_.pos_;
      } else if (from_pd.matched_ways_.empty()) {
        bee.reason_ = "no_start_matches";
        bee.from_point_ = from_pd.loc_.pos_;
        // to info is in dest_matches
      } else {
        bee.reason_ = "no_destinations_reached";
      }
      bee.distance_ = seg.beeline_dist_;
      bee.cost_ = seg.min_cost_;
      dbg_seg.beeline_ = bee;
    }

    // Collect additional edges
    for (auto const& [node_idx, edges] : seg.additional_edges_) {
      for (auto const& edge : edges) {
        auto const idx = dbg_ways.size();
        auto way_obj = boost::json::object{};
        way_obj["idx"] = static_cast<std::int64_t>(idx);
        if (edge.underlying_way_ != way_idx_t::invalid()) {
          way_obj["internalId"] =
              static_cast<std::int64_t>(to_idx(edge.underlying_way_));
          way_obj["underlyingWayIdx"] = static_cast<std::int64_t>(
              register_way(edge.underlying_way_, false, std::nullopt, nullptr));
          if (edge.underlying_way_ < w.way_osm_idx_.size()) {
            way_obj["osmId"] = static_cast<std::int64_t>(
                to_idx(w.way_osm_idx_[edge.underlying_way_]));
          }
        } else {
          way_obj["internalId"] = 0;
        }
        way_obj["isAdditionalEdge"] = true;
        way_obj["fromNodeId"] = static_cast<std::int64_t>(to_idx(node_idx));
        way_obj["toNodeId"] = static_cast<std::int64_t>(to_idx(edge.node_));
        way_obj["reverse"] = edge.reverse_;

        auto geom = boost::json::array{};
        for (auto const& pt : edge.polyline_) {
          geom.emplace_back(boost::json::array{pt.lng(), pt.lat()});
        }
        way_obj["geometry"] = std::move(geom);

        dbg_ways.emplace_back(std::move(way_obj));
        dbg_seg.additional_edge_ways_.push_back(idx);
      }
    }

    // Collect labels for nodes within 1500m radius of segment start/end
    constexpr auto kLabelRadius = 1500.0;  // meters
    auto const start_pos = from_pd.loc_.pos_;
    auto const end_pos = to_pd.loc_.pos_;

    // Find ways near start and end, register their nodes, then collect labels
    auto radius_nodes = hash_set<node_idx_t>{};

    // Helper to collect nodes from ways in a radius around a point
    auto const find_nodes_near = [&](geo::latlng const& pos) {
      l.find(geo::box{pos, kLabelRadius}, [&](way_idx_t const way) {
        for (auto const node : w.r_->way_nodes_[way]) {
          auto const node_pos = w.get_node_pos(node).as_latlng();
          if (geo::distance(node_pos, pos) <= kLabelRadius) {
            radius_nodes.insert(node);
          }
        }
      });
    };

    find_nodes_near(start_pos);
    find_nodes_near(end_pos);

    // Now process each node in radius
    for (auto const node_idx : radius_nodes) {
      // Skip additional nodes
      if (w.is_additional_node(node_idx)) {
        continue;
      }

      // Register the node if not already registered
      auto const arr_idx = register_node(node_idx, false);

      P::resolve_all(*w.r_, node_idx, kNoLevel, [&](auto const& n) {
        auto const cost = seg.d_.get_cost(n);
        if (cost != kInfeasible) {
          auto label = debug_node_label{};
          label.cost_ = cost;

          // Get label string from node.print()
          auto ss = std::stringstream{};
          n.print(ss, w);
          label.label_str_ = ss.str();

          // Get predecessor
          auto const& entry_it = seg.d_.cost_.find(n.get_key());
          if (entry_it != seg.d_.cost_.end()) {
            auto const pred = entry_it->second.pred(n);
            if (pred.has_value()) {
              auto const pred_node_idx = pred->get_node();
              // Register predecessor too if needed
              auto const pred_arr_idx = register_node(
                  pred_node_idx, w.is_additional_node(pred_node_idx));
              if (pred_arr_idx != std::numeric_limits<std::size_t>::max()) {
                label.pred_node_array_idx_ = pred_arr_idx;
              }
            }
          }

          dbg_seg.node_labels_[arr_idx].push_back(std::move(label));
        }
      });
    }

    // Collect labels for start additional nodes
    for (auto const& dm : dbg_seg.start_matches_) {
      auto const add_node_arr_idx = dm.additional_node_array_idx_;
      if (add_node_arr_idx == std::numeric_limits<std::size_t>::max()) {
        continue;
      }

      // Get the matched_way to access the fwd/bwd nodes
      auto const& from_mw = from_pd.matched_ways_[dm.match_idx_];

      auto const add_start_label_for_node = [&](typename P::node const& n) {
        if (n == P::node::invalid()) {
          return;
        }
        auto const cost = seg.d_.get_cost(n);
        if (cost == kInfeasible) {
          return;
        }

        auto label = debug_node_label{};
        label.cost_ = cost;

        // Get label string from node.print()
        auto ss = std::stringstream{};
        n.print(ss, w);
        label.label_str_ = ss.str();

        // Get predecessor
        auto const& entry_it = seg.d_.cost_.find(n.get_key());
        if (entry_it != seg.d_.cost_.end()) {
          auto const pred = entry_it->second.pred(n);
          if (pred.has_value()) {
            auto const pred_node_idx = pred->get_node();
            auto const pred_is_add =
                w.is_additional_node(pred_node_idx) ||
                (seg.sharing_ &&
                 seg.sharing_->is_additional_node(pred_node_idx));
            auto const pred_arr_idx = register_node(pred_node_idx, pred_is_add);
            if (pred_arr_idx != std::numeric_limits<std::size_t>::max()) {
              label.pred_node_array_idx_ = pred_arr_idx;
            }
          }
        }

        dbg_seg.node_labels_[add_node_arr_idx].push_back(std::move(label));
      };

      add_start_label_for_node(from_mw.fwd_node_);
      add_start_label_for_node(from_mw.bwd_node_);
    }

    // Collect labels for destination additional nodes
    for (auto const& dm : dbg_seg.dest_matches_) {
      auto const add_node_arr_idx = dm.additional_node_array_idx_;
      if (add_node_arr_idx == std::numeric_limits<std::size_t>::max()) {
        continue;
      }

      // Get the matched_way to access the fwd/bwd nodes
      auto const& to_mw = to_pd.matched_ways_[dm.match_idx_];

      auto const add_label_for_node = [&](typename P::node const& n) {
        if (n == P::node::invalid()) {
          return;
        }
        auto const cost = seg.d_.get_cost(n);
        if (cost == kInfeasible) {
          return;
        }

        auto label = debug_node_label{};
        label.cost_ = cost;

        // Get label string from node.print()
        auto ss = std::stringstream{};
        n.print(ss, w);
        label.label_str_ = ss.str();

        // Get predecessor
        auto const& entry_it = seg.d_.cost_.find(n.get_key());
        if (entry_it != seg.d_.cost_.end()) {
          auto const pred = entry_it->second.pred(n);
          if (pred.has_value()) {
            auto const pred_node_idx = pred->get_node();
            auto const pred_is_add =
                w.is_additional_node(pred_node_idx) ||
                (seg.sharing_ &&
                 seg.sharing_->is_additional_node(pred_node_idx));
            auto const pred_arr_idx = register_node(pred_node_idx, pred_is_add);
            if (pred_arr_idx != std::numeric_limits<std::size_t>::max()) {
              label.pred_node_array_idx_ = pred_arr_idx;
            }
          }
        }

        dbg_seg.node_labels_[add_node_arr_idx].push_back(std::move(label));
      };

      add_label_for_node(to_mw.fwd_node_);
      add_label_for_node(to_mw.bwd_node_);
    }

    dbg_route_segments.push_back(std::move(dbg_seg));
  }

  auto json_input_points = boost::json::array{};
  for (auto const& loc : points) {
    json_input_points.emplace_back(
        boost::json::object{{"lat", loc.pos_.lat()},
                            {"lng", loc.pos_.lng()},
                            {"level", loc.lvl_.to_float()}});
  }

  auto json_ways = boost::json::array{};
  for (auto const& way_obj : dbg_ways) {
    json_ways.emplace_back(way_obj);
  }

  auto json_nodes = boost::json::array{};
  for (auto const& node_obj : dbg_nodes) {
    json_nodes.emplace_back(node_obj);
  }

  auto json_route_segments = boost::json::array{};
  for (auto const& seg : dbg_route_segments) {
    auto seg_obj = boost::json::object{
        {"segmentIdx", static_cast<std::int64_t>(seg.segment_idx_)},
        {"fromPointIdx", static_cast<std::int64_t>(seg.from_point_idx_)},
        {"toPointIdx", static_cast<std::int64_t>(seg.to_point_idx_)},
        {"allBeelined", seg.all_beelined_},
        {"minCost", static_cast<std::int64_t>(seg.min_cost_)},
        {"maxCost", static_cast<std::int64_t>(seg.max_cost_)},
        {"dijkstraCostLimit",
         static_cast<std::int64_t>(seg.dijkstra_cost_limit_)},
        {"dijkstraEarlyTerminationMaxCost",
         static_cast<std::int64_t>(seg.dijkstra_early_termination_max_cost_)},
        {"maxReachedInDijkstra", seg.max_reached_in_dijkstra_},
        {"dijkstraTerminatedEarlyMaxCost",
         seg.dijkstra_terminated_early_max_cost_},
        {"dijkstraRemainingDestinations", seg.dijkstra_remaining_destinations_},
        {"dijkstraDurationUs", seg.d_dijkstra_.count()}};

    auto start_matches = boost::json::array{};
    for (auto const& dm : seg.start_matches_) {
      start_matches.emplace_back(to_json_match(dm, true, false));
    }
    seg_obj["startMatches"] = std::move(start_matches);

    auto dest_matches = boost::json::array{};
    for (auto const& dm : seg.dest_matches_) {
      dest_matches.emplace_back(to_json_match(dm, false, true));
    }
    seg_obj["destMatches"] = std::move(dest_matches);

    if (seg.beeline_) {
      seg_obj["beeline"] = boost::json::object{
          {"reason", seg.beeline_->reason_},
          {"fromMatchIdx", to_json_opt_size_t(seg.beeline_->from_match_idx_)},
          {"toMatchIdx", to_json_opt_size_t(seg.beeline_->to_match_idx_)},
          {"fromPoint", to_json_coord(seg.beeline_->from_point_)},
          {"toPoint", to_json_coord(seg.beeline_->to_point_)},
          {"distance", static_cast<std::int64_t>(seg.beeline_->distance_)},
          {"cost", static_cast<std::int64_t>(seg.beeline_->cost_)}};
    }

    // Serialize node labels
    if (!seg.node_labels_.empty()) {
      auto labels_obj = boost::json::object{};
      for (auto const& [node_arr_idx, labels] : seg.node_labels_) {
        auto labels_arr = boost::json::array{};
        for (auto const& label : labels) {
          auto label_obj = boost::json::object{
              {"label", label.label_str_},
              {"cost", static_cast<std::int64_t>(label.cost_)}};
          if (label.pred_node_array_idx_) {
            label_obj["predNodeIdx"] =
                static_cast<std::int64_t>(*label.pred_node_array_idx_);
          } else {
            label_obj["predNodeIdx"] = nullptr;
          }
          labels_arr.emplace_back(std::move(label_obj));
        }
        labels_obj[std::to_string(node_arr_idx)] = std::move(labels_arr);
      }
      seg_obj["nodeLabels"] = std::move(labels_obj);
    }

    // Serialize additional edge ways
    if (!seg.additional_edge_ways_.empty()) {
      auto add_edges = boost::json::array{};
      for (auto const idx : seg.additional_edge_ways_) {
        add_edges.emplace_back(static_cast<std::int64_t>(idx));
      }
      seg_obj["additionalEdgeWays"] = std::move(add_edges);
    }

    json_route_segments.emplace_back(std::move(seg_obj));
  }

  // Final route geometry
  auto json_final_route = boost::json::object{
      {"totalCost", static_cast<std::int64_t>(result.path_.cost_)}};
  auto final_geom = boost::json::array{};
  for (auto const& seg : result.path_.segments_) {
    for (auto const& pt : seg.polyline_) {
      final_geom.emplace_back(to_json_coord(pt));
    }
  }
  json_final_route["geometry"] = std::move(final_geom);

  // Compute bounding box
  auto min_lng = std::numeric_limits<double>::max();
  auto min_lat = std::numeric_limits<double>::max();
  auto max_lng = std::numeric_limits<double>::lowest();
  auto max_lat = std::numeric_limits<double>::lowest();

  auto const update_bounds = [&](double lng, double lat) {
    min_lng = std::min(min_lng, lng);
    min_lat = std::min(min_lat, lat);
    max_lng = std::max(max_lng, lng);
    max_lat = std::max(max_lat, lat);
  };

  for (auto const& pt : points) {
    update_bounds(pt.pos_.lng(), pt.pos_.lat());
  }
  for (auto const& node_obj : dbg_nodes) {
    if (auto const* pos = node_obj.if_contains("pos")) {
      if (auto const* arr = pos->if_array(); arr && arr->size() >= 2) {
        update_bounds((*arr)[0].as_double(), (*arr)[1].as_double());
      }
    }
  }

  auto const debug_json = boost::json::object{
      {"metadata",
       boost::json::object{
           {"nInputPoints", static_cast<std::int64_t>(points.size())},
           {"nRouteSegments", static_cast<std::int64_t>(n_route_segments)},
           {"nRouted", static_cast<std::int64_t>(result.n_routed_)},
           {"nBeelined", static_cast<std::int64_t>(result.n_beelined_)},
           {"maxSegmentCost", static_cast<std::int64_t>(max_segment_cost)},
           {"boundingBox",
            boost::json::array{boost::json::array{min_lng, min_lat},
                               boost::json::array{max_lng, max_lat}}}}},
      {"inputPoints", std::move(json_input_points)},
      {"ways", std::move(json_ways)},
      {"nodes", std::move(json_nodes)},
      {"routeSegments", std::move(json_route_segments)},
      {"finalRoute", std::move(json_final_route)},
      {"totalDurationMs", result.d_total_.count()}};

  auto out_path = debug_path;
  out_path += ".json.gz";
  auto ofs = boost::iostreams::filtering_ostream{};
  ofs.push(boost::iostreams::gzip_compressor(
      boost::iostreams::gzip_params(boost::iostreams::gzip::best_compression)));
  ofs.push(boost::iostreams::file_sink(out_path.string(), std::ios::binary));
  ofs << boost::json::serialize(debug_json);
}

template void write_map_match_debug<foot<false, elevator_tracking>>(
    ways const&,
    lookup const&,
    foot<false, elevator_tracking>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<foot<false, elevator_tracking>>> const&,
    std::vector<segment_data<foot<false, elevator_tracking>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<foot<true, elevator_tracking>>(
    ways const&,
    lookup const&,
    foot<true, elevator_tracking>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<foot<true, elevator_tracking>>> const&,
    std::vector<segment_data<foot<true, elevator_tracking>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void
write_map_match_debug<bike<bike_costing::kSafe, kElevationNoCost>>(
    ways const&,
    lookup const&,
    bike<bike_costing::kSafe, kElevationNoCost>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<bike<bike_costing::kSafe, kElevationNoCost>>> const&,
    std::vector<
        segment_data<bike<bike_costing::kSafe, kElevationNoCost>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void
write_map_match_debug<bike<bike_costing::kFast, kElevationNoCost>>(
    ways const&,
    lookup const&,
    bike<bike_costing::kFast, kElevationNoCost>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<bike<bike_costing::kFast, kElevationNoCost>>> const&,
    std::vector<
        segment_data<bike<bike_costing::kFast, kElevationNoCost>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void
write_map_match_debug<bike<bike_costing::kSafe, kElevationLowCost>>(
    ways const&,
    lookup const&,
    bike<bike_costing::kSafe, kElevationLowCost>::parameters const&,
    std::vector<location> const&,
    std::vector<
        point_data<bike<bike_costing::kSafe, kElevationLowCost>>> const&,
    std::vector<
        segment_data<bike<bike_costing::kSafe, kElevationLowCost>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void
write_map_match_debug<bike<bike_costing::kSafe, kElevationHighCost>>(
    ways const&,
    lookup const&,
    bike<bike_costing::kSafe, kElevationHighCost>::parameters const&,
    std::vector<location> const&,
    std::vector<
        point_data<bike<bike_costing::kSafe, kElevationHighCost>>> const&,
    std::vector<
        segment_data<bike<bike_costing::kSafe, kElevationHighCost>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<car>(
    ways const&,
    lookup const&,
    car::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<car>> const&,
    std::vector<segment_data<car>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<car_parking<false, false>>(
    ways const&,
    lookup const&,
    car_parking<false, false>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<car_parking<false, false>>> const&,
    std::vector<segment_data<car_parking<false, false>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<car_parking<true, false>>(
    ways const&,
    lookup const&,
    car_parking<true, false>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<car_parking<true, false>>> const&,
    std::vector<segment_data<car_parking<true, false>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<car_parking<false, true>>(
    ways const&,
    lookup const&,
    car_parking<false, true>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<car_parking<false, true>>> const&,
    std::vector<segment_data<car_parking<false, true>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<car_parking<true, true>>(
    ways const&,
    lookup const&,
    car_parking<true, true>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<car_parking<true, true>>> const&,
    std::vector<segment_data<car_parking<true, true>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<bike_sharing>(
    ways const&,
    lookup const&,
    bike_sharing::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<bike_sharing>> const&,
    std::vector<segment_data<bike_sharing>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<car_sharing<track_node_tracking>>(
    ways const&,
    lookup const&,
    car_sharing<track_node_tracking>::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<car_sharing<track_node_tracking>>> const&,
    std::vector<segment_data<car_sharing<track_node_tracking>>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<bus>(
    ways const&,
    lookup const&,
    bus::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<bus>> const&,
    std::vector<segment_data<bus>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<railway>(
    ways const&,
    lookup const&,
    railway::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<railway>> const&,
    std::vector<segment_data<railway>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

template void write_map_match_debug<ferry>(
    ways const&,
    lookup const&,
    ferry::parameters const&,
    std::vector<location> const&,
    std::vector<point_data<ferry>> const&,
    std::vector<segment_data<ferry>> const&,
    matched_route const&,
    std::function<geo::latlng(node_idx_t)> const&,
    std::filesystem::path const&);

}  // namespace osr
