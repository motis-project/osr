#include "boost/json.hpp"

#include "fmt/ranges.h"
#include "fmt/std.h"

#include "utl/pairwise.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"

namespace osr {

boost::json::array to_array(point const& p) { return {p.lng(), p.lat()}; }

boost::json::array to_array(geo::latlng const& p) { return {p.lng(), p.lat()}; }

template <typename Collection>
boost::json::value to_line_string(Collection const& line) {
  auto x = boost::json::array{};
  for (auto const& p : line) {
    x.emplace_back(boost::json::array{p.lng(), p.lat()});
  }
  return {{"type", "LineString"}, {"coordinates", x}};
}

boost::json::value to_point(point const& p) {
  return {{"type", "Point"}, {"coordinates", to_array(p)}};
}

struct geojson_writer {
  void write_way(way_idx_t const i) {
    auto const nodes = w_.r_->way_nodes_[i];
    auto const way_nodes = utl::nwise<2>(nodes);
    auto const dists = w_.r_->way_node_dist_[i];
    auto way_nodes_it = std::begin(way_nodes);
    auto dist_it = std::begin(dists);
    auto const p = w_.r_->way_properties_[i];
    for (; dist_it != end(dists); ++way_nodes_it, ++dist_it) {
      auto const& [from, to] = *way_nodes_it;
      auto const dist = *dist_it;
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties",
           {{"type", "edge"},
            {"osm_way_id", to_idx(w_.way_osm_idx_[i])},
            {"internal_id", to_idx(i)},
            {"distance", dist},
            {"car", p.is_car_accessible()},
            {"bike", p.is_bike_accessible()},
            {"foot", p.is_foot_accessible()},
            {"is_destination", p.is_destination()},
            {"oneway_car", p.is_oneway_car()},
            {"oneway_bike", p.is_oneway_bike()},
            {"max_speed", p.max_speed_km_per_h()},
            {"level", to_float(level_t{p.from_level()})},
            {"to_level", to_float(level_t{p.to_level()})},
            {"is_elevator", p.is_elevator()},
            {"is_steps", p.is_steps()},
            {"is_parking", p.is_parking_}}},
          {"geometry", to_line_string(std::initializer_list<geo::latlng>{
                           w_.get_node_pos(from), w_.get_node_pos(to)})}});
    }

    features_.emplace_back(
        boost::json::value{{"type", "Feature"},
                           {"properties",
                            {{"type", "geometry"},
                             {"osm_way_id", to_idx(w_.way_osm_idx_[i])},
                             {"internal_id", to_idx(i)},
                             {"car", p.is_car_accessible()},
                             {"bike", p.is_bike_accessible()},
                             {"foot", p.is_foot_accessible()},
                             {"is_destination", p.is_destination()},
                             {"oneway_car", p.is_oneway_car()},
                             {"oneway_bike", p.is_oneway_bike()},
                             {"max_speed", p.max_speed_km_per_h()},
                             {"from_level", to_float(level_t{p.from_level()})},
                             {"to_level", to_float(level_t{p.to_level()})},
                             {"is_elevator", p.is_elevator()},
                             {"is_steps", p.is_steps()}}},
                           {"geometry", to_line_string(w_.way_polylines_[i])}});

    nodes_.insert(begin(nodes), end(nodes));
  }

  template <typename Dijkstra>
  std::string finish(Dijkstra const& s) {
    for (auto const n : nodes_) {
      auto const p = w_.r_->node_properties_[n];

      auto ss = std::stringstream{};
      Dijkstra::profile_t::resolve_all(*w_.r_, n, level_t::invalid(),
                                       [&](auto const n) {
                                         auto const cost = s.get_cost(n);
                                         if (cost != kInfeasible) {
                                           ss << "{";
                                           n.print(ss, w_);
                                           ss << ", " << cost << "}\n";
                                         }
                                       });

      auto levels = std::stringstream{};
      foot<true>::for_each_elevator_level(
          *w_.r_, n, [&](auto&& l) { levels << to_float(level_t{l}) << " "; });

      auto properties = boost::json::object{
          {"osm_node_id", to_idx(w_.node_to_osm_[n])},
          {"internal_id", to_idx(n)},
          {"car", p.is_car_accessible()},
          {"bike", p.is_bike_accessible()},
          {"foot", p.is_walk_accessible()},
          {"is_restricted", w_.r_->node_is_restricted_[n]},
          {"is_entrance", p.is_entrance()},
          {"is_elevator", p.is_elevator()},
          {"is_parking", p.is_parking()},
          {"multi_level", p.is_multi_level()},
          {"levels", levels.str()},
          {"ways", fmt::format("{}", w_.r_->node_ways_[n] |
                                         std::views::transform([&](auto&& w) {
                                           return w_.way_osm_idx_[w];
                                         }))},
          {"restrictions",
           fmt::format("{}",
                       w_.r_->node_restrictions_[n] |
                           std::views::transform([&](restriction const r) {
                             return std::pair{
                                 w_.way_osm_idx_[w_.r_->node_ways_[n][r.from_]],
                                 w_.way_osm_idx_[w_.r_->node_ways_[n][r.to_]]};
                           }))},
          {"label", ss.str().empty() ? "unreachable" : ss.str()}};
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties", properties},
          {"geometry",
           {{"type", "Point"},
            {"coordinates", to_array(w_.get_node_pos(n))}}}});
    }

    return boost::json::serialize(boost::json::value{
        {"type", "FeatureCollection"}, {"features", features_}});
  }

  ways const& w_;
  boost::json::array features_;
  hash_set<node_idx_t> nodes_;
};

}  // namespace osr