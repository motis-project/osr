#include "boost/json.hpp"

#include "utl/pairwise.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

#include "osr/dijkstra.h"
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
    auto const nodes = w_.way_nodes_[i];
    auto const way_nodes = utl::nwise<2>(nodes);
    auto const dists = w_.way_node_dist_[i];
    auto way_nodes_it = std::begin(way_nodes);
    auto dist_it = std::begin(dists);
    auto const p = w_.way_properties_[i];
    for (; dist_it != end(dists); ++way_nodes_it, ++dist_it) {
      auto const& [from, to] = *way_nodes_it;
      auto const dist = *dist_it;
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties",
           {{"type", "edge"},
            {"osm_way_id", to_idx(w_.way_osm_idx_[i])},
            {"distance", dist},
            {"car", p.is_car_accessible()},
            {"bike", p.is_bike_accessible()},
            {"foot", p.is_foot_accessible()},
            {"oneway_car", p.is_oneway_car()},
            {"oneway_bike", p.is_oneway_bike()},
            {"max_speed", p.max_speed_km_per_h()},
            {"level", to_float(level_t{p.level_})},
            {"is_elevator", p.is_elevator()}}},
          {"geometry", to_line_string(std::initializer_list<geo::latlng>{
                           w_.get_node_pos(from), w_.get_node_pos(to)})}});
    }

    features_.emplace_back(
        boost::json::value{{"type", "Feature"},
                           {"properties",
                            {{"type", "geometry"},
                             {"osm_way_id", to_idx(w_.way_osm_idx_[i])},
                             {"access_car", p.is_car_accessible()},
                             {"access_bike", p.is_bike_accessible()},
                             {"access_foot", p.is_foot_accessible()},
                             {"oneway_car", p.is_oneway_car()},
                             {"oneway_bike", p.is_oneway_bike()},
                             {"max_speed", p.max_speed_km_per_h()},
                             {"level", to_float(level_t{p.level_})},
                             {"is_elevator", p.is_elevator()}}},
                           {"geometry", to_line_string(w_.way_polylines_[i])}});

    nodes_.insert(begin(nodes), end(nodes));
  }

  std::string finish(dijkstra_state const* s) {
    for (auto const n : nodes_) {
      auto const p = w_.node_properties_[n];
      //      auto const e = s == nullptr ? std::nullopt : s->get(n);
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties",
           {{"osm_node_id", to_idx(w_.node_to_osm_[n])},
            {"car", p.is_car_accessible()},
            {"bike", p.is_bike_accessible()},
            {"foot", p.is_walk_accessible()},
            //            {"pred", to_idx(e && e->pred_ != node_idx_t::invalid()
            //                                ? w_.node_to_osm_[e->pred_]
            //                                : osm_node_idx_t{0U})},
            //            {"dist", e ? e->dist_ : -1},
            {"is_entrance", static_cast<bool>(p.is_entrance_)},
            {"is_elevator", static_cast<bool>(p.is_elevator_)}}},
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