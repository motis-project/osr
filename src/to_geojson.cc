#include "osr/to_geojson.h"

#include "fmt/ranges.h"

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "utl/pairwise.h"

#include "osr/dijkstra.h"
#include "osr/geojson.h"
#include "osr/ways.h"

namespace osr {

template <typename Writer>
void write(ways const& x,
           Writer& w,
           dijkstra_state const* s,
           std::vector<geo::latlng> const* start_left_path,
           std::vector<geo::latlng> const* start_right_path) {
  w.StartObject();
  w.String("type");
  w.String("FeatureCollection");
  w.String("features");
  w.StartArray();

  auto const n_nodes = x.node_to_osm_.size();
  for (auto n = node_idx_t{0U}; n != n_nodes; ++n) {
    write_node(x, w, n, s);
  }

  auto const n_ways = x.n_ways();
  for (auto n = way_idx_t{0U}; n != n_ways; ++n) {
    write_way(x, w, n);
  }

  if (start_left_path != nullptr && start_left_path->size() >= 2U) {
    w.StartObject();
    w.String("type");
    w.String("Feature");

    w.String("geometry");
    write_line_string(w, *start_left_path);
    fmt::println("LEFT PATH: {}", *start_left_path);

    w.String("properties");
    w.StartObject();
    w.String("stroke");
    w.String("#9141ac");
    w.String("stroke-width");
    w.Int(4);
    w.EndObject();

    w.EndObject();
  }

  if (start_right_path != nullptr && start_right_path->size() >= 2U) {
    w.StartObject();
    w.String("type");
    w.String("Feature");

    w.String("geometry");
    write_line_string(w, *start_right_path);

    w.String("properties");
    w.StartObject();
    w.String("stroke");
    w.String("#33d17a");
    w.String("stroke-width");
    w.Int(4);
    w.EndObject();

    w.EndObject();
  }

  w.EndArray();
  w.EndObject();
}

std::string to_geojson(ways const& x,
                       dijkstra_state const* s,
                       std::vector<geo::latlng> const* start_path_left,
                       std::vector<geo::latlng> const* start_path_right) {
  auto sb = rapidjson::StringBuffer{};
  auto w = rapidjson::PrettyWriter<rapidjson::StringBuffer>{sb};
  write(x, w, s, start_path_left, start_path_right);
  return sb.GetString();
}

}  // namespace osr