#include "osr/to_geojson.h"

#include "fmt/ranges.h"

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "utl/pairwise.h"

#include "osr/ways.h"

namespace osr {

template <typename Writer, typename Location>
void write_lon_lat(Writer& w, Location const& location) {
  w.StartArray();
  w.Double(location.lng());
  w.Double(location.lat());
  w.EndArray();
}

template <typename Writer, typename Location>
void write_point(Writer& w, Location const& loc) {
  w.StartObject();
  w.String("type");
  w.String("Point");
  w.String("coordinates");
  write_lon_lat(w, loc);
  w.EndObject();
}

template <typename Writer>
void write_node(ways const& x, Writer& w, node_idx_t const i) {
  w.StartObject();
  w.String("type");
  w.String("Feature");

  w.String("properties");
  w.StartObject();
  w.String("id");
  w.Uint64(to_idx(i));
  w.String("osm_node_id");
  w.Uint64(to_idx(x.node_to_osm_[i]));
  w.String("ways");
  w.String(
      fmt::format("{}", x.node_ways_[i] | std::views::transform([&](auto&& j) {
                          return x.way_osm_idx_[j];
                        }))
          .c_str());
  w.EndObject();

  w.String("geometry");
  write_point(w, x.get_node_pos(i));

  w.String("id");
  w.Uint64(to_idx(i));
  w.EndObject();
}

template <typename Writer, typename Vec>
void write_line_string(Writer& writer, Vec const& line) {
  writer.StartObject();
  writer.String("type");
  writer.String("LineString");
  writer.String("coordinates");
  writer.StartArray();
  for (auto const& loc : line) {
    write_lon_lat(writer, loc);
  }
  writer.EndArray();
  writer.EndObject();
}

template <typename Writer>
void write(ways const& x, Writer& w) {
  w.StartObject();
  w.String("type");
  w.String("FeatureCollection");
  w.String("features");
  w.StartArray();

  auto const n_nodes = x.node_to_osm_.size();
  for (auto n = node_idx_t{0U}; n != n_nodes; ++n) {
    write_node(x, w, n);
  }

  auto const n_ways = x.way_osm_idx_.size();
  for (auto n = way_idx_t{0U}; n != n_ways; ++n) {
    auto const way_nodes = utl::nwise<2>(x.way_nodes_[n]);
    auto const dists = x.way_node_dist_[n];
    auto way_nodes_it = std::begin(way_nodes);
    auto dist_it = std::begin(dists);
    utl::verify(way_nodes.size() == dists.size(),
                "way {}: {} node pairs vs {} distances", n, way_nodes.size(),
                dists.size());
    for (; dist_it != end(dists); ++way_nodes_it, ++dist_it) {
      auto const& [from, to] = *way_nodes_it;
      auto const dist = *dist_it;
      w.StartObject();
      w.String("type");
      w.String("Feature");

      w.String("properties");
      w.StartObject();
      w.String("distance");
      w.Uint(dist);
      w.String("osm_way_id");
      w.Uint64(to_idx(x.way_osm_idx_[n]));
      w.EndObject();

      w.String("geometry");
      auto const from_pos = x.get_node_pos(from);
      auto const to_pos = x.get_node_pos(to);
      write_line_string(w,
                        std::initializer_list<geo::latlng>{from_pos, to_pos});

      w.String("style");
      w.StartObject();
      w.String("stroke");
      w.String("red");
      w.EndObject();

      w.EndObject();
    }

    w.StartObject();
    w.String("type");
    w.String("Feature");

    w.String("properties");
    w.StartObject();
    w.String("osm_way_id");
    w.Uint64(to_idx(x.way_osm_idx_[n]));
    w.EndObject();

    w.String("geometry");
    write_line_string(w, x.way_polylines_[n]);

    w.String("style");
    w.StartObject();
    w.String("stroke");
    w.String("#00FF00");
    w.EndObject();

    w.EndObject();
  }

  w.EndArray();
  w.EndObject();
}

std::string to_geojson(ways const& x) {
  auto sb = rapidjson::StringBuffer{};
  auto w = rapidjson::PrettyWriter<rapidjson::StringBuffer>{sb};
  write(x, w);
  return sb.GetString();
}

}  // namespace osr