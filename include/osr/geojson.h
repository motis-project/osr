#pragma once

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

template <typename Writer, typename State>
void write_node(ways const& x, Writer& w, node_idx_t const i, State const* s) {
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
  if (s != nullptr) {
    w.String("dist");
    w.Int(s->dist_[i]);
    w.String("pred");
    w.Uint64(to_idx(s->pred_[i]));
  }
  w.EndObject();

  w.String("geometry");
  write_point(w, x.get_node_pos(i));

  w.String("id");
  w.Uint64(to_idx(i));
  w.EndObject();
}

template <typename Writer, typename Vec>
void write_line_string(Writer& w, Vec const& line) {
  w.StartObject();
  w.String("type");
  w.String("LineString");
  w.String("coordinates");
  w.StartArray();
  for (auto const& loc : line) {
    write_lon_lat(w, loc);
  }
  w.EndArray();
  w.EndObject();
}

template <typename Writer>
void write_way(ways const& x, Writer& w, way_idx_t const n) {
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
    w.String("way_idx");
    w.Uint64(to_idx(n));
    w.EndObject();

    w.String("geometry");
    auto const from_pos = x.get_node_pos(from);
    auto const to_pos = x.get_node_pos(to);
    write_line_string(w, std::initializer_list<geo::latlng>{from_pos, to_pos});

    w.String("style");
    w.StartObject();
    w.String("stroke");
    w.String("#FF0000");
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
  w.String("way_idx");
  w.Uint64(to_idx(n));
  w.String("stroke");
  w.String("#00FF00");
  w.String("stroke-opacity");
  w.Double(0.3);
  w.EndObject();

  w.String("geometry");
  write_line_string(w, x.way_polylines_[n]);

  w.String("style");
  w.StartObject();
  w.EndObject();

  w.EndObject();
}

}  // namespace osr