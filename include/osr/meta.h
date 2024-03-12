#pragma once

#include "cista/memory_holder.h"

#include "fmt/ranges.h"

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include "utl/zip.h"

#include "osr/db.h"
#include "osr/graph.h"

namespace osr {

struct meta {
  meta(config const& c)
      : db_{c.db_.c_str(), c.db_max_size_},
        graph_{graph::read(cista::memory_holder{
            cista::file{c.graph_.c_str(), "r"}.content()})},
        osm_nodes_{
            cista::mmap{c.node_map_.c_str(), cista::mmap::protection::READ}},
        osm_ways_{
            cista::mmap{c.edge_map_.c_str(), cista::mmap::protection::READ}} {}

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
  void write_node(Writer& w, node_idx_t const i) {
    static node_info nfo;
    auto const osm_idx = get_node(i, nfo);

    w.StartObject();
    w.String("type");
    w.String("Feature");

    w.String("properties");
    w.StartObject();
    w.String("id");
    w.Uint64(to_idx(i));
    w.String("osm_node_id");
    w.Int64(to_idx(osm_idx));
    w.String("ways");
    w.String(fmt::format("{}", nfo.ways_).c_str());
    w.EndObject();

    w.String("geometry");
    write_point(w, nfo.p_);

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
  void write_edge(Writer& w, edge_idx_t const i) {
    w.StartObject();
    w.String("type");
    w.String("Feature");

    w.String("properties");
    w.StartObject();
    w.String("distance");
    w.Uint(graph_->edge_distance_[i]);
    w.String("osm_way_id");
    w.Uint64(to_idx(osm_ways_[i]));
    w.String("edge_idx");
    w.Uint(to_idx(i));
    w.EndObject();

    w.String("geometry");
    auto const from = get_node_pos(graph_->edge_from_[i]);
    auto const to = get_node_pos(graph_->edge_to_[i]);
    write_line_string(w, std::initializer_list<geo::latlng>{from, to});

    //    w.String("style");
    //    w.StartObject();
    //    write_edge_style(w, e.info(rg));
    //    w.EndObject();

    w.EndObject();
  }

  template <typename Writer>
  void write(Writer& w) {
    w.StartObject();
    w.String("type");
    w.String("FeatureCollection");
    w.String("features");
    w.StartArray();

    auto const n_nodes = graph_->out_edges_.size();
    for (auto n = node_idx_t{0U}; n != n_nodes; ++n) {
      write_node(w, n);
    }

    auto const n_edges = graph_->edge_distance_.size();
    for (auto n = edge_idx_t{0U}; n != n_edges; ++n) {
      write_edge(w, n);
    }

    w.EndArray();
    w.EndObject();
  }

  std::string write() {
    auto sb = rapidjson::StringBuffer{};
    auto w = rapidjson::PrettyWriter<rapidjson::StringBuffer>{sb};
    write(w);
    return sb.GetString();
  }

  point get_node_pos(node_idx_t const i) {
    auto const osm_idx = osm_nodes_[i];
    return db_.get_node_pos(osm_idx);
  }

  osm_node_idx_t get_node(node_idx_t const i, node_info& nfo) {
    auto const osm_idx = osm_nodes_[i];
    db_.get_node(osm_idx, nfo);
    return osm_idx;
  }

  osm_way_idx_t get_way(edge_idx_t const i, way_info& nfo) {
    auto const osm_idx = osm_ways_[i];
    db_.get_way(osm_idx, nfo);
    return osm_idx;
  }

  db db_;
  cista::wrapped<graph> graph_;
  edge_map_t osm_ways_;
  node_map_t osm_nodes_;
};

}  // namespace osr