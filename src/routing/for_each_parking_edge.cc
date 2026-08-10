#include "osr/routing/for_each_parking_edge.h"

#include <functional>

namespace osr {

void for_each_parking_edge(ways::routing const& r,
                           node_idx_t const node_idx,
                           std::function<void(parking_edge_idx_t)> const& f) {
  if (r.has_parking_edges_.test(node_idx)) {
    auto it = std::lower_bound(
        begin(r.node_parking_edges_), end(r.node_parking_edges_), node_idx,
        [&](pair<node_idx_t, parking_edge_idx_t> const& p, node_idx_t const n) {
          return p.first < n;
        });
    for (; it != end(r.node_parking_edges_) && it->first == node_idx; ++it) {
      f(it->second);
    }
  }
}

vec<point> parking_edge_offset_polyline(
    ways const& w,
    ways::routing::parking_edge const& parking_edge,
    bool const is_from,
    bool const is_left) {
  auto const offset = is_from ? parking_edge.from_ : parking_edge.to_;
  return parking_edge_offset_polyline(w,
                                      is_from ? parking_edge.connection_.front()
                                              : parking_edge.connection_.back(),
                                      is_left ? offset.left_ : offset.right_,
                                      offset.way_, offset.segment_, is_left);
}

vec<point> parking_edge_offset_polyline(
    ways const& w,
    ways::routing::parking_edge const& parking_edge,
    bool const is_from,
    node_idx_t const node_idx) {
  auto const& offset = is_from ? parking_edge.from_ : parking_edge.to_;
  auto const is_left = offset.left_ == node_idx;
  return parking_edge_offset_polyline(w, parking_edge, is_from, is_left);
}

vec<point> parking_edge_offset_polyline(ways const& w,
                                        point const& start_point,
                                        node_idx_t const end_node,
                                        way_idx_t const way_idx,
                                        unsigned const segment_idx,
                                        bool const is_left) {
  auto path = vec{start_point};

  auto const stop_node = w.node_to_osm_[end_node];
  auto const add_point = [&](unsigned const i) {
    path.push_back(w.way_polylines_[way_idx][i]);
  };

  auto i = is_left ? segment_idx : segment_idx + 1;
  if (is_left) {
    for (; i != 0U && w.way_osm_nodes_[way_idx][i] != stop_node; --i) {
      add_point(i);
    }
    add_point(i);
  } else {
    for (; i < w.way_osm_nodes_[way_idx].size(); ++i) {
      add_point(i);
      if (w.way_osm_nodes_[way_idx][i] == stop_node) {
        break;
      }
    }
  }

  return path;
}

}  // namespace osr
