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

vec<point> parking_edge_connection_polyline(
    ways::routing::parking_edge const& parking_edge) {
  auto line = vec<point>{};
  line.reserve(parking_edge.connection_.size() + 2);
  line.push_back(parking_edge.from_.additional_point_);
  for (auto const& p : parking_edge.connection_) {
    line.push_back(p);
  }
  line.push_back(parking_edge.to_.additional_point_);

  return line;
}

vec<point> parking_edge_offset_polyline(
    ways const& w,
    ways::routing::parking_edge::offset const& offset,
    bool const is_left) {
  auto line = vec{offset.additional_point_};

  auto const way_idx = offset.way_;
  auto const target_node = is_left ? offset.left_ : offset.right_;
  if (target_node == node_idx_t::invalid()) {
    return {};
  }
  auto const stop_node = w.node_to_osm_[target_node];
  auto const add_point = [&](unsigned const i) {
    line.push_back(w.way_polylines_[way_idx][i]);
  };

  auto i = is_left ? offset.segment_ : offset.segment_ + 1;
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

  return line;
}

geo::polyline parking_edge_polyline(
    ways const& w,
    ways::routing::parking_edge const& parking_edge,
    node_idx_t const from,
    node_idx_t const to) {
  auto line = geo::polyline{};
  auto const reverse = [](vec<point>&& points) {
    std::reverse(begin(points), end(points));
    return points;
  };
  auto previous = geo::latlng();
  auto const add_points = [&](vec<point> const& points) {
    for (auto const& p : points) {
      if (line.empty() || previous != p) {
        line.emplace_back(p.as_latlng());
        previous = p;
      }
    }
  };

  add_points(reverse(parking_edge_offset_polyline(
      w, parking_edge.from_, parking_edge.from_.left_ == from)));
  add_points(parking_edge_connection_polyline(parking_edge));
  add_points(parking_edge_offset_polyline(w, parking_edge.to_,
                                          parking_edge.to_.left_ == to));

  return line;
}

}  // namespace osr
