#include "osr/routing/additional_connection.h"

#include <ranges>

#include "osr/types.h"
#include "utl/pipes/transform.h"
#include "utl/to_vec.h"

#include "osr/lookup.h"

namespace osr {

namespace {
connection_idx_t to_connection_idx(ways::routing const& r,
                                   way_idx_t const way_idx) {
  utl::verify(way_idx >= r.way_component_.size(), "way_idx too low: {} < {}",
              way_idx, r.way_component_.size());
  return connection_idx_t{to_idx(way_idx) - r.way_component_.size()};
}

way_idx_t to_way_idx(ways::routing const& r, connection_idx_t const conn_idx) {
  return way_idx_t{to_idx(conn_idx) + r.way_component_.size()};
}
vec<point> get_connection(vec<point> const& interior,
                          point const& from,
                          point const& to) {
  auto connection = vec<point>{from};
  connection.reserve(interior.size() + 2);
  for (auto const& p : interior) {
    connection.emplace_back(p);
  }
  connection.emplace_back(to);
  return connection;
}

direction invert(direction const dir, bool const revert) {
  return revert ? dir == direction::kForward ? direction::kBackward
                                             : direction::kForward
                : dir;
}

vec<point> reverse(vec<point>&& points) {
  std::reverse(begin(points), end(points));
  return points;
}

geo::polyline to_polyline(vec<point> const& points) {
  return utl::to_vec(points, [](point const& p) { return p.as_latlng(); });
}
}  // namespace

void for_each_connection(ways::routing const& r,
                         node_idx_t const node_idx,
                         std::function<void(connection_idx_t)> const& f) {
  if (r.has_additional_connections_.test(node_idx)) {
    auto it = std::lower_bound(begin(r.additional_node_connections_),
                               end(r.additional_node_connections_), node_idx,
                               [&](pair<node_idx_t, connection_idx_t> const& p,
                                   node_idx_t const n) { return p.first < n; });
    for (; it != end(r.additional_node_connections_) && it->first == node_idx;
         ++it) {
      f(it->second);
    }
  }
}

bool is_additional_connection(ways::routing const& r, way_idx_t const way_idx) {
  return way_idx != way_idx_t::invalid() && way_idx >= r.way_component_.size();
}

ways::routing::additional_connection const& get_additional_connection(
    ways::routing const& r, way_idx_t const way_idx) {
  auto const conn_idx = to_connection_idx(r, way_idx);
  utl::verify(conn_idx < r.additional_connections_.size(),
              "Invalid connection index: {} >= {}", conn_idx,
              r.additional_connections_.size());
  return r.additional_connections_[conn_idx];
}

way_idx_t add_additional_connection(
    ways::routing& r,
    ways::routing::additional_connection::offset&& from,
    ways::routing::additional_connection::offset&& to,
    vec<point>&& connection,
    bool const is_parking) {
  utl::verify((from.left_ != node_idx_t::invalid() ||
               from.right_ != node_idx_t::invalid()) &&
                  (from.left_ != node_idx_t::invalid() ||
                   from.right_ != node_idx_t::invalid()),
              "Cannot add offset without valid node");
  auto const conn_idx = connection_idx_t{r.additional_connections_.size()};
  auto const add_node = [&](node_idx_t const node_idx) {
    r.additional_node_connections_.emplace_back(node_idx, conn_idx);
    if (is_parking) {
      r.has_additional_connections_.set(node_idx);
    }
  };

  for (auto const node_idx : {from.left_, from.right_, to.left_, to.right_}) {
    if (node_idx != node_idx_t::invalid()) {
      add_node(node_idx);
    }
  }
  r.additional_connections_.emplace_back(
      std::move(connection), std::move(from), std::move(to),
      geo::length(to_polyline(get_connection(connection, from.connecting_point_,
                                             to.connecting_point_))));
  return to_way_idx(r, conn_idx);
}

geo::polyline get_additional_connection_polyline(
    ways const& w,
    ways::routing::additional_connection const& conn,
    node_idx_t from,
    node_idx_t to) {
  auto polyline = geo::polyline{};
  auto const append_to_polyline = [&](vec<point> const& points) {
    polyline.reserve(polyline.size() + points.size());
    for (auto const& p : points) {
      polyline.emplace_back(p.as_latlng());
    }
  };
  append_to_polyline(reverse(get_additional_connection_offset_points(
      w, conn.from_, conn.from_.left_ == from)));
  append_to_polyline(get_additional_connection_points(conn));
  append_to_polyline(get_additional_connection_offset_points(
      w, conn.to_, conn.to_.left_ == to));

  return polyline;
}

vec<point> get_additional_connection_points(
    ways::routing::additional_connection const& connection) {
  return get_connection(connection.connection_,
                        connection.from_.connecting_point_,
                        connection.to_.connecting_point_);
}

vec<point> get_additional_connection_offset_points(
    ways const& w,
    ways::routing::additional_connection::offset const& offset,
    bool const is_left) {
  auto line = vec{offset.connecting_point_};

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

ways::routing::additional_connection::offset to_offset(
    ways const& w, way_candidate const& wc) {
  auto offset = ways::routing::additional_connection::offset{
      .connecting_point_ = point::from_latlng(wc.closest_point_on_way_),
      .way_ = wc.way_,
      .segment_ = wc.segment_idx_,
      .left_ = wc.left_.node_,
      .right_ = wc.right_.node_,
      .dist_left_ = 0U,
      .dist_right_ = 0U};

  if (wc.left_.node_ != node_idx_t::invalid()) {
    offset.dist_left_ = geo::length(
        to_polyline(get_additional_connection_offset_points(w, offset, true)));
  }
  if (wc.right_.node_ != node_idx_t::invalid()) {
    offset.dist_right_ = geo::length(
        to_polyline(get_additional_connection_offset_points(w, offset, false)));
  }

  return offset;
}

void for_each_addional_connection(
    ways::routing const& r,
    node_idx_t const node_idx,
    direction const dir,
    std::function<void(ways::routing::additional_connection const&,
                       way_idx_t,
                       node_idx_t,
                       direction,
                       additional_connection_offset from,
                       additional_connection_offset to)> const& f) {
  for_each_connection(r, node_idx, [&](connection_idx_t const connection_idx) {
    auto const& conn = r.additional_connections_[connection_idx];
    auto const g = [&](direction const conn_dir,
                       ways::routing::additional_connection::offset const& from,
                       ways::routing::additional_connection::offset const& to) {
      for (auto const from_left : {true, false}) {
        auto const start = from_left ? from.left_ : from.right_;
        if (start != node_idx) {
          continue;
        }
        for (auto const to_left : {true, false}) {
          auto const target = to_left ? to.left_ : to.right_;
          if (target != node_idx_t::invalid()) {
            f(conn, to_way_idx(r, connection_idx), target, conn_dir,
              {.offset_ = from,
               .node_ = start,
               .dist_ = from_left ? from.dist_left_ : from.dist_right_,
               .dir_ = invert(
                   from_left ? direction::kForward : direction::kBackward,
                   conn_dir == direction::kBackward)},
              {.offset_ = to,
               .node_ = target,
               .dist_ = to_left ? to.dist_left_ : to.dist_right_,
               .dir_ =
                   invert(to_left ? direction::kBackward : direction::kForward,
                          conn_dir == direction::kBackward)});
          }
        }
      }
    };
    if (dir == direction::kForward /*|| conn.reverse_allowed*/) {
      g(direction::kForward, conn.from_, conn.to_);
    }
    if (dir == direction::kBackward /*|| conn.reverse_allowed*/) {
      g(direction::kBackward, conn.to_, conn.from_);
    }
  });
}

};  // namespace osr
