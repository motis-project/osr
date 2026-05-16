#pragma once

#include "boost/url/parse.hpp"

#include "net/web_server/web_server.h"

#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/with_profile.h"

namespace osr::backend {

enum geometry { polyline, geojson };

struct osrm_request {
  search_profile profile_;
  std::vector<location> coords_;
  unsigned alt_{0};
  geometry geom_{polyline};
  bool steps_{false};
  bool annotations_{false};
  unsigned number_{1};
};

inline unsigned parse_alternative(std::string_view const& c) {
  if (c != "true" && c != "false") {
    unsigned n;
    std::from_chars(c.data(), c.data() + c.size(), n);
    return n;
  }
  return 0;
}

inline location parse_location(std::string_view const& c) {
  double lat, lon;
  auto const sep = c.find(',');
  std::from_chars(c.data(), c.data() + sep, lon);
  std::from_chars(c.data() + sep + 1, c.data() + c.size(), lat);
  return {{lat, lon}, kNoLevel};
}

inline std::vector<location> parse_locations(std::string_view c) {
  std::vector<location> locs;
  size_t it;
  while ((it = c.find(';')) != std::string_view::npos) {
    locs.emplace_back(parse_location(c.substr(0, it)));
    c.remove_prefix(it + 1);
  }
  locs.emplace_back(parse_location(c));
  return locs;
}

inline boost::json::value to_line_string(path const& p) {
  auto x = boost::json::array{};
  for (auto const& s : p.segments_) {
    x.emplace_back(
        boost::json::array{s.polyline_[0].lng(), s.polyline_[0].lat()});
  }
  auto const last = p.segments_.front().polyline_.front();
  x.emplace_back(boost::json::array{last.lng(), last.lat()});
  return {{"type", "LineString"}, {"coordinates", x}};
}

inline std::string osrm_nearest_response(const osrm_request& req,
                                         lookup const& l,
                                         ways const& w) {
  const auto from = req.coords_[0];
  boost::json::object r;
  const auto candidates = with_profile(req.profile_, [&]<Profile P>(P&&) {
    double dist = 100;
    match_t match;
    while (match.size() < req.number_) {
      match = l.get_way_candidates<P>(
          std::get<typename P::parameters>(get_parameters(req.profile_)), from,
          false, direction::kForward, dist, nullptr);
      dist *= 2;
    }
    return match;
  });
  boost::json::array waypoints;
  for (std::size_t i = 0; i < req.number_; ++i) {
    const auto& c = candidates[i];
    boost::json::object wp;
    auto id = [&](const auto& n) {
      return n.valid() ? w.node_to_osm_[n.node_] : osm_node_idx_t{0};
    };
    wp["nodes"] = boost::json::array{to_idx(id(c.left_)), to_idx(id(c.right_))};
    wp["distance"] = geo::distance(c.closest_point_on_way_, from.pos_);
    wp["name"] = w.get_way_name(c.way_);
    wp["location"] = boost::json::array{c.closest_point_on_way_.lng(),
                                        c.closest_point_on_way_.lat()};
    waypoints.emplace_back(wp);
  }
  r["waypoints"] = waypoints;
  r["code"] = "Ok";
  return boost::json::serialize(r);
}

inline std::string osrm_route_response(const osrm_request& req,
                                       std::optional<path> const& p) {
  boost::json::object r;
  if (!p.has_value()) {
    r["code"] = "NoRoute";
    return boost::json::serialize(r);
  }

  r["code"] = "Ok";

  boost::json::object route;
  route["distance"] = p->dist_;
  route["duration"] = p->cost_;
  route["geometry"] =
      req.geom_ == geometry::geojson ? to_line_string(p.value()) : "polyline";
  route["weight"] = p->cost_;
  route["weight_name"] = "duration";

  boost::json::array legs;
  for (auto const& s : p->segments_) {
    boost::json::object leg;
    leg["distance"] = s.dist_;
    leg["duration"] = s.cost_;
    leg["weight"] = s.cost_;

    // turn by turn instructions
    boost::json::array steps;
    std::string summary;
    if (req.steps_) {
      // TODO:: summary, steps
    }

    leg["steps"] = steps;
    leg["summary"] = summary;
    if (req.annotations_) {
      // TODO
    }
    legs.emplace_back(leg);
  }

  route["legs"] = legs;

  boost::json::array routes;
  routes.emplace_back(route);
  r["routes"] = routes;

  return boost::json::serialize(r);
}

inline osrm_request parse_request(net::web_server::http_req_t const& r) {
  auto const url = boost::urls::parse_uri_reference(r.target());
  osrm_request req{};
  auto segs = url->segments().begin();
  auto params = url->params();
  auto const version = *(++segs);
  auto const profile = *(++segs);
  auto const coords = *(++segs);

  req.profile_ = to_profile(profile);
  req.coords_ = parse_locations(coords);

  auto get = [&](const auto& str) {
    std::optional<std::string> value;
    auto const v = params.find(str);
    if (v != params.end()) value = (*v).value;
    return value;
  };

  const auto alt = get("alternatives");
  const auto g = get("geometries");
  const auto s = get("steps");
  const auto a = get("annotations");
  const auto n = get("number");

  if (alt) {
    req.alt_ = parse_alternative(*alt);
  }
  if (g) {
    req.geom_ = g == "geojson" ? geometry::geojson : geometry::polyline;
  }
  if (s) {
    req.steps_ = s == "true";
  }
  if (a) {
    req.annotations_ = a == "true";
  }
  if (n) {
    std::from_chars(n->data(), n->data() + n->size(), req.number_);
  }
  return req;
}

}  // namespace osr::backend
