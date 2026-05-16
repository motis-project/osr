#pragma once

#include <net/web_server/web_server.h>
#include <boost/json/array.hpp>
#include <boost/json/object.hpp>
#include <boost/json/value.hpp>
#include <boost/url/parse.hpp>

#include "osr/location.h"
#include "osr/routing/profile.h"

namespace osr::backend {

enum geometry { polyline, geojson };

struct osrm_request {
  search_profile profile_;
  std::vector<location> coords_;
  unsigned alt_{0};
  geometry geom_{polyline};
  bool steps_{false};
  bool annotations_{false};
};

inline unsigned parse_alt(std::string_view const& c) {
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

inline boost::json::object osrm_route_response(const osrm_request& req,
                                               std::optional<path> const& p) {
  boost::json::object r;
  if (!p.has_value()) {
    r["code"] = "NoRoute";
    return r;
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

  return r;
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

  if (auto const alt = params.find("alternatives"); alt != params.end()) {
    req.alt_ = parse_alt((*alt).value);
  }

  if (auto const geom = params.find("geometries"); geom != params.end()) {
    req.geom_ =
        (*geom).value == "geojson" ? geometry::geojson : geometry::polyline;
  }

  return req;
}

}  // namespace osr::backend
