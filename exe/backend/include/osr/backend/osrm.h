#pragma once

#include <net/web_server/web_server.h>
#include <boost/json/array.hpp>
#include <boost/json/object.hpp>
#include <boost/url/parse.hpp>
#include "osr/location.h"
#include "osr/routing/profile.h"

namespace osr::backend {

struct osrm_request {
  search_profile profile_;
  std::vector<location> coords_;
  unsigned alt_{0};
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
  auto sep = c.find(',');
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

inline boost::json::object osrm_route_response(std::optional<path> const& p) {
  boost::json::object r;
  if (!p.has_value()) {
    r["code"] = "NoRoute";
    return r;
  }

  r["code"] = "Ok";

  boost::json::array legs;
  for (auto const& seg : p->segments_) {
    boost::json::object leg;
    leg["distance"] = seg.dist_;
    leg["duration"] = seg.cost_;
    leg["weight"] = seg.cost_;
    leg["summary"] = "";

    boost::json::array coords;
    for (auto const& pt : seg.polyline_) {
      boost::json::array c;
      c.emplace_back(pt.lng_);
      c.emplace_back(pt.lat_);
      coords.emplace_back(std::move(c));
    }
    leg["geometry"] = std::move(coords);
    legs.emplace_back(std::move(leg));
  }

  boost::json::object route;
  route["distance"] = p->dist_;
  route["duration"] = p->cost_;
  route["weight"] = p->cost_;
  route["weight_name"] = "routability";
  route["legs"] = std::move(legs);

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
  segs++;
  auto const version = *(segs++);
  auto const profile = *(segs++);
  auto const coords = *(segs++);

  req.profile_ = to_profile(profile);
  req.coords_ = parse_locations(coords);

  if (auto const alt = params.find("alternatives"); alt != params.end()) {
    req.alt_ = parse_alt((*alt).value);
  }

  return req;
}

}  // namespace osr::backend
