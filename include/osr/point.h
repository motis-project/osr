#pragma once

#include <ostream>

#include "osmium/osm/location.hpp"

#include "geo/latlng.h"

namespace osr {

struct point {
  friend std::ostream& operator<<(std::ostream& out, point const& c) {
    auto const l = c.as_location();
    return out << '(' << l.lat() << ", " << l.lon() << ')';
  }

  static point from_latlng(geo::latlng const& x) {
    auto const l = osmium::Location{x.lng_, x.lat_};
    return point{.lat_ = l.x(), .lng_ = l.y()};
  }

  static point from_location(osmium::Location const l) {
    return {l.x(), l.y()};
  }

  osmium::Location as_location() const { return osmium::Location{lat_, lng_}; }

  geo::latlng as_latlng() const {
    auto const l = as_location();
    return {l.lat(), l.lon()};
  }

  operator geo::latlng() const { return as_latlng(); }

  double lat() const { return as_latlng().lat_; }
  double lng() const { return as_latlng().lng_; }

  std::int32_t lat_, lng_;
};

inline auto format_as(point const p) { return std::pair{p.lat(), p.lng()}; }

}  // namespace osr