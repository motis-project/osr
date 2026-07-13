#pragma once

#include <memory>

#include "geo/latlng.h"
#include "osmium/handler.hpp"

#include "osr/conditional.h"

namespace osr {

struct ways;

struct osm_areas {
  explicit osm_areas(ways&);
  ~osm_areas();

  osm_areas(osm_areas const&) = delete;
  osm_areas& operator=(osm_areas const&) = delete;

  void add_area(osmium::Area const& area);

  bool is_in_low_emission_zone(osmium::Way const& way) const;
  conditional_timezone_idx_t get_timezone(geo::latlng const&) const;

  struct impl;
  std::unique_ptr<impl> impl_;
};

struct osm_area_handler : public osmium::handler::Handler {
  explicit osm_area_handler(osm_areas& areas) : areas_{areas} {}
  void area(osmium::Area const& area) { areas_.add_area(area); }
  osm_areas& areas_;
};

}  // namespace osr
