#pragma once

#include <cstdint>
#include <vector>

#include "osmium/handler.hpp"

struct rtree;
struct tg_geom;

namespace osr {

struct low_emission_zones {
  low_emission_zones();
  ~low_emission_zones();

  low_emission_zones(low_emission_zones const&) = delete;
  low_emission_zones& operator=(low_emission_zones const&) = delete;

  static bool is_low_emission_zone(osmium::TagList const& tags);

  void add_area(osmium::Area const& area);

  bool empty() const { return zones_.empty(); }

  bool intersects(osmium::Way const& way) const;

  rtree* rtree_{nullptr};
  std::vector<tg_geom*> zones_;
};

struct low_emission_zone_handler : public osmium::handler::Handler {
  explicit low_emission_zone_handler(low_emission_zones& zones)
      : zones_{zones} {}
  void area(osmium::Area const& area) { zones_.add_area(area); }
  low_emission_zones& zones_;
};

struct low_emission_zone_relation_handler : public osmium::handler::Handler {
  void relation(osmium::Relation const& relation);
  bool has_low_emission_zone_{false};
};

}  // namespace osr
