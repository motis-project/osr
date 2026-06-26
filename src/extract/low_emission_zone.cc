#include "osr/extract/low_emission_zone.h"

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <vector>

#include "osmium/osm/area.hpp"
#include "osmium/osm/relation.hpp"
#include "osmium/osm/way.hpp"

#include "utl/to_vec.h"
#include "utl/verify.h"

#include "rtree.h"
#include "tg.h"

namespace osr {

namespace {

struct geom_deleter {
  void operator()(tg_geom* geom) const { tg_geom_free(geom); }
};

using geom_ptr = std::unique_ptr<tg_geom, geom_deleter>;

geom_ptr make_way_geom(std::vector<tg_point> const& points) {
  if (points.empty()) {
    return nullptr;
  }
  if (points.size() == 1U) {
    return geom_ptr{tg_geom_new_point(points.front())};
  }

  auto* line = tg_line_new_ix(points.data(), static_cast<int>(points.size()),
                              TG_NATURAL);
  if (line == nullptr) {
    return nullptr;
  }
  auto* geom = tg_geom_new_linestring(line);
  tg_line_free(line);
  return geom_ptr{geom};
}

tg_ring* make_ring(std::vector<tg_point>& tmp, auto&& ring) {
  tmp.clear();
  for (auto const& n : ring) {
    tmp.emplace_back(tg_point{n.lon(), n.lat()});
  }
  return tg_ring_new_ix(tmp.data(), static_cast<int>(tmp.size()), TG_YSTRIPES);
}

tg_geom* make_geom(osmium::Area const& area) {
  auto polys = std::vector<tg_poly*>{};
  auto inner_rings = std::vector<tg_ring*>{};
  auto ring_points = std::vector<tg_point>{};

  for (auto const& outer_ring : area.outer_rings()) {
    inner_rings.clear();
    for (auto const& inner_ring : area.inner_rings(outer_ring)) {
      if (auto* inner = make_ring(ring_points, inner_ring); inner != nullptr) {
        inner_rings.emplace_back(inner);
      }
    }

    auto* outer = make_ring(ring_points, outer_ring);
    if (outer != nullptr) {
      polys.emplace_back(tg_poly_new(outer, inner_rings.data(),
                                     static_cast<int>(inner_rings.size())));
      tg_ring_free(outer);
    }

    for (auto* inner : inner_rings) {
      tg_ring_free(inner);
    }
  }

  auto* geom =
      tg_geom_new_multipolygon(polys.data(), static_cast<int>(polys.size()));
  for (auto* poly : polys) {
    tg_poly_free(poly);
  }
  return geom;
}

}  // namespace

bool low_emission_zones::is_low_emission_zone(osmium::TagList const& tags) {
  return tags.has_tag("boundary", "low_emission_zone");
}

low_emission_zones::low_emission_zones() : rtree_{rtree_new()} {
  utl::verify(rtree_ != nullptr, "low emission zone rtree creation failed");
}

low_emission_zones::~low_emission_zones() {
  for (auto* geom : zones_) {
    tg_geom_free(geom);
  }
  rtree_free(rtree_);
}

void low_emission_zones::add_area(osmium::Area const& area) {
  if (!is_low_emission_zone(area.tags())) {
    return;
  }

  auto geom = make_geom(area);
  if (geom == nullptr || tg_geom_error(geom) != nullptr ||
      tg_geom_is_empty(geom)) {
    if (geom != nullptr) {
      tg_geom_free(geom);
    }
    return;
  }

  auto const idx = zones_.size();
  zones_.emplace_back(geom);

  for (auto const& outer : area.outer_rings()) {
    auto const env = outer.envelope();
    if (!env.valid()) {
      continue;
    }
    auto const min =
        std::array{env.bottom_left().lon(), env.bottom_left().lat()};
    auto const max = std::array{env.top_right().lon(), env.top_right().lat()};
    rtree_insert(rtree_, min.data(), max.data(), reinterpret_cast<void*>(idx));
  }
}

bool low_emission_zones::intersects(osmium::Way const& way) const {
  if (empty() || way.nodes().empty()) {
    return false;
  }

  auto min = std::array{std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max()};
  auto max = std::array{std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::lowest()};

  for (auto const& n : way.nodes()) {
    if (!n.location().valid()) {
      return false;
    }
    min[0] = std::min(min[0], n.location().lon());
    min[1] = std::min(min[1], n.location().lat());
    max[0] = std::max(max[0], n.location().lon());
    max[1] = std::max(max[1], n.location().lat());
  }

  auto candidates = std::vector<std::size_t>{};
  rtree_search(
      rtree_, min.data(), max.data(),
      [](double const*, double const*, void const* item, void* udata) {
        auto& c = *reinterpret_cast<std::vector<std::size_t>*>(udata);
        c.emplace_back(reinterpret_cast<std::size_t>(item));
        return true;
      },
      &candidates);

  if (candidates.empty()) {
    return false;
  }

  auto const geom = make_way_geom(utl::to_vec(way.nodes(), [](auto const& n) {
    return tg_point{n.location().lon(), n.location().lat()};
  }));
  if (geom == nullptr) {
    return false;
  }

  for (auto const idx : candidates) {
    if (idx < zones_.size() && tg_geom_intersects(zones_[idx], geom.get())) {
      return true;
    }
  }

  return false;
}

void low_emission_zone_relation_handler::relation(
    osmium::Relation const& relation) {
  if (!has_low_emission_zone_ &&
      low_emission_zones::is_low_emission_zone(relation.tags())) {
    has_low_emission_zone_ = true;
  }
}

}  // namespace osr
