#include "osr/extract/osm_areas.h"

#include <cassert>
#include <charconv>
#include <cstdint>
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "date/tz.h"

#include "osmium/osm/area.hpp"
#include "osmium/osm/way.hpp"

#include "utl/parser/arg_parser.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "rtree.h"
#include "tg.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace {

struct geom_deleter {
  void operator()(tg_geom* geom) const { tg_geom_free(geom); }
};

using geom_ptr = std::unique_ptr<tg_geom, geom_deleter>;

geom_ptr make_way_geom(osmium::Way const& way) {
  assert(way.nodes().size() >= 2U);

  auto const points = utl::to_vec(way.nodes(), [](auto const& n) {
    return tg_point{n.location().lon(), n.location().lat()};
  });

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

geom_ptr make_area_geom(osmium::Area const& area) {
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
  return geom_ptr{geom};
}

constexpr auto const kTimezoneAdminLevel = std::uint8_t{13U};

std::optional<std::uint8_t> get_admin_level(osmium::Area const& area) {
  auto const* value = area.tags()["admin_level"];
  if (value == nullptr) {
    return kTimezoneAdminLevel;
  }

  auto const level = utl::parse<std::uint8_t>(value);
  if (level < 2U || level > 11U) {
    std::clog << "osr: ignored timezone area " << area.id()
              << " with invalid admin_level=" << value << '\n';
    return std::nullopt;
  }
  return level;
}

}  // namespace

struct osm_areas::impl {
  struct area {
    tg_geom* geom_{nullptr};
    conditional_timezone_idx_t timezone_{conditional_timezone_idx_t::invalid()};
    std::uint8_t admin_level_{0U};
  };

  explicit impl(ways& w)
      : w_{w},
        low_emission_zone_rtree_{rtree_new()},
        timezone_rtree_{rtree_new()} {
    utl::verify(low_emission_zone_rtree_ != nullptr,
                "low emission zone rtree creation failed");
    utl::verify(timezone_rtree_ != nullptr, "timezone rtree creation failed");
  }

  ~impl() {
    for (auto const& area : areas_) {
      tg_geom_free(area.geom_);
    }
    rtree_free(low_emission_zone_rtree_);
    rtree_free(timezone_rtree_);
  }

  std::optional<conditional_timezone_idx_t> register_timezone(
      osmium::Area const& area, std::string_view const name) {
    if (auto const it = timezone_idx_by_name_.find(name);
        it != end(timezone_idx_by_name_)) {
      return it->second;
    }

    try {
      date::locate_zone(name);
    } catch (std::runtime_error const&) {
      std::clog << "osr: ignored timezone area " << area.id()
                << " with unknown timezone=" << name << '\n';
      return std::nullopt;
    }

    utl::verify(
        w_.r_->timezones_.size() < std::numeric_limits<std::uint16_t>::max(),
        "too many timezones");
    auto const idx = conditional_timezone_idx_t{
        static_cast<std::uint16_t>(w_.r_->timezones_.size())};
    w_.r_->timezones_.emplace_back(name);
    timezone_idx_by_name_.emplace(std::string{name}, idx);
    return idx;
  }

  void add_area(osmium::Area const& osm_area) {
    auto const& tags = osm_area.tags();
    auto const is_low_emission_zone =
        tags.has_tag("boundary", "low_emission_zone");
    auto const* timezone_name = tags["timezone"];
    auto const has_timezone = timezone_name != nullptr &&
                              std::string_view{timezone_name}.empty() == false;
    if (!is_low_emission_zone && !has_timezone) {
      return;
    }

    auto admin_level = std::optional<std::uint8_t>{};
    auto timezone = std::optional<conditional_timezone_idx_t>{};
    if (has_timezone) {
      admin_level = get_admin_level(osm_area);
      if (admin_level.has_value()) {
        timezone = register_timezone(osm_area, timezone_name);
      }
    }

    if (!is_low_emission_zone && !timezone.has_value()) {
      return;
    }

    auto geom = make_area_geom(osm_area);
    if (geom == nullptr || tg_geom_error(geom.get()) != nullptr ||
        tg_geom_is_empty(geom.get())) {
      return;
    }

    auto const idx = areas_.size();
    areas_.emplace_back(area{
        .geom_ = geom.release(),
        .timezone_ = timezone.value_or(conditional_timezone_idx_t::invalid()),
        .admin_level_ = admin_level.value_or(0U),
    });

    for (auto const& outer : osm_area.outer_rings()) {
      auto const env = outer.envelope();
      if (!env.valid()) {
        continue;
      }
      auto const min =
          std::array{env.bottom_left().lon(), env.bottom_left().lat()};
      auto const max = std::array{env.top_right().lon(), env.top_right().lat()};
      if (is_low_emission_zone) {
        rtree_insert(low_emission_zone_rtree_, min.data(), max.data(),
                     reinterpret_cast<void*>(idx));
      }
      if (timezone.has_value()) {
        rtree_insert(timezone_rtree_, min.data(), max.data(),
                     reinterpret_cast<void*>(idx));
      }
    }
  }

  ways& w_;
  rtree* low_emission_zone_rtree_{nullptr};
  rtree* timezone_rtree_{nullptr};
  std::vector<area> areas_;
  hash_map<std::string, conditional_timezone_idx_t> timezone_idx_by_name_;
};

osm_areas::osm_areas(ways& w) : impl_{std::make_unique<impl>(w)} {}

osm_areas::~osm_areas() = default;

void osm_areas::add_area(osmium::Area const& area) { impl_->add_area(area); }

bool osm_areas::is_in_low_emission_zone(osmium::Way const& way) const {
  if (rtree_count(impl_->low_emission_zone_rtree_) == 0U) {
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

  struct search_state {
    impl const& impl_;
    osmium::Way const& way_;
    geom_ptr way_geom_;
    bool intersects_{false};
  } state{*impl_, way, nullptr, false};
  rtree_search(
      impl_->low_emission_zone_rtree_, min.data(), max.data(),
      [](double const*, double const*, void const* item, void* udata) {
        auto& s = *static_cast<search_state*>(udata);
        if (s.way_geom_ == nullptr) {
          s.way_geom_ = make_way_geom(s.way_);
          if (s.way_geom_ == nullptr) {
            return false;
          }
        }
        auto const idx = reinterpret_cast<std::size_t>(item);
        assert(idx < s.impl_.areas_.size());
        if (tg_geom_intersects(s.impl_.areas_[idx].geom_, s.way_geom_.get())) {
          s.intersects_ = true;
          return false;
        }
        return true;
      },
      &state);
  return state.intersects_;
}

conditional_timezone_idx_t osm_areas::get_timezone(
    geo::latlng const& pos) const {
  if (rtree_count(impl_->timezone_rtree_) == 0U) {
    return conditional_timezone_idx_t::invalid();
  }

  struct search_state {
    impl const& impl_;
    double lon_;
    double lat_;
    std::size_t area_idx_{std::numeric_limits<std::size_t>::max()};
  } state{*impl_, pos.lng(), pos.lat()};
  auto const point = pos.lnglat();
  rtree_search(
      impl_->timezone_rtree_, point.data(), point.data(),
      [](double const*, double const*, void const* item, void* udata) {
        auto& s = *static_cast<search_state*>(udata);
        auto const idx = reinterpret_cast<std::size_t>(item);
        assert(idx < s.impl_.areas_.size());
        auto const& candidate = s.impl_.areas_[idx];
        if (!tg_geom_intersects_xy(candidate.geom_, s.lon_, s.lat_)) {
          return true;
        }
        if (s.area_idx_ == std::numeric_limits<std::size_t>::max()) {
          s.area_idx_ = idx;
          return true;
        }
        auto const& current = s.impl_.areas_[s.area_idx_];
        if (candidate.admin_level_ > current.admin_level_ ||
            (candidate.admin_level_ == current.admin_level_ &&
             idx < s.area_idx_)) {
          s.area_idx_ = idx;
        }
        return true;
      },
      &state);
  return state.area_idx_ == std::numeric_limits<std::size_t>::max()
             ? conditional_timezone_idx_t::invalid()
             : impl_->areas_[state.area_idx_].timezone_;
}

}  // namespace osr
