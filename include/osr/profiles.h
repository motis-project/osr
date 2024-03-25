#pragma once

#include "cista/hash.h"

#include "osmium/osm/object.hpp"

namespace osr {

enum class override : std::uint8_t { kNone, kWhitelist, kBlacklist };

template <typename T>
bool is_accessible(osmium::OSMObject const& o) {
  auto const override = T::override_access(o);
  return override == override::kWhitelist ||
         (T::default_access(o) && override == override::kBlacklist);
}

struct foot_profile {
  static override access_override(osmium::OSMObject const& o) {
    auto const foot = o.tags()["foot"];
    if (foot == nullptr) {
      return override::kNone;
    }

    switch (cista::hash(std::string_view{foot})) {
      case cista::hash("no"):
      case cista::hash("private"):
      case cista::hash("use_sidepath"): [[fallthrough]];
      case cista::hash("destination"): return override::kBlacklist;

      case cista::hash("yes"):
      case cista::hash("permissive"): [[fallthrough]];
      case cista::hash("designated"): return override::kWhitelist;

      default: return override::kNone;
    }
  }

  static bool default_access(osmium::OSMObject const& o, bool const is_way) {
    if (is_way) {
      auto const highway = std::string_view{o.tags()["highway"]};
      switch (cista::hash(highway)) {
        case cista::hash("primary"):
        case cista::hash("primary_link"):
        case cista::hash("secondary"):
        case cista::hash("secondary_link"):
        case cista::hash("tertiary"):
        case cista::hash("tertiary_link"):
        case cista::hash("unclassified"):
        case cista::hash("residential"):
        case cista::hash("road"):
        case cista::hash("living_street"):
        case cista::hash("service"):
        case cista::hash("track"):
        case cista::hash("path"):
        case cista::hash("steps"):
        case cista::hash("pedestrian"):
        case cista::hash("footway"):
        case cista::hash("pier"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }
};

struct bike_profile {
  static override access_override(osmium::OSMObject const& o) {}

  static bool default_access(osmium::OSMObject const& o, bool const is_way) {
    if (is_way) {
      auto const highway = std::string_view{o.tags()["highway"]};
      switch (cista::hash(highway)) {
        case cista::hash("cycleway"):
        case cista::hash("primary"):
        case cista::hash("primary_link"):
        case cista::hash("secondary"):
        case cista::hash("secondary_link"):
        case cista::hash("tertiary"):
        case cista::hash("tertiary_link"):
        case cista::hash("residential"):
        case cista::hash("unclassified"):
        case cista::hash("living_street"):
        case cista::hash("road"):
        case cista::hash("service"):
        case cista::hash("track"):
        case cista::hash("path"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }
};

struct car_profile {
  static override access_override(osmium::OSMObject const& o) {}

  static bool default_access(osmium::OSMObject const& o, bool const is_way) {
    if (is_way) {
      auto const highway = std::string_view{o.tags()["highway"]};
      switch (cista::hash(highway)) {
        case cista::hash("motorway"):
        case cista::hash("motorway_link"):
        case cista::hash("trunk"):
        case cista::hash("trunk_link"):
        case cista::hash("primary"):
        case cista::hash("primary_link"):
        case cista::hash("secondary"):
        case cista::hash("secondary_link"):
        case cista::hash("tertiary"):
        case cista::hash("tertiary_link"):
        case cista::hash("residential"):
        case cista::hash("living_street"):
        case cista::hash("unclassified"):
        case cista::hash("service"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }
};

}  // namespace osr