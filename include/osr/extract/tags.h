#pragma once

#include "cista/hash.h"

#include "osmium/osm/object.hpp"

#include "osr/types.h"

namespace osr {

enum class osm_obj_type : std::uint8_t { kWay, kNode };

enum class override : std::uint8_t { kNone, kWhitelist, kBlacklist };

struct tags {
  explicit tags(osmium::OSMObject const& o) {
    auto const add_levels = [](auto&& t, level_bits_t& level_bits) {
      auto s = utl::cstr{t.value()};
      while (s) {
        auto l = 0.0F;
        utl::parse_arg(s, l);
        auto const lvl = level_t{std::clamp(l, kMinLevel, kMaxLevel)};
        level_bits |= (static_cast<level_bits_t>(1) << to_idx(lvl));
        if (s) {
          ++s;
        }
      }
    };

    auto circular = false;
    auto oneway_defined = false;
    for (auto const& t : o.tags()) {
      switch (cista::hash(std::string_view{t.key()})) {
        using namespace std::string_view_literals;
        case cista::hash("ramp"): is_ramp_ |= t.value() != "no"sv; break;
        case cista::hash("type"): is_route_ |= t.value() == "route"sv; break;
        case cista::hash("parking"): is_parking_ = true; break;
        case cista::hash("amenity"):
          is_parking_ |=
              (t.value() == "parking"sv || t.value() == "parking_entrance"sv);
          break;
        case cista::hash("building"):
          is_parking_ |= t.value() == "parking"sv;
          landuse_ = true;
          break;
        case cista::hash("landuse"): landuse_ = true; break;
        case cista::hash("railway"):
          railway_ = t.value();
          landuse_ |= railway_ == "station_area"sv;
          break;
        case cista::hash("oneway"):
          oneway_defined = true;
          oneway_ |= t.value() == "yes"sv;
          break;
        case cista::hash("junction"):
          oneway_ |= t.value() == "roundabout"sv;
          circular |= t.value() == "circular"sv;
          break;
        case cista::hash("oneway:bicycle"):
          not_oneway_bike_ = t.value() == "no"sv;
          break;
        case cista::hash("oneway:psv"):
          not_oneway_psv_ = t.value() == "no"sv;
          break;
        case cista::hash("motor_vehicle:forward"):
        case cista::hash("motor_vehicle"):
          motor_vehicle_ = t.value();
          is_destination_ |= motor_vehicle_ == "destination"sv;
          break;
        case cista::hash("foot"): foot_ = t.value(); break;
        case cista::hash("bicycle"): bicycle_ = t.value(); break;
        case cista::hash("highway"):
          highway_ = t.value();
          if (highway_ == "elevator") {
            is_elevator_ = true;
          }
          if (highway_ == "bus_stop") {
            is_platform_ = true;
          }
          break;
        case cista::hash("indoor:level"): [[fallthrough]];
        case cista::hash("level"):
          has_level_ = true;
          add_levels(t, level_bits_);
          break;
        case cista::hash("name"): name_ = t.value(); break;
        case cista::hash("ref"): ref_ = t.value(); break;
        case cista::hash("entrance"): is_entrance_ = true; break;
        case cista::hash("sidewalk"):
        case cista::hash("sidewalk:both"):
        case cista::hash("sidewalk:left"): [[fallthrough]];
        case cista::hash("sidewalk:right"):
          if (t.value() == "separate"sv) {
            sidewalk_separate_ = true;
          }
          break;
        case cista::hash("cycleway"): cycleway_ = t.value(); break;
        case cista::hash("motorcar"):
          motorcar_ = t.value();
          is_destination_ |= motorcar_ == "destination";
          break;
        case cista::hash("barrier"): barrier_ = t.value(); break;
        case cista::hash("platform_edge"): is_platform_ = true; break;
        case cista::hash("public_transport"):
          switch (cista::hash(std::string_view{t.value()})) {
            case cista::hash("platform"):
            case cista::hash("stop_position"): is_platform_ = true;
          }
          break;
        case cista::hash("construction"): is_construction_ = true; break;
        case cista::hash("vehicle"):
          switch (cista::hash(std::string_view{t.value()})) {
            case cista::hash("private"):
            case cista::hash("delivery"):
            case cista::hash("no"): vehicle_ = override::kBlacklist; break;

            case cista::hash("destination"):
              is_destination_ = true;
              [[fallthrough]];
            case cista::hash("permissive"): [[fallthrough]];
            case cista::hash("yes"): vehicle_ = override::kWhitelist; break;
          }
          break;
        case cista::hash("psv"):
          if (bus_ == override::kNone) {
            bus_ = t.value() == "no"sv ? override::kBlacklist
                                       : override::kWhitelist;
          }
          break;
        case cista::hash("bus"):  // more specific than psv
          bus_ =
              t.value() == "no"sv ? override::kBlacklist : override::kWhitelist;
          break;
        case cista::hash("access"):
          switch (cista::hash(std::string_view{t.value()})) {
            case cista::hash("no"):
            case cista::hash("agricultural"):
            case cista::hash("forestry"):
            case cista::hash("emergency"): [[fallthrough]];
            case cista::hash("delivery"): access_ = override::kBlacklist; break;

            case cista::hash("private"):
              access_ = override::kBlacklist;
              private_access_ = true;
              break;

            case cista::hash("designated"):
            case cista::hash("dismount"):
            case cista::hash("customers"):
            case cista::hash("permissive"): [[fallthrough]];
            case cista::hash("yes"): access_ = override::kWhitelist; break;

            case cista::hash("psv"):
            case cista::hash("bus"):
              access_ = override::kBlacklist;
              bus_ = override::kWhitelist;
              break;
          }
          break;
        case cista::hash("access:conditional"): {
          constexpr auto const kPrefix = "no @ ("sv;
          constexpr auto const kPostfix = ")"sv;
          auto const value = std::string_view{t.value()};
          if (value.starts_with(kPrefix) && value.ends_with(kPostfix)) {
            access_conditional_no_ =
                value.substr(kPrefix.size(), value.length() - kPrefix.length() -
                                                 kPostfix.length());
          }
        } break;
        case cista::hash("maxspeed"): max_speed_ = t.value(); break;
        case cista::hash("toll"): toll_ = t.value() == "yes"sv; break;
        case cista::hash("incline"): {
          auto const value = std::string_view{t.value()};
          is_incline_down_ = t.value() == "down"sv || value.starts_with("-"sv);
        } break;
        case cista::hash("route"):
          is_ferry_route_ = t.value() == "ferry"sv;
          break;
      }
    }
    if (circular && !oneway_defined) {
      oneway_ = true;
    }
  }

  bool is_platform() const { return is_platform_ && !is_construction_; }

  // https://wiki.openstreetmap.org/wiki/Relation:route
  bool is_route_{false};

  // https://wiki.openstreetmap.org/wiki/Tag:route%3Dferry
  bool is_ferry_route_{false};

  // https://wiki.openstreetmap.org/wiki/Key:oneway
  // https://wiki.openstreetmap.org/wiki/Tag:junction=roundabout
  bool oneway_{false};

  // https://wiki.openstreetmap.org/wiki/Key:oneway:bicycle
  bool not_oneway_bike_{false};

  // https://wiki.openstreetmap.org/wiki/Key:oneway:psv
  bool not_oneway_psv_{false};

  // https://wiki.openstreetmap.org/wiki/Key:barrier
  std::string_view barrier_;

  // https://wiki.openstreetmap.org/wiki/Key:motorcar
  std::string_view motorcar_;

  // https://wiki.openstreetmap.org/wiki/Key:motor_vehicle
  std::string_view motor_vehicle_;

  // https://wiki.openstreetmap.org/wiki/Key:foot
  std::string_view foot_;

  // https://wiki.openstreetmap.org/wiki/Key:bicycle
  std::string_view bicycle_;

  // https://wiki.openstreetmap.org/wiki/DE:Key:highway
  std::string_view highway_;

  // https://wiki.openstreetmap.org/wiki/Key:railway
  std::string_view railway_;

  // https://wiki.openstreetmap.org/wiki/Key:sidewalk
  bool sidewalk_separate_{false};

  // https://wiki.openstreetmap.org/wiki/Key:cycleway
  std::string_view cycleway_;

  // https://wiki.openstreetmap.org/wiki/Key:maxspeed
  std::string_view max_speed_;

  // https://wiki.openstreetmap.org/wiki/Key:name
  std::string_view name_;

  // https://wiki.openstreetmap.org/wiki/Key:ref
  std::string_view ref_;

  // https://wiki.openstreetmap.org/wiki/Key:vehicle
  bool is_destination_{false};
  override vehicle_{override::kNone};

  // https://wiki.openstreetmap.org/wiki/Key:access
  override access_{override::kNone};
  bool private_access_{false};

  // https://wiki.openstreetmap.org/wiki/Key:psv
  // https://wiki.openstreetmap.org/wiki/Key:bus
  override bus_{override::kNone};

  // https://wiki.openstreetmap.org/wiki/Key:landuse
  bool landuse_{false};

  // https://wiki.openstreetmap.org/wiki/Key:construction
  bool is_construction_{false};

  // https://wiki.openstreetmap.org/wiki/Key:public%20transport
  bool is_platform_{false};

  // https://wiki.openstreetmap.org/wiki/Key:ramp
  bool is_ramp_{false};

  // https://wiki.openstreetmap.org/wiki/Tag:highway=elevator
  bool is_elevator_{false};

  // https://wiki.openstreetmap.org/wiki/Key:entrance
  bool is_entrance_{false};

  // https://wiki.openstreetmap.org/wiki/Tag:amenity%3Dparking
  bool is_parking_{false};

  // https://wiki.openstreetmap.org/wiki/Key:level
  bool has_level_{false};
  level_bits_t level_bits_{0U};

  // https://wiki.openstreetmap.org/wiki/Key:incline
  bool is_incline_down_{false};

  // https://wiki.openstreetmap.org/wiki/Key:toll
  bool toll_{false};

  // https://wiki.openstreetmap.org/wiki/Conditional_restrictions
  std::string_view access_conditional_no_;
};

template <typename T>
bool is_accessible(tags const& o, osm_obj_type const type) {
  auto const override = T::access_override(o, type);
  return override == override::kWhitelist ||
         (T::default_access(o, type) && override != override::kBlacklist);
}

template <typename T>
bool is_accessible_with_penalty(tags const& o, osm_obj_type const type) {
  return T::access_with_penalty(o, type);
}

struct foot_profile {
  static override access_override(tags const& t, osm_obj_type) {
    if (t.is_route_ || t.sidewalk_separate_ || t.is_ferry_route_) {
      return override::kBlacklist;
    }

    switch (cista::hash(t.barrier_)) {
      case cista::hash("yes"):
      case cista::hash("wall"):
      case cista::hash("fence"): return override::kBlacklist;
    }

    switch (cista::hash(t.foot_)) {
      case cista::hash("no"):
      case cista::hash("private"): [[fallthrough]];
      case cista::hash("use_sidepath"): return override::kBlacklist;

      case cista::hash("yes"):
      case cista::hash("permissive"): [[fallthrough]];
      case cista::hash("designated"): return override::kWhitelist;
    }

    if (t.is_platform_ || t.is_parking_) {
      return override::kWhitelist;
    }

    if (t.access_ == override::kBlacklist) {
      return override::kBlacklist;
    }

    return override::kNone;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      if (t.is_elevator_ || t.is_parking_) {
        return true;
      }
      switch (cista::hash(t.highway_)) {
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
        case cista::hash("platform"):
        case cista::hash("corridor"):
        case cista::hash("footway"): [[fallthrough]];
        case cista::hash("pier"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }

  static bool access_with_penalty(tags const&, osm_obj_type const) {
    return false;
  }
};

struct bike_profile {
  static override access_override(tags const& t, osm_obj_type) {
    if (t.is_route_ || t.is_ferry_route_) {
      return override::kBlacklist;
    }

    switch (cista::hash(t.barrier_)) {
      case cista::hash("yes"):
      case cista::hash("wall"):
      case cista::hash("fence"): return override::kBlacklist;
    }

    switch (cista::hash(t.bicycle_)) {
      case cista::hash("no"):
      case cista::hash("private"):
      case cista::hash("optional_sidepath"): [[fallthrough]];
      case cista::hash("use_sidepath"): return override::kBlacklist;

      case cista::hash("yes"):
      case cista::hash("permissive"): [[fallthrough]];
      case cista::hash("designated"): return override::kWhitelist;
    }

    if (t.access_ == override::kBlacklist) {
      return override::kBlacklist;
    }

    return t.vehicle_;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      switch (cista::hash(t.highway_)) {
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

  static bool access_with_penalty(tags const&, osm_obj_type const) {
    return false;
  }
};

struct car_profile {
  static override access_override(tags const& t, osm_obj_type const type) {
    if (t.access_ == override::kBlacklist || t.is_route_ || t.is_ferry_route_ ||
        (type == osm_obj_type::kWay && t.highway_.empty())) {
      return override::kBlacklist;
    }

    if (!t.barrier_.empty()) {
      switch (cista::hash(t.barrier_)) {
        case cista::hash("cattle_grid"):
        case cista::hash("border_control"):
        case cista::hash("toll_booth"):
        case cista::hash("sally_port"):
        case cista::hash("gate"):
        case cista::hash("lift_gate"):
        case cista::hash("no"):
        case cista::hash("entrance"):
        case cista::hash("coupure"):
        case cista::hash("height_restrictor"): [[fallthrough]];
        case cista::hash("arch"): break;
        default: return override::kBlacklist;
      }
    }

    auto const get_override = [](std::string_view tag) {
      switch (cista::hash(tag)) {
        case cista::hash("private"):
        case cista::hash("optional_sidepath"):
        case cista::hash("agricultural"):
        case cista::hash("forestry"):
        case cista::hash("agricultural;forestry"):
        case cista::hash("permit"):
        case cista::hash("customers"):
        case cista::hash("delivery"): [[fallthrough]];
        case cista::hash("no"): return override::kBlacklist;

        case cista::hash("designated"):
        case cista::hash("permissive"): [[fallthrough]];
        case cista::hash("yes"): return override::kWhitelist;
        default: return override::kNone;
      }
    };

    if (auto mv = get_override(t.motor_vehicle_); mv != override::kNone) {
      return mv;
    } else if (auto mc = get_override(t.motorcar_); mc != override::kNone) {
      return mc;
    }

    if (t.is_parking_) {
      return override::kWhitelist;
    }

    return t.vehicle_;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      switch (cista::hash(t.highway_)) {
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
  static bool access_with_penalty(tags const&, osm_obj_type const) {
    return false;
  }
};

struct bus_profile {
  static override access_override(tags const& t, osm_obj_type const type) {
    if (t.bus_ != override::kNone) {
      return t.bus_;
    } else if (t.access_ == override::kBlacklist || t.is_route_ ||
               t.is_ferry_route_) {
      return override::kBlacklist;
    } else if (t.barrier_ == "bus_trap") {
      return override::kWhitelist;
    }

    return car_profile::access_override(t, type);
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    return car_profile::default_access(t, type);
  }

  static bool access_with_penalty(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kNode) {
      return t.private_access_ && t.barrier_ == "gate";
    } else {
      return t.private_access_ && default_access(t, type);
    }
  }
};

struct railway_profile {
  static override access_override(tags const&, osm_obj_type) {
    return override::kNone;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      switch (cista::hash(t.railway_)) {
        case cista::hash("rail"):
        case cista::hash("light_rail"):
        case cista::hash("monorail"):
        case cista::hash("narrow_gauge"):
        case cista::hash("subway"):
        case cista::hash("tram"):
        case cista::hash("funicular"): return true;
        default: return false;
      }
    } else {
      return true;
    }
  }

  static bool access_with_penalty(tags const&, osm_obj_type const) {
    return false;
  }
};

struct ferry_profile {
  static override access_override(tags const&, osm_obj_type) {
    return override::kNone;
  }

  static bool default_access(tags const& t, osm_obj_type const type) {
    if (type == osm_obj_type::kWay) {
      return t.is_ferry_route_;
    } else {
      return true;
    }
  }

  static bool access_with_penalty(tags const&, osm_obj_type const) {
    return false;
  }
};

}  // namespace osr
