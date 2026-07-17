#ifdef _WIN_32
// Otherwise
// winnt.h(169): fatal error C1189: #error:  "No Target Architecture"
#include <windows.h>
#endif

#include <cstdint>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include "osr/extract/extract.h"

#ifndef _WIN32
#include <sys/mman.h>
#endif

#include "boost/thread/tss.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "osmium/memory/buffer.hpp"
#include "osmium/visitor.hpp"

#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/parallel_for.h"
#include "utl/parser/arg_parser.h"
#include "utl/progress_tracker.h"
#include "utl/verify.h"

#include "osm/decoder.h"
#include "osm/hnidx/hybrid_node_index.h"
#include "osm/inflate.h"
#include "osm/mp_manager.h"
#include "osm/osmium_builder.h"
#include "osm/raw_reader.h"
#include "osm/types.h"

#include "geo/box.h"

#include "osr/elevation_storage.h"
#include "osr/extract/conditional_parser.h"
#include "osr/extract/osm_areas.h"
#include "osr/extract/tag_parser.h"
#include "osr/extract/tags.h"
#include "osr/lookup.h"
#include "osr/platforms.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace std::string_view_literals;

namespace osr {

struct osm_restriction {
  bool valid() const {
    return from_ != osm_way_idx_t::invalid() &&
           to_ != osm_way_idx_t::invalid() && via_ != osm_node_idx_t::invalid();
  }

  osm_way_idx_t from_{osm_way_idx_t::invalid()};
  osm_way_idx_t to_{osm_way_idx_t::invalid()};
  osm_node_idx_t via_{osm_node_idx_t::invalid()};
};

struct conditional_turn_restriction {
  resolved_restriction::type type_{};
  std::string_view condition_{};
};

bool is_number(std::string_view s) {
  return !s.empty() &&
         utl::all_of(s, [](char const c) { return std::isdigit(c); });
}

std::optional<resolved_restriction::type> parse_turn_restriction_type(
    std::string_view const value) {
  if (value.starts_with("no"sv)) {
    return resolved_restriction::type::kNo;
  }
  if (value.starts_with("only"sv)) {
    return resolved_restriction::type::kOnly;
  }
  return std::nullopt;
}

std::optional<conditional_turn_restriction> parse_conditional_turn_restriction(
    std::string_view const value) {
  auto const at = value.find('@');
  if (at == std::string_view::npos) {
    return std::nullopt;
  }

  auto const type = parse_turn_restriction_type(trim(value.substr(0U, at)));
  auto const condition = trim(value.substr(at + 1U));
  if (!type.has_value() || condition.empty()) {
    return std::nullopt;
  }
  return conditional_turn_restriction{*type, condition};
}

template <typename T, typename Value>
void set_hgv_info_value(hgv_way_info& info,
                        hgv_info_field const field,
                        T hgv_way_info::* member,
                        std::optional<Value> const value) {
  if (!value.has_value()) {
    return;
  }
  info.fields_ |= to_mask(field);
  info.*member = *value;
}

std::string_view pick_hgv_variant(std::string_view base, std::string_view hgv) {
  return hgv.empty() ? base : hgv;
}

std::optional<access_value> get_access_value(std::string_view value) {
  switch (cista::hash(value)) {
    case cista::hash("yes"): return access_value::kYes;
    case cista::hash("designated"): return access_value::kDesignated;
    case cista::hash("permissive"): return access_value::kPermissive;
    case cista::hash("private"): return access_value::kPrivate;
    case cista::hash("delivery"): return access_value::kDelivery;
    case cista::hash("destination"): return access_value::kDestination;
    case cista::hash("no"): return access_value::kNo;
    case cista::hash("discouraged"): return access_value::kDiscouraged;
    default: return std::nullopt;
  }
}

std::optional<hgv_way_info> get_hgv_way_info(tags const& t) {
  auto info = hgv_way_info{};

  auto const hgv_access = get_access_value(t.hgv_);
  auto const hgv_fwd = get_access_value(t.hgv_forward_);
  auto const hgv_bwd = get_access_value(t.hgv_backward_);

  if (hgv_access.has_value()) {
    info.fields_ |= to_mask(hgv_info_field::kAccessFwd);
    info.hgv_access_fwd_ = static_cast<std::uint8_t>(*hgv_access);
    info.fields_ |= to_mask(hgv_info_field::kAccessBwd);
    info.hgv_access_bwd_ = static_cast<std::uint8_t>(*hgv_access);
  }
  if (hgv_fwd.has_value()) {
    info.fields_ |= to_mask(hgv_info_field::kAccessFwd);
    info.hgv_access_fwd_ = static_cast<std::uint8_t>(*hgv_fwd);
  }
  if (hgv_bwd.has_value()) {
    info.fields_ |= to_mask(hgv_info_field::kAccessBwd);
    info.hgv_access_bwd_ = static_cast<std::uint8_t>(*hgv_bwd);
  }

  set_hgv_info_value(
      info, hgv_info_field::kMaxSpeed, &hgv_way_info::maxspeed_km_h_,
      to_integer<std::uint8_t>(parse_speed_km_h(t.max_speed_hgv_)));
  set_hgv_info_value(
      info, hgv_info_field::kMaxLength, &hgv_way_info::maxlength_cm_,
      to_integer<std::uint16_t>(
          parse_length_m(pick_hgv_variant(t.max_length_, t.max_length_hgv_)),
          100.0));
  set_hgv_info_value(info, hgv_info_field::kMaxWeightRating,
                     &hgv_way_info::maxweightrating_100kg_,
                     to_integer<std::uint16_t>(
                         parse_weight_t(pick_hgv_variant(
                             t.max_weightrating_, t.max_weightrating_hgv_)),
                         10.0));
  set_hgv_info_value(
      info, hgv_info_field::kMaxHeight, &hgv_way_info::maxheight_cm_,
      to_integer<std::uint16_t>(parse_length_m(t.max_height_), 100.0));
  set_hgv_info_value(
      info, hgv_info_field::kMaxWidth, &hgv_way_info::maxwidth_cm_,
      to_integer<std::uint16_t>(parse_length_m(t.max_width_), 100.0));
  set_hgv_info_value(
      info, hgv_info_field::kMaxWeight, &hgv_way_info::maxweight_100kg_,
      to_integer<std::uint16_t>(parse_weight_t(t.max_weight_), 10.0));
  set_hgv_info_value(
      info, hgv_info_field::kMaxAxleLoad, &hgv_way_info::maxaxleload_100kg_,
      to_integer<std::uint16_t>(parse_weight_t(t.max_axle_load_), 10.0));
  set_hgv_info_value(info, hgv_info_field::kMaxAxles, &hgv_way_info::maxaxles_,
                     to_integer<std::uint8_t>(parse_unitless(t.max_axles_)));

  if (auto const hazmat = get_access_value(t.hazmat_); hazmat.has_value()) {
    info.fields_ |= to_mask(hgv_info_field::kHazmat);
    info.hazmat_access_ = static_cast<std::uint8_t>(*hazmat);
  }

  if (auto const hazmat_water = get_access_value(t.hazmat_water_);
      hazmat_water.has_value()) {
    info.fields_ |= to_mask(hgv_info_field::kHazmatWater);
    info.hazmat_water_access_ = static_cast<std::uint8_t>(*hazmat_water);
  }

  if (auto const trailer = get_access_value(t.hgv_trailer_);
      trailer.has_value()) {
    info.fields_ |= to_mask(hgv_info_field::kTrailer);
    info.trailer_access_ = static_cast<std::uint8_t>(*trailer);
  }

  return info.fields_ == 0U ? std::nullopt : std::optional{info};
}

bool is_big_street(tags const& t) {
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
    case cista::hash("unclassified"): return true;
    default: return false;
  }
}

speed_limit get_speed_limit(tags const& t) {
  if (auto const speed = parse_speed_km_h(t.max_speed_); speed.has_value()) {
    return get_speed_limit(static_cast<unsigned>(speed.value()));
  } else {
    switch (cista::hash(t.highway_)) {
      case cista::hash("motorway"): return get_speed_limit(100);
      case cista::hash("motorway_link"): return get_speed_limit(45);
      case cista::hash("trunk"): return get_speed_limit(85);
      case cista::hash("trunk_link"): return get_speed_limit(40);
      case cista::hash("primary"):
        return t.name_.empty() ? get_speed_limit(80) : get_speed_limit(40);
      case cista::hash("primary_link"): return get_speed_limit(30);
      case cista::hash("secondary"):
        return t.name_.empty() ? get_speed_limit(80) : get_speed_limit(60);
      case cista::hash("secondary_link"): return get_speed_limit(25);
      case cista::hash("tertiary"):
        return t.name_.empty() ? get_speed_limit(60) : get_speed_limit(40);
      case cista::hash("tertiary_link"): return get_speed_limit(20);
      case cista::hash("unclassified"): return get_speed_limit(40);
      case cista::hash("residential"): return get_speed_limit(20);
      case cista::hash("living_street"): return get_speed_limit(10);
      case cista::hash("service"): return get_speed_limit(20);
      case cista::hash("track"): return get_speed_limit(12);
      case cista::hash("path"): return get_speed_limit(13);
      case cista::hash("bus_guideway"):
      case cista::hash("busway"): return get_speed_limit(50);
      default:
        switch (cista::hash(t.railway_)) {
          case cista::hash("rail"):
          case cista::hash("narrow_gauge"): return get_speed_limit(80);
          case cista::hash("light_rail"): return get_speed_limit(50);
          case cista::hash("subway"): return get_speed_limit(50);
          case cista::hash("tram"): return get_speed_limit(30);
          default: return speed_limit::kmh_10;
        }
    }
  }
}

struct rel_way {
  way_properties p_;
  platform_idx_t pl_{platform_idx_t::invalid()};
};

using rel_ways_t = hash_map<osm_way_idx_t, rel_way>;

std::tuple<level_t, level_t, bool> get_levels(tags const& t) {
  return get_levels(t.has_level_, t.level_bits_);
}

way_properties get_way_properties(
    tags const& t, osm_obj_type const obj_type = osm_obj_type::kWay) {
  auto const [from, to, _] = get_levels(t);
  auto p = way_properties{};
  std::memset(&p, 0, sizeof(way_properties));
  p.is_foot_accessible_ = is_accessible<foot_profile>(t, obj_type);
  p.is_bike_accessible_ = is_accessible<bike_profile>(t, obj_type);
  p.is_car_accessible_ = is_accessible<car_profile>(t, obj_type);
  p.is_bus_accessible_ = is_accessible<bus_profile>(t, obj_type);
  p.is_railway_accessible_ = is_accessible<railway_profile>(t, obj_type);
  p.is_destination_ = t.is_destination_;
  p.is_oneway_car_ = t.oneway_;
  p.is_oneway_bike_ = t.oneway_ && !t.not_oneway_bike_;
  p.is_oneway_bus_psv_ = t.oneway_ && !t.not_oneway_bus_psv_;
  p.is_oneway_reverse_ = t.oneway_reverse_;
  p.is_elevator_ = t.is_elevator_;
  p.is_steps_ = (t.highway_ == "steps"sv);
  p.is_parking_ = t.is_parking_;
  p.speed_limit_ = get_speed_limit(t);
  p.from_level_ = to_idx(from);
  p.to_level_ = to_idx(to);
  p.is_incline_down_ = t.is_incline_down_;
  p.is_platform_ = t.is_platform();
  p.is_ramp_ = t.is_ramp_ || (t.highway_ == "footway"sv && t.has_incline_);
  p.is_sidewalk_separate_ = t.sidewalk_separate_;
  p.motor_vehicle_no_ =
      (t.motor_vehicle_ == "no"sv) || (t.vehicle_ == override::kBlacklist);
  p.has_toll_ = t.toll_;
  p.is_big_street_ = is_big_street(t);
  p.in_route_ = t.is_route_ && t.is_public_transport_route();
  p.is_bus_accessible_with_penalty_ =
      is_accessible_with_penalty<bus_profile>(t, obj_type);
  p.is_ferry_accessible_ = is_accessible<ferry_profile>(t, obj_type);
  p.is_railway_accessible_with_penalty_ =
      is_accessible_with_penalty<railway_profile>(t, obj_type);
  p.is_detour_ = t.is_detour_route();
  return p;
}

std::pair<node_properties, level_bits_t> get_node_properties(tags const& t) {
  auto const [from, to, is_multi] = get_levels(t);
  auto p = node_properties{};
  std::memset(&p, 0, sizeof(node_properties));
  p.from_level_ = to_idx(from);
  p.is_foot_accessible_ = is_accessible<foot_profile>(t, osm_obj_type::kNode);
  p.is_bike_accessible_ = is_accessible<bike_profile>(t, osm_obj_type::kNode);
  p.is_car_accessible_ = is_accessible<car_profile>(t, osm_obj_type::kNode);
  p.is_bus_accessible_ = is_accessible<bus_profile>(t, osm_obj_type::kNode);
  p.is_elevator_ = t.is_elevator_;
  p.is_entrance_ = t.is_entrance_;
  p.is_multi_level_ = is_multi;
  p.is_parking_ = t.is_parking_;
  p.to_level_ = to_idx(to);
  p.is_bus_accessible_with_penalty_ =
      is_accessible_with_penalty<bus_profile>(t, osm_obj_type::kNode);
  return {p, t.level_bits_};
}

struct way_handler : public osmium::handler::Handler {
  using is_transparent = void;

  struct strings_hash {
    using is_transparent = void;

    cista::hash_t operator()(string_idx_t const& x) const {
      return (*this)(((*strings_)[x]).view());
    }

    cista::hash_t operator()(std::string_view x) const {
      return cista::hash(x);
    }

    mm_vecvec<string_idx_t, char, std::uint64_t> const* strings_{nullptr};
  };

  struct strings_equals {
    using is_transparent = void;

    bool operator()(string_idx_t const a, string_idx_t const b) const {
      return a == b;
    }

    bool operator()(std::string_view a, string_idx_t const b) const {
      return a == (*strings_)[b].view();
    }

    mm_vecvec<string_idx_t, char, std::uint64_t> const* strings_{nullptr};
  };

  way_handler(ways& w,
              platforms* platforms,
              rel_ways_t const& rel_ways,
              hash_map<osm_node_idx_t, level_bits_t>& elevator_nodes)
      : w_{w},
        platforms_{platforms},
        rel_ways_{rel_ways},
        elevator_nodes_{elevator_nodes} {
    strings_set_.hash_function().strings_ = &w_.strings_;
    strings_set_.key_eq().strings_ = &w_.strings_;
  }

  void way(osmium::Way const& w) {
    if (w.nodes().size() < 2U) {
      return;
    }

    auto const osm_way_idx = to_osm_way_idx(w.id());
    auto const it = rel_ways_.find(osm_way_idx);
    auto t = tags{w};

    if (t.is_elevator_) {
      if (w.nodes().front() != w.nodes().back()) {
        return;  // way elevators have to be loops
      }
      auto const first_node = to_osm_node_idx(w.nodes().front().ref());
      auto const l = std::scoped_lock{elevator_nodes_mutex_};
      elevator_nodes_.emplace(first_node, t.level_bits_);
    }

    if (!t.is_elevator_ &&  // elevators tagged as building would be landuse
        !t.is_parking_ &&
        ((it == end(rel_ways_) && t.highway_.empty() && t.railway_.empty() &&
          !t.is_ferry_route_ && !t.is_platform()) ||
         (t.highway_.empty() && !t.is_platform() && it != end(rel_ways_) &&
          t.landuse_))) {
      return;
    }

    auto const in_route = it != end(rel_ways_) && it->second.p_.in_route_;
    auto const hgv_info = get_hgv_way_info(t);

    auto p = (t.is_platform() || t.is_parking_ || !t.highway_.empty() ||
              (!t.railway_.empty() && (in_route || it == end(rel_ways_))) ||
              t.is_ferry_route_)
                 ? get_way_properties(t)
                 : it->second.p_;
    p.has_hgv_info_ = hgv_info.has_value();

    if (!t.has_level_ && it != end(rel_ways_)) {
      p.from_level_ = it->second.p_.from_level_;
      p.to_level_ = it->second.p_.to_level_;
    }

    if (in_route) {
      p.in_route_ = true;
    }
    if (it != end(rel_ways_) && it->second.p_.is_detour_) {
      p.is_detour_ = true;
    }

    auto const get_point = [](osmium::NodeRef const& n) {
      return point::from_location(n.location());
    };

    auto const get_node_id = [&](osmium::NodeRef const& n) {
      auto const idx = to_osm_node_idx(n.ref());
      w_.node_way_counter_.increment(to_idx(idx));
      return idx;
    };

    auto const register_string = [&](std::string_view s) {
      auto str_idx = string_idx_t::invalid();
      if (auto const string_it = strings_set_.find(s);
          string_it != end(strings_set_)) {
        str_idx = *string_it;
      } else {
        str_idx = string_idx_t{w_.strings_.size()};
        w_.strings_.emplace_back(s);
        strings_set_.insert(str_idx);
      }
      return str_idx;
    };

    auto l = std::scoped_lock{mutex_};
    auto const way_idx = way_idx_t{w_.way_osm_idx_.size()};

    // Timezones are not known yet (areas are assembled in the same pass) -
    // they are patched into the condition sets by osm_areas::annotate_ways().
    auto conditional_builder = conditional_storage_builder{.routing_ = *w_.r_};
    for (auto const& [key, value] : t.conditional_tags_) {
      if (!parse_conditional_restriction_tag(key, value, conditional_builder)) {
        std::clog << "osr: ignored unsupported conditional restriction on way "
                  << to_idx(osm_way_idx) << ": " << key << '=' << value << '\n';
      }
    }
    p.has_conditionals_ = !conditional_builder.way_.empty();
    if (!p.is_accessible()) {
      return;
    }

    if (platforms_ != nullptr &&
        (t.is_platform() || p.is_platform_ ||
         (it != end(rel_ways_) && it->second.p_.is_platform_))) {
      platforms_->way(way_idx, w);
    }

    if (it != end(rel_ways_) && it->second.pl_ != platform_idx_t::invalid()) {
      platforms_->platform_ref_[it->second.pl_].push_back(to_value(way_idx));
    }

    w_.way_osm_idx_.push_back(to_osm_way_idx(w.id()));
    w_.r_->way_properties_.emplace_back(p);
    if (hgv_info.has_value()) {
      w_.r_->way_hgv_info_.emplace_back(way_idx, *hgv_info);
    }
    if (p.has_conditionals()) {
      w_.r_->way_conditionals_.emplace_back(way_idx, conditional_builder.way_);
    }

    w_.way_polylines_.emplace_back(w.nodes() |
                                   std::views::transform(get_point));
    w_.way_osm_nodes_.emplace_back(w.nodes() |
                                   std::views::transform(get_node_id));

    auto const name = t.name_.empty() ? t.ref_ : t.name_;
    if (!name.empty()) {
      w_.way_names_.emplace_back(register_string(name));
    } else {
      w_.way_names_.emplace_back(string_idx_t::invalid());
    }

    w_.way_has_conditional_access_no_.resize(to_idx(way_idx) + 1U);
    if (!t.access_conditional_no_.empty()) {
      w_.way_has_conditional_access_no_.set(way_idx, true);
      w_.way_conditional_access_no_.emplace_back(
          way_idx, register_string(t.access_conditional_no_));
    }
  }

  using strings_set_t = hash_set<string_idx_t, strings_hash, strings_equals>;
  strings_set_t strings_set_;

  std::mutex mutex_;
  ways& w_;
  platforms* platforms_;
  rel_ways_t const& rel_ways_;

  std::mutex elevator_nodes_mutex_;
  hash_map<osm_node_idx_t, level_bits_t>& elevator_nodes_;
};

struct node_handler : public osmium::handler::Handler {
  struct cache {
    void clear() {
      from_.clear();
      to_.clear();
    }
    std::vector<way_idx_t> from_, to_;
  };

  node_handler(ways& w,
               platforms* platforms,
               std::vector<resolved_restriction>& r,
               hash_map<osm_node_idx_t, level_bits_t> const& elevator_nodes)
      : platforms_{platforms}, r_{r}, w_{w}, elevator_nodes_{elevator_nodes} {
    w_.r_->node_properties_.resize(w_.n_nodes());
    w_.r_->node_positions_.resize(w_.n_nodes());
  }

  void node(osmium::Node const& n) {
    auto const osm_node_idx = to_osm_node_idx(n.id());
    if (auto const node_idx = w_.find_node_idx(osm_node_idx);
        node_idx.has_value()) {
      auto const t = tags{n};
      auto const [p, level_bits] = get_node_properties(t);
      w_.r_->node_properties_[*node_idx] = p;
      w_.r_->node_positions_[*node_idx] = point::from_location(n.location());

      if (platforms_ != nullptr && t.is_platform()) {
        auto const l = std::lock_guard{platforms_mutex_};
        platforms_->node(*node_idx, n);
      }

      if (p.is_elevator() && p.is_multi_level()) {
        auto const l = std::scoped_lock{multi_level_elevators_mutex_};
        w_.r_->multi_level_elevators_.emplace_back(*node_idx, level_bits);
      } else if (auto const it = elevator_nodes_.find(osm_node_idx);
                 it != end(elevator_nodes_)) {
        auto const [from, to, is_multi] = get_levels(true, it->second);
        auto& x = w_.r_->node_properties_[*node_idx];
        x.is_elevator_ = true;
        x.from_level_ = to_idx(from);
        x.to_level_ = to_idx(to);
        x.is_multi_level_ = is_multi;
        if (is_multi) {
          auto const l = std::scoped_lock{multi_level_elevators_mutex_};
          w_.r_->multi_level_elevators_.emplace_back(*node_idx, it->second);
        }
      }
    }
  }

  void relation(osmium::Relation const& r) {
    static auto c = boost::thread_specific_ptr<cache>{};
    if (c.get() == nullptr) {
      c.reset(new cache{});
    }
    c->clear();

    auto const type = r.tags()["type"];
    if (type == nullptr || type != "restriction"sv) {
      return;
    }

    auto const restriction_ptr = r.tags()["restriction"];
    auto const hgv_restriction_ptr = r.tags()["restriction:hgv"];
    auto const conditional_restriction_ptr =
        r.tags()["restriction:conditional"];
    auto const hgv_conditional_restriction_ptr =
        r.tags()["restriction:hgv:conditional"];
    if (restriction_ptr == nullptr && hgv_restriction_ptr == nullptr &&
        conditional_restriction_ptr == nullptr &&
        hgv_conditional_restriction_ptr == nullptr) {
      return;
    }

    auto applies_to_hgv = true;
    auto applies_to_bus = true;
    if (auto const except_ptr = r.tags()["except"]; except_ptr != nullptr) {
      auto val = std::string_view{except_ptr};
      while (!val.empty()) {
        auto const sep = val.find(';');
        auto const token =
            trim(sep == std::string_view::npos ? val : val.substr(0, sep));
        if (token == "bus"sv || token == "psv"sv) {
          applies_to_bus = false;
        } else if (token == "hgv"sv) {
          applies_to_hgv = false;
        }
        val.remove_prefix(sep == std::string_view::npos ? val.size()
                                                        : sep + 1U);
      }
    }

    auto via = node_idx_t::invalid();
    for (auto const& m : r.members()) {
      switch (cista::hash(std::string_view{m.role()})) {
        case cista::hash("to"): {
          auto const to = w_.find_way(to_osm_way_idx(m.ref()));
          if (to.has_value()) {
            c->to_.emplace_back(*to);
          }
          break;
        }

        case cista::hash("from"): {
          auto const from = w_.find_way(to_osm_way_idx(m.ref()));
          if (from.has_value()) {
            c->from_.emplace_back(*from);
          }
          break;
        }

        case cista::hash("via"):
          if (m.type() == osmium::item_type::node) {
            auto const v = w_.find_node_idx(to_osm_node_idx(m.ref()));
            if (v.has_value()) {
              via = *v;
            }
          }
          break;
      }
    }

    if (via == node_idx_t::invalid() || c->from_.empty() || c->to_.empty()) {
      return;
    }

    auto const l = std::scoped_lock{r_mutex_};
    auto conditional_builder = conditional_storage_builder{.routing_ = *w_.r_};

    auto const append_restriction =
        [&](resolved_restriction::type const restriction_type,
            bool const applies_to_default, bool const applies_to_bus_value,
            bool const applies_to_hgv_value,
            conditional_condition_set_idx_t const condition_set) {
          if (!applies_to_default && !applies_to_bus_value &&
              !applies_to_hgv_value) {
            return;
          }
          for (auto const& from : c->from_) {
            for (auto const& to : c->to_) {
              r_.emplace_back(resolved_restriction{
                  restriction_type, from, to, via, applies_to_default,
                  applies_to_bus_value, applies_to_hgv_value, condition_set});
            }
          }
        };

    if (restriction_ptr != nullptr) {
      if (auto const restriction_type =
              parse_turn_restriction_type(std::string_view{restriction_ptr});
          restriction_type.has_value()) {
        append_restriction(*restriction_type, true, applies_to_bus,
                           applies_to_hgv,
                           conditional_condition_set_idx_t::invalid());
      }
    }

    if (hgv_restriction_ptr != nullptr && applies_to_hgv) {
      if (auto const restriction_type = parse_turn_restriction_type(
              std::string_view{hgv_restriction_ptr});
          restriction_type.has_value()) {
        append_restriction(*restriction_type, false, false, true,
                           conditional_condition_set_idx_t::invalid());
      }
    }

    auto const append_conditional = [&](char const* key, char const* value,
                                        bool const applies_to_hgv_value) {
      if (value == nullptr || !applies_to_hgv_value) {
        return;
      }
      auto const parsed =
          parse_conditional_turn_restriction(std::string_view{value});
      if (!parsed.has_value()) {
        log_conditional_turn_restriction_parse_error(r.id(), key, value);
        return;
      }
      auto const condition_set = parse_conditional_condition_set(
          parsed->condition_, conditional_builder);
      if (!condition_set.has_value()) {
        log_conditional_turn_restriction_parse_error(r.id(), key, value);
        return;
      }
      append_restriction(parsed->type_, false, false, true, *condition_set);
    };

    append_conditional("restriction:conditional", conditional_restriction_ptr,
                       applies_to_hgv);
    append_conditional("restriction:hgv:conditional",
                       hgv_conditional_restriction_ptr, applies_to_hgv);
  }

  void log_conditional_turn_restriction_parse_error(
      std::int64_t const rel,
      std::string_view const key,
      std::string_view const value) {
    std::clog << "osr: ignored unsupported conditional turn restriction on "
                 "relation "
              << rel << ": " << key << '=' << value << '\n';
  }

  std::mutex platforms_mutex_;
  platforms* platforms_;

  std::mutex multi_level_elevators_mutex_;

  std::vector<resolved_restriction>& r_;
  std::mutex r_mutex_;

  ways& w_;

  hash_map<osm_node_idx_t, level_bits_t> const& elevator_nodes_;
};

struct rel_ways_handler : public osmium::handler::Handler {
  explicit rel_ways_handler(platforms* pl, rel_ways_t& rel_ways)
      : pl_{pl}, rel_ways_{rel_ways} {}

  void relation(osmium::Relation const& r) {
    auto const p = get_way_properties(tags{r}, osm_obj_type::kRelation);
    if (!p.is_accessible() && !p.in_route() && !p.is_detour()) {
      return;
    }

    auto const platform = p.is_platform_ && pl_ != nullptr
                              ? pl_->relation(r)
                              : platform_idx_t::invalid();

    for (auto const& m : r.members()) {
      if (m.type() == osmium::item_type::way) {
        auto rw =
            rel_ways_.emplace(to_osm_way_idx(m.ref()), rel_way{p, platform});
        if (!rw.second) {
          rw.first->second.p_.in_route_ |= p.in_route_;
          rw.first->second.p_.is_detour_ |= p.is_detour_;
        }
      }
    }
  }

  platforms* pl_;
  rel_ways_t& rel_ways_;
};

void extract(bool const with_platforms,
             fs::path const& in,
             fs::path const& out,
             fs::path const& elevation_dir) {
  auto ec = std::error_code{};
  fs::remove_all(out, ec);
  if (!fs::is_directory(out)) {
    fs::create_directories(out);
  }

  utl::verify(fs::exists(in), "load_osm failed [file={}]", in);
  auto reader = osm::raw_reader{
      .file_ = cista::mmap{in.generic_string().c_str(),
                           cista::mmap::protection::READ}};

  // All passes read the PBF strictly forward: hint the kernel to ramp up
  // readahead and evict pages behind the cursor.
#ifdef MADV_SEQUENTIAL
  ::madvise(const_cast<std::uint8_t*>(reader.file_.data()),
            reader.file_.size(), MADV_SEQUENTIAL);
#endif

  auto blocks = std::vector<osm::buf>{};
  while (auto const b = reader.read()) {
    blocks.emplace_back(*b);
  }

  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  auto node_idx = osm::hybrid_node_idx{
      cista::mmap{out.generic_string().c_str(),
                  cista::mmap::protection::TMPFILE},
      cista::mmap{out.generic_string().c_str(),
                  cista::mmap::protection::TMPFILE}};
  auto node_idx_merger = osm::hybrid_block_merger{node_idx};

  auto rel_ways = rel_ways_t{};
  auto w = ways{out, cista::mmap::protection::WRITE};
  auto pl = std::unique_ptr<platforms>{};
  if (with_platforms) {
    pl = std::make_unique<platforms>(out, cista::mmap::protection::WRITE);
  }
  auto mp = osm::polygon_manager{/*assemble_way_polygons=*/false};
  auto areas = osm_areas{w};

  // Timezone / low emission zone relations, stored (owned) in pass 1 for
  // assembly after the ways pass collected the member way geometries.
  struct area_relation {
    std::int64_t id_;
    std::vector<std::tuple<std::int64_t, std::string, osm::member_type>>
        members_;
    std::vector<std::pair<std::string, std::string>> tags_;
  };
  auto area_relations = std::vector<area_relation>{};

  auto const is_area_relation = [](osmium::TagList const& tags) {
    auto const* boundary = tags["boundary"];
    return tags.has_key("timezone") ||
           (boundary != nullptr &&
            std::string_view{boundary} == "low_emission_zone"sv);
  };

  auto const to_member_type = [](osmium::item_type const t) {
    switch (t) {
      case osmium::item_type::node: return osm::member_type::kNode;
      case osmium::item_type::way: return osm::member_type::kWay;
      default: return osm::member_type::kRelation;
    }
  };

  struct block_local {
    osm::inflate decompressor_;
    std::string decompressed_;
    std::vector<std::string_view> strings_;
    osmium::memory::Buffer scratch_{1U << 12U,
                                    osmium::memory::Buffer::auto_grow::yes};
  };

  auto const decompress = [&blocks](block_local& local, std::size_t const i) {
    local.decompressed_.resize(blocks[i].raw_size_);
    local.decompressor_.decompress(blocks[i].compressed_,
                                   local.decompressed_);
  };

  w.node_way_counter_.reserve(14'000'000'000);
  {  // Pass 1: node index, blocking nodes, relations.
    pt->status("Load OSM / Pass 1").in_high(blocks.size()).out_bounds(0, 15);

    auto rel_ways_h = rel_ways_handler{pl.get(), rel_ways};

    struct pass1_result {
      osm::hybrid_block nodes_;
      std::vector<std::uint64_t> blocking_nodes_;
      osmium::memory::Buffer rels_{1U << 12U,
                                   osmium::memory::Buffer::auto_grow::yes};
    };

    utl::parallel_ordered_collect_threadlocal<block_local>(
        blocks.size(),
        [&](block_local& local, std::size_t const i) {
          auto res = pass1_result{};
          auto enc = osm::hybrid_block_encoder{};
          decompress(local, i);
          osm::decode_primitive(
              local.decompressed_, local.strings_,
              /*read_nodes=*/true, /*read_ways=*/false,
              /*read_relations=*/true,
              [&](std::int64_t const id, geo::latlng const& pos,
                  auto&& tag_range) {
                enc.node(osm::node{id, geo::fixed_latlng::from_latlng(pos)});
                if (std::ranges::begin(tag_range) !=
                    std::ranges::end(tag_range)) {
                  auto const& n =
                      osm::build_node(local.scratch_, id, pos, tag_range);
                  auto const t = tags{n};
                  auto const accessible =
                      is_accessible<car_profile>(t, osm_obj_type::kNode) &&
                      is_accessible<bike_profile>(t, osm_obj_type::kNode) &&
                      is_accessible<foot_profile>(t, osm_obj_type::kNode);
                  if (!accessible || t.is_elevator_ || t.is_platform()) {
                    res.blocking_nodes_.push_back(
                        to_idx(to_osm_node_idx(id)));
                  }
                  if (pl != nullptr && t.is_platform()) {
                    // Ensure nodes are created even if they are not part of
                    // a routable way.
                    res.blocking_nodes_.push_back(
                        to_idx(to_osm_node_idx(id)));
                  }
                }
              },
              [](auto&&...) {},
              [&](std::int64_t const id, auto&& members, auto&& tag_range) {
                auto const& rel = osm::build_relation(local.scratch_, id,
                                                      members, tag_range);
                auto const p =
                    get_way_properties(tags{rel}, osm_obj_type::kRelation);
                if (p.is_accessible() || p.in_route() || p.is_detour() ||
                    is_area_relation(rel.tags())) {
                  res.rels_.add_item(rel);
                  res.rels_.commit();
                }
              });
          res.nodes_ = std::move(enc).finish();
          return res;
        },
        [&](std::size_t, pass1_result&& res) {
          node_idx_merger.merge(std::move(res.nodes_));
          for (auto const id : res.blocking_nodes_) {
            w.node_way_counter_.increment(id);
          }
          osmium::apply(res.rels_, rel_ways_h);
          for (auto const& rel : res.rels_.select<osmium::Relation>()) {
            if (!is_area_relation(rel.tags())) {
              continue;
            }
            auto ar = area_relation{.id_ = rel.id(), .members_ = {}, .tags_ = {}};
            for (auto const& m : rel.members()) {
              ar.members_.emplace_back(m.ref(), std::string{m.role()},
                                       to_member_type(m.type()));
            }
            for (auto const& t : rel.tags()) {
              ar.tags_.emplace_back(t.key(), t.value());
            }
            mp.save_ways_of_relation(ar.id_, ar.members_, ar.tags_);
            area_relations.push_back(std::move(ar));
          }
        },
        [&](std::size_t const i) { pt->update_monotonic(i); });

    node_idx_merger.finish();
    mp.index_relation_members();
  }

  auto elevator_nodes = hash_map<osm_node_idx_t, level_bits_t>{};
  {  // Pass 2: extract streets, collect area member way geometries.
    pt->status("Load OSM / Ways").in_high(blocks.size()).out_bounds(15, 38);

    auto h = way_handler{w, pl.get(), rel_ways, elevator_nodes};

    struct pass2_local : public block_local {
      std::vector<osm::way> ways_;
      std::vector<std::vector<std::pair<std::string_view, std::string_view>>>
          tags_;
    };

    struct pass2_result {
      osmium::memory::Buffer ways_{1U << 16U,
                                   osmium::memory::Buffer::auto_grow::yes};
      std::vector<osm::way> member_ways_;
    };

    utl::parallel_ordered_collect_threadlocal<pass2_local>(
        blocks.size(),
        [&](pass2_local& local, std::size_t const i) {
          auto res = pass2_result{};
          decompress(local, i);
          local.ways_.clear();
          local.tags_.clear();
          osm::decode_primitive(
              local.decompressed_, local.strings_,
              /*read_nodes=*/false, /*read_ways=*/true,
              /*read_relations=*/false, [](auto&&...) {},
              [&](std::int64_t const id, auto&& refs, auto&& tag_range) {
                auto& ow = local.ways_.emplace_back();
                ow.id = id;
                for (auto const ref : refs) {
                  ow.node_refs.emplace_back(osm::node_ref{ref, {}});
                }
                auto& t = local.tags_.emplace_back();
                for (auto const& [k, v] : tag_range) {
                  t.emplace_back(k, v);
                }
              },
              [](auto&&...) {});

          osm::update_locations(node_idx, local.ways_);

          for (auto const [ow, t] : utl::zip(local.ways_, local.tags_)) {
            res.ways_.add_item(osm::build_way(local.scratch_, ow, t));
            res.ways_.commit();
            if (mp.is_relation_member(ow.id)) {
              res.member_ways_.push_back(std::move(ow));
            }
          }
          return res;
        },
        [&](std::size_t, pass2_result&& res) {
          osmium::apply(res.ways_, h);
          static auto const kNoTags =
              std::vector<std::pair<std::string_view, std::string_view>>{};
          for (auto& mw : res.member_ways_) {
            mp.save_ways(std::move(mw), kNoTags);
          }
        },
        [&](std::size_t const i) { pt->update_monotonic(i); });
  }

  {  // Assemble timezone and low emission zone polygons.
    pt->status("Load OSM / Areas")
        .in_high(std::max(std::size_t{1U}, area_relations.size()))
        .out_bounds(38, 39);
    auto scratch = osmium::memory::Buffer{
        1U << 12U, osmium::memory::Buffer::auto_grow::yes};
    for (auto const& ar : area_relations) {
      auto const pa = mp.assemble_area(ar.id_, ar.members_, ar.tags_);
      if (pa.valid) {
        areas.add_area(osm::build_area(scratch, pa, ar.tags_));
      } else {
        std::clog << "osr: area assembly failed for relation " << ar.id_
                  << " (" << ar.members_.size() << " members)\n";
      }
      pt->increment();
    }
  }

  {  // Set way properties that depend on the assembled areas.
    pt->status("Annotate Areas").in_high(1).out_bounds(39, 40);
    areas.annotate_ways();
    pt->update(pt->in_high_);
  }

  w.r_->write(out);
  w.sync();

  w.connect_ways();
  w.build_components();

  auto r = std::vector<resolved_restriction>{};
  {  // Pass 3: node properties + turn restrictions.
    pt->status("Load OSM / Node Properties")
        .in_high(blocks.size())
        .out_bounds(90, 95);

    auto h = node_handler{w, pl.get(), r, elevator_nodes};

    struct pass3_result {
      osmium::memory::Buffer objs_{1U << 14U,
                                   osmium::memory::Buffer::auto_grow::yes};
    };

    utl::parallel_ordered_collect_threadlocal<block_local>(
        blocks.size(),
        [&](block_local& local, std::size_t const i) {
          auto res = pass3_result{};
          decompress(local, i);
          osm::decode_primitive(
              local.decompressed_, local.strings_,
              /*read_nodes=*/true, /*read_ways=*/false,
              /*read_relations=*/true,
              [&](std::int64_t const id, geo::latlng const& pos,
                  auto&& tag_range) {
                if (w.find_node_idx(to_osm_node_idx(id)).has_value()) {
                  res.objs_.add_item(
                      osm::build_node(local.scratch_, id, pos, tag_range));
                  res.objs_.commit();
                }
              },
              [](auto&&...) {},
              [&](std::int64_t const id, auto&& members, auto&& tag_range) {
                auto is_restriction = false;
                for (auto const& [k, v] : tag_range) {
                  if (std::string_view{k} == "type"sv &&
                      std::string_view{v} == "restriction"sv) {
                    is_restriction = true;
                    break;
                  }
                }
                if (is_restriction) {
                  res.objs_.add_item(osm::build_relation(local.scratch_, id,
                                                         members, tag_range));
                  res.objs_.commit();
                }
              });
          return res;
        },
        [&](std::size_t, pass3_result&& res) { osmium::apply(res.objs_, h); },
        [&](std::size_t const i) { pt->update_monotonic(i); });
  }

  w.add_restriction(r);

  utl::sort(w.r_->multi_level_elevators_);

  if (pl) {
    utl::sort(pl->node_pos_,
              [](auto&& a, auto&& b) { return a.first < b.first; });
  }

  if (!elevation_dir.empty()) {
    auto const provider =
        osr::preprocessing::elevation::provider{elevation_dir};
    if (provider.driver_count() > 0) {
      auto elevations = elevation_storage{out, cista::mmap::protection::WRITE};
      elevations.set_elevations(w, provider);
    }
  }

  pt->status("Big Street Neighbors").in_high(w.n_ways()).out_bounds(95, 99);
  w.compute_big_street_neighbors();
  w.r_->write(out);

  pt->status("Build R-Tree").in_high(1).out_bounds(99, 100);
  lookup{w, out, cista::mmap::protection::WRITE}.build_rtree();
}

}  // namespace osr
