#pragma once

#include <variant>

#include "osr/ways.h"

namespace osr {

struct foot {
  dist_t operator()(way_properties const& e,
                    direction,
                    std::uint16_t const dist) {
    if (e.is_foot_accessible()) {
      return static_cast<dist_t>(std::round(dist / 1.4F));
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_walk_accessible() ? 0U : kInfeasible;
  }
};

struct bike {
  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    if (e.is_bike_accessible() &&
        (dir == direction::kForward || !e.is_oneway_bike())) {
      return static_cast<dist_t>(std::round(dist / 3.5F));
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_bike_accessible() ? 0U : kInfeasible;
  }
};

struct car {
  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s());
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }
};

struct generic {
  explicit generic(search_profile const p) : fn_{get(p)} {}

  static std::variant<foot, bike, car> get(search_profile const p) {
    switch (p) {
      case search_profile::kFoot: return foot{};
      case search_profile::kBike: return bike{};
      case search_profile::kCar: return car{};
    }
    std::unreachable();
  }

  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    return std::visit([&](auto&& f) { return f(e, dir, dist); }, fn_);
  }

  dist_t operator()(node_properties const& n) {
    return std::visit([&](auto&& f) { return f(n); }, fn_);
  }

  std::variant<foot, bike, car> fn_;
};

}  // namespace osr