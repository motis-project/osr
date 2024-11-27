#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace osr::preprocessing::elevation {

struct location {
  inline int32_t x() const { return x_; }
  inline int32_t y() const { return y_; }

  inline void set_x(int32_t x) { x_ = x; }
  inline void set_y(int32_t y) { y_ = y; }

  static constexpr int32_t PRECISION = 10000000;

  inline double lon() const { return fix_to_double(x_); }
  inline double lat() const { return fix_to_double(y_); }

  inline void set_lon(double lon) { x_ = double_to_fix(lon); }
  inline void set_lat(double lat) { y_ = double_to_fix(lat); }

  inline static int32_t double_to_fix(double c) {
    auto const v = std::round(c * PRECISION);
    if (v > std::numeric_limits<int32_t>::max()) {
      return std::numeric_limits<int32_t>::max();
    } else if (v < std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<int32_t>::min();
    } else {
      return static_cast<int32_t>(v);
    }
  }

  inline static double fix_to_double(int32_t c) {
    return static_cast<double>(c) / PRECISION;
  }

  inline bool valid() const {
    return x_ >= (-180 * PRECISION) && x_ <= (180 * PRECISION) &&
           y_ >= (-90 * PRECISION) && y_ <= (90 * PRECISION);
  }

  int32_t x_{std::numeric_limits<int32_t>::max()};
  int32_t y_{std::numeric_limits<int32_t>::max()};
};

inline location make_location(int32_t x, int32_t y) { return location{x, y}; }

inline location make_location(double lon, double lat) {
  return location{location::double_to_fix(lon), location::double_to_fix(lat)};
}

inline bool operator==(location const& lhs, location const& rhs) {
  return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

inline bool operator!=(location const& lhs, location const& rhs) {
  return !(lhs == rhs);
}

inline bool operator<(location const& lhs, location const& rhs) {
  return (lhs.x() == rhs.x() && lhs.y() < rhs.x()) || lhs.x() < rhs.x();
}

inline bool operator>(location const& lhs, location const& rhs) {
  return rhs < lhs;
}

inline bool operator<=(location const& lhs, location const& rhs) {
  return !(rhs < lhs);
}

inline bool operator>=(location const& lhs, location const& rhs) {
  return !(lhs < rhs);
}

inline location operator+(location const& lhs, location const& rhs) {
  return {lhs.x() + rhs.x(), lhs.y() + rhs.y()};
}

inline location operator-(location const& lhs, location const& rhs) {
  return {lhs.x() - rhs.x(), lhs.y() - rhs.y()};
}

inline std::ostream& operator<<(std::ostream& os, location const& loc) {
  auto const precision = os.precision(12);
  os << "location{lon=" << loc.lon() << ", lat=" << loc.lat() << "}";
  os.precision(precision);
  return os;
}

}  // namespace osr::preprocessing::elevation
