#include "osr/routing/instructions/directions.h"

#include "boost/beast/zlib/zlib.hpp"

#include "geo/latlng.h"
#include "geo/webmercator.h"

namespace osr {

relative_direction get_relative_direction(const double angle) {
  bool is_left = angle > 0;
  double abs_angle = std::abs(angle);
  if (abs_angle <= 20) {
    return relative_direction::kStraight;
  }
  if (abs_angle <= 45) {
    return is_left ? relative_direction::kSlightLeft : relative_direction::kSlightRight;
  }
  if (abs_angle <= 135) {
    return is_left ? relative_direction::kLeft : relative_direction::kRight;
  }
  if (abs_angle < 180) {
    return is_left ? relative_direction::kSharpLeft : relative_direction::kSharpRight;
  }

  return relative_direction::kTurnAround;
}

double get_angle(geo::latlng const& p1,
                 geo::latlng const& shared,
                 geo::latlng const& p2) {
  auto normalize = [](double angle) {
    double x = std::fmod(angle + 180.0, 360.0);
    if (x < 0) {
      x += 360.0;
    }
    return x - 180.0;
  };

  const double b1 = geo::bearing(p1, shared);
  const double b2 = geo::bearing(shared, p2);
  return normalize(b2 - b1); // [-180,180]
}

} // namespace osr