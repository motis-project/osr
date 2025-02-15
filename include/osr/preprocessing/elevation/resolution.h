#pragma once

#include <cmath>
#include <limits>

namespace osr::preprocessing::elevation {

struct resolution {
  resolution& update(resolution const& o) {
    if (std::isnan(x_) || o.x_ < x_) {
      x_ = o.x_;
    }
    if (std::isnan(y_) || o.y_ < y_) {
      y_ = o.y_;
    }
    return *this;
  }
  double x_{std::numeric_limits<double>::quiet_NaN()};
  double y_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace osr::preprocessing::elevation