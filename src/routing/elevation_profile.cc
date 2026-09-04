#include "osr/routing/elevation_profile.h"
#include <algorithm>
#include <iterator>
#include <vector>

#include "utl/helpers/algorithm.h"

#include "geo/latlng.h"

namespace osr {

height_profile::height_profile(
    path& p,
    preprocessing::elevation::provider const& provider,
    double resolution) {
  resolution_ = std::max(
      {resolution, provider.max_resolution().x_, provider.max_resolution().y_});
  auto n_samples = p.dist_ / resolution_;

  auto seg_it = p.segments_.begin();
  auto poly_it = seg_it->polyline_.begin();
  auto const next = [&]() {
    if (++poly_it == std::end(seg_it->polyline_)) {
      ++seg_it;
      poly_it = (++seg_it)->polyline_.begin();
    }

    return *poly_it;
  };

  auto const adjust_lng = [&](double x) { return x; };
  auto const squared_distance = [&](geo::latlng a, geo::latlng b) {
    return std::pow(b.lat() - a.lat(), 2) +
           std::pow(adjust_lng(b.lng() - a.lng()), 2);
  };

  points_.resize(n_samples + 1);
  elevation_.resize(n_samples + 1);

  auto from = *poly_it;
  auto const baseline = provider.get(from);

  auto const add = [&](geo::latlng const& coord) {
    points_.push_back(coord);
    auto const z = provider.get(coord);
    elevation_.push_back(z);

    if (z == value_t::invalid()) {
      return;
    }

    if (z < baseline) {
      down_ += z.v_;
    } else {
      up_ += z.v_;
    }
    min_ = std::min(z, min_);
    max_ = std::max(z, max_);
  };

  add(from);
  --n_samples;
  auto to = next();

  auto acc = 0.0;
  while (n_samples > 0) {
    acc += squared_distance(from, to);

    while (acc >= resolution_) {
      acc -= resolution_;
      auto const normed_diff = acc / std::abs(squared_distance(from, to));
      auto const lat_diff = from.lat() - to.lat();
      auto const lng_diff = adjust_lng(from.lng() - to.lng());
      auto sample_point = geo::latlng{to.lat() + lat_diff * normed_diff,
                                      to.lng() + lng_diff * normed_diff};
      add(sample_point);
      --n_samples;
    }

    from = to;
    to = next();
  }

  to = p.segments_.back().polyline_.back();
}

height_profile::value_t height_profile::median() {
  if (elevation_.empty()) {
    return value_t::invalid();
  }

  if (elevation_.size() == 1) {
    return elevation_[0];
  }

  auto sorted = std::vector<value_t>{elevation_};
  auto const n = sorted.size() / 2;
  utl::nth_element(sorted, n);

  if (n % 2 == 0) {
    return (sorted[n] + sorted[n - 1]) / 2;
  }

  return sorted[n];
}

}  // namespace osr