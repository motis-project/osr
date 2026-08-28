#include "osr/routing/elevation_profile.h"
#include "geo/latlng.h"

namespace osr {

height_profile::height_profile(
    path& p,
    preprocessing::elevation::provider const& provider,
    unsigned n_samples) {

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

  auto const sample_dist = std::pow(p.dist_ / n_samples, 2);

  auto const add = [&](geo::latlng const& coord) {
    points_.push_back(coord);
    elevation_.push_back(provider.get(coord));
  };

  points_.resize(n_samples + 1);
  elevation_.resize(n_samples + 1);

  auto from = *poly_it;
  add({from.lat(), from.lng()});
  --n_samples;
  auto to = next();

  auto acc = 0.0;
  while (n_samples > 0) {
    acc += squared_distance(from, to);

    if (acc >= sample_dist) {
      --n_samples;
      auto const normed_diff =
          (acc - sample_dist) / std::abs(squared_distance(from, to));
      auto const lat_diff = from.lat() - to.lat();
      auto const lng_diff = adjust_lng(from.lng() - to.lng());
      auto sample_point = geo::latlng{to.lat() + lat_diff * normed_diff,
                                      to.lng() + lng_diff * normed_diff};
      add(sample_point);

      acc = 0.0;
    }

    from = to;
    to = next();
  }

  to = p.segments_.back().polyline_.back();
}

}  // namespace osr