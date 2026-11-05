#include "osr/routing/elevation_profile.h"

#include <algorithm>
#include <iterator>
#include <vector>

#include "cista/strong.h"

#include "osr/point.h"
#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace ev = preprocessing::elevation;

elevation_profile::elevation_profile(ways const& w,
                                     std::span<path::segment const> segments,
                                     double resolution)
    : resolution_(resolution) {

  auto const adjust_lng = [](double const x) {
    if (x < -180.) {
      return x + 360.;
    } else if (x <= 180.) {
      return x;
    } else {
      return x - 360.;
    }
  };

  auto s = utl::find_if(segments, [&](auto const& seg) {
    return seg.elevation_.first_ != nullptr && seg.elevation_.n_ != 0;
  });
  if (s == end(segments)) {
    return;
  }

  baseline_ = *s->elevation_.first_;
  min_ = baseline_;
  max_ = baseline_;

  auto const add = [&](point const& pos, ev::elevation_meters_t z) {
    points_.push_back(pos);
    elevation_.push_back(z);

    min_ = std::min(z, min_);
    max_ = std::max(z, max_);
  };

  // TODO -> start at seg_it
  for (; s != end(segments); s++) {
    if (s->elevation_.n_ <= 2) {
      add(w.get_node_pos(s->from_), *(s->elevation_.first_));
      add(w.get_node_pos(s->to_), (s->elevation_.n_ == 1)
                                      ? *(s->elevation_.first_)
                                      : *std::next(s->elevation_.first_));
      continue;
    }

    auto const step_dist = 1 / (s->elevation_.n_ - 1);
    auto const from_pos = w.get_node_pos(s->from_);
    auto const to_pos = w.get_node_pos(s->to_);
    auto const lng_dist = adjust_lng(to_pos.lng() - from_pos.lng()) * step_dist;
    auto const lat_dist = (to_pos.lat() - from_pos.lat()) * step_dist;
    for (auto i = 1U; i < s->elevation_.n_; ++i) {
      auto const new_lng = from_pos.lng() + lng_dist * i;
      auto const new_lat = from_pos.lat() + lat_dist * i;

      add(point::from_latlng({.lng_ = new_lng, .lat_ = new_lat}),
          *std::next(s->elevation_.first_, i));
    }
  }
}

ev::elevation_meters_t elevation_profile::median() const {
  if (elevation_.empty()) {
    return ev::elevation_meters_t::invalid();
  }

  if (elevation_.size() == 1) {
    return baseline_;
  }

  auto sorted = std::vector<ev::elevation_meters_t>{elevation_};
  auto const n = sorted.size() / 2;
  std::nth_element(
      begin(sorted),
      next(
          begin(sorted),
          static_cast<std::vector<ev::elevation_meters_t>::difference_type>(n)),
      end(sorted));

  if (n % 2 == 0) {
    return (sorted[n] + sorted[n - 1]) / 2;
  }

  return sorted[n];
}

}  // namespace osr