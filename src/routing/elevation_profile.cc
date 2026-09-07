#include "osr/routing/elevation_profile.h"
#include <cista/strong.h>
#include <algorithm>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

height_profile::height_profile(ways const& w, path const& p, unsigned steps) {
  auto const adjust_lng = [](double const x) {
    if (x < -180.) {
      return x + 360.;
    } else if (x <= 180.) {
      return x;
    } else {
      return x - 360.;
    }
  };

  auto const add = [&](point const& a, point const& b,
                       elevation_difference_t z) {
    auto const lat = a.lat() + (b.lat() - a.lat()) * 0.5;
    auto const lng = adjust_lng(a.lng() + (b.lng() - a.lng()) * 0.5);
    points_.emplace_back(lat, lng);
    elevation_.push_back(z);

    min_ = std::min(static_cast<elevation_absolute_t>(to_idx(z)) + baseline_,
                    min_);
    max_ = std::max(static_cast<elevation_absolute_t>(to_idx(z)) + baseline_,
                    max_);
  };

  steps = std::min(steps, static_cast<unsigned>(p.segments_.size()));
  auto const step_size = p.dist_ / steps;

  points_.resize(steps);
  elevation_.resize(steps);
  baseline_ = p.segments_.front().elevation_.absolute_;
  points_.push_back(w.get_node_pos(p.segments_.front().from_));
  elevation_.emplace_back(0);

  auto dist_acc = distance_t{0};
  auto z_acc = elevation_difference_t{0};
  auto from = w.get_node_pos(p.segments_.front().from_);
  add(from, w.get_node_pos(p.segments_.front().to_), z_acc);
  for (auto seg = begin(p.segments_); seg != end(p.segments_); seg++) {
    dist_acc += seg->dist_;
    z_acc += static_cast<cista::base_t<elevation_difference_t>>(
                 to_idx(seg->elevation_.up_)) -
             static_cast<cista::base_t<elevation_difference_t>>(
                 to_idx(seg->elevation_.down_));

    if (dist_acc < step_size && seg != prev(end(p.segments_))) {
      seg++;
      continue;
    }

    auto const to = w.get_node_pos(seg->to_);
    add(from, to, z_acc);
    from = to;
    dist_acc = 0;
    z_acc = elevation_difference_t{0};
  }
}

elevation_absolute_t height_profile::median() const {
  if (elevation_.empty()) {
    return elevation_absolute_t::invalid();
  }

  if (elevation_.size() == 1) {
    return static_cast<elevation_absolute_t>(to_idx(elevation_[0])) + baseline_;
  }

  auto sorted = std::vector<elevation_difference_t>{elevation_};
  auto const n = sorted.size() / 2;
  std::nth_element(
      begin(sorted),
      next(
          begin(sorted),
          static_cast<std::vector<elevation_difference_t>::difference_type>(n)),
      end(sorted));

  if (n % 2 == 0) {
    return static_cast<elevation_absolute_t>(
               to_idx((sorted[n] + sorted[n - 1]) / 2)) +
           baseline_;
  }

  return static_cast<elevation_absolute_t>(to_idx(sorted[n])) + baseline_;
}

}  // namespace osr