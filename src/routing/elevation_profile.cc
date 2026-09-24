#include "osr/routing/elevation_profile.h"

#include <algorithm>
#include <vector>

#include "utl/helpers/algorithm.h"

#include "cista/strong.h"

#include "osr/elevation_storage.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

elevation_profile::elevation_profile(ways const& w,
                                     std::span<path::segment const> segments,
                                     double resolution)
    : resolution_(resolution) {
  if (segments.empty() || utl::all_of(segments, [](auto const& s) {
        return s.elevation_.absolute_ == elevation_absolute_t::invalid();
      })) {
    return;
  }

  auto const adjust_lng = [](double const x) {
    if (x < -180.) {
      return x + 360.;
    } else if (x <= 180.) {
      return x;
    } else {
      return x - 360.;
    }
  };

  auto it = utl::find_if(segments, [&](auto const& s) {
    return s.elevation_.absolute_ != elevation_absolute_t::invalid();
  });
  baseline_ = it->elevation_.absolute_;

  auto const add = [&](node_idx_t a, node_idx_t b, elevation_absolute_t z) {
    auto const a_pos = w.get_node_pos(a);
    auto const b_pos = w.get_node_pos(b);

    auto const lat = a_pos.lat() + (b_pos.lat() - a_pos.lat()) * 0.5;
    auto const lng =
        adjust_lng(a_pos.lng() + (b_pos.lng() - a_pos.lng()) * 0.5);
    points_.push_back(point::from_latlng({lat, lng}));
    elevation_.push_back(z);

    min_ = std::min(z, min_);
    max_ = std::max(z, max_);
  };

  auto dist_acc = distance_t{0};
  add(it->from_, it->to_, baseline_);
  while (++it != prev(end(segments), 2)) {
    dist_acc += it->dist_;

    if (dist_acc < resolution ||
        it->elevation_.absolute_ == elevation_absolute_t::invalid()) {
      continue;
    }

    add(it->from_, it->to_, it->elevation_.absolute_);
    dist_acc = 0;
  }
  if (it->elevation_.absolute_ != elevation_absolute_t::invalid()) {
    add(it->from_, it->to_, it->elevation_.absolute_);
  }
}

elevation_absolute_t elevation_profile::median() const {
  if (elevation_.empty()) {
    return elevation_absolute_t::invalid();
  }

  if (elevation_.size() == 1) {
    return baseline_;
  }

  auto sorted = std::vector<elevation_absolute_t>{elevation_};
  auto const n = sorted.size() / 2;
  std::nth_element(
      begin(sorted),
      next(begin(sorted),
           static_cast<std::vector<elevation_absolute_t>::difference_type>(n)),
      end(sorted));

  if (n % 2 == 0) {
    return (sorted[n] + sorted[n - 1]) / 2;
  }

  return sorted[n];
}

}  // namespace osr