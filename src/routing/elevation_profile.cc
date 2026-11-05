#include "osr/routing/elevation_profile.h"
#include <cstddef>

#include <algorithm>
#include <iterator>
#include <vector>

#include "cista/strong.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace ev = preprocessing::elevation;

template <typename T>
struct moving_average {
  moving_average(std::size_t window_size, std::vector<double> weights)
      : window_size_(window_size), weights_(weights) {
    assert(window_size % 2 == 1);
  }

  moving_average(unsigned window_size) : window_size_(window_size) {
    assert(window_size % 2 == 1);
    weights_.resize(window_size, 1.0 / window_size);
  }

  void init_from(std::vector<T> const& values) {
    auto const left = window_size_ / 2 + 1;
    auto const right = window_size_ - left;
    window_.resize(left, values[0]);
    for (auto i = std::size_t{1}; i <= right; ++i) {
      window_.push_back(values[i]);
    }
  }

  T next(T value) {
    window_.push_back(value);
    if (window_.size() > window_size_) {
      window_.erase(window_.begin());
    }

    double sum = 0.0;
    for (auto i = std::size_t{0}; i < window_size_; ++i) {
      sum = sum + to_idx(window_[i]) * weights_[i];
    }

    return static_cast<T>(sum);
  }

  std::size_t window_size_;
  std::vector<T> window_{};
  std::vector<double> weights_{};
};

elevation_profile::elevation_profile(ways const& w,
                                     std::span<path::segment const> segments) {

  auto const adjust_lng = [](double const x) {
    if (x < -180.) {
      return x + 360.;
    } else if (x <= 180.) {
      return x;
    } else {
      return x - 360.;
    }
  };

  if (utl::find_if(segments, [&](auto const& seg) {
        return seg.elevation_.first_ != nullptr && seg.elevation_.n_ != 0;
      }) == end(segments)) {
    return;
  }

  baseline_ = *segments[1].elevation_.first_;
  min_ = baseline_;
  max_ = baseline_;

  auto const add = [&](point const& pos, ev::elevation_meters_t z) {
    points_.push_back(pos);
    elevation_.push_back(z);

    min_ = std::min(z, min_);
    max_ = std::max(z, max_);
  };

  for (auto s : segments) {
    if (s.elevation_.n_ <= 2) {
      add(w.get_node_pos(s.from_), *s.elevation_.first_);
      add(w.get_node_pos(s.to_), (s.elevation_.n_ == 1)
                                     ? *s.elevation_.first_
                                     : *std::next(s.elevation_.first_));
      continue;
    }

    auto const step_dist = 1 / (s.elevation_.n_ - 1);
    auto const from_pos = w.get_node_pos(s.from_);
    auto const to_pos = w.get_node_pos(s.to_);
    auto const lng_dist = adjust_lng(to_pos.lng() - from_pos.lng()) * step_dist;
    auto const lat_dist = (to_pos.lat() - from_pos.lat()) * step_dist;
    for (auto i = 1U; i < s.elevation_.n_; ++i) {
      auto const new_lng = from_pos.lng() + lng_dist * i;
      auto const new_lat = from_pos.lat() + lat_dist * i;

      add(point::from_latlng({.lng_ = new_lng, .lat_ = new_lat}),
          *std::next(s.elevation_.first_, i));
    }
  }

  if (elevation_.size() < 5) {
    return;
  }

  auto avg = moving_average<ev::elevation_meters_t>{5};
  avg.init_from(elevation_);
  for (auto i = avg.window_size_; i < elevation_.size() + avg.window_size_;
       ++i) {
    auto const curr = i - avg.window_size_;
    auto const next =
        (i < elevation_.size()) ? elevation_[i] : elevation_.back();
    elevation_[curr] = avg.next(next);
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