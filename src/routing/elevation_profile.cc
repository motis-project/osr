#include "osr/routing/elevation_profile.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>

#include <algorithm>
#include <iterator>
#include <string>
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
  static std::vector<T> apply(std::vector<T> const& values,
                              std::vector<double> const& distances,
                              double window_distance,
                              T pad_left,
                              T pad_right) {
    if (values.empty() || values.size() != distances.size()) {
      return {};
    }

    auto const half_window_d = window_distance / 2.0;

    auto res = std::vector<T>(values.size());

    auto w_start = 0U;
    auto w_end = 0U;
    for (auto i = 0U; i < values.size(); ++i) {
      auto const pos = distances[i];
      auto const left_edge = pos - half_window_d;
      auto const right_edge = pos + half_window_d;

      auto const evaluate = [&](auto a, auto b, auto x) {
        double const x0 = distances[a];
        double const x1 = distances[b];
        double const y0 = static_cast<double>(values[a].v_);
        double const y1 = static_cast<double>(values[b].v_);
        double const m = (y1 - y0) / (x1 - x0);

        return 0.5 * m * std::pow((x - x0), 2) + y0 * x;
      };

      while (w_start < i && distances[w_start] < left_edge) {
        ++w_start;
      }

      while (w_end < values.size() - 1 && distances[w_end + 1] <= right_edge) {
        ++w_end;
      }

      auto sum = 0.0;
      // edge to first point
      if (w_start > 0) {
        double const val = evaluate(w_start - 1, w_start, distances[w_start]) -
                           evaluate(w_start - 1, w_start, left_edge);
        sum += val;
      } else {
        double const val =
            (distances[0] - left_edge) * static_cast<double>(pad_left.v_);
        sum += val;
      }
      // points in window
      for (auto j = w_start; j < w_end; ++j) {
        double const val = evaluate(j, j + 1, distances[j + 1]) -
                           evaluate(j, j + 1, distances[j]);
        sum += val;
      }
      // last point to edge
      if (w_end < values.size() - 1) {
        double const val = evaluate(w_end, w_end + 1, right_edge) -
                           evaluate(w_end, w_end + 1, distances[w_end]);
        sum += val;
      } else {
        double const val =
            (right_edge - distances[w_end]) * static_cast<double>(pad_right.v_);
        sum += val;
      }

      res[i] = static_cast<T>(sum / window_distance);
    }

    return res;
  }
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

  auto it = utl::find_if(segments, [&](auto const& seg) {
    return seg.elevation_.first_ != nullptr && seg.elevation_.n_ != 0;
  });
  if (it == end(segments)) {
    return;
  }

  auto distances = std::vector<double>{};
  auto const add = [&](point const& pos, double d, ev::elevation_meters_t z) {
    points_.push_back(pos);
    elevation_.push_back(z);
    distances.push_back(d);
  };

  auto dist_acc = 0U;
  add(w.get_node_pos(it->from_), 0, *(it->elevation_.first_));
  for (; it != end(segments); it++) {
    auto const s = *it;
    if (s.elevation_.n_ == 0) {
      continue;
    }

    if (s.elevation_.n_ == 1) {
      dist_acc += s.dist_;
      add(w.get_node_pos(s.to_), dist_acc, *(s.elevation_.first_));
      continue;
    }

    auto from_lng = w.get_node_pos(s.from_).lng();
    auto from_lat = w.get_node_pos(s.from_).lat();
    auto to_lng = w.get_node_pos(s.to_).lng();
    auto to_lat = w.get_node_pos(s.to_).lat();
    auto const step_fak = 1.0 / (s.elevation_.n_ - 1);
    auto const step_dist = s.dist_ * step_fak;
    auto const lng_dist = adjust_lng(to_lng - from_lng) * step_fak;
    auto const lat_dist = (to_lat - from_lat) * step_fak;
    for (auto i = 1U; i < s.elevation_.n_; ++i) {
      to_lng = from_lng + lng_dist * i;
      to_lat = from_lat + lat_dist * i;
      dist_acc += step_dist;

      add(point::from_latlng({to_lat, to_lng}), dist_acc,
          *std::next(s.elevation_.first_, i));
    }
  }

  elevation_ = moving_average<ev::elevation_meters_t>::apply(
      elevation_, distances, static_cast<double>(dist_acc) / 4,
      elevation_.front(), elevation_.back());
  min_ = *std::min_element(begin(elevation_), end(elevation_));
  max_ = *std::max_element(begin(elevation_), end(elevation_));
  baseline_ = elevation_[0];

  auto prev = baseline_;
  for (auto const e : elevation_) {
    if (prev > e) {
      down_ = down_ + static_cast<elevation_monotonic_t>((prev - e).v_);
    } else {
      up_ = up_ + static_cast<elevation_monotonic_t>((e - prev).v_);
    }

    prev = e;
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