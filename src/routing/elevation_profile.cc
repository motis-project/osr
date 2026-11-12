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
  moving_average(double window_distance, T pad_left, T pad_right)
      : window_distance_(window_distance),
        pad_left_(pad_left),
        pad_right_(pad_right) {}

  std::vector<T> apply(std::vector<T> const& values,
                       std::vector<double> const& distances) {
    if (values.empty() || values.size() != distances.size()) {
      return {};
    }

    auto const half_window_d = window_distance_ / 2.0;

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
      if (w_start > 0) {
        double const val = evaluate(w_start - 1, w_start, distances[w_start]) -
                           evaluate(w_start - 1, w_start, left_edge);
        sum += val;
      } else {
        double const val =
            (distances[0] - left_edge) * static_cast<double>(values[0].v_);
        sum += val;
      }
      for (auto j = w_start; j < w_end; ++j) {
        double const val = evaluate(j, j + 1, distances[j + 1]) -
                           evaluate(j, j + 1, distances[j]);
        sum += val;
      }

      if (w_end < values.size() - 1) {
        double const val = evaluate(w_end, w_end + 1, right_edge) -
                           evaluate(w_end, w_end + 1, distances[w_end]);
        sum += val;
      } else {
        double const val = (right_edge - distances[w_end]) *
                           static_cast<double>(values[w_end].v_);
        sum += val;
      }

      res[i] = static_cast<T>(sum / window_distance_);
    }

    return res;
  }

  double window_distance_;
  T pad_left_;
  T pad_right_;
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

  auto distances = std::vector<double>{};
  auto const add = [&](point const& pos, double d, ev::elevation_meters_t z) {
    points_.push_back(pos);
    elevation_.push_back(z);
    distances.push_back(d);
  };

  auto dist_acc = 0U;
  for (auto s : segments) {
    if (s.elevation_.n_ == 0) {
      continue;
    }

    if (baseline_ == ev::elevation_meters_t::invalid()) {
      baseline_ = *s.elevation_.first_;
      min_ = baseline_;
      max_ = baseline_;
      add(w.get_node_pos(s.from_), 0, baseline_);
    }

    auto from_lng = w.get_node_pos(s.from_).lng();
    auto from_lat = w.get_node_pos(s.from_).lat();
    auto to_lng = w.get_node_pos(s.to_).lng();
    auto to_lat = w.get_node_pos(s.to_).lat();
    auto const step_dist_fak = 1.0 / (s.elevation_.n_ - 1);
    auto const lng_dist = adjust_lng(to_lng - from_lng) * step_dist_fak;
    auto const lat_dist = (to_lat - from_lat) * step_dist_fak;
    auto const step_dist = s.dist_ * step_dist_fak;

    for (auto i = 1U; i < s.elevation_.n_; ++i) {
      to_lng = from_lng + lng_dist * i;
      to_lat = from_lat + lat_dist * i;
      dist_acc += step_dist;

      add(point::from_latlng({to_lat, to_lng}), dist_acc,
          *std::next(s.elevation_.first_, i));
    }
  }

  auto avg = moving_average<ev::elevation_meters_t>{
      static_cast<double>(dist_acc) / 4, ev::elevation_meters_t{0},
      ev::elevation_meters_t{0}};
  elevation_ = avg.apply(elevation_, distances);
  min_ = *std::min_element(begin(elevation_), end(elevation_));
  max_ = *std::max_element(begin(elevation_), end(elevation_));
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