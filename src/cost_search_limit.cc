#include "osr/routing/cost_search_limit.h"

#include <cmath>
#include <algorithm>

namespace osr {
namespace {

struct cost_policy {
  double factor_;
  cost_t allowance_;
};

constexpr auto const kDefaultPolicy = cost_policy{1.25, 300U};
constexpr auto const kDurationPolicy = cost_policy{1.0, 60U};

template <bool Wheelchair, typename Tracking>
cost_policy policy(foot<Wheelchair, Tracking> const&,
                   typename foot<Wheelchair, Tracking>::parameters const&) {
  return kDefaultPolicy;
}

template <bike_costing Costing, unsigned Elevation, unsigned Exponent>
cost_policy policy(
    bike<Costing, Elevation, Exponent> const&,
    typename bike<Costing, Elevation, Exponent>::parameters const&) {
  if (Elevation != 0U) {
    return {kDefaultPolicy.factor_, kDefaultPolicy.allowance_ + Elevation};
  }
  return Costing == bike_costing::kFast ? kDurationPolicy : kDefaultPolicy;
}

template <bool Bus>
cost_policy policy(generic_car<Bus> const&,
                   typename generic_car<Bus>::parameters const&) {
  return kDurationPolicy;
}

cost_policy policy(hgv const&, hgv::parameters const&) {
  return kDefaultPolicy;
}

cost_policy policy(railway const&, railway::parameters const& p) {
  return {4.0, clamp_cost(1000ULL + p.slight_curve_penalty_ +
                          p.tight_curve_penalty_ + p.extreme_turn_penalty_)};
}

cost_policy policy(ferry const&, ferry::parameters const&) {
  return kDurationPolicy;
}

cost_policy combine(cost_policy const a, cost_policy const b) {
  return {std::max(a.factor_, b.factor_),
          clamp_cost(static_cast<std::uint64_t>(a.allowance_) + b.allowance_)};
}

template <bool Wheelchair, bool Parking>
cost_policy policy(
    car_parking<Wheelchair, Parking> const&,
    typename car_parking<Wheelchair, Parking>::parameters const& p) {
  return combine(policy(car{}, p.car_), policy(foot<Wheelchair>{}, p.foot_));
}

cost_policy policy(bike_sharing const&, bike_sharing::parameters const& p) {
  return combine(policy(bike_sharing::bikep{}, p.bike_),
                 policy(bike_sharing::footp{}, p.foot_));
}

template <typename Tracking>
cost_policy policy(car_sharing<Tracking> const&,
                   typename car_sharing<Tracking>::parameters const& p) {
  return combine(policy(car{}, p.car_), policy(foot<false>{}, p.foot_));
}

}  // namespace

cost_t cost_search_limit(profile_parameters const& params,
                         duration_t const duration) {
  return std::visit(
      [&](auto const& p) {
        using P = typename std::decay_t<decltype(p)>::profile_t;
        auto const [factor, allowance] = policy(P{}, p);
        auto const limit =
            std::ceil(static_cast<double>(duration.count()) * factor) +
            allowance;
        return static_cast<cost_t>(
            std::min(limit, static_cast<double>(kMaxDurationSearchCost - 1U)));
      },
      params);
}

}  // namespace osr
