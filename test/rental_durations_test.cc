#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <random>
#include <string>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

// one_to_many_state::rental_durations() reports where the ride is on a sharing
// path from the search chain alone. It has to agree with what reconstruct()
// lays out: the ride starts where the first segment with the vehicle's mode
// starts and ends where the last one ends, measured in duration from the start
// of the path in travel order.

namespace {

constexpr auto const kMaxMatchDistance = 100.0;
constexpr auto const kMax = std::chrono::seconds{1800};
constexpr auto const kNumDestinations = 200U;
constexpr auto const kNumVehicles = 30U;
constexpr auto const kNumSearches = 4U;

node_idx_t random_node(ways const& w, std::mt19937& prng) {
  return node_idx_t{static_cast<std::uint32_t>(prng() % w.n_nodes())};
}

// A random node on a way the predicate accepts.
template <typename Accessible>
node_idx_t random_node(ways const& w,
                       std::mt19937& prng,
                       Accessible&& accessible) {
  while (true) {
    auto const n = random_node(w, prng);
    for (auto const way : w.r_->node_ways_[n]) {
      if (accessible(w.r_->way_properties_[way])) {
        return n;
      }
    }
  }
}

struct graph {
  graph() {
    dir_ = fs::temp_directory_path() / "osr_rental_durations_test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, "test/miraustr.osm.pbf", dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  fs::path dir_;
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
};

// Free-floating vehicles at kNumVehicles random nodes on ways the vehicle can
// use; they can be returned anywhere (end allowed everywhere). car_sharing
// boards at any allowed street node (GTFS-Flex zones), bike_sharing only at an
// additional node (GBFS station or vehicle), so with `additional_nodes` each
// vehicle is an additional node tied to its street node by zero-length edges.
struct vehicles {
  template <typename Accessible>
  vehicles(ways const& w,
           std::mt19937& prng,
           bool const additional_nodes,
           Accessible&& accessible) {
    auto const size = static_cast<bitvec<node_idx_t>::size_type>(
        w.n_nodes() + (additional_nodes ? kNumVehicles : 0U));
    start_allowed_.resize(size);
    end_allowed_.resize(size);
    through_allowed_.resize(size);
    start_allowed_.zero_out();
    end_allowed_.one_out();
    through_allowed_.one_out();
    for (auto placed = 0U; placed != kNumVehicles; ++placed) {
      auto const n = random_node(w, prng, accessible);
      if (additional_nodes) {
        auto const a = node_idx_t{w.n_nodes() + placed};
        start_allowed_.set(a, true);
        additional_node_coordinates_.push_back(w.get_node_pos(n).as_latlng());
        additional_edges_[n].push_back({.to_ = a, .distance_ = 0U});
        additional_edges_[a].push_back({.to_ = n, .distance_ = 0U});
      } else {
        start_allowed_.set(n, true);
      }
    }
  }

  sharing_data view(ways const& w) const {
    return {.start_allowed_ = &start_allowed_,
            .end_allowed_ = &end_allowed_,
            .through_allowed_ = &through_allowed_,
            .additional_node_offset_ = w.n_nodes(),
            .additional_node_coordinates_ = additional_node_coordinates_,
            .additional_edges_ = additional_edges_};
  }

  bitvec<node_idx_t> start_allowed_{};
  bitvec<node_idx_t> end_allowed_{};
  bitvec<node_idx_t> through_allowed_{};
  std::vector<geo::latlng> additional_node_coordinates_{};
  hash_map<node_idx_t, std::vector<additional_edge>> additional_edges_{};
};

// Duration from the start of the path (travel order) to the start of the first
// and to the end of the last segment travelled with `ride_mode`.
struct ride_position {
  duration_t pickup_, drop_off_;
};

std::optional<ride_position> ride_from_segments(path const& p,
                                                mode const ride_mode) {
  auto pos = std::optional<ride_position>{};
  auto at = duration_t{0U};
  for (auto const& s : p.segments_) {
    if (s.mode_ == ride_mode) {
      if (!pos.has_value()) {
        pos = ride_position{.pickup_ = at, .drop_off_ = at};
      }
      pos->drop_off_ = at + s.duration_;
    }
    at += s.duration_;
  }
  return pos;
}

struct outcome {
  unsigned n_rides_{0U}, n_walks_{0U}, n_returned_at_destination_{0U};
};

constexpr auto const kReturnAnywhere = [](geo::latlng const&) { return true; };

// Routes from a random position to random destinations and checks every
// reached destination's rental_durations() against its reconstructed path.
// With `return_at_destination`, the vehicle may be returned at the destination
// coordinate: the matching piece at that end is then ridden.
template <typename Accessible>
outcome check(graph const& g,
              search_profile const profile,
              profile_parameters const& params,
              mode const ride_mode,
              bool const additional_nodes,
              bool const return_at_destination,
              direction const dir,
              unsigned const seed,
              Accessible&& accessible) {
  auto const& w = *g.w_;
  auto const& l = *g.l_;
  auto prng = std::mt19937{seed};
  auto const veh = vehicles{w, prng, additional_nodes, accessible};
  auto const sharing = veh.view(w);

  // The search start is on a way the vehicle can use: backward, it is the
  // travel destination, where the vehicle may be returned.
  auto const from = location{w.get_node_pos(random_node(w, prng, accessible))};
  auto to = std::vector<location>{};
  for (auto i = 0U; i != kNumDestinations; ++i) {
    to.push_back(location{w.get_node_pos(random_node(w, prng))});
  }

  auto from_m = match_result{};
  l.match(params, from, false, dir, kMaxMatchDistance, nullptr, profile, {},
          from_m);
  auto to_m = match_result{};
  for (auto const& x : to) {
    l.match(params, x, true, dir, kMaxMatchDistance, nullptr, profile, {},
            to_m);
  }
  auto const from_match = from_m[match_idx_t{0U}];
  EXPECT_FALSE(from_match.empty());
  if (from_match.empty()) {
    return {};
  }

  auto const state =
      route_one_to_many(params, w, l, profile, from, to, from_match, to_m, kMax,
                        dir, nullptr, &sharing);

  // Out of range and unreachable destinations: nullopt, no crash.
  EXPECT_FALSE(state->rental_durations(to.size()).has_value());
  EXPECT_FALSE(state->rental_durations(to.size() + 7U).has_value());

  auto out = outcome{};
  for (auto k = 0U; k != to.size(); ++k) {
    auto const& r = state->results()[k];
    auto const rc = state->rental_durations(k);
    if (!r.has_value()) {
      EXPECT_FALSE(rc.has_value()) << k;
      continue;
    }
    auto const p = state->reconstruct(w, l, k, &sharing);
    EXPECT_TRUE(p.has_value()) << k;
    if (!p.has_value()) {
      continue;
    }
    // The reconstructed path lasts as long as the search tracked: the
    // matching pieces at both ends are laid out with their tracked durations,
    // not with their costs.
    EXPECT_EQ(r->duration_, p->duration_) << k;
    auto const ride = ride_from_segments(*p, ride_mode);

    // A ride is reported exactly when the path has one.
    EXPECT_EQ(ride.has_value(), rc.has_value()) << k;
    if (!rc.has_value() || !ride.has_value()) {
      ++out.n_walks_;
      continue;
    }
    ++out.n_rides_;

    // The chain plus the final matching piece is the whole path as
    // reconstruct() shows it.
    auto const total = rc->dest_node_ + rc->dest_match_;
    EXPECT_EQ(p->duration_, total) << k;

    // Durations along the chain, in search order.
    EXPECT_LE(rc->before_min_, rc->min_) << k;
    EXPECT_LE(rc->min_, rc->max_) << k;
    EXPECT_LE(rc->max_, rc->after_max_) << k;
    EXPECT_LE(rc->after_max_, total) << k;
    if (!return_at_destination) {
      // The switch takes time (the search never starts on the vehicle).
      EXPECT_LT(rc->before_min_, rc->min_) << k;
    }

    // The vehicle is returned at the destination coordinate exactly if the
    // final matching piece, the path's last segment, is ridden. The ride then
    // reaches the end of the path: forward, the destination node is the last
    // rental label and the piece follows it; backward, the search root is the
    // first rental label and nothing precedes it.
    auto const returned_at_destination = p->segments_.back().mode_ == ride_mode;
    if (returned_at_destination) {
      EXPECT_TRUE(return_at_destination) << k;
      ++out.n_returned_at_destination_;
      if (dir == direction::kForward) {
        EXPECT_EQ(rc->max_, total) << k;
        EXPECT_EQ(rc->after_max_, total) << k;
      } else {
        EXPECT_EQ(rc->before_min_, duration_t{0U}) << k;
      }
    } else if (dir == direction::kForward) {
      EXPECT_LE(rc->after_max_, rc->dest_node_) << k;
    }

    // Ride start and end as the segments show them. Forward, the chain runs
    // in travel order; backward, it runs from the travel end, so positions
    // are measured from the far end.
    auto const pickup =
        dir == direction::kForward ? rc->before_min_ : total - rc->max_;
    auto const drop_off =
        dir == direction::kForward ? rc->max_ : total - rc->before_min_;
    EXPECT_EQ(ride->pickup_, pickup) << k;
    EXPECT_EQ(ride->drop_off_, drop_off) << k;
  }
  return out;
}

template <typename Accessible>
void check_both_directions(search_profile const profile,
                           profile_parameters const& params,
                           mode const ride_mode,
                           bool const additional_nodes,
                           bool const return_at_destination,
                           unsigned const seed,
                           Accessible&& accessible) {
  auto const g = graph{};
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    SCOPED_TRACE(std::string{to_str(dir)});
    // Several searches: backward, one search has a single travel destination,
    // and whether a vehicle gets returned at it depends on the place.
    auto out = outcome{};
    for (auto i = 0U; i != kNumSearches; ++i) {
      auto const o = check(g, profile, params, ride_mode, additional_nodes,
                           return_at_destination, dir, seed + i, accessible);
      out.n_rides_ += o.n_rides_;
      out.n_walks_ += o.n_walks_;
      out.n_returned_at_destination_ += o.n_returned_at_destination_;
    }
    // All cases have to occur for the checks above to mean anything.
    EXPECT_GT(out.n_rides_, 5U);
    EXPECT_GT(out.n_walks_, 5U);
    if (return_at_destination) {
      EXPECT_GT(out.n_returned_at_destination_, 0U);
    } else {
      EXPECT_EQ(out.n_returned_at_destination_, 0U);
    }
  }
}

constexpr auto const kCarAccessible = [](way_properties const& p) {
  return p.is_car_accessible();
};
constexpr auto const kBikeAccessible = [](way_properties const& p) {
  return p.is_bike_accessible();
};

}  // namespace

TEST(rental_durations, car_sharing) {
  check_both_directions(
      search_profile::kCarSharing,
      profile_parameters{car_sharing<track_node_tracking>::parameters{}},
      mode::kCar, false, false, 1U, kCarAccessible);
}

TEST(rental_durations, bike_sharing) {
  check_both_directions(search_profile::kBikeSharing,
                        profile_parameters{bike_sharing::parameters{}},
                        mode::kBike, true, false, 2U, kBikeAccessible);
}

TEST(rental_durations, car_sharing_return_at_destination) {
  auto params = car_sharing<track_node_tracking>::parameters{};
  params.vehicle_return_allowed_ = kReturnAnywhere;
  check_both_directions(search_profile::kCarSharing, profile_parameters{params},
                        mode::kCar, false, true, 1U, kCarAccessible);
}

TEST(rental_durations, bike_sharing_return_at_destination) {
  auto params = bike_sharing::parameters{};
  params.vehicle_return_allowed_ = kReturnAnywhere;
  check_both_directions(search_profile::kBikeSharing,
                        profile_parameters{params}, mode::kBike, true, true, 2U,
                        kBikeAccessible);
}

// Profiles without rental nodes never report a ride.
TEST(rental_durations, foot_has_no_ride) {
  auto const g = graph{};
  auto const& w = *g.w_;
  auto const& l = *g.l_;
  auto const params =
      profile_parameters{foot<false, elevator_tracking>::parameters{}};
  auto prng = std::mt19937{3U};
  auto const from = location{w.get_node_pos(random_node(w, prng))};
  auto to = std::vector<location>{};
  for (auto i = 0U; i != 50U; ++i) {
    to.push_back(location{w.get_node_pos(random_node(w, prng))});
  }
  auto from_m = match_result{};
  l.match(params, from, false, direction::kForward, kMaxMatchDistance, nullptr,
          search_profile::kFoot, {}, from_m);
  auto to_m = match_result{};
  for (auto const& x : to) {
    l.match(params, x, true, direction::kForward, kMaxMatchDistance, nullptr,
            search_profile::kFoot, {}, to_m);
  }
  auto const state = route_one_to_many(params, w, l, search_profile::kFoot,
                                       from, to, from_m[match_idx_t{0U}], to_m,
                                       kMax, direction::kForward);
  auto n_found = 0U;
  for (auto k = 0U; k != to.size(); ++k) {
    auto const& r = state->results()[k];
    EXPECT_FALSE(state->rental_durations(k).has_value()) << k;
    if (!r.has_value()) {
      continue;
    }
    ++n_found;
    auto const p = state->reconstruct(w, l, k, nullptr);
    ASSERT_TRUE(p.has_value()) << k;
    EXPECT_EQ(r->duration_, p->duration_) << k;
  }
  EXPECT_GT(n_found, 5U);
}
