#include "gtest/gtest.h"

#include <filesystem>
#include <memory>
#include <random>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {

constexpr auto const kMaxMatchDistance = 100.0;
constexpr auto const kMax = cost_t{900U};
constexpr auto const kNumDestinations = 200U;

struct graph {
  graph() {
    dir_ = fs::temp_directory_path() / "osr_one_to_many_state_test";
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

cost_t segment_cost_sum(path const& p) {
  auto sum = cost_t{0U};
  for (auto const& s : p.segments_) {
    sum += s.cost_;
  }
  return sum;
}

// The segments have to form one chain from start to destination.
void expect_contiguous(path const& p) {
  ASSERT_GE(p.segments_.size(), 2U);
  for (auto i = 1U; i < p.segments_.size(); ++i) {
    if (p.segments_[i - 1U].to_ != node_idx_t::invalid() &&
        p.segments_[i].from_ != node_idx_t::invalid()) {
      EXPECT_EQ(p.segments_[i - 1U].to_, p.segments_[i].from_) << i;
    }
    EXPECT_FALSE(p.segments_[i].polyline_.empty()) << i;
  }
}

// Returns true if the paths are identical (cost, duration, distance,
// segments). `uses_elevator_` is not compared: the legacy one-to-many
// reconstruct forces it to true, the state keeps the tracked value.
bool same_path(path const& a, path const& b) {
  if (a.cost_ != b.cost_ || a.duration_ != b.duration_ || a.dist_ != b.dist_ ||
      a.segments_.size() != b.segments_.size()) {
    return false;
  }
  for (auto i = 0U; i != a.segments_.size(); ++i) {
    if (a.segments_[i].from_ != b.segments_[i].from_ ||
        a.segments_[i].to_ != b.segments_[i].to_ ||
        a.segments_[i].way_ != b.segments_[i].way_ ||
        a.segments_[i].cost_ != b.segments_[i].cost_ ||
        a.segments_[i].polyline_.size() != b.segments_[i].polyline_.size()) {
      return false;
    }
  }
  return true;
}

// Reconstructing from the retained one-to-many search has to give what
// `route()` with `do_reconstruct` gives, in both directions. The only allowed
// difference: `route()` reconstructs as soon as a destination is settled,
// while later start candidates can still improve the search state, so the
// deferred reconstruct may return a path that is at most as expensive.
template <typename P>
void check(graph const& g,
           search_profile const profile,
           direction const dir,
           unsigned const seed) {
  auto const& w = *g.w_;
  auto const& l = *g.l_;
  auto const params = profile_parameters{typename P::parameters{}};

  auto prng = std::mt19937{seed};
  auto distr = std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
  auto const from = location{w.get_node_pos(node_idx_t{distr(prng)})};
  auto to = std::vector<location>{};
  for (auto i = 0U; i != kNumDestinations; ++i) {
    to.emplace_back(w.get_node_pos(node_idx_t{distr(prng)}));
  }

  auto const& pp = std::get<typename P::parameters>(params);
  auto from_m = match_result{};
  l.match<P>(pp, from, false, dir, kMaxMatchDistance, nullptr, from_m,
             std::nullopt);
  auto to_m = match_result{};
  for (auto const& x : to) {
    l.match<P>(pp, x, true, dir, kMaxMatchDistance, nullptr, to_m,
               std::nullopt);
  }
  auto const from_match = from_m[match_idx_t{0U}];
  ASSERT_FALSE(from_match.empty());

  auto const expected =
      route_one_to_many(params, w, l, profile, from, to, from_match, to_m, kMax,
                        dir, nullptr, nullptr, nullptr,
                        [](path const&) { return true; })
          ->results();
  auto const state = route_one_to_many(params, w, l, profile, from, to,
                                       from_match, to_m, kMax, dir);

  ASSERT_EQ(expected.size(), state->results().size());
  auto n_found = 0U, n_identical = 0U;
  for (auto k = 0U; k != to.size(); ++k) {
    ASSERT_EQ(expected[k].has_value(), state->results()[k].has_value()) << k;
    auto const reconstructed = state->reconstruct(w, l, k, nullptr);
    ASSERT_EQ(expected[k].has_value(), reconstructed.has_value()) << k;
    if (!expected[k].has_value()) {
      continue;
    }
    ++n_found;
    EXPECT_EQ(expected[k]->cost_, state->results()[k]->cost_) << k;
    EXPECT_EQ(expected[k]->cost_, reconstructed->cost_) << k;
    // The rendered segments must not exceed the search cost (catches a wrong
    // start candidate, e.g. the far side of a loop way); they can be cheaper
    // when a later start candidate improved the path.
    EXPECT_LE(segment_cost_sum(*reconstructed), reconstructed->cost_) << k;
    if (same_path(*expected[k], *reconstructed)) {
      ++n_identical;
    } else {
      expect_contiguous(*reconstructed);
      EXPECT_LE(segment_cost_sum(*reconstructed),
                segment_cost_sum(*expected[k]))
          << k;
    }
    // Reconstructing twice is fine (the state is not consumed).
    EXPECT_TRUE(
        same_path(*reconstructed, *state->reconstruct(w, l, k, nullptr)))
        << k;
  }
  EXPECT_GT(n_found, 10U);
  EXPECT_GE(n_identical * 2U, n_found);  // mostly identical
}

}  // namespace

TEST(one_to_many_state, foot_forward) {
  auto const g = graph{};
  check<foot<false, elevator_tracking>>(g, search_profile::kFoot,
                                        direction::kForward, 1U);
}

TEST(one_to_many_state, foot_backward) {
  auto const g = graph{};
  check<foot<false, elevator_tracking>>(g, search_profile::kFoot,
                                        direction::kBackward, 2U);
}

TEST(one_to_many_state, bike_forward) {
  auto const g = graph{};
  check<bike<bike_costing::kSafe, kElevationNoCost>>(g, search_profile::kBike,
                                                     direction::kForward, 3U);
}

TEST(one_to_many_state, bike_backward) {
  auto const g = graph{};
  check<bike<bike_costing::kSafe, kElevationNoCost>>(g, search_profile::kBike,
                                                     direction::kBackward, 4U);
}

TEST(one_to_many_state, empty_from_match) {
  auto const g = graph{};
  auto const params =
      profile_parameters{foot<false, elevator_tracking>::parameters{}};
  auto const to =
      std::vector<location>{location{g.w_->get_node_pos(node_idx_t{0U})}};
  auto to_m = match_result{};
  g.l_->match<foot<false, elevator_tracking>>(
      std::get<foot<false, elevator_tracking>::parameters>(params), to[0], true,
      direction::kForward, kMaxMatchDistance, nullptr, to_m, std::nullopt);
  auto const state =
      route_one_to_many(params, *g.w_, *g.l_, search_profile::kFoot, to[0], to,
                        match_view_t{}, to_m, kMax, direction::kForward);
  ASSERT_EQ(1U, state->results().size());
  EXPECT_FALSE(state->results()[0].has_value());
  EXPECT_FALSE(state->reconstruct(*g.w_, *g.l_, 0U, nullptr).has_value());
  EXPECT_FALSE(state->reconstruct(*g.w_, *g.l_, 5U, nullptr).has_value());
}
