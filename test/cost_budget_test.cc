#include "gtest/gtest.h"

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <random>
#include <vector>

#include "utl/to_vec.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/cost_search_limit.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace {

// Berlin, Miraustr.: regression query for a 315 s walk that bidirectional A*
// missed with MOTIS's reconstruction limit (leg duration + 5 min).
constexpr auto const kFrom =
    osr::location{52.5863794, 13.3109247, osr::kNoLevel};
constexpr auto const kTo = osr::location{52.5840800, 13.3096810, osr::kNoLevel};

// What MOTIS passes for a 360s access leg (360 + 5min).
constexpr auto const kReconstructionLimit = std::chrono::seconds{660};

constexpr auto const kLongEdgeOsm = R"(
<osm version="0.6">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.001000"/>
  <node id="3" lat="49.080000" lon="8.001000"/>
  <node id="4" lat="49.080000" lon="8.002000"/>
  <way id="1">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="2">
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="3">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
)";

struct graph {
  graph() {
    dir_ = fs::temp_directory_path() / "osr_cost_budget_test";

    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);

    osr::extract(false, "test/miraustr.osm.pbf", dir_, {});
    w_ = std::make_unique<osr::ways>(dir_, cista::mmap::protection::READ);
    l_ =
        std::make_unique<osr::lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  std::optional<osr::path> route(std::chrono::seconds const max,
                                 osr::routing_algorithm const algo) const {
    return osr::route(osr::foot<false, osr::elevator_tracking>::parameters{},
                      *w_, *l_, osr::search_profile::kFoot, kFrom, kTo, max,
                      osr::direction::kForward, /*max_match_distance=*/25.0,
                      nullptr, nullptr, nullptr, algo);
  }

  fs::path dir_;
  std::unique_ptr<osr::ways> w_;
  std::unique_ptr<osr::lookup> l_;
};

}  // namespace

TEST(cost_budget, duration_policy_is_monotonic) {
  for (auto const profile :
       {osr::search_profile::kFoot, osr::search_profile::kWheelchair,
        osr::search_profile::kBike, osr::search_profile::kBikeFast,
        osr::search_profile::kBikeElevationLow,
        osr::search_profile::kBikeElevationHigh, osr::search_profile::kCar,
        osr::search_profile::kHgv, osr::search_profile::kBikeSharing,
        osr::search_profile::kCarSharing, osr::search_profile::kCarParking,
        osr::search_profile::kCarParkingWheelchair,
        osr::search_profile::kCarDropOff,
        osr::search_profile::kCarDropOffWheelchair, osr::search_profile::kBus,
        osr::search_profile::kRailway, osr::search_profile::kFerry}) {
    auto const params = osr::get_parameters(profile);
    auto prev = osr::cost_t{0U};
    for (auto const t : {0, 1, 60, 900, 3600, 65534}) {
      auto const limit = osr::cost_search_limit(params, osr::duration_t{t});
      EXPECT_GE(limit, prev);
      EXPECT_LT(limit, osr::kMaxDurationSearchCost);
      prev = limit;
    }
  }
}

TEST(cost_budget, duration_filters_the_preferred_route) {
  auto const dir = fs::temp_directory_path() / "osr_duration_limit_test";
  fs::create_directories(dir);
  osr::extract(false, osr::test::write_osm_pbf("osr_duration_limit", R"(
<osm version="0.6">
  <node id="1" lat="49" lon="8"/>
  <node id="2" lat="49" lon="8.001"/>
  <node id="3" lat="49" lon="8.003"/>
  <node id="4" lat="49" lon="8.004"/>
  <node id="5" lat="49.002" lon="8.001"/>
  <node id="6" lat="49.002" lon="8.003"/>
  <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/><tag k="maxspeed" v="50"/></way>
  <way id="2"><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/><tag k="motor_vehicle" v="destination"/><tag k="maxspeed" v="50"/></way>
  <way id="3"><nd ref="3"/><nd ref="4"/><tag k="highway" v="residential"/><tag k="maxspeed" v="50"/></way>
  <way id="4"><nd ref="2"/><nd ref="5"/><tag k="highway" v="residential"/><tag k="maxspeed" v="50"/></way>
  <way id="5"><nd ref="5"/><nd ref="6"/><tag k="highway" v="residential"/><tag k="maxspeed" v="50"/></way>
  <way id="6"><nd ref="6"/><nd ref="3"/><tag k="highway" v="residential"/><tag k="maxspeed" v="50"/></way>
</osm>)"),
               dir, {});
  auto const w = osr::ways{dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, dir, cista::mmap::protection::READ};
  auto const params = osr::get_parameters(osr::search_profile::kCar);
  auto const from = osr::location{49, 8.0005};
  auto const to = osr::location{49, 8.0035};
  EXPECT_ANY_THROW(osr::route(params, w, l, osr::search_profile::kCar, from, to,
                              std::chrono::seconds{-1},
                              osr::direction::kForward, 2.0));
  EXPECT_ANY_THROW(osr::route(params, w, l, osr::search_profile::kCar, from, to,
                              std::chrono::seconds{65535},
                              osr::direction::kForward, 2.0));
  auto blocked = osr::bitvec<osr::node_idx_t>{w.n_nodes()};
  blocked.set(*w.find_node_idx(osr::osm_node_idx_t{5U}));
  auto const alternative = osr::route(params, w, l, osr::search_profile::kCar,
                                      from, to, std::chrono::seconds{1000},
                                      osr::direction::kForward, 2.0, &blocked);
  auto const preferred =
      osr::route(params, w, l, osr::search_profile::kCar, from, to,
                 std::chrono::seconds{1000}, osr::direction::kForward, 2.0);
  ASSERT_TRUE(preferred.has_value());
  ASSERT_TRUE(alternative.has_value());
  ASSERT_LT(preferred->cost_, alternative->cost_);
  ASSERT_GT(preferred->duration_, alternative->duration_);

  for (auto const direction :
       {osr::direction::kForward, osr::direction::kBackward}) {
    auto const a = direction == osr::direction::kForward ? from : to;
    auto const b = direction == osr::direction::kForward ? to : from;
    for (auto const algo : {osr::routing_algorithm::kDijkstra,
                            osr::routing_algorithm::kAStarBi}) {
      for (auto const t :
           {alternative->duration_.count(), preferred->duration_.count(),
            static_cast<osr::duration_t::rep>(preferred->duration_.count() +
                                              60U)}) {
        auto const max_duration = std::chrono::seconds{t};
        auto const p = osr::route(params, w, l, osr::search_profile::kCar, a, b,
                                  max_duration, direction, 2.0, nullptr,
                                  nullptr, nullptr, algo);
        if (t < preferred->duration_.count()) {
          EXPECT_FALSE(p.has_value());
        } else {
          ASSERT_TRUE(p.has_value());
          EXPECT_EQ(p->cost_, preferred->cost_);
          EXPECT_EQ(p->duration_, preferred->duration_);
        }
      }
    }
    auto const max_duration =
        std::chrono::seconds{alternative->duration_.count()};
    auto const many =
        osr::route(params, w, l, osr::search_profile::kCar, a,
                   std::vector<osr::location>{b}, max_duration, direction, 2.0);
    ASSERT_EQ(many.size(), 1U);
    EXPECT_FALSE(many.front().has_value());
  }
}

TEST(cost_budget, short_walk_within_budget) {
  auto const g = graph{};

  auto const dijkstra =
      g.route(kReconstructionLimit, osr::routing_algorithm::kDijkstra);
  ASSERT_TRUE(dijkstra.has_value());
  EXPECT_LE(dijkstra->duration_.count(), kReconstructionLimit.count());

  auto const astar_bi =
      g.route(kReconstructionLimit, osr::routing_algorithm::kAStarBi);
  ASSERT_TRUE(astar_bi.has_value())
      << "bidirectional A* found no path within "
      << kReconstructionLimit.count() << " s, but dijkstra routes it at cost "
      << dijkstra->cost_;
  EXPECT_EQ(dijkstra->cost_, astar_bi->cost_);
}

TEST(cost_budget, short_walk_with_generous_budget) {
  auto const g = graph{};

  auto const dijkstra =
      g.route(std::chrono::seconds{3600}, osr::routing_algorithm::kDijkstra);
  auto const astar_bi =
      g.route(std::chrono::seconds{3600}, osr::routing_algorithm::kAStarBi);

  ASSERT_TRUE(dijkstra.has_value());
  ASSERT_TRUE(astar_bi.has_value());
  EXPECT_EQ(dijkstra->cost_, astar_bi->cost_);
}

TEST(cost_budget, internal_edge_longer_than_bidirectional_radius) {
  auto const dir =
      fs::temp_directory_path() / "osr_bidirectional_long_edge_test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  osr::extract(
      false,
      osr::test::write_osm_pbf("osr_bidirectional_long_edge", kLongEdgeOsm),
      dir, {});

  auto const w = osr::ways{dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, dir, cista::mmap::protection::READ};
  auto const params = osr::foot<false, osr::elevator_tracking>::parameters{};
  auto const from = osr::location{49.000000, 8.000500};
  auto const to = osr::location{49.080000, 8.001500};
  auto constexpr kLimit = std::chrono::seconds{7600};
  auto const route_with = [&](osr::routing_algorithm const algo) {
    return osr::route(params, w, l, osr::search_profile::kFoot, from, to,
                      kLimit, osr::direction::kForward,
                      /*max_match_distance=*/25.0, nullptr, nullptr, nullptr,
                      algo);
  };

  auto const dijkstra = route_with(osr::routing_algorithm::kDijkstra);
  ASSERT_TRUE(dijkstra.has_value());
  ASSERT_LE(dijkstra->duration_.count(), kLimit.count());

  auto const astar_bi = route_with(osr::routing_algorithm::kAStarBi);
  ASSERT_TRUE(astar_bi.has_value());
  EXPECT_EQ(dijkstra->cost_, astar_bi->cost_);
}

// Exercise the engine directly: public routing may fall back to Dijkstra.
TEST(cost_budget, bidirectional_competing_routes_asymmetric_roots) {
  using P = osr::foot<false, osr::elevator_tracking>;
  auto const dir = fs::temp_directory_path() / "osr_bidir_competing_routes";
  fs::create_directories(dir);
  osr::extract(false, osr::test::write_osm_pbf("osr_bidir_competing_routes", R"(
<osm version="0.6">
  <node id="1" lat="49" lon="7.999"/>
  <node id="2" lat="49" lon="8"/>
  <node id="3" lat="49.04" lon="8"/>
  <node id="4" lat="49.08" lon="8"/>
  <node id="5" lat="49.04" lon="8.04"/>
  <node id="6" lat="49.08" lon="8.001"/>
  <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="footway"/></way>
  <way id="2"><nd ref="2"/><nd ref="3"/><tag k="highway" v="footway"/></way>
  <way id="3"><nd ref="3"/><nd ref="4"/><tag k="highway" v="footway"/></way>
  <way id="4"><nd ref="2"/><nd ref="5"/><tag k="highway" v="footway"/></way>
  <way id="5"><nd ref="5"/><nd ref="4"/><tag k="highway" v="footway"/></way>
  <way id="6"><nd ref="4"/><nd ref="6"/><tag k="highway" v="footway"/></way>
</osm>)"),
               dir, {});
  auto const w = osr::ways{dir, cista::mmap::protection::READ};
  auto const a =
      P::node{w.get_node_idx(osr::osm_node_idx_t{2U}), osr::kNoLevel};
  auto const z =
      P::node{w.get_node_idx(osr::osm_node_idx_t{4U}), osr::kNoLevel};
  auto blocked = osr::bitvec<osr::node_idx_t>{w.n_nodes()};
  blocked.set(w.get_node_idx(osr::osm_node_idx_t{3U}));
  for (auto const reverse : {false, true}) {
    auto const start = reverse ? z : a;
    auto const goal = reverse ? a : z;
    auto const direction =
        reverse ? osr::direction::kBackward : osr::direction::kForward;
    for (auto const penalties : {std::pair{7U, 311U}, std::pair{6000U, 9000U},
                                 std::pair{9000U, 6000U}}) {
      auto const [start_cost, end_cost] = penalties;
      auto d = osr::dijkstra<P, false>{};
      auto sp = osr::search_params<P::parameters>{
          .w_ = &w,
          .max_ = 60000U,
          .dir_ = direction,
          .start_loc_ = osr::location{w.get_node_pos(start.n_).as_latlng()},
          .end_loc_ = osr::location{w.get_node_pos(goal.n_).as_latlng()}};
      d.reset(sp);
      d.add_start(P::label{start, start_cost}, osr::duration_t{3U});
      d.run();
      auto const expected = d.get_cost(goal) + end_cost;
      auto const duration =
          d.cost_.at(goal.get_key()).duration(goal) + osr::duration_t{11U};
      ASSERT_LT(expected, 60000U);
      ASSERT_GT(d.get_cost(goal) - start_cost,
                2U * osr::bidirectional<P>::kLongestNodeDistance);
      sp.blocked_ = &blocked;
      d.reset(sp);
      d.add_start(P::label{start, start_cost}, osr::duration_t{3U});
      d.run();
      ASSERT_GT(d.get_cost(goal) + end_cost, expected);
      ASSERT_LT(d.get_cost(goal) + end_cost, 60000U);
      sp.blocked_ = nullptr;
      for (auto const max :
           {expected - 1U, expected, expected + 1U, expected * 2U, 60000U}) {
        SCOPED_TRACE(testing::Message()
                     << "reverse=" << reverse << " roots=" << start_cost << ","
                     << end_cost << " max=" << max);
        sp.max_ = max;
        auto b = osr::bidirectional<P>{};
        b.reset(sp);
        ASSERT_TRUE(b.search_bounds_valid_);
        b.add_start(P::label{start, start_cost}, osr::duration_t{3U});
        b.add_end(P::label{goal, end_cost}, osr::duration_t{11U});
        b.run();
        auto const found = b.best_cost_ < max;
        EXPECT_EQ(found, expected < max);
        if (found) {
          EXPECT_EQ(b.best_cost_, expected);
          EXPECT_EQ(b.best_duration(), duration);
        }
      }
    }
  }
}

// Early stopping must preserve the generous search's preferred route,
// rejecting it if it exceeds the duration limit.
TEST(cost_budget, duration_stop_matches_generous_search) {
  auto const dir = fs::temp_directory_path() / "osr_duration_stop_test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  osr::extract(false, "test/miraustr.osm.pbf", dir, {});
  auto const w = osr::ways{dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto rng = std::mt19937{42U};
  auto random_node = std::uniform_int_distribution<osr::node_idx_t::value_t>{
      0U, w.n_nodes() - 1U};
  auto offset = std::uniform_real_distribution{-0.00015, 0.00015};
  auto const random_location = [&]() {
    auto pos = w.get_node_pos(osr::node_idx_t{random_node(rng)}).as_latlng();
    pos.lat_ += offset(rng);
    pos.lng_ += offset(rng);
    return osr::location{pos, osr::kNoLevel};
  };

  constexpr auto const kGenerousLimit = std::chrono::seconds{30000};
  auto n_found = 0U;
  auto n_too_slow = 0U;
  for (auto const profile :
       {osr::search_profile::kFoot, osr::search_profile::kBike,
        osr::search_profile::kCar}) {
    auto const params = osr::get_parameters(profile);
    for (auto sample = 0U; sample != 40U; ++sample) {
      auto const from = random_location();
      auto to = std::vector<osr::location>(4U);
      std::generate(begin(to), end(to), random_location);
      auto const references = utl::to_vec(to, [&](osr::location const& target) {
        return osr::route(params, w, l, profile, from, target, kGenerousLimit,
                          osr::direction::kForward, 25.0, nullptr, nullptr,
                          nullptr, osr::routing_algorithm::kDijkstra);
      });
      for (auto const t : {60, 180, 420}) {
        auto const max_duration = std::chrono::seconds{t};
        auto const cap = osr::cost_search_limit(params, osr::duration_t{t});
        auto const many =
            osr::route(params, w, l, profile, from, to, max_duration,
                       osr::direction::kForward, 25.0);
        ASSERT_EQ(many.size(), to.size());
        for (auto i = 0U; i != to.size(); ++i) {
          SCOPED_TRACE(testing::Message() << "profile=" << osr::to_str(profile)
                                          << " sample=" << sample << " t=" << t
                                          << " destination=" << i);
          auto const& reference = references[i];
          auto const single =
              osr::route(params, w, l, profile, from, to[i], max_duration,
                         osr::direction::kForward, 25.0, nullptr, nullptr,
                         nullptr, osr::routing_algorithm::kDijkstra);
          auto const reference_fits =
              reference.has_value() &&
              static_cast<int>(reference->duration_.count()) <= t;
          for (auto const* p : {&single, &many[i]}) {
            if (p->has_value()) {
              ASSERT_TRUE(reference.has_value());
              EXPECT_EQ((*p)->cost_, reference->cost_);
              EXPECT_EQ((*p)->duration_, reference->duration_);
              EXPECT_TRUE(reference_fits);
            } else {
              EXPECT_FALSE(reference_fits && reference->cost_ < cap)
                  << "missing route with cost " << reference->cost_
                  << " below cap " << cap;
            }
          }
          n_found += single.has_value() ? 1U : 0U;
          n_too_slow += reference.has_value() && !reference_fits ? 1U : 0U;
        }
      }
    }
  }
  EXPECT_GT(n_found, 100U);
  EXPECT_GT(n_too_slow, 100U);
}
