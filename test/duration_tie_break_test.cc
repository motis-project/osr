#include "gtest/gtest.h"

#include <filesystem>
#include <memory>

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/routing/astar.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/search_params.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {

// Two routes reach node 6 at the same cost but with very different durations,
// and both arrive there in the same routing state, so they compete for one
// entry slot:
//
//   1 --way 1 (2x 236m @ 10km/h)--> 2 --> 3 --way 3 (7m @ 120km/h)--> 5
//   1 --way 2 (28m @ 10km/h, motor_vehicle=destination)--> 4 --way 4 (7m)--> 5
//   5 --way 5 (7m @ 120km/h)--> 6
//
// The car profile charges `5 * base + 120` on a `motor_vehicle=destination`
// way but keeps the plain travel time as the duration, which is what makes the
// two routes cost-equal and duration-different:
//
//   via node 3: cost 85 + 85 = 170, duration 170
//   via node 4: cost 5 * 10 + 120 = 170, duration 10
//
// Ways 3, 4 and 5 are short enough at 120km/h that `rint(dist * s_per_m)`
// rounds to 0, so they are edges with cost 0. That is what puts nodes 3 and 4
// in the *same* cost bucket as nodes 5 and 6: the state at node 6 can still be
// improved from inside the bucket it is settled in.
//
// Node 2 carries a stub way (way 6) so that it stays a junction and node 3 is
// therefore pushed while bucket 85 is processed - after node 4, which is
// pushed from bucket 0. `dial` pops a bucket LIFO, so the slow route is
// expanded first and a search that stops at the first pop of node 6 settles
// the 170s duration and never looks at node 4.
constexpr auto const kOsm = R"(
<osm version="0.6">
  <node id="1" lat="49.0000000" lon="8.0000000"/>
  <node id="2" lat="49.0021208" lon="8.0001000"/>
  <node id="3" lat="49.0000000" lon="8.0002000"/>
  <node id="4" lat="49.0000000" lon="8.0003833"/>
  <node id="5" lat="49.0000000" lon="8.0002900"/>
  <node id="6" lat="49.0000600" lon="8.0002900"/>
  <node id="7" lat="49.0021208" lon="8.0004000"/>
  <way id="1">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="10"/>
  </way>
  <way id="2">
    <nd ref="1"/>
    <nd ref="4"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="10"/>
    <tag k="motor_vehicle" v="destination"/>
  </way>
  <way id="3">
    <nd ref="3"/>
    <nd ref="5"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="120"/>
  </way>
  <way id="4">
    <nd ref="4"/>
    <nd ref="5"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="120"/>
  </way>
  <way id="5">
    <nd ref="5"/>
    <nd ref="6"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="120"/>
  </way>
  <way id="6">
    <nd ref="2"/>
    <nd ref="7"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="10"/>
  </way>
  <way id="7">
    <nd ref="6"/>
    <nd ref="7"/>
    <tag k="highway" v="residential"/>
    <tag k="maxspeed" v="10"/>
  </way>
</osm>
)";

constexpr auto const kMaxCost = cost_t{10'000U};

struct fixture {
  fixture() {
    dir_ = fs::temp_directory_path() / "osr_duration_tie_break_test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    osr::extract(false,
                 osr::test::write_osm_pbf("osr_duration_tie_break", kOsm), dir_,
                 {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);

    start_ = *w_->find_node_idx(osm_node_idx_t{1U});
    auto const end_node = *w_->find_node_idx(osm_node_idx_t{6U});
    // the state both routes arrive in: node 6, along way 5, forwards
    end_ = car::node{
        .n_ = end_node,
        .way_ = w_->r_->get_way_pos(end_node, *w_->find_way(osm_way_idx_t{5U})),
        .dir_ = direction::kForward};
  }

  template <typename Search>
  cost_and_duration result(Search const& search) const {
    auto const it = search.cost_.find(end_.get_key());
    return it == end(search.cost_)
               ? infeasible_cost_and_duration()
               : cost_and_duration{.cost_ = it->second.cost(end_),
                                   .duration_ = it->second.duration(end_)};
  }

  template <typename Fn>
  void for_each_start_state(Fn&& f) const {
    auto const ways = w_->r_->node_ways_[start_];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      f(car::node{.n_ = start_, .way_ = i, .dir_ = direction::kForward});
      f(car::node{.n_ = start_, .way_ = i, .dir_ = direction::kBackward});
    }
  }

  search_params<car::parameters> make_search_params() const {
    return {.profile_ = params_,
            .w_ = w_.get(),
            .max_ = kMaxCost,
            .dir_ = direction::kForward,
            .start_loc_ = location{w_->get_node_pos(start_), kNoLevel},
            .end_loc_ = location{w_->get_node_pos(end_.get_node()), kNoLevel}};
  }

  template <bool EarlyTermination>
  cost_and_duration run_dijkstra(dijkstra<car, EarlyTermination>& d) const {
    d.reset(make_search_params());
    if constexpr (EarlyTermination) {
      d.add_destination(end_);
    }
    for_each_start_state([&](car::node const n) {
      d.add_start(car::label{n, 0U}, duration_t{0});
    });
    d.run();
    return result(d);
  }

  template <bool EarlyTermination>
  cost_and_duration run_astar(astar<car, EarlyTermination>& a) const {
    a.reset(make_search_params());
    a.add_destination(end_);
    for_each_start_state([&](car::node const n) {
      a.add_start(car::label{n, 0U}, duration_t{0});
    });
    a.run();
    return result(a);
  }

  fs::path dir_;
  std::unique_ptr<ways> w_;
  car::parameters params_{};
  node_idx_t start_{node_idx_t::invalid()};
  car::node end_{car::node::invalid()};
};

}  // namespace

// The fixture is only a regression test for the tie break as long as it really
// does offer two cost-equal routes with different durations.
// A change to the cost model should cause a failure here, so that the
// tests can be updated to reflect the new cost model.
TEST(duration_tie_break, fixture_has_a_cost_tie_with_distinct_durations) {
  auto const f = fixture{};
  auto d = dijkstra<car, false>{};
  auto const r = f.run_dijkstra(d);

  EXPECT_EQ(cost_t{170U}, r.cost_);
  EXPECT_EQ(10, r.duration_.count());
}

TEST(duration_tie_break, early_termination_dijkstra_drains_the_bucket) {
  auto const f = fixture{};

  auto exhaustive = dijkstra<car, false>{};
  auto const expected = f.run_dijkstra(exhaustive);
  ASSERT_NE(kInfeasible, expected.cost_);

  auto early = dijkstra<car, true>{};
  auto const actual = f.run_dijkstra(early);

  EXPECT_EQ(expected.cost_, actual.cost_);
  EXPECT_EQ(expected.duration_.count(), actual.duration_.count())
      << "early termination settled cost " << actual.cost_ << " with duration "
      << actual.duration_.count() << ", but an equal cost route with duration "
      << expected.duration_.count() << " was still pending in the bucket";
}

TEST(duration_tie_break, early_termination_astar_drains_the_bucket) {
  auto const f = fixture{};

  auto exhaustive = dijkstra<car, false>{};
  auto const expected = f.run_dijkstra(exhaustive);
  ASSERT_NE(kInfeasible, expected.cost_);

  auto early = astar<car, true>{};
  auto const actual = f.run_astar(early);

  EXPECT_EQ(expected.cost_, actual.cost_);
  EXPECT_EQ(expected.duration_.count(), actual.duration_.count())
      << "early termination settled cost " << actual.cost_ << " with duration "
      << actual.duration_.count() << ", but an equal cost route with duration "
      << expected.duration_.count() << " was still pending in the bucket";
}
