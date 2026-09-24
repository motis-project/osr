#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <string_view>
#include <utility>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

// An elevator (levels -3, -2, -1): way 1 on level -1, way 2 on level -2.
constexpr auto const kElevatorOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="10" lat="49.000020" lon="8.000100">
    <tag k="highway" v="elevator"/><tag k="level" v="-3;-2;-1"/>
  </node>
  <node id="11" lat="49.000050" lon="7.999000"/>
  <node id="12" lat="49.000050" lon="8.000100"/>
  <node id="21" lat="49.000020" lon="7.999000"/>
  <way id="1">
    <nd ref="11"/><nd ref="12"/><nd ref="10"/>
    <tag k="highway" v="footway"/><tag k="level" v="-1"/>
  </way>
  <way id="2">
    <nd ref="21"/><nd ref="10"/>
    <tag k="highway" v="footway"/><tag k="level" v="-2"/>
  </way>
</osm>
)";

// Ways 1 and 2 only connected at node 2, a private gate (a penalty for bus).
constexpr auto const kGateOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.002000">
    <tag k="barrier" v="gate"/><tag k="access" v="private"/>
  </node>
  <node id="3" lat="49.000000" lon="8.004000"/>
  <way id="1">
    <nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/>
  </way>
  <way id="2">
    <nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/>
  </way>
</osm>
)";

struct origin_node_cost_test : ::testing::Test {
  void load(char const* name, std::string_view const osm) {
    dir_ = fs::temp_directory_path() / name;
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf(name, osm), dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  void TearDown() override {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  // Calls `fn(from, to, dir, algo)` for both algorithms, both search
  // directions, and both orders of `a` and `b`.
  template <typename Fn>
  static void for_each_query(location const& a, location const& b, Fn&& fn) {
    for (auto const algo :
         {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
      for (auto const dir : {direction::kForward, direction::kBackward}) {
        for (auto const& [from, to] : {std::pair{a, b}, std::pair{b, a}}) {
          SCOPED_TRACE(::testing::Message()
                       << "algo=" << static_cast<int>(algo) << " dir=" << dir);
          fn(from, to, dir, algo);
        }
      }
    }
  }

  fs::path dir_;
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
};

TEST_F(origin_node_cost_test, elevator_next_to_the_origin_is_charged) {
  load("osr-origin-node-cost-elevator", kElevatorOsm);
  // On level -1, the closer way 2 on level -2 matches through the elevator.
  auto const upper = location{{49.000000, 8.000000}, level_t{-1.0F}};
  auto const lower = location{{49.000020, 7.999800}, level_t{-2.0F}};
  for_each_query(upper, lower,
                 [&](location const& from, location const& to,
                     direction const dir, routing_algorithm const algo) {
                   auto const p =
                       route(foot<false, elevator_tracking>::parameters{}, *w_,
                             *l_, search_profile::kFoot, from, to,
                             std::chrono::seconds{3600}, dir, 50.0, nullptr,
                             nullptr, nullptr, algo);
                   ASSERT_TRUE(p.has_value());
                   EXPECT_TRUE(p->uses_elevator_);
                   EXPECT_GE(p->duration_.count(), 90);
                 });
}

TEST_F(origin_node_cost_test, gate_next_to_the_origin_is_charged) {
  load("osr-origin-node-cost-gate", kGateOsm);
  // On way 1 and on way 2: every route passes the gate.
  auto const a = location{{49.000100, 8.001000}, kNoLevel};
  auto const b = location{{49.000100, 8.003000}, kNoLevel};
  for_each_query(
      a, b,
      [&](location const& from, location const& to, direction const dir,
          routing_algorithm const algo) {
        auto const cost = [&](cost_t const penalty) {
          auto const p =
              route(bus::parameters{.private_gate_penalty_ = penalty}, *w_, *l_,
                    search_profile::kBus, from, to, std::chrono::seconds{3600},
                    dir, 50.0, nullptr, nullptr, nullptr, algo);
          EXPECT_TRUE(p.has_value());
          return p.has_value() ? p->cost_ : cost_t{0U};
        };
        EXPECT_EQ(cost(0U) + 60U, cost(60U));
      });
}

}  // namespace
}  // namespace osr
