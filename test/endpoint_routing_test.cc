#include <filesystem>
#include <numeric>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace osr {
namespace {

struct endpoint_routing : testing::Test {
  void SetUp() override {
    std::filesystem::create_directories(dir_);
    extract(false, test::write_osm_pbf("osr-endpoint-routing", R"(
<osm version="0.6">
  <node id="1" lat="49" lon="8"/>
  <node id="2" lat="49" lon="8.001"/>
  <node id="3" lat="49.001" lon="8.001"/>
  <node id="4" lat="49.1" lon="8"/>
  <node id="5" lat="49.1" lon="8.02"/>
  <node id="6" lat="49.099" lon="8"/>
  <node id="7" lat="49.101" lon="8.02"/>
  <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
  <way id="2"><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
  <way id="3"><nd ref="4"/><nd ref="5"/><tag k="highway" v="footway"/></way>
  <way id="4"><nd ref="4"/><nd ref="6"/><tag k="highway" v="footway"/></way>
  <way id="5"><nd ref="5"/><nd ref="7"/><tag k="highway" v="footway"/></way>
</osm>)"),
            dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  std::filesystem::path dir_{std::filesystem::temp_directory_path() /
                             "osr-endpoint-routing"};
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
};

TEST_F(endpoint_routing, direct_without_affordable_graph_root) {
  auto const params = get_parameters(search_profile::kFoot);
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    auto const from = location{49.1, 8.01};
    auto const to = location{49.1, 8.01001};
    auto const single = route(params, *w_, *l_, search_profile::kFoot, from, to,
                              100U, dir, 2.0);
    auto const many = route(params, *w_, *l_, search_profile::kFoot, from,
                            std::vector<location>{to}, 100U, dir, 2.0);
    ASSERT_TRUE(single.has_value());
    ASSERT_EQ(many.size(), 1U);
    ASSERT_TRUE(many.front().has_value());
    EXPECT_EQ(single->cost_, many.front()->cost_);
    EXPECT_EQ(single->duration_, many.front()->duration_);
  }
}

TEST_F(endpoint_routing, unaffordable_direct_does_not_hide_graph_route) {
  auto const params = get_parameters(search_profile::kFoot);
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    for (auto const algo :
         {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
      for (auto const max : {30U, 60U}) {
        auto const from = location{49., 8.00099};
        auto const to = location{49.00001, 8.001};
        auto const p = route(params, *w_, *l_, search_profile::kFoot, from, to,
                             max, dir, 2.0, nullptr, nullptr, nullptr, algo);
        ASSERT_TRUE(p.has_value());
        EXPECT_LT(p->cost_, max);
        auto const a = route_astar(params, *w_, *l_, search_profile::kFoot,
                                   from, to, max, dir, 2.0);
        ASSERT_TRUE(a.has_value());
        EXPECT_LT(a->cost_, max);
      }
    }
  }
}

TEST_F(endpoint_routing, meeting_turn_is_in_segment_totals) {
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    auto const from = location{49., 8.0005};
    auto const to = location{49.0005, 8.001};
    auto const bwd = dir == direction::kBackward;
    auto const p =
        route(get_parameters(search_profile::kBus), *w_, *l_,
              search_profile::kBus, bwd ? to : from, bwd ? from : to, 3600U,
              dir, 2.0, nullptr, nullptr, nullptr, routing_algorithm::kAStarBi);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->cost_, 35U);
    EXPECT_EQ(
        p->cost_,
        std::accumulate(begin(p->segments_), end(p->segments_), cost_t{0U},
                        [](auto sum, auto const& s) { return sum + s.cost_; }));
    EXPECT_EQ(p->duration_,
              std::accumulate(
                  begin(p->segments_), end(p->segments_), duration_t{0U},
                  [](auto sum, auto const& s) { return sum + s.duration_; }));
  }
}

TEST_F(endpoint_routing, geometry_follows_physical_travel) {
  auto const params = get_parameters(search_profile::kFoot);
  for (auto const& [from, to] :
       {std::pair{location{49., 8.0005}, location{49.0005, 8.001}},
        std::pair{location{49.0999, 8.}, location{49.1001, 8.02}},
        std::pair{location{49.1, 8.01}, location{49.1, 8.01001}}}) {
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      auto const bwd = dir == direction::kBackward;
      for (auto const algo :
           {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
        auto const p = route(params, *w_, *l_, search_profile::kFoot,
                             bwd ? to : from, bwd ? from : to, 100'000U, dir,
                             2.0, nullptr, nullptr, nullptr, algo);
        ASSERT_TRUE(p.has_value());
        ASSERT_FALSE(p->segments_.empty());
        EXPECT_LT(
            geo::distance(from.pos_, p->segments_.front().polyline_.front()),
            0.01);
        EXPECT_LT(geo::distance(to.pos_, p->segments_.back().polyline_.back()),
                  0.01);
        for (auto i = std::size_t{1U}; i < p->segments_.size(); ++i) {
          auto const& prev = p->segments_[i - 1U];
          auto const& next = p->segments_[i];
          EXPECT_EQ(prev.to_, next.from_);
          EXPECT_LT(
              geo::distance(prev.polyline_.back(), next.polyline_.front()),
              0.01);
        }
        auto const many = route(params, *w_, *l_, search_profile::kFoot, to,
                                std::vector<location>{from}, 100'000U,
                                direction::kBackward, 2.0, nullptr, nullptr,
                                nullptr, [](path const&) { return true; });
        ASSERT_TRUE(many.front().has_value());
        EXPECT_EQ(p->cost_, many.front()->cost_);
        EXPECT_EQ(p->duration_, many.front()->duration_);
      }
    }
  }
}

}  // namespace
}  // namespace osr
