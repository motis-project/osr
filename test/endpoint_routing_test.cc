#include <filesystem>

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

}  // namespace
}  // namespace osr
