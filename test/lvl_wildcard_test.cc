#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>

#include "boost/json.hpp"

#include "utl/pipes.h"

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace json = boost::json;
namespace fs = std::filesystem;
using namespace osr;

TEST(routing, no_lvl_wildcard) {
  constexpr auto const kTestFolder = "/tmp/osr_stuttgart";

  auto ec = std::error_code{};
  fs::remove_all(kTestFolder, ec);
  fs::create_directories(kTestFolder, ec);

  extract(false, "test/stuttgart.osm.pbf", kTestFolder);

  auto w = osr::ways{kTestFolder, cista::mmap::protection::READ};
  auto l = osr::lookup{w, kTestFolder, cista::mmap::protection::READ};

  auto const p = route(w, l, search_profile::kFoot,  //
                       {{48.7829, 9.18212}, level_t{0.F}},
                       {{48.7847, 9.18337}, level_t{1.F}},  //
                       3600, direction::kForward, 100);

  ASSERT_TRUE(p.has_value());

  auto const path = json::serialize(json::object{
      {"type", "FeatureCollection"},
      {"metadata", {{"duration", p->cost_}, {"distance", p->dist_}}},
      {"features",
       utl::all(p->segments_) | utl::transform([&](const path::segment& s) {
         return json::object{
             {"type", "Feature"},
             {
                 "properties",
                 {{"level", s.from_level_.to_float()},
                  {"osm_way_id", s.way_ == way_idx_t::invalid()
                                     ? 0U
                                     : to_idx(w.way_osm_idx_[s.way_])},
                  {"cost", s.cost_},
                  {"distance", s.dist_}},
             },
             {"geometry", to_line_string(s.polyline_)}};
       }) | utl::emplace_back_to<json::array>()}});
  EXPECT_EQ(
      "{\"type\":\"FeatureCollection\",\"metadata\":{\"duration\":385,"
      "\"distance\":4.2683610950337305E2},\"features\":[{\"type\":\"Feature\","
      "\"properties\":{\"level\":0E0,\"osm_way_id\":0,\"cost\":32,\"distance\":"
      "36},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[9.18212E0,4."
      "87829E1],[9.1824098E0,4.87829011E1],[9.1825297E0,4.87829369E1],[9."
      "1826092E0,4.87829279E1]]}},{\"type\":\"Feature\",\"properties\":{"
      "\"level\":1E0,\"osm_way_id\":1016875485,\"cost\":308,\"distance\":339},"
      "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[9.1826092E0,4."
      "87829279E1],[9.1827967E0,4.87828835E1],[9.1830696E0,4.87828214E1],[9."
      "1836147E0,4.87829191E1],[9.1844751E0,4.87833829E1],[9.1845214E0,4."
      "87834284E1],[9.1845395E0,4.87835258E1],[9.1845748E0,4.87835747E1],[9."
      "1846319E0,4.8783605E1],[9.1846862E0,4.87836279E1],[9.1847208E0,4."
      "8783654E1],[9.1847473E0,4.8783684E1],[9.1847559E0,4.87837162E1],[9."
      "1847516E0,4.8783756E1],[9.1846579E0,4.87840841E1],[9.1846341E0,4."
      "8784113E1],[9.1845471E0,4.87841486E1],[9.1844849E0,4.87841546E1],[9."
      "1841289E0,4.87841629E1],[9.1840357E0,4.87841863E1],[9.1838475E0,4."
      "87842531E1],[9.1835E0,4.8784369E1],[9.1835041E0,4.87843773E1]]}},{"
      "\"type\":\"Feature\",\"properties\":{\"level\":1E0,\"osm_way_id\":"
      "713810685,\"cost\":4,\"distance\":4},\"geometry\":{\"type\":"
      "\"LineString\",\"coordinates\":[[9.1835041E0,4.87843773E1],[9.1835086E0,"
      "4.87844171E1]]}},{\"type\":\"Feature\",\"properties\":{\"level\":1E0,"
      "\"osm_way_id\":0,\"cost\":41,\"distance\":47},\"geometry\":{\"type\":"
      "\"LineString\",\"coordinates\":[[9.1835086E0,4.87844171E1],[9.1834509E0,"
      "4.87844364E1],[9.1832201E0,4.87845268E1],[9.183214598396768E0,4."
      "8784528970208854E1],[9.18337E0,4.87847E1]]}}]}",
      path);
}
