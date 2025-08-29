#include "gtest/gtest.h"
#include "osr/geojson.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"
#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

std::string extract_and_route(std::string_view path,
                              osr::location const& from,
                              osr::location const& to) {
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const p = osr::route(osr::foot<false, osr::elevator_tracking>::parameters{}, w, l, osr::search_profile::kFoot, from, {to}, 900,
                            osr::direction::kForward, 250.0, nullptr, nullptr,
                            nullptr, osr::routing_algorithm::kDijkstra);
  utl::verify(p.has_value(), "{}: from={} to={} -> no route", path,
              fmt::streamed(from), fmt::streamed(to));
  return osr::to_featurecollection(w, *p, false);
}

TEST(routing, island) {
  auto const from = osr::location{49.872715, 8.651534, osr::level_t{0.F}};
  auto const to = osr::location{49.873023, 8.651523, osr::level_t{0.F}};
  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":3,"distance":3},"geometry":{"type":"LineString","coordinates":[[8.651514431787469E0,4.987274386418564E1],[8.6515174E0,4.98727447E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1201551426,"cost":3,"distance":3},"geometry":{"type":"LineString","coordinates":[[8.6515174E0,4.98727447E1],[8.6515563E0,4.98727556E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":22937760,"cost":12,"distance":12},"geometry":{"type":"LineString","coordinates":[[8.6515563E0,4.98727556E1],[8.6515406E0,4.98727706E1],[8.6514528E0,4.98728367E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":847710844,"cost":6,"distance":7},"geometry":{"type":"LineString","coordinates":[[8.6514528E0,4.98728367E1],[8.6514754E0,4.98728959E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":23511479,"cost":10,"distance":10},"geometry":{"type":"LineString","coordinates":[[8.6514754E0,4.98728959E1],[8.651539E0,4.98729497E1],[8.6515596E0,4.98729618E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":8,"distance":9},"geometry":{"type":"LineString","coordinates":[[8.6515596E0,4.98729618E1],[8.651493334251755E0,4.987300334757231E1]]}}]})",
      extract_and_route("test/luisenplatz-darmstadt.osm.pbf", from, to));
}

TEST(routing, ferry) {
  auto const from = osr::location{41.921472, 8.742216, osr::level_t{0.F}};
  auto const to = osr::location{41.921436, 8.740166, osr::level_t{0.F}};
  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":84,"distance":101},"geometry":{"type":"LineString","coordinates":[[8.7415295E0,4.19221369E1],[8.7415321E0,4.19222147E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":147,"distance":177},"geometry":{"type":"LineString","coordinates":[[8.7415321E0,4.19222147E1],[8.7408991E0,4.19222267E1],[8.7400905E0,4.19218375E1]]}}]})",
      extract_and_route("test/ajaccio-ferry.osm.pbf", from, to));
}

TEST(routing, corridor) {
  auto const from =
      osr::location{51.54663831994142, -0.05622849779558692, osr::level_t{0.F}};
  auto const to =
      osr::location{51.547004658329, -0.05694437499428773, osr::level_t{0.F}};
  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":1,"distance":1},"geometry":{"type":"LineString","coordinates":[[-5.6227579350404025E-2,5.154663151628954E1],[-5.62413E-2,5.15466308E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1057782512,"cost":6,"distance":6},"geometry":{"type":"LineString","coordinates":[[-5.62413E-2,5.15466308E1],[-5.62459E-2,5.15466862E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":881103497,"cost":3,"distance":4},"geometry":{"type":"LineString","coordinates":[[-5.62459E-2,5.15466862E1],[-5.62474E-2,5.15467056E1],[-5.62483E-2,5.15467228E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":881103497,"cost":6,"distance":7},"geometry":{"type":"LineString","coordinates":[[-5.62483E-2,5.15467228E1],[-5.62524E-2,5.15467815E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1079378708,"cost":5,"distance":6},"geometry":{"type":"LineString","coordinates":[[-5.62524E-2,5.15467815E1],[-5.62531E-2,5.15468061E1],[-5.62538E-2,5.15468316E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1079378709,"cost":3,"distance":4},"geometry":{"type":"LineString","coordinates":[[-5.62538E-2,5.15468316E1],[-5.62546E-2,5.15468484E1],[-5.62621E-2,5.15468663E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":881103498,"cost":3,"distance":4},"geometry":{"type":"LineString","coordinates":[[-5.62621E-2,5.15468663E1],[-5.62628E-2,5.15469052E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1063762150,"cost":1,"distance":1},"geometry":{"type":"LineString","coordinates":[[-5.62628E-2,5.15469052E1],[-5.6263E-2,5.15469163E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1063762149,"cost":2,"distance":3},"geometry":{"type":"LineString","coordinates":[[-5.6263E-2,5.15469163E1],[-5.62635E-2,5.1546946E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":881103496,"cost":7,"distance":9},"geometry":{"type":"LineString","coordinates":[[-5.62635E-2,5.1546946E1],[-5.62657E-2,5.15470109E1],[-5.62908E-2,5.15470107E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":41,"distance":49},"geometry":{"type":"LineString","coordinates":[[-5.62908E-2,5.15470107E1],[-5.62911E-2,5.1547031E1],[-5.66186E-2,5.15470287E1],[-5.69331E-2,5.15470277E1],[-5.694401949978242E-2,5.1547027764967275E1]]}}]})",
      extract_and_route("test/london-corridor.osm.pbf", from, to));
}
