#include <filesystem>

#include "gtest/gtest.h"
#include "osr/geojson.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

std::string extract_and_route(std::string_view path,
                              osr::location const& from,
                              osr::location const& to) {
  auto const dir = fs::temp_directory_path() / path;
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const p = osr::route(
      osr::foot<false, osr::elevator_tracking>::parameters{}, w, l,
      osr::search_profile::kFoot, from, {to}, 900, osr::direction::kForward,
      250.0, nullptr, nullptr, nullptr, osr::routing_algorithm::kDijkstra);
  utl::verify(p.has_value(), "{}: from={} to={} -> no route", path,
              fmt::streamed(from), fmt::streamed(to));
  return osr::to_featurecollection(w, *p, false);
}

TEST(routing, island) {
  auto const from = osr::location{49.872715, 8.651534, osr::level_t{0.F}};
  auto const to = osr::location{49.873023, 8.651523, osr::level_t{0.F}};
  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":3,"distance":3},"geometry":{"type":"LineString","coordinates":[[8.651514431787469E0,4.987274386418564E1],[8.6515174E0,4.98727447E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1201551426,"cost":3,"distance":3},"geometry":{"type":"LineString","coordinates":[[8.6515174E0,4.98727447E1],[8.6515563E0,4.98727556E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":22937760,"cost":2,"distance":2},"geometry":{"type":"LineString","coordinates":[[8.6515563E0,4.98727556E1],[8.6515406E0,4.98727706E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":22937760,"cost":10,"distance":10},"geometry":{"type":"LineString","coordinates":[[8.6515406E0,4.98727706E1],[8.6514528E0,4.98728367E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":847710844,"cost":6,"distance":7},"geometry":{"type":"LineString","coordinates":[[8.6514528E0,4.98728367E1],[8.6514754E0,4.98728959E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":23511479,"cost":8,"distance":8},"geometry":{"type":"LineString","coordinates":[[8.6514754E0,4.98728959E1],[8.651539E0,4.98729497E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":23511479,"cost":2,"distance":2},"geometry":{"type":"LineString","coordinates":[[8.651539E0,4.98729497E1],[8.6515596E0,4.98729618E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":8,"distance":9},"geometry":{"type":"LineString","coordinates":[[8.6515596E0,4.98729618E1],[8.651493334251755E0,4.987300334757231E1]]}}]})",
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

TEST(routing, stop_area) {
  auto const from =
      osr::location{48.725296645530705, 2.2612587304760723, osr::level_t{0.F}};
  auto const to =
      osr::location{48.725480463902784, 2.2588322597458728, osr::level_t{0.F}};
  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":13,"distance":13},"geometry":{"type":"LineString","coordinates":[[2.261314500627132E0,4.8725266362834425E1],[2.2613838E0,4.87253219E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1290289280,"cost":6,"distance":6},"geometry":{"type":"LineString","coordinates":[[2.2613838E0,4.87253219E1],[2.2613206E0,4.87253586E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1290289280,"cost":5,"distance":5},"geometry":{"type":"LineString","coordinates":[[2.2613206E0,4.87253586E1],[2.2612721E0,4.87253867E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1290289280,"cost":3,"distance":3},"geometry":{"type":"LineString","coordinates":[[2.2612721E0,4.87253867E1],[2.2612448E0,4.87254022E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":892871341,"cost":6,"distance":7},"geometry":{"type":"LineString","coordinates":[[2.2612448E0,4.87254022E1],[2.2611731E0,4.87254412E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1419149027,"cost":7,"distance":9},"geometry":{"type":"LineString","coordinates":[[2.2611731E0,4.87254412E1],[2.2610764E0,4.87254927E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1419149026,"cost":16,"distance":19},"geometry":{"type":"LineString","coordinates":[[2.2610764E0,4.87254927E1],[2.260814E0,4.87254962E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1359929257,"cost":20,"distance":24},"geometry":{"type":"LineString","coordinates":[[2.260814E0,4.87254962E1],[2.2605996E0,4.87253278E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1429030767,"cost":10,"distance":12},"geometry":{"type":"LineString","coordinates":[[2.2605996E0,4.87253278E1],[2.2604973E0,4.87252419E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1429030769,"cost":2,"distance":2},"geometry":{"type":"LineString","coordinates":[[2.2604973E0,4.87252419E1],[2.2604761E0,4.87252248E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1429030768,"cost":7,"distance":8},"geometry":{"type":"LineString","coordinates":[[2.2604761E0,4.87252248E1],[2.2604036E0,4.87251661E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1359929257,"cost":22,"distance":26},"geometry":{"type":"LineString","coordinates":[[2.2604036E0,4.87251661E1],[2.2602208E0,4.87250223E1],[2.2601787E0,4.87249893E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":222040526,"cost":25,"distance":30},"geometry":{"type":"LineString","coordinates":[[2.2601787E0,4.87249893E1],[2.2600672E0,4.87250507E1],[2.2600272E0,4.87250721E1],[2.2598576E0,4.87251608E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":222040526,"cost":11,"distance":13},"geometry":{"type":"LineString","coordinates":[[2.2598576E0,4.87251608E1],[2.2597231E0,4.87252337E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":222040526,"cost":4,"distance":5},"geometry":{"type":"LineString","coordinates":[[2.2597231E0,4.87252337E1],[2.2596673E0,4.87252638E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":222040526,"cost":93,"distance":112},"geometry":{"type":"LineString","coordinates":[[2.2596673E0,4.87252638E1],[2.2596292E0,4.8725284E1],[2.2584789E0,4.87258964E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":222040526,"cost":13,"distance":16},"geometry":{"type":"LineString","coordinates":[[2.2584789E0,4.87258964E1],[2.2583131E0,4.87259901E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":642263621,"cost":8,"distance":10},"geometry":{"type":"LineString","coordinates":[[2.2583131E0,4.87259901E1],[2.2582647E0,4.87259491E1],[2.2582257E0,4.8725916E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":642578765,"cost":2,"distance":3},"geometry":{"type":"LineString","coordinates":[[2.2582257E0,4.8725916E1],[2.2581937E0,4.87259329E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":642263625,"cost":6,"distance":7},"geometry":{"type":"LineString","coordinates":[[2.2581937E0,4.87259329E1],[2.2581458E0,4.87258947E1],[2.2581625E0,4.87258856E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":642578757,"cost":7,"distance":8},"geometry":{"type":"LineString","coordinates":[[2.2581625E0,4.87258856E1],[2.2582443E0,4.87258411E1]]}},{"type":"Feature","properties":{"level":5E-1,"osm_way_id":642263631,"cost":12,"distance":14},"geometry":{"type":"LineString","coordinates":[[2.2582443E0,4.87258411E1],[2.2581285E0,4.87257422E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1192127892,"cost":1,"distance":1},"geometry":{"type":"LineString","coordinates":[[2.2581285E0,4.87257422E1],[2.2581167E0,4.87257321E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1192127892,"cost":2,"distance":2},"geometry":{"type":"LineString","coordinates":[[2.2581167E0,4.87257321E1],[2.2581393E0,4.87257206E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":0,"cost":55,"distance":66},"geometry":{"type":"LineString","coordinates":[[2.2581393E0,4.87257206E1],[2.2582250818254357E0,4.872579309702946E1]]}}]})",
      extract_and_route("test/station-border.osm.pbf", from, to));
}
