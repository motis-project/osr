#include "gtest/gtest.h"

#include <filesystem>
#include <memory>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

namespace {

// Berlin, Miraustr. - a query coordinate and the tram stop ~380m away. MOTIS
// reconstructs an access leg between these two with a budget of
// `leg duration + 5min`, and gets no path back although the walk is 315s.
constexpr auto const kFrom = osr::location{52.5863794, 13.3109247, osr::kNoLevel};
constexpr auto const kTo = osr::location{52.5840800, 13.3096810, osr::kNoLevel};

// What MOTIS passes for a 360s access leg (360 + 5min).
constexpr auto const kReconstructionBudget = osr::cost_t{660U};

class graph {
public:
  graph() {
    dir_ = fs::temp_directory_path() / "osr_cost_budget_test";

    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);

    osr::extract(false, "test/miraustr.osm.pbf", dir_, {});
    w_ = std::make_unique<osr::ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<osr::lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  std::optional<osr::path> route(osr::cost_t const max,
                                 osr::routing_algorithm const algo) const {
    return osr::route(osr::foot<false, osr::elevator_tracking>::parameters{},
                      *w_, *l_, osr::search_profile::kFoot, kFrom, kTo, max,
                      osr::direction::kForward, /*max_match_distance=*/25.0,
                      nullptr, nullptr, nullptr, algo);
  }

private:
  fs::path dir_;
  std::unique_ptr<osr::ways> w_;
  std::unique_ptr<osr::lookup> l_;
};

}  // namespace

// The walk exists and is far cheaper than the budget it is searched with, so
// both algorithms have to find it. Bidirectional A* returns nothing until the
// budget is raised well above the actual path cost.
TEST(cost_budget, short_walk_within_budget) {
  auto const g = graph{};

  auto const dijkstra = g.route(kReconstructionBudget,
                                osr::routing_algorithm::kDijkstra);
  ASSERT_TRUE(dijkstra.has_value());
  EXPECT_LT(dijkstra->cost_, kReconstructionBudget);

  auto const astar_bi = g.route(kReconstructionBudget,
                                osr::routing_algorithm::kAStarBi);
  ASSERT_TRUE(astar_bi.has_value())
      << "bidirectional A* found no path within " << kReconstructionBudget
      << ", but dijkstra routes it at cost " << dijkstra->cost_;
  EXPECT_EQ(dijkstra->cost_, astar_bi->cost_);
}

// The same query with a budget far above the path cost. This one passes today:
// the search only fails while the budget is tight, which is what makes it a
// budget handling bug rather than a matching or connectivity problem.
TEST(cost_budget, short_walk_with_generous_budget) {
  auto const g = graph{};

  auto const dijkstra = g.route(3600U, osr::routing_algorithm::kDijkstra);
  auto const astar_bi = g.route(3600U, osr::routing_algorithm::kAStarBi);

  ASSERT_TRUE(dijkstra.has_value());
  ASSERT_TRUE(astar_bi.has_value());
  EXPECT_EQ(dijkstra->cost_, astar_bi->cost_);
}
