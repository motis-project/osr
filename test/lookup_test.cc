#include <algorithm>
#include <filesystem>
#include <iterator>
#include <memory>

#include "gtest/gtest.h"

#include "cista/mmap.h"

#include "utl/verify.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

struct lookup_test : public ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-lookup-test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::osm_to_pbf("test/sharing-routing.osm"), dir_, {});
    ways_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    lookup_ =
        std::make_unique<lookup>(*ways_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    lookup_.reset();
    ways_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static inline fs::path dir_{};
  static inline std::unique_ptr<ways> ways_{};
  static inline std::unique_ptr<lookup> lookup_{};
};

way_idx_t find_osm_way(ways const& w, std::int64_t const osm_way) {
  auto const it = std::find(begin(w.way_osm_idx_), end(w.way_osm_idx_),
                            to_osm_way_idx(osm_way));
  utl::verify(it != end(w.way_osm_idx_) && *it == to_osm_way_idx(osm_way),
              "OSM way {} not found", osm_way);
  return way_idx_t{static_cast<way_idx_t::value_t>(
      std::distance(begin(w.way_osm_idx_), it))};
}

bool matches_way(match_result::view const& m, way_idx_t const way) {
  for (auto i = std::size_t{0U}; i != m.size(); ++i) {
    if (m.way_[i] == way) {
      return true;
    }
  }
  return false;
}

TEST_F(lookup_test, unlevelled_foot_way_is_only_ground_level_fallback) {
  using foot_t = foot<false, elevator_tracking>;

  auto const params = foot_t::parameters{};
  auto const pos = geo::latlng{49.000000, 8.001500};

  auto ground_matches = match_result{};
  lookup_->match<foot_t>(params, location{pos, level_t{0.F}}, false,
                         direction::kForward, 25.0, nullptr, false,
                         ground_matches);
  ASSERT_EQ(1U, ground_matches.size());
  EXPECT_FALSE(ground_matches[match_idx_t{0U}].empty());

  auto level_one_matches = match_result{};
  lookup_->match<foot_t>(params, location{pos, level_t{1.F}}, false,
                         direction::kForward, 25.0, nullptr, false,
                         level_one_matches);
  ASSERT_EQ(1U, level_one_matches.size());
  EXPECT_TRUE(level_one_matches[match_idx_t{0U}].empty());
}

// OSM way 100 carries no `level` tag, so at an explicit level 1 the foot half
// of a sharing profile resolves to nothing. The vehicle half ignores levels,
// so whether the candidate survives depends entirely on the exact return flag.
TEST_F(lookup_test, exact_return_decides_vehicle_only_match) {
  auto const params = bike_sharing::parameters{};
  auto const query = location{geo::latlng{49.000000, 8.001500}, level_t{1.F}};
  auto const way = find_osm_way(*ways_, 100);

  auto without_exact_return = match_result{};
  lookup_->match<bike_sharing>(params, query, true, direction::kForward, 25.0,
                               nullptr, false, without_exact_return);
  EXPECT_FALSE(matches_way(without_exact_return[match_idx_t{0U}], way));

  auto with_exact_return = match_result{};
  lookup_->match<bike_sharing>(params, query, true, direction::kForward, 25.0,
                               nullptr, true, with_exact_return);
  EXPECT_TRUE(matches_way(with_exact_return[match_idx_t{0U}], way));
}

TEST_F(lookup_test, exact_return_matches_car_only_way) {
  auto const params = car_sharing<>::parameters{};
  auto const query = location{geo::latlng{49.070000, 8.001000}};
  auto const way = find_osm_way(*ways_, 1200);

  auto without_exact_return = match_result{};
  lookup_->match<car_sharing<>>(params, query, true, direction::kForward, 25.0,
                                nullptr, false, without_exact_return);
  EXPECT_FALSE(matches_way(without_exact_return[match_idx_t{0U}], way));

  auto with_exact_return = match_result{};
  lookup_->match<car_sharing<>>(params, query, true, direction::kForward, 25.0,
                                nullptr, true, with_exact_return);
  EXPECT_TRUE(matches_way(with_exact_return[match_idx_t{0U}], way));
}

}  // namespace
}  // namespace osr
