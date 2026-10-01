#include <filesystem>
#include <memory>

#include "gtest/gtest.h"

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
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

TEST_F(lookup_test, unlevelled_foot_way_is_only_ground_level_fallback) {
  using foot_t = foot<false, elevator_tracking>;

  auto const params = foot_t::parameters{};
  auto const pos = geo::latlng{49.000000, 8.001500};

  auto ground_matches = match_result{};
  lookup_->match<foot_t>(params, location{pos, level_t{0.F}}, false,
                         direction::kForward, 25.0, nullptr, ground_matches);
  ASSERT_EQ(1U, ground_matches.size());
  EXPECT_FALSE(ground_matches[match_idx_t{0U}].empty());

  auto level_one_matches = match_result{};
  lookup_->match<foot_t>(params, location{pos, level_t{1.F}}, false,
                         direction::kForward, 25.0, nullptr, level_one_matches);
  ASSERT_EQ(1U, level_one_matches.size());
  EXPECT_TRUE(level_one_matches[match_idx_t{0U}].empty());
}

}  // namespace
}  // namespace osr
