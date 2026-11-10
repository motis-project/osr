#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <filesystem>

#include "cista/mmap.h"

#include "osr/elevation_storage.h"
#include "osr/extract/extract.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(elevation, flat_test) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false,
          osr::test::osm_to_pbf("test/da_hbf_elevation.osm").generic_string(),
          p, "test/elevation_test_data", 1, 0.1);

  auto const elevation = elevation_storage::try_open(p);
  auto w = ways{p, cista::mmap::protection::READ};
  auto const way_idx = w.find_way(osm_way_idx_t{4208830}).value();

  ASSERT_EQ(w.r_->way_nodes_[way_idx].size() - 1U,
            elevation->offsets_[way_idx].size());

  for (auto i = 0U; i < w.r_->way_nodes_[way_idx].size() - 1U; ++i) {
    auto const res = elevation->get_elevations(way_idx, i);

    // sampled from geoportal hessen (https://www.geoportal.hessen.de/)
    auto const groundTruth = 132;
    auto const kTolerance = 1;
    auto const min =
        preprocessing::elevation::elevation_meters_t{groundTruth - kTolerance};
    auto const max =
        preprocessing::elevation::elevation_meters_t{groundTruth + kTolerance};
    ASSERT_EQ(1, res.n_);
    EXPECT_THAT(*res.first_,
                ::testing::AllOf(::testing::Gt(min), ::testing::Lt(max)));
    ASSERT_EQ(elevation_monotonic_t{0}, res.down_);
    ASSERT_EQ(elevation_monotonic_t{0}, res.up_);
  }
}

TEST(elevation, up_down_test) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false,
          osr::test::osm_to_pbf("test/taunus_elevation.osm").generic_string(),
          p, "test/elevation_test_data", 0, 0.0);

  auto const elevation = elevation_storage::try_open(p);
  auto w = ways{p, cista::mmap::protection::READ};
  auto const start_osm = osm_node_idx_t{300146683};
  auto const end_osm = osm_node_idx_t{300146687};
  auto const from_idx = w.find_node_idx(start_osm);
  auto const to_idx = w.find_node_idx(end_osm);
  auto const way_idx = w.find_way(osm_way_idx_t{43798200}).value();

  auto const kTolerance = 1;
  // sampled from geoportal hessen (https://www.geoportal.hessen.de/)
  auto const kGroundTruthMin = 416;
  auto const kGroundTruthMax = 424;

  auto i = 0U;
  auto up = elevation_monotonic_t{0};
  auto down = elevation_monotonic_t{0};
  for (auto const [from, to] : utl::pairwise(w.r_->way_nodes_[way_idx])) {
    if (from < from_idx || to > to_idx) {
      break;
    }

    auto const res = elevation->get_elevations(way_idx, i);

    up += res.up_.v_;
    down += res.down_.v_;

    for (int j = 0U; j < res.n_; ++j) {
      EXPECT_THAT(
          *std::next(res.first_, j),
          ::testing::AllOf(::testing::Gt(kGroundTruthMin - kTolerance),
                           ::testing::Lt(kGroundTruthMax + kTolerance)));
    }
  }

  EXPECT_THAT(up, ::testing::Lt(elevation_monotonic_t{12 + kTolerance * 2}));
  EXPECT_THAT(down, ::testing::Lt(elevation_monotonic_t{4 + kTolerance * 2}));
}
