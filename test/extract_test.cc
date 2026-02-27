#include "gtest/gtest.h"

#include <filesystem>

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, string_cache) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/map.osm", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  // 140186757 and 519430215 have the same name=Pankratiusstraße
  const auto pankratius1 = w.find_way(osm_way_idx_t{140186757});
  const auto pankratius2 = w.find_way(osm_way_idx_t{519430215});
  ASSERT_TRUE(pankratius1.has_value());
  ASSERT_TRUE(pankratius2.has_value());

  const auto name_idx1 = w.way_names_[pankratius1.value()];
  const auto name_idx2 = w.way_names_[pankratius2.value()];
  ASSERT_EQ(name_idx1, name_idx2);
}

TEST(extract, bus_only_on_highway) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/luisenplatz-darmstadt.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto const luisenplatz_outer = w.find_way(osm_way_idx_t{53341306});
  ASSERT_TRUE(luisenplatz_outer.has_value());

  auto const& wp = w.r_->way_properties_[luisenplatz_outer.value()];
  ASSERT_FALSE(wp.is_bus_accessible());
  ASSERT_TRUE(wp.is_foot_accessible());
}
