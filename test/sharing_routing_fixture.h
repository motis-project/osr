#pragma once

#include <filesystem>
#include <memory>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace osr::test {

struct sharing_routing_fixture : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = std::filesystem::temp_directory_path() / "osr-sharing-routing-test";
    auto ec = std::error_code{};
    std::filesystem::remove_all(dir_, ec);
    std::filesystem::create_directories(dir_, ec);
    extract(false, test::osm_to_pbf("test/sharing-routing.osm"), dir_, {});
    ways_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    lookup_ =
        std::make_unique<lookup>(*ways_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    lookup_.reset();
    ways_.reset();
    auto ec = std::error_code{};
    std::filesystem::remove_all(dir_, ec);
  }

  static inline std::filesystem::path dir_{};
  static inline std::unique_ptr<ways> ways_{};
  static inline std::unique_ptr<lookup> lookup_{};
};

}  // namespace osr::test
