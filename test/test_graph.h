#pragma once

#include <filesystem>
#include <memory>
#include <string_view>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/ways.h"

namespace osr::test {

// Extracts `osm` into a fresh directory `name` below the temp directory and
// opens it.
struct graph {
  graph(std::filesystem::path const& osm, std::string_view const name)
      : dir_{std::filesystem::temp_directory_path() / name} {
    auto ec = std::error_code{};
    std::filesystem::remove_all(dir_, ec);
    std::filesystem::create_directories(dir_, ec);
    extract(false, osm, dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  std::filesystem::path dir_;
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
};

}  // namespace osr::test
