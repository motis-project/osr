#include <iostream>

#include "fmt/core.h"
#include "fmt/std.h"

#include "utl/progress_tracker.h"

#include "osr/extract.h"
#include "osr/to_geojson.h"
#include "osr/ways.h"

using namespace osr;
namespace fs = std::filesystem;

int main(int ac, char const** av) {
  if (ac < 2) {
    fmt::println("usage: osr-serve [PATH]");
    return 1;
  }

  auto const path = fs::path{av[1]};

  if (!fs::is_regular_file(path)) {
    fmt::println("input file {} not found", path);
    return 1;
  }

  auto w = ways{path, cista::mmap::protection::READ};
  fmt::println("{}", to_geojson(w));
}