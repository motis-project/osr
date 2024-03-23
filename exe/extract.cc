#include <iostream>

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/progress_tracker.h"

#include "osr/extract.h"

using namespace osr;
using namespace boost::program_options;
namespace fs = std::filesystem;

struct config : public conf::configuration {
  config(std::filesystem::path in, std::filesystem::path const& out)
      : configuration{"Options"}, in_{std::move(in)}, out_{out} {
    param(in_, "in,i", "OpenStreetMap .osm.pbf input path");
    param(out_, "out,o", "output directory");
  }

  std::filesystem::path in_, out_;
};

int main(int ac, char const** av) {
  auto c = config{"./planet-latest.osm.pbf", "./osr"};

  conf::options_parser parser({&c});
  parser.read_command_line_args(ac, av);

  parser.read_configuration_file();

  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::is_regular_file(c.in_)) {
    fmt::println("input file {} not found", c.in_);
    return 1;
  }

  utl::activate_progress_tracker("osr");
  auto const silencer = utl::global_progress_bars{false};

  extract(c.in_, c.out_);
}