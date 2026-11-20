#include <cstdint>
#include <iostream>

#include "fmt/core.h"

#include "conf/options_parser.h"

#include "osr/elevation_storage.h"
#include "utl/progress_tracker.h"

#include "osr/extract/extract.h"

using namespace osr;
using namespace boost::program_options;
namespace fs = std::filesystem;

struct config : public conf::configuration {
  config(std::filesystem::path in, std::filesystem::path out)
      : configuration{"Options"}, in_{std::move(in)}, out_{std::move(out)} {
    param(in_, "in,i", "OpenStreetMap .osm.pbf input path");
    param(out_, "out,o", "output directory");
    param(elevation_data_, "elevation_data,e", "directory with elevation data");
    param(with_platforms_, "with_platforms,p", "extract platform info");
    param(min_elevation_, "min_elevation",
          "minimum distance to consider not flat");
    param(min_incline_, "min_incline", "minimum incline to consider not flat");
    param(max_incline_, "max_incline",
          "maximum elevation difference (relative to sampling resolution)");
  }

  std::filesystem::path in_, out_, elevation_data_;
  bool with_platforms_{false};
  std::uint32_t min_elevation_{1};
  double min_incline_{0.1};
  double max_incline_{0.45};
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

  extract(c.with_platforms_, c.in_, c.out_, c.elevation_data_,
          preprocessing::elevation::elevation_params{
              .min_elevation_ = c.min_elevation_,
              .min_incline_ = c.min_incline_,
              .max_incline_ = c.max_incline_});
}