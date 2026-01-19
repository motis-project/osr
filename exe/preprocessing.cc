#include <iostream>

#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/progress_tracker.h"

#include "osr/preprocessing/contraction_hierarchy/contraction.h"
#include "osr/preprocessing/contraction_hierarchy/node_order.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/types.h"

using namespace osr;
using namespace boost::program_options;
namespace fs = std::filesystem;

struct config : public conf::configuration {
  config(std::filesystem::path dir,
         size_t const seed,
         cost_t const detour_limit,
         std::string const& method)
      : configuration{"Options"},
        dir_{std::move(dir)},
        seed_(seed),
        detour_limit_(detour_limit),
        ordering_method_(method) {
    param(dir_, "folder,f",
          "directory of extracted data and to save shortcuts to");
    param(seed_, "seed,s",
          "(optional) the seed to for random node ordering (default: 1337)");
    param(detour_limit_, "detour,d",
          "(optional) max feasible cost to detour turning restrictions "
          "(default: 10 min)");
    param(ordering_method_, "method,m",
          "(optional) the method to order the nodes: random, real (orders by "
          "OSM metadata) (default: real)");
  }

  std::filesystem::path dir_;
  size_t seed_;
  cost_t detour_limit_;
  std::string ordering_method_;
};

int main(int ac, char const** av) {
  auto c = config{"", 1337, 10 * 60U, "real"};

  conf::options_parser parser({&c});
  parser.read_command_line_args(ac, av);

  parser.read_configuration_file();

  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::is_directory(c.dir_)) {
    fmt::println("input extract directory {} not found", c.dir_);
    return 1;
  }

  utl::activate_progress_tracker("osr-preprocessing");
  auto const silencer = utl::global_progress_bars{false};

  preprocess_ch<car>(
      c.dir_, search_profile::kCar,
      std::get<car::parameters>(get_parameters(search_profile::kCar)), c.seed_,
      c.detour_limit_, node_order::string2method(c.ordering_method_));
}