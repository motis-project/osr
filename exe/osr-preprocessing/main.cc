#include <iostream>

#include "boost/filesystem.hpp"

#include <filesystem>

#include "conf/options_parser.h"
#include "fmt/ostream.h"
#include "osr/preprocessing/contraction_hierarchies/node_order_strategy.h"
#include "osr/preprocessing/contraction_hierarchies/preprocessor.h"

#include "utl/progress_tracker.h"

using namespace osr;
using namespace osr::ch;
namespace fs = std::filesystem;

struct config : public conf::configuration {
  config(fs::path in) : configuration{"Options"}, in_{std::move(in)} {
    param(in_, "in,i", "input directory for extracted Data");
    param(out_, "out,o", "output directory for preprocessed Data");
    param(order_, "order", "Order which to use (default rand, predefined = pre)");
    param(seed_, "seed", "Seed for any random order");
    param(stall_, "stall", "Percentage of the nodes that should be contracted [0-100] (default: 95)");
  }

  fs::path in_;
  fs::path out_;
  std::string order_{"importance"};
  int seed_{-1};
  size_t stall_{95};
};

std::unique_ptr<OrderStrategy> get_order_strategy(const config& c) {
  if (c.order_ == "rand") {
    return std::make_unique<RandomOrderStrategy>(c.seed_);
  }
  return std::make_unique<node_importance_order_strategy>(c.seed_); // importance
}

int main(int argc, const char** argv) {
  auto c = config{"osr-germany"};

  conf::options_parser parser({&c});
  parser.read_command_line_args(argc, argv);

  parser.read_configuration_file();

  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::is_directory(c.in_)) {
    fmt::println("directory not found: {}", c.in_);
    return 1;
  }

  if (fs::exists(c.out_)) {
    fmt::println("Removing existing contraction hierarchy directory: {}", c.out_);
    fs::remove_all(c.out_);
  }

  fmt::println("Creating contraction hierarchy directory: {}", c.out_);
  fs::create_directories(c.out_);
  for (auto const& file : fs::directory_iterator(c.in_)) {
    fs::copy(file.path(), c.out_ / file.path().filename(), fs::copy_options::recursive);
  }

  utl::activate_progress_tracker("contraction-hierarchy");
  auto const silencer = utl::global_progress_bars{false};

  auto order = get_order_strategy(c);

  process_ch(c.in_,c.out_, order, c.stall_);

  return 0;
}