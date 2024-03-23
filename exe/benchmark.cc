#include <filesystem>
#include <thread>
#include <vector>

#include "boost/asio/executor_work_guard.hpp"
#include "boost/asio/io_context.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "osr/lookup.h"
#include "osr/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

class settings : public conf::configuration {
public:
  explicit settings() : configuration("Options") {
    param(data_dir_, "data,d", "Data directory");
    param(threads_, "threads,t", "Number of routing threads");
    param(n_queries_, "n", "Number of queries");
  }

  fs::path data_dir_{"osr"};
  unsigned n_queries_{100};
  unsigned max_dist_{900U};
  unsigned threads_{std::thread::hardware_concurrency()};
};

node_idx_t random_node() { return node_idx_t{0U}; }

int main(int argc, char const* argv[]) {
  auto opt = settings{};
  auto parser = conf::options_parser({&opt});
  parser.read_command_line_args(argc, argv);

  if (parser.help()) {
    parser.print_help(std::cout);
    return 0;
  } else if (parser.version()) {
    return 0;
  }

  parser.read_configuration_file();
  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::is_directory(opt.data_dir_)) {
    fmt::println("directory not found: {}", opt.data_dir_);
    return 1;
  }

  auto const w = ways{opt.data_dir_, cista::mmap::protection::READ};

  auto threads = std::vector<std::thread>(std::max(1U, opt.threads_));
  auto i = std::atomic_size_t{0U};
  for (auto& t : threads) {
    t = std::thread([&]() {
      auto s = routing_state{};
      auto h = cista::BASE_HASH;
      auto n = 0U;
      while (i.fetch_add(1U) < opt.n_queries_) {
        auto const start =
            node_idx_t{cista::hash_combine(h, ++n, i.load()) % w.n_nodes()};
        s.dijkstra_state_.reset(w, opt.max_dist_);
        s.dijkstra_state_.add_start(start, 0U);
        dijkstra(w, s.dijkstra_state_, opt.max_dist_, pedestrian{});
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }
}
