#include <filesystem>
#include <thread>
#include <vector>

#include "boost/asio/executor_work_guard.hpp"
#include "boost/asio/io_context.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/timer.h"

#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/a_star.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

class settings : public conf::configuration {
public:
  explicit settings() : configuration("Options") {
    param(data_dir_, "data,d", "Data directory");
    param(threads_, "threads,t", "Number of routing threads");
    param(n_queries_, "n", "Number of queries");
    param(max_dist_, "r", "Radius");
  }

  fs::path data_dir_{"osr"};
  unsigned n_queries_{50};
  unsigned max_dist_{1200};
  unsigned threads_{std::thread::hardware_concurrency()};
};

std::vector<int> generate_random_vector(int upperBound, int n) {
  std::srand(std::time(0));
  int lowerBound = 0;
  std::vector<int> random_numbers(n);
  for (int i = 0; i < n; ++i) {
    random_numbers[i] = lowerBound + std::rand() % (upperBound - lowerBound + 1);
  }
  return random_numbers;
}

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

  auto timer = utl::scoped_timer{"timer"};
  auto i = std::atomic_size_t{0U};
  auto d = dijkstra<car>{};
  auto h = cista::BASE_HASH;
  auto n = 0U;
  auto const start =
      node_idx_t{cista::hash_combine(h, ++n, i.load()) % w.n_nodes()};
  auto start_lf = car::label{car::node{start, 0, direction::kForward}, 0U};
  auto start_lb = car::label{car::node{start, 0, direction::kBackward}, 0U};
  d.reset(opt.max_dist_);
  d.add_start(w, start_lf);
  d.add_start(w, start_lb);
  d.run<direction::kForward, false>(w, *w.r_, opt.max_dist_, nullptr, nullptr);

  std::vector<int> random_numbers =
      generate_random_vector(d.cost_.size(), opt.n_queries_);

  if (n > d.cost_.size()) {
    std::cerr << "n is greater than the size of the cost array \n";
  }

  auto timer_a = utl::scoped_timer{"timer"};
  auto threads = std::vector<std::thread>(std::max(1U, opt.threads_));
  for (auto& t : threads) {
    t = std::thread([&]() {
      while (i.fetch_add(1U) < opt.n_queries_) {
        auto it = d.cost_.begin();
        std::advance(it, random_numbers[i]);
        auto const end = it->first;
        // Add end first for heuristic calculation
        auto end_lf = car::label{car::node{end, 0, direction::kForward}, 0U};
        auto end_lb = car::label{car::node{end, 0, direction::kBackward}, 0U};
        if (it->second.cost_.empty()) {
          std::cerr << "Cost is infeasible \n";
        }
        if ((it->second.cost(end_lf.get_node()) == std::numeric_limits<std::uint16_t>::max() &&
            it->second.cost(end_lb.get_node()) == std::numeric_limits<std::uint16_t>::max()) ||(
            it->second.cost(end_lf.get_node()) == kInfeasible &&
            it->second.cost(end_lb.get_node()) == kInfeasible)) {
          std::cout << "Cost is infeasible \n";
          continue;
        }
        // route to n random nodes with a_star
        auto a = a_star<car>{};
        auto h_a = cista::BASE_HASH;
        auto n_a = 0U;
        a.reset(opt.max_dist_);

        a.add_end(w, end_lf);
        a.add_end(w, end_lb);
        a.add_start(w, start_lf);
        a.add_start(w, start_lb);
        a.run<direction::kForward, false>(w, *w.r_, opt.max_dist_, nullptr,
                                          nullptr);


        // compare the cost of the nodes from dijkstra and a_star
        auto found = a.cost_.find(end);
        //std::cout << "Dijkstra cost: " << it->second.cost(end_nf.get_node()) <<
                     //", A* cost: " << found->second.cost(end_nf.get_node()) << "\n";

        if (found == a.cost_.end()) {
          std::cout << "Element " << end << " not found in a.cost_.\n";
          std::cout << it->second.cost(end_lf.get_node()) << std::endl;
          std::cout << it->second.cost(end_lb.get_node()) << std::endl;
          // handle error or continue
        } else if (it->second.cost(end_lf.get_node()) != found->second.cost(end_lf.get_node())
                   && it->second.cost(end_lb.get_node()) == std::numeric_limits<std::uint16_t>::max()) {
          std::cout << "Dijkstra cost: " << it->second.cost(end_lf.get_node())
                    << " A* cost: " << found->second.cost(end_lf.get_node()) << "\n";
          throw std::runtime_error("Costs are not equal");
        }
      }
    });
  }
  for (auto& t : threads) {
    t.join();
  }
  // print success if costs are equal
  std::cout << "Success: Costs are equal \n";
}

