#include <chrono>
#include <filesystem>
#include <mutex>
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
    param(n_queries_, ",n", "Number of queries");
    param(max_dist_, "radius,r", "Radius");
  }

  fs::path data_dir_{"osr"};
  unsigned n_queries_{50};
  unsigned max_dist_{1200};
  unsigned threads_{std::thread::hardware_concurrency()};
};

struct benchmark_result {
  friend std::ostream& operator<<(std::ostream& out,
                                  benchmark_result const& br) {
    using double_milliseconds_t =
        std::chrono::duration<double, std::ratio<1, 1000>>;
    out << "(duration: " << std::fixed << std::setprecision(3) << std::setw(10)
        << std::chrono::duration_cast<double_milliseconds_t>(br.duration_)
        << ")";
    return out;
  }

  std::chrono::microseconds duration_;
};

// needs sorted vector
benchmark_result quantile(std::vector<benchmark_result> const& v, double q) {
  q = std::clamp(q, 0.0, 1.0);
  if (q == 1.0) {
    return v.back();
  }
  return v[static_cast<std::size_t>(v.size() * q)];
}

void print_result(std::vector<benchmark_result> const& var,
                  std::string const& var_name) {
  std::cout << "\n--- " << var_name << " --- (n = " << var.size() << ")"
            << "\n  10%: " << quantile(var, 0.1)
            << "\n  20%: " << quantile(var, 0.2)
            << "\n  30%: " << quantile(var, 0.3)
            << "\n  40%: " << quantile(var, 0.4)
            << "\n  50%: " << quantile(var, 0.5)
            << "\n  60%: " << quantile(var, 0.6)
            << "\n  70%: " << quantile(var, 0.7)
            << "\n  80%: " << quantile(var, 0.8)
            << "\n  90%: " << quantile(var, 0.9)
            << "\n  99%: " << quantile(var, 0.99)
            << "\n99.9%: " << quantile(var, 0.999) << "\n  max: " << var.back()
            << "\n-----------------------------\n";
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

  auto threads = std::vector<std::thread>(std::max(1U, opt.threads_));
  auto results = std::vector<benchmark_result>{};
  results.reserve(opt.n_queries_);
  auto i = std::atomic_size_t{0U};
  auto m = std::mutex{};
  for (auto& t : threads) {
    t = std::thread([&]() {
      auto d = dijkstra<car>{};
      auto h = cista::BASE_HASH;
      auto n = 0U;
      while (i.fetch_add(1U) < opt.n_queries_) {
        auto const start_time = std::chrono::steady_clock::now();
        auto const start =
            node_idx_t{cista::hash_combine(h, ++n, i.load()) % w.n_nodes()};
        d.reset(opt.max_dist_);
        d.add_start(w,
                    car::label{car::node{start, 0, direction::kForward}, 0U});
        d.add_start(w,
                    car::label{car::node{start, 0, direction::kBackward}, 0U});
        d.run<direction::kForward, false>(w, *w.r_, opt.max_dist_, nullptr,
                                          nullptr);
        auto const end_time = std::chrono::steady_clock::now();
        {
          auto const guard = std::lock_guard{m};
          results.emplace_back(
              std::chrono::duration_cast<decltype(benchmark_result::duration_)>(
                  end_time - start_time));
        }
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  std::ranges::sort(results, std::less<>{},
                    [](benchmark_result const& res) { return res.duration_; });

  print_result(results, "duration");
}
