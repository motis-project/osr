#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include "boost/asio/executor_work_guard.hpp"
#include "boost/asio/io_context.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/timer.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/routing/with_profile.h"
#include "osr/types.h"
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
    param(from_coords_, "matching,m", "Include node matching to coords");
    param(speed_, "speed,s", "Walking speed");
  }

  fs::path data_dir_{"osr"};
  unsigned n_queries_{50};
  unsigned max_dist_{32768};
  bool from_coords_{false};
  unsigned threads_{std::thread::hardware_concurrency()};
  float speed_{1.2F};
};

struct benchmark_result {
  friend std::ostream& operator<<(std::ostream& out,
                                  benchmark_result const& br) {
    using double_milliseconds_t =
        std::chrono::duration<double, std::ratio<1, 1000>>;
    out << "(duration: " << std::fixed << std::setprecision(3) << std::setw(10)
        << std::chrono::duration_cast<double_milliseconds_t>(br.duration_)
               .count()
        << "ms)";
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
                  std::string const& profile) {
  auto const avg = benchmark_result{
      std::accumulate(var.begin(), var.end(), std::chrono::microseconds{0U},
                      [](auto&& sum, auto const& res) {
                        return std::move(sum) + res.duration_;
                      }) /
      var.size()};
  std::cout << "\n--- profile: " << profile << " --- (n = " << var.size() << ")"
            << "\n  max: " << var.back() << "\n  avg: " << avg << "\n----------"
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
            << "\n99.9%: " << quantile(var, 0.999)
            << "\n-----------------------------\n";
}

template <Profile P>
void set_start(dijkstra<P>& d, ways const& w, node_idx_t const start) {
  d.add_start(w, typename P::label{typename P::node{start}, 0U});
}

template <>
void set_start<car>(dijkstra<car>& d, ways const& w, node_idx_t const start) {
  d.add_start(w, car::label{car::node{start, 0, direction::kForward}, 0U});
  d.add_start(w, car::label{car::node{start, 0, direction::kBackward}, 0U});
};

template <Profile P>
void set_start(typename P::parameters const& params,
               bidirectional<P>& b,
               ways const& w,
               node_idx_t const start) {
  b.add_start(params, w, typename P::label{typename P::node{start}, 0U},
              nullptr);
}

template <>
void set_start<car>(car::parameters const& params,
                    bidirectional<car>& b,
                    ways const& w,
                    node_idx_t const start) {
  b.add_start(params, w,
              car::label{car::node{start, 0, direction::kForward}, 0U},
              nullptr);
  b.add_start(params, w,
              car::label{car::node{start, 0, direction::kBackward}, 0U},
              nullptr);
};

template <Profile P>
std::vector<typename P::label> set_end(typename P::parameters const& params,
                                       bidirectional<P>& b,
                                       ways const& w,
                                       node_idx_t const end) {
  auto const l = typename P::label{typename P::node{end}, 0U};
  b.add_end(params, w, l, nullptr);
  return {l};
}

template <>
std::vector<typename car::label> set_end<car>(car::parameters const& params,
                                              bidirectional<car>& b,
                                              ways const& w,
                                              node_idx_t const end) {
  std::vector<typename car::label> ends;
  auto const ways = w.r_->node_ways_[end];

  for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
    auto const l1 = car::label{car::node{end, i, direction::kForward}, 0U};
    auto const l2 = car::label{car::node{end, i, direction::kBackward}, 0U};
    b.add_end(params, w, l1, nullptr);
    b.add_end(params, w, l2, nullptr);
    ends.push_back(l1);
    ends.push_back(l2);
  }
  return ends;
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
  auto const l = osr::lookup{w, opt.data_dir_, cista::mmap::protection::READ};
  auto const elevations = elevation_storage::try_open(opt.data_dir_);

  auto threads = std::vector<std::thread>(std::max(1U, opt.threads_));
  auto results = std::vector<benchmark_result>{};
  results.reserve(opt.n_queries_);

  auto const run_benchmark = [&]<Profile P>(
                                 typename P::parameters const& params,
                                 search_profile const profile,
                                 const char* profile_label) {
    results.clear();
    auto i = std::atomic_size_t{0U};
    auto m = std::mutex{};
    for (auto& t : threads) {
      t = std::thread([&]() {
        auto d = dijkstra<P>{};
        auto b = bidirectional<P>{};
        auto h = cista::BASE_HASH;
        auto n = 0U;
        while (i.fetch_add(1U) < opt.n_queries_ - 1) {
          auto const start =
              node_idx_t{cista::hash_combine(h, ++n, i.load()) % w.n_nodes()};
          auto const end =
              node_idx_t{cista::hash_combine(h, ++n, i.load()) % w.n_nodes()};

          if (w.r_->node_ways_[start].empty() ||
              w.r_->node_ways_[end].empty()) {
            continue;
          }
          auto const start_loc =
              location{w.get_node_pos(start).as_latlng(), level_t{0.F}};
          auto const end_loc =
              location{w.get_node_pos(end).as_latlng(), level_t{0.F}};
          std::cout << location{w.get_node_pos(start).as_latlng(), level_t{0.F}}
                    << " fromto "
                    << location{w.get_node_pos(end).as_latlng(), level_t{0.F}}
                    << std::endl;
          if (opt.from_coords_) {
            auto const start_time = std::chrono::steady_clock::now();
            auto const d_res =
                route(params, w, l, profile, start_loc, end_loc, opt.max_dist_,
                      direction::kForward, 250, nullptr, nullptr, nullptr,
                      routing_algorithm::kDijkstra);
            auto const middle_time = std::chrono::steady_clock::now();
            auto const b_res =
                route(params, w, l, profile, start_loc, end_loc, opt.max_dist_,
                      direction::kForward, 250, nullptr, nullptr, nullptr,
                      routing_algorithm::kAStarBi);
            auto const end_time = std::chrono::steady_clock::now();

            /*std::cout << "took "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             middle_time - start_time)
                      << " vs "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             end_time - middle_time)
                      << std::endl;*/

            utl::verify(!d_res.has_value() && !b_res.has_value() ||
                            d_res.has_value() && b_res.has_value() &&
                                d_res->cost_ == b_res->cost_,
                        "not equal {} {}", d_res->cost_, b_res->cost_);
            {
              auto const guard = std::lock_guard{m};
              results.emplace_back(benchmark_result{std::chrono::duration_cast<
                  decltype(benchmark_result::duration_)>(end_time -
                                                         middle_time)});
            }
          } else {
            if (w.r_->way_component_[w.r_->node_ways_[start][0]] !=
                w.r_->way_component_[w.r_->node_ways_[end][0]]) {
              std::cout << "skipping" << std::endl;
              continue;
            }
            d.reset(opt.max_dist_);
            b.reset(params, opt.max_dist_, start_loc, end_loc);
            set_start<P>(d, w, start);
            set_start<P>(params, b, w, start);

            auto const ends = set_end<P>(params, b, w, end);
            auto const start_time = std::chrono::steady_clock::now();
            d.template run<direction::kForward, false>(
                params, w, *w.r_, opt.max_dist_, nullptr, nullptr,
                elevations.get());
            auto const middle_time = std::chrono::steady_clock::now();
            b.template run<direction::kForward, false>(
                params, w, *w.r_, opt.max_dist_, nullptr, nullptr,
                elevations.get());
            auto const end_time = std::chrono::steady_clock::now();
            /*std::cout << "took "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             middle_time - start_time)
                      << " vs "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             end_time - middle_time)
                      << std::endl;*/
            auto const b_res =
                b.get_cost_to_mp(b.meet_point_1_, b.meet_point_2_);
            if (!utl::any_of(ends, [&](auto&& e) {
                  auto const it = d.cost_.find(e.get_node().get_key());
                  auto const d_res = d.get_cost(e.get_node());
                  std::cout << " " << d_res << " vs " << b_res << std::endl;
                  return b_res == d_res;
                })) {

              std::cout << "not equal" << std::endl;
            }
            {
              auto const guard = std::lock_guard{m};
              results.emplace_back(benchmark_result{std::chrono::duration_cast<
                  decltype(benchmark_result::duration_)>(end_time -
                                                         start_time)});
            }
          }
        }
      });
    }

    for (auto& t : threads) {
      t.join();
    }

    std::ranges::sort(results, std::less<>{}, [](benchmark_result const& res) {
      return res.duration_;
    });

    print_result(results, profile_label);
  };

  auto const run_speed_benchmark = [&](search_profile const profile,
                                       std::string_view label,
                                       float const speed = 0.0F) {
    with_profile(profile, [&]<Profile P>(P&&) {
      auto const params = [&]() {
        if constexpr (requires { typename P::parameters::speed_; }) {
          return speed > 0.0F ? typename P::parameters{.speed_ = opt.speed_}
                              : typename P::parameters{};
        } else {
          return typename P::parameters{};
        }
      }();
      run_benchmark.template operator()<P>(params, profile, label.data());
    });
  };
  auto const walk_speed = opt.speed_;
  auto const bike_speed = 3.5F * walk_speed;
  run_speed_benchmark(search_profile::kFoot, "foot", walk_speed);
  run_speed_benchmark(search_profile::kCar, "car");
  run_speed_benchmark(search_profile::kBike, "bike", bike_speed);
  run_speed_benchmark(search_profile::kBikeElevationLow,
                      "bike (low elevation costs)", bike_speed);
  run_speed_benchmark(search_profile::kBikeElevationHigh,
                      "bike (high elevation costs)", bike_speed);
}
