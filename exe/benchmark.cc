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

#include "utl/memory_usage_printer.h"
#include "utl/timer.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include <fstream>

#include "routingkit/contraction_hierarchy.h"

#include "osr/routing/bidirectional.h"
#include "osr/routing/ch.h"
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
    param(mem_usage_, "mem", "Track memory usage");
    param(ch_, "ch", "Run CH materialization test (foot) and exit");
    param(car_, "car", "Use car (turn-expanded) graph instead of foot for --ch");
    param(stops_file_, "stops_file", "CSV of real stop lat,lon for Bucket-CH");
  }

  bool car_{false};
  std::string stops_file_{};
  fs::path data_dir_{"osr"};
  unsigned n_queries_{50};
  unsigned max_dist_{32768};
  bool from_coords_{false};
  unsigned threads_{std::thread::hardware_concurrency()};
  float speed_{1.2F};
  bool mem_usage_{false};
  bool ch_{false};
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

  auto mem_usage = opt.mem_usage_
                       ? std::make_unique<utl::memory_usage_printer>()
                       : std::unique_ptr<utl::memory_usage_printer>{};

  auto const w = ways{opt.data_dir_, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, opt.data_dir_, cista::mmap::protection::READ};
  auto const elevations = elevation_storage::try_open(opt.data_dir_);

  if (opt.ch_) {
    auto const ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    auto const now = std::chrono::steady_clock::now;

    auto t0 = now();
    auto const g = opt.car_ ? materialize_car(w) : materialize_foot(w);
    fmt::println("{} graph: {} vertices, {} edges (materialize {:.0f}ms)",
                 opt.car_ ? "car" : "foot", g.n_vertices(), g.n_edges(),
                 ms(t0, now()));

    // CSR -> tail/head/weight for RoutingKit
    auto const n = static_cast<unsigned>(g.n_vertices());
    // foot: base vertices [0, n_src) are real nodes (osr can match to them).
    // car: vertices are (node,way,dir) turn states, not nodes -> use all as ids.
    auto const n_src =
        opt.car_ ? n : static_cast<unsigned>(w.n_nodes());
    auto tail = std::vector<unsigned>{};
    auto head = std::vector<unsigned>{};
    auto weight = std::vector<unsigned>{};
    tail.reserve(g.n_edges());
    head.reserve(g.n_edges());
    weight.reserve(g.n_edges());
    for (auto u = 0U; u != n; ++u) {
      for (auto e = g.first_out_[u]; e != g.first_out_[u + 1U]; ++e) {
        tail.push_back(u);
        head.push_back(g.edge_target_[e]);
        weight.push_back(g.edge_cost_[e]);
      }
    }

    t0 = now();
    auto const rkch =
        RoutingKit::ContractionHierarchy::build(n, tail, head, weight);
    fmt::println("RoutingKit CH built (contraction {:.0f}ms)", ms(t0, now()));

    auto q = RoutingKit::ContractionHierarchyQuery{rkch};
    auto targets = std::vector<unsigned>(n);
    std::iota(begin(targets), end(targets), 0U);
    q.pin_targets(targets);

    // correctness gate: RoutingKit one-to-all vs Dijkstra (every vertex)
    auto hh = cista::BASE_HASH;
    auto nn = 0U;
    auto mismatches = 0ULL, checked = 0ULL;
    for (auto qi = 0U; qi != 3U; ++qi) {
      auto const s = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
      auto const dd = dijkstra_all(g, s, kInfeasible);
      q.reset_source().add_source(s).run_to_pinned_targets();
      auto const rk = q.get_distances_to_targets();
      for (auto v = 0U; v != n; ++v) {
        ++checked;
        auto const a_inf = dd[v] == kInfeasible;
        auto const b_inf = rk[v] >= RoutingKit::inf_weight;
        if (a_inf != b_inf || (!a_inf && dd[v] != rk[v])) {
          ++mismatches;
        }
      }
    }
    fmt::println("correctness: {} / {} vertices mismatch (must be 0)",
                 mismatches, checked);

    // last-mile setup: same CH, reverse query; oracle = transposed Dijkstra
    auto const grev = transpose(g);
    auto q2 = RoutingKit::ContractionHierarchyQuery{rkch};
    q2.pin_sources(targets);
    {
      auto rmism = 0ULL, rchk = 0ULL;
      for (auto qi = 0U; qi != 3U; ++qi) {
        auto const d = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        auto const dd = dijkstra_all(grev, d, kInfeasible);
        q2.reset_target().add_target(d).run_to_pinned_sources();
        auto const rk = q2.get_distances_to_sources();
        for (auto v = 0U; v != n; ++v) {
          ++rchk;
          auto const a_inf = dd[v] == kInfeasible;
          auto const b_inf = rk[v] >= RoutingKit::inf_weight;
          if (a_inf != b_inf || (!a_inf && dd[v] != rk[v])) {
            ++rmism;
          }
        }
      }
      fmt::println("last-mile correctness: {} / {} mismatch (must be 0)", rmism,
                   rchk);
    }

    // radius sweep: first- and last-mile, Dijkstra (radius-bounded) vs CH
    fmt::println(
        "\n radius |   first-mile: Dijkstra       CH  speedup |    last-mile: "
        "Dijkstra       CH  speedup");
    fmt::println(
        "--------+-------------------------------------------+---------------"
        "----------------------------");
    for (auto const R : {900U, 1800U, 3600U, 7200U, 14400U, 32768U}) {
      auto fdj = 0.0, fch = 0.0, ldj = 0.0, lch = 0.0;
      for (auto qi = 0U; qi != opt.n_queries_; ++qi) {
        auto const s = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        auto a = now();
        auto const d1 = dijkstra_all(g, s, R);
        fdj += ms(a, now());
        a = now();
        q.reset_source().add_source(s).run_to_pinned_targets();
        auto const r1 = q.get_distances_to_targets();
        fch += ms(a, now());

        auto const d = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        a = now();
        auto const d2 = dijkstra_all(grev, d, R);
        ldj += ms(a, now());
        a = now();
        q2.reset_target().add_target(d).run_to_pinned_sources();
        auto const r2 = q2.get_distances_to_sources();
        lch += ms(a, now());
      }
      auto const N = static_cast<double>(opt.n_queries_);
      fmt::println(
          "{:>6}s | {:>9.3f}ms {:>8.3f}ms {:>6.2f}x | {:>9.3f}ms {:>8.3f}ms "
          "{:>6.2f}x",
          R, fdj / N, fch / N, fdj / std::max(1e-9, fch), ldj / N, lch / N,
          ldj / std::max(1e-9, lch));
    }

    // ================= Bucket-CH on a realistic stop-subset =================
    auto const K = std::min(2000U, n);
    auto stops = std::vector<unsigned>{};
    stops.reserve(K);
    for (auto i = 0U; i != K; ++i) {
      stops.push_back(
          static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n_src));
    }
    auto const INF = RoutingKit::inf_weight;

    // upward CH search WITH STALL-ON-DEMAND. `main` = search-direction graph,
    // `stall` = opposite-direction graph (used to prune sub-optimally-settled
    // nodes, the dominant Bucket-CH optimization).
    auto up_search = [&](auto const& main, auto const& stall, unsigned const src,
                         std::vector<unsigned>& dist,
                         std::vector<unsigned>& touched) {
      using qe = std::pair<unsigned, unsigned>;
      auto pq = std::priority_queue<qe, std::vector<qe>, std::greater<>>{};
      dist[src] = 0U;
      touched.push_back(src);
      pq.push({0U, src});
      while (!pq.empty()) {
        auto const [d, u] = pq.top();
        pq.pop();
        if (d > dist[u]) continue;
        // stall-on-demand: a neighbor in the opposite graph offering a shorter
        // path into u means u is settled sub-optimally -> do not expand it.
        auto stalled = false;
        for (auto e = stall.first_out[u]; e != stall.first_out[u + 1U]; ++e) {
          auto const v = stall.head[e];
          if (dist[v] != INF && dist[v] + stall.weight[e] < d) {
            stalled = true;
            break;
          }
        }
        if (stalled) continue;
        for (auto e = main.first_out[u]; e != main.first_out[u + 1U]; ++e) {
          auto const v = main.head[e];
          auto const nd = d + main.weight[e];
          if (nd < dist[v]) {
            dist[v] = nd;
            touched.push_back(v);
            pq.push({nd, v});
          }
        }
      }
    };

    // ---- memory-scaling measurement: linear in #stops? radius-cappable? ----
    fmt::println("\nBucket-CH memory scaling (foot graph n={}):", n);
    fmt::println(
        "  #stops |   entries  entries/stop   full MB | <=30min MB  <=15min MB");
    fmt::println(
        "---------+--------------------------------------+--------------------"
        "--");
    {
      auto db = std::vector<unsigned>(n, INF);
      auto tou = std::vector<unsigned>{};
      for (auto const KK : {2000U, 8000U, 32000U, 128000U}) {
        if (KK > n) break;
        auto ent = 0ULL, cap30 = 0ULL, cap15 = 0ULL;
        auto hs = cista::BASE_HASH;
        auto cnt = 0U;
        for (auto i = 0U; i != KK; ++i) {
          auto const s =
              static_cast<unsigned>(cista::hash_combine(hs, ++cnt) % n);
          up_search(rkch.backward, rkch.forward, rkch.rank[s], db, tou);
          for (auto const v : tou) {
            ++ent;
            if (db[v] <= 1800U) ++cap30;
            if (db[v] <= 900U) ++cap15;
            db[v] = INF;
          }
          tou.clear();
        }
        fmt::println("{:>8} | {:>9} {:>10.0f}  {:>9.1f} | {:>9.1f}  {:>9.1f}",
                     KK, ent, static_cast<double>(ent) / KK, ent * 8.0 / 1e6,
                     cap30 * 8.0 / 1e6, cap15 * 8.0 / 1e6);
      }
    }

    // ---- REAL stops (match GTFS stop coords to graph nodes, then measure) ----
    if (!opt.stops_file_.empty() && !opt.car_) {
      auto real_stops = std::vector<unsigned>{};
      auto f = std::ifstream{opt.stops_file_};
      auto line = std::string{};
      auto const fp = typename foot<false>::parameters{};
      auto total = 0U;
      while (std::getline(f, line)) {
        auto const comma = line.find(',');
        if (comma == std::string::npos) continue;
        ++total;
        auto const lat = std::stod(line.substr(0, comma));
        auto const lon = std::stod(line.substr(comma + 1));
        auto const m = l.match<foot<false>>(
            fp, location{geo::latlng{lat, lon}, level_t{0.F}}, false,
            direction::kForward, 100.0, nullptr);
        if (m.empty()) continue;
        auto const& nc = m.front().left_.valid() ? m.front().left_ : m.front().right_;
        if (nc.valid()) real_stops.push_back(to_idx(nc.node_));
      }
      auto db = std::vector<unsigned>(n, INF);
      auto tou = std::vector<unsigned>{};
      auto ent = 0ULL, cap30 = 0ULL, cap15 = 0ULL;
      for (auto const s : real_stops) {
        up_search(rkch.backward, rkch.forward, rkch.rank[s], db, tou);
        for (auto const v : tou) {
          ++ent;
          if (db[v] <= 1800U) ++cap30;
          if (db[v] <= 900U) ++cap15;
          db[v] = INF;
        }
        tou.clear();
      }
      fmt::println(
          "\nREAL stops ({}): {} of {} matched to graph nodes", opt.stops_file_,
          real_stops.size(), total);
      fmt::println(
          "  {} real stops | {} entries {:.0f}/stop  full {:.1f} MB | "
          "<=30min {:.1f} MB  <=15min {:.1f} MB",
          real_stops.size(), ent,
          static_cast<double>(ent) / std::max<std::size_t>(1, real_stops.size()),
          ent * 8.0 / 1e6, cap30 * 8.0 / 1e6, cap15 * 8.0 / 1e6);
    }

    // build buckets: for each stop, backward up-search; bucket[v] += (dist, i).
    // Sorted by distance so the query can break early under a radius bound.
    auto bucket = std::vector<std::vector<std::pair<unsigned, unsigned>>>(n);
    {
      auto db = std::vector<unsigned>(n, INF);
      auto tou = std::vector<unsigned>{};
      auto const tb = now();
      for (auto i = 0U; i != K; ++i) {
        up_search(rkch.backward, rkch.forward, rkch.rank[stops[i]], db, tou);
        for (auto const v : tou) {
          bucket[v].push_back({db[v], i});
          db[v] = INF;
        }
        tou.clear();
      }
      for (auto& b : bucket) {
        std::sort(begin(b), end(b));
      }
      auto const build_ms = ms(tb, now());
      auto entries = 0ULL, nonempty = 0ULL;
      for (auto const& b : bucket) {
        entries += b.size();
        nonempty += b.empty() ? 0U : 1U;
      }
      auto const ch_edges = rkch.forward.head.size() + rkch.backward.head.size();
      fmt::println(
          "\nBucket-CH: {} stops built in {:.0f}ms\n"
          "  buckets: {} entries = {:.1f} MB  ({:.0f}/stop, {} of {} nodes "
          "non-empty)\n"
          "  CH graph: {} edges = {:.1f} MB   (base graph: {} edges = {:.1f} "
          "MB)",
          K, build_ms, entries, entries * 8.0 / 1e6,
          static_cast<double>(entries) / K, nonempty, n, ch_edges,
          ch_edges * 8.0 / 1e6, g.n_edges(), g.n_edges() * 8.0 / 1e6);
    }

    // last-mile buckets: forward up-search from each stop (stop -> node up)
    auto bucket_lm = std::vector<std::vector<std::pair<unsigned, unsigned>>>(n);
    {
      auto db = std::vector<unsigned>(n, INF);
      auto tl = std::vector<unsigned>{};
      for (auto i = 0U; i != K; ++i) {
        up_search(rkch.forward, rkch.backward, rkch.rank[stops[i]], db, tl);
        for (auto const v : tl) {
          bucket_lm[v].push_back({db[v], i});
          db[v] = INF;
        }
        tl.clear();
      }
      for (auto& b : bucket_lm) {
        std::sort(begin(b), end(b));
      }
    }

    // Bucket-CH query with radius R: forward up-search (stall) + pruned sweep.
    auto df = std::vector<unsigned>(n, INF);
    auto tou = std::vector<unsigned>{};
    auto res = std::vector<unsigned>(K);
    auto bucket_query = [&](unsigned const s, unsigned const R) {
      std::fill(begin(res), end(res), INF);
      up_search(rkch.forward, rkch.backward, rkch.rank[s], df, tou);
      for (auto const u : tou) {
        auto const du = df[u];
        for (auto const& [dbk, i] : bucket[u]) {
          auto const c = du + dbk;
          if (c >= R) break;  // sorted -> remaining stops are farther
          if (c < res[i]) res[i] = c;
        }
        df[u] = INF;
      }
      tou.clear();
    };

    // last-mile query: backward up-search from dest, scan last-mile buckets
    auto res_lm = std::vector<unsigned>(K);
    auto last_mile_query = [&](unsigned const d, unsigned const R) {
      std::fill(begin(res_lm), end(res_lm), INF);
      up_search(rkch.backward, rkch.forward, rkch.rank[d], df, tou);
      for (auto const u : tou) {
        auto const du = df[u];
        for (auto const& [dbk, i] : bucket_lm[u]) {
          auto const c = du + dbk;
          if (c >= R) break;
          if (c < res_lm[i]) res_lm[i] = c;
        }
        df[u] = INF;
      }
      tou.clear();
    };

    // correctness vs RoutingKit-pinned
    auto qp = RoutingKit::ContractionHierarchyQuery{rkch};
    qp.pin_targets(stops);
    auto qp_lm = RoutingKit::ContractionHierarchyQuery{rkch};
    qp_lm.pin_sources(stops);
    {
      auto bmism = 0ULL;
      for (auto qi = 0U; qi != 5U; ++qi) {
        auto const s = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        bucket_query(s, INF);
        qp.reset_source().add_source(s).run_to_pinned_targets();
        auto const ref = qp.get_distances_to_targets();
        for (auto i = 0U; i != K; ++i) {
          if (res[i] != ref[i]) ++bmism;
        }
      }
      fmt::println("Bucket-CH correctness vs RK-pinned: {} mismatch (must be 0)",
                   bmism);
    }

    // last-mile correctness vs RoutingKit reverse
    {
      auto bmism = 0ULL;
      for (auto qi = 0U; qi != 5U; ++qi) {
        auto const d = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        last_mile_query(d, INF);
        qp_lm.reset_target().add_target(d).run_to_pinned_sources();
        auto const ref = qp_lm.get_distances_to_sources();
        for (auto i = 0U; i != K; ++i) {
          if (res_lm[i] != ref[i]) ++bmism;
        }
      }
      fmt::println("Bucket-CH last-mile correctness vs RK: {} mismatch (0 ok)",
                   bmism);
    }

    // ===== presentation sweep: Dijkstra vs Bucket-CH, first & last mile =====
    fmt::println("\n radius |          first mile             |          last mile");
    fmt::println(
        "  (min) |  Dijkstra   Bucket-CH  speedup  |  Dijkstra   Bucket-CH  "
        "speedup");
    fmt::println(
        "--------+---------------------------------+-------------------------"
        "--------");
    for (auto const R : {300U, 600U, 900U, 1200U, 1800U, 2400U, 3000U, 3600U,
                         5400U, 7200U, 10800U, 14400U, 21600U, 32768U}) {
      auto fdj = 0.0, fb = 0.0, ldj = 0.0, lb = 0.0;
      for (auto qi = 0U; qi != opt.n_queries_; ++qi) {
        auto const s = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        auto a = now();
        auto const d1 = dijkstra_all(g, s, R);
        fdj += ms(a, now());
        a = now();
        bucket_query(s, R);
        fb += ms(a, now());
        auto const d = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
        a = now();
        auto const d2 = dijkstra_all(grev, d, R);
        ldj += ms(a, now());
        a = now();
        last_mile_query(d, R);
        lb += ms(a, now());
      }
      auto const N = static_cast<double>(opt.n_queries_);
      fmt::println(
          "{:>5.0f}m | {:>8.3f}ms {:>9.3f}ms {:>6.1f}x  | {:>8.3f}ms {:>9.3f}ms "
          "{:>6.1f}x",
          R / 60.0, fdj / N, fb / N, fdj / std::max(1e-9, fb), ldj / N, lb / N,
          ldj / std::max(1e-9, lb));
    }

    // ===== Bucket-CH vs osr PRODUCTION dial-Dijkstra (what MOTIS runs) =====
    // osr's route() one-to-many uses this dijkstra<foot> (dial queue, on-the-fly
    // (node,level) expansion). Same source -> same K stops, same radius.
    // (foot only: car uses a different (node,way,dir) key; see note below.)
    if (!opt.car_) {
      auto d = dijkstra<foot<false>>{};
      auto const fpar = typename foot<false>::parameters{};
      fmt::println(
          "\nBucket-CH vs osr PRODUCTION dial-Dijkstra (source -> {} stops):", K);
      fmt::println(
          "  radius | osr-Dijkstra   Bucket-CH   speedup | mismatch");
      fmt::println(
          "--------+---------------------------------+---------");
      for (auto const R : {900U, 1800U, 3600U, 7200U, 14400U}) {
        auto tdj = 0.0, tb = 0.0;
        auto mism = 0ULL;
        for (auto qi = 0U; qi != opt.n_queries_; ++qi) {
          auto const s =
              static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n_src);
          auto a = now();
          d.reset(R);
          set_start<foot<false>>(d, w, node_idx_t{s});
          d.template run<direction::kForward, false>(fpar, w, *w.r_, R, nullptr,
                                                     nullptr, elevations.get());
          auto dcost = std::vector<cost_t>(K);
          for (auto i = 0U; i != K; ++i) {
            dcost[i] = d.get_cost(typename foot<false>::node{node_idx_t{stops[i]}});
          }
          tdj += ms(a, now());
          a = now();
          bucket_query(s, R);
          tb += ms(a, now());
          for (auto i = 0U; i != K; ++i) {
            auto const dj_in = dcost[i] != kInfeasible && dcost[i] < R;
            auto const bk_in = res[i] != INF;
            if (dj_in != bk_in || (dj_in && dcost[i] != res[i])) {
              ++mism;
              if (mism <= 3) {
                fmt::println(
                    "   MISMATCH R={} src_node={} stop_node={} osm={} "
                    "osr(lvl0)={} bucketCH={} (dj_in={} bk_in={})",
                    R, s, stops[i], to_idx(w.node_to_osm_[node_idx_t{stops[i]}]),
                    dcost[i], res[i], dj_in, bk_in);
              }
            }
          }
        }
        auto const N = static_cast<double>(opt.n_queries_);
        fmt::println("{:>6}s | {:>10.3f}ms {:>10.3f}ms {:>6.1f}x | {}", R,
                     tdj / N, tb / N, tdj / std::max(1e-9, tb), mism);
      }
    } else {
      // car: Bucket-CH vs osr PRODUCTION car dial-Dijkstra, vertex -> vertices
      // on the exact turn-expanded graph. vmap[id] = the (node,way,dir) state.
      auto vmap = std::vector<car::node>{};
      vmap.reserve(n);
      for (auto i = 0U; i != static_cast<unsigned>(w.n_nodes()); ++i) {
        auto const nways = w.r_->node_ways_[node_idx_t{i}].size();
        for (auto way = 0U; way != nways; ++way) {
          for (auto dd = 0U; dd != 2U; ++dd) {
            vmap.push_back(car::node{node_idx_t{i}, static_cast<way_pos_t>(way),
                                     dd == 0U ? direction::kForward
                                              : direction::kBackward});
          }
        }
      }
      auto d = dijkstra<car>{};
      auto const cpar = typename car::parameters{};
      fmt::println(
          "\nBucket-CH vs osr PRODUCTION car dial-Dijkstra (vertex -> {} "
          "stops):",
          K);
      fmt::println("  radius | osr-Dijkstra   Bucket-CH   speedup | mismatch");
      fmt::println("--------+---------------------------------+---------");
      for (auto const R : {900U, 1800U, 3600U, 7200U, 14400U}) {
        auto tdj = 0.0, tb = 0.0;
        auto mism = 0ULL;
        for (auto qi = 0U; qi != opt.n_queries_; ++qi) {
          auto s = 0U;
          do {
            s = static_cast<unsigned>(cista::hash_combine(hh, ++nn) % n);
          } while (g.first_out_[s] == g.first_out_[s + 1U]);
          auto a = now();
          d.reset(R);
          d.add_start(w, car::label{vmap[s], 0U});
          d.template run<direction::kForward, false>(cpar, w, *w.r_, R, nullptr,
                                                     nullptr, nullptr);
          auto dcost = std::vector<cost_t>(K);
          for (auto i = 0U; i != K; ++i) {
            dcost[i] = d.get_cost(vmap[stops[i]]);
          }
          tdj += ms(a, now());
          a = now();
          bucket_query(s, R);
          tb += ms(a, now());
          for (auto i = 0U; i != K; ++i) {
            auto const dj_in = dcost[i] != kInfeasible && dcost[i] < R;
            auto const bk_in = res[i] != INF;
            if (dj_in != bk_in || (dj_in && dcost[i] != res[i])) ++mism;
          }
        }
        auto const N = static_cast<double>(opt.n_queries_);
        fmt::println("{:>6}s | {:>10.3f}ms {:>10.3f}ms {:>6.1f}x | {}", R,
                     tdj / N, tb / N, tdj / std::max(1e-9, tb), mism);
      }
    }
    return 0;
  }

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
