#include "osr/ways.h"

#include <algorithm>
#include <atomic>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "date/tz.h"

#include "utl/parallel_for.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "cista/io.h"

namespace osr {

namespace {

template <typename Polyline>
quantized_angle_t get_prev_bearing(Polyline const& polyline,
                                   std::size_t const idx) {
  if (idx == 0U) {
    return 0U;
  }

  auto const origin = polyline[idx].as_latlng();
  for (auto i = idx; i != 0U; --i) {
    auto const candidate = polyline[i - 1U].as_latlng();
    if (origin != candidate) {
      return quantize_angle(geo::bearing(origin, candidate));
    }
  }

  return 0U;
}

template <typename Polyline>
quantized_angle_t get_next_bearing(Polyline const& polyline,
                                   std::size_t const idx) {
  if (idx + 1U >= polyline.size()) {
    return 0U;
  }

  auto const origin = polyline[idx].as_latlng();
  for (auto i = static_cast<std::size_t>(idx) + 1U; i < polyline.size(); ++i) {
    auto const candidate = polyline[i].as_latlng();
    if (origin != candidate) {
      return quantize_angle(geo::bearing(origin, candidate));
    }
  }

  return 0U;
}

constexpr auto const kNoComponent = std::numeric_limits<std::uint32_t>::max();

std::uint32_t find_component_root(std::vector<std::uint32_t>& parent,
                                  std::uint32_t way) {
  while (parent[way] != way) {
    parent[way] = parent[parent[way]];
    way = parent[way];
  }
  return way;
}

std::vector<std::uint32_t> build_component_forest(ways::routing const& r,
                                                  component_class const cls) {
  auto parent =
      std::vector<std::uint32_t>(r.way_properties_.size(), kNoComponent);
  for (auto i = 0U; i != parent.size(); ++i) {
    if (is_accessible(cls, r.way_properties_[way_idx_t{i}])) {
      parent[i] = i;
    }
  }
  for (auto n = 0U; n != r.node_ways_.size(); ++n) {
    auto anchor = kNoComponent;
    for (auto const w : r.node_ways_[node_idx_t{n}]) {
      if (parent[to_idx(w)] == kNoComponent) {
        continue;
      }
      auto const root = find_component_root(parent, to_idx(w));
      if (anchor == kNoComponent) {
        anchor = root;
      } else {
        auto const a = find_component_root(parent, anchor);
        anchor = std::min(a, root);
        parent[std::max(a, root)] = anchor;
      }
    }
  }
  return parent;
}

// Every non-root points to a smaller index, which is already renumbered.
std::vector<std::uint32_t> number_component_roots(
    std::vector<std::uint32_t>& parent) {
  auto sizes = std::vector<std::uint32_t>{};
  for (auto i = 0U; i != parent.size(); ++i) {
    if (parent[i] == kNoComponent) {
      continue;
    }
    if (parent[i] == i) {
      utl::verify(sizes.size() < kNoComponent, "too many components");
      parent[i] = static_cast<std::uint32_t>(sizes.size());
      sizes.push_back(0U);
    } else {
      parent[i] = parent[parent[i]];
    }
    ++sizes[parent[i]];
  }
  return sizes;
}

}  // namespace

class_components compute_class_components(ways::routing const& r,
                                          component_class const cls) {
  auto parent = build_component_forest(r, cls);
  auto const sizes = number_component_roots(parent);
  return class_components::build(parent, sizes);
}

class_components class_components::build(
    std::span<std::uint32_t const> const comp,
    std::span<std::uint32_t const> const sizes) {
  utl::verify(comp.size() <= std::numeric_limits<std::uint32_t>::max() &&
                  sizes.size() < std::numeric_limits<std::uint32_t>::max(),
              "component graph exceeds index range");
  auto ranked = std::vector<std::uint32_t>(sizes.size());
  std::iota(begin(ranked), end(ranked), 0U);
  std::erase_if(ranked, [&](auto const id) { return sizes[id] < 2U; });
  std::sort(begin(ranked), end(ranked), [&](auto const a, auto const b) {
    return sizes[a] != sizes[b] ? sizes[a] > sizes[b] : a < b;
  });
  auto ids = std::vector<std::uint32_t>(sizes.size(), 0U);
  for (auto i = std::size_t{0U}; i != ranked.size(); ++i) {
    ids[ranked[i]] = static_cast<std::uint32_t>(i + 1U);
  }

  auto out = class_components{};
  out.way_component_.resize(static_cast<std::uint32_t>(comp.size()),
                            kNoComponent);
  auto exception_ways = std::uint64_t{0U};
  for (auto i = std::size_t{kException - 1U}; i < ranked.size(); ++i) {
    exception_ways += sizes[ranked[i]];
  }
  utl::verify(exception_ways <= comp.size(), "invalid component sizes");
  out.exception_way_.reserve(static_cast<std::uint32_t>(exception_ways));
  out.exception_component_.reserve(static_cast<std::uint32_t>(exception_ways));
  for (auto i = std::size_t{0U}; i != comp.size(); ++i) {
    auto const c = comp[i];
    if (c == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }
    utl::verify(c < sizes.size(), "invalid component id {}", c);
    auto const id = ids[c];
    auto const way = way_idx_t{static_cast<std::uint32_t>(i)};
    if (id < kException) {
      out.way_component_[way] = static_cast<std::uint8_t>(id);
    } else {
      out.way_component_[way] = kException;
      out.exception_way_.push_back(way);
      out.exception_component_.push_back(id);
    }
  }
  return out;
}

ways::ways(std::filesystem::path p, cista::mmap::protection const mode)
    : p_{std::move(p)},
      mode_{mode},
      r_{mode == cista::mmap::protection::READ
             ? routing::read(p_)
             : cista::wrapped<routing>{cista::raw::make_unique<routing>()}},
      timezones_{utl::to_vec(
          r_->timezones_,
          [](auto const& tz) { return date::locate_zone(tz.view()); })},
      node_to_osm_{mm("node_to_osm.bin")},
      osm_node_order_{mm("osm_node_order.bin")},
      way_osm_idx_{mm("way_osm_idx.bin")},
      osm_way_order_{mm("osm_way_order.bin")},
      way_polylines_{mm_vec<point>{mm("way_polylines_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_polylines_index.bin")}},
      way_osm_nodes_{mm_vec<osm_node_idx_t>{mm("way_osm_nodes_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_osm_nodes_index.bin")}},
      strings_{mm_vec<char>(mm("strings_data.bin")),
               mm_vec<std::uint64_t>(mm("strings_idx.bin"))},
      way_names_{mm("way_names.bin")},
      way_has_conditional_access_no_{
          mm_vec<std::uint64_t>(mm("way_has_conditional_access_no"))},
      way_conditional_access_no_{mm("way_conditional_access_no")} {}

void ways::build_class_components() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Build components")
      .in_high(kNumComponentClasses)
      .out_bounds(75, 90);

  r_->class_components_.clear();
  r_->class_components_.resize(kNumComponentClasses);
  auto const threads =
      std::min(static_cast<unsigned>(kNumComponentClasses),
               std::max(1U, std::thread::hardware_concurrency()));
  utl::parallel_for_run(
      kNumComponentClasses,
      [&](std::size_t const i) {
        r_->class_components_[static_cast<std::uint32_t>(i)] =
            compute_class_components(*r_, kAllComponentClasses[i]);
        pt->increment();
      },
      utl::noop_progress_update{}, utl::parallel_error_strategy::QUIT_EXEC,
      threads);
}

void ways::add_restriction(std::vector<resolved_restriction>& rs) {
  using it_t = std::vector<resolved_restriction>::iterator;
  utl::sort(rs, [](auto&& a, auto&& b) { return a.via_ < b.via_; });
  utl::equal_ranges_linear(
      begin(rs), end(rs), [](auto&& a, auto&& b) { return a.via_ == b.via_; },
      [&](it_t const& lb, it_t const& ub) {
        auto const range = std::span{lb, ub};
        r_->node_restrictions_.resize(to_idx(range.front().via_) + 1U);
        r_->node_is_restricted_.set(range.front().via_, true);

        for (auto const& x : range) {
          // Skip a restriction whose way does not meet the via node:
          // get_way_pos() would return 0, i.e. whichever way comes first at
          // the node.
          auto const pos = [&](way_idx_t const way) {
            auto const ways = r_->node_ways_[x.via_];
            auto const it = std::find(begin(ways), end(ways), way);
            return it != end(ways)
                       ? std::optional{static_cast<way_pos_t>(it - begin(ways))}
                       : std::nullopt;
          };
          auto const from_pos = pos(x.from_);
          auto const to_pos = pos(x.to_);
          if (!from_pos.has_value() || !to_pos.has_value()) {
            continue;
          }
          if (x.type_ == resolved_restriction::type::kNo) {
            r_->node_restrictions_[x.via_].push_back(restriction{
                *from_pos, *to_pos, x.applies_to_default_, x.applies_to_bus_,
                x.applies_to_hgv_, x.condition_set_});
          } else /* kOnly */ {
            for (auto const [i, from] :
                 utl::enumerate(r_->node_ways_[x.via_])) {
              for (auto const [j, to] :
                   utl::enumerate(r_->node_ways_[x.via_])) {
                if (x.from_ == from && x.to_ != to) {
                  r_->node_restrictions_[x.via_].push_back(restriction{
                      static_cast<way_pos_t>(i), static_cast<way_pos_t>(j),
                      x.applies_to_default_, x.applies_to_bus_,
                      x.applies_to_hgv_, x.condition_set_});
                }
              }
            }
          }
        }
      });
  r_->node_restrictions_.resize(node_to_osm_.size());
}

void ways::compute_big_street_neighbors() {
  struct state {
    hash_set<way_idx_t> done_;
  };

  auto pt = utl::get_active_progress_tracker();

  auto is_orig_big_street = std::vector<bool>(n_ways());
  for (auto const [i, p] : utl::enumerate(r_->way_properties_)) {
    is_orig_big_street[i] = p.is_big_street();
  }

  utl::parallel_for_run_threadlocal<state>(
      n_ways(), [&](state& s, std::size_t const i) {
        auto const way = way_idx_t{i};

        if (is_orig_big_street[to_idx(way)]) {
          pt->update_monotonic(i);
          return;
        }

        s.done_.clear();

        auto const expand = [&](way_idx_t const x, bool const go_further,
                                auto&& recurse) {
          for (auto const& n : r_->way_nodes_[x]) {
            for (auto const& w : r_->node_ways_[n]) {
              if (is_orig_big_street[to_idx(w)]) {
                r_->way_properties_[way].is_big_street_ = true;
                return true;
              }

              if (s.done_.emplace(w).second && go_further) {
                if (recurse(x, false, recurse)) {
                  return true;
                }
              }
            }
          }
          return false;
        };

        s.done_.emplace(way);
        expand(way, true, expand);
        pt->update_monotonic(i);
      });
}

std::vector<way_idx_t> ways::order_ways() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Order ways").in_high(n_ways()).out_bounds(40, 41);

  // (key, old index): equal keys keep the OSM id order
  auto order = std::vector<std::pair<std::uint64_t, way_idx_t>>(n_ways());
  utl::parallel_for_run(
      n_ways(),
      [&](std::size_t const i) {
        auto const way = way_idx_t{i};
        auto const polyline = way_polylines_[way];
        order[i] = {geo::morton_encode(polyline[polyline.size() / 2U]), way};
      },
      pt->update_fn());
  std::sort(begin(order), end(order));

  auto new_idx = std::vector<way_idx_t>(n_ways());
  for (auto i = 0U; i != order.size(); ++i) {
    new_idx[to_idx(order[i].second)] = way_idx_t{i};
  }

  // Ways were added in OSM id order: the old index is the rank by OSM id.
  osm_way_order_.resize(n_ways());
  for (auto i = 0U; i != new_idx.size(); ++i) {
    osm_way_order_[i] = new_idx[i];
  }

  auto const permute = [&](auto& v) {
    auto const old = std::vector(begin(v), end(v));
    for (auto i = 0U; i != order.size(); ++i) {
      v[way_idx_t{i}] = old[to_idx(order[i].second)];
    }
  };
  permute(way_osm_idx_);
  permute(way_names_);
  permute(r_->way_properties_);

  auto const permute_bits = [&](auto& v) {
    auto old = std::vector<bool>(n_ways());
    for (auto i = 0U; i != v.size(); ++i) {
      old[i] = v.test(way_idx_t{i});
    }
    v.resize(n_ways());
    for (auto i = 0U; i != order.size(); ++i) {
      v.set(way_idx_t{i}, old[to_idx(order[i].second)]);
    }
  };
  permute_bits(way_has_conditional_access_no_);

  // (way, value) lists, looked up by binary search
  auto const remap = [&](auto& v) {
    for (auto& x : v) {
      x.first = new_idx[to_idx(x.first)];
    }
    std::sort(begin(v), end(v),
              [](auto const& a, auto const& b) { return a.first < b.first; });
  };
  remap(r_->way_hgv_info_);
  remap(r_->way_conditionals_);
  remap(way_conditional_access_no_);

  // Rebuilt through temporary files: on Europe both are 8 GB.
  auto const permute_buckets = [&](auto& v) {
    {
      auto tmp = std::decay_t<decltype(v)>{
          decltype(v.data_){mm("order_ways_tmp_data.bin")},
          decltype(v.bucket_starts_){mm("order_ways_tmp_index.bin")}};
      for (auto const& [_, old] : order) {
        tmp.emplace_back(v[old]);
      }
      v.data_.resize(0U);  // mmap_vec has no clear()
      v.bucket_starts_.resize(0U);
      for (auto const b : tmp) {
        v.emplace_back(b);
      }
    }
    // cista::mmap unmaps only the used part of the mapping; the rest keeps
    // the file alive until exit. Truncate first so removing frees the space.
    for (auto const* f :
         {"order_ways_tmp_data.bin", "order_ways_tmp_index.bin"}) {
      std::filesystem::resize_file(p_ / f, 0U);
      std::filesystem::remove(p_ / f);
    }
  };
  permute_buckets(way_polylines_);
  permute_buckets(way_osm_nodes_);

  return new_idx;
}

void ways::connect_ways() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  {  // Assign graph node ids to every node with >1 way, in Morton order of
     // the node positions: nearby nodes get nearby ids, so a search touches
     // fewer cache lines.
    pt->status("Create graph nodes")
        .in_high(way_osm_nodes_.size())
        .out_bounds(40, 50);

    auto osm_ids = std::vector<osm_node_idx_t>{};  // ascending
    node_way_counter_.for_each_multi([&](std::uint64_t const b_idx) {
      osm_ids.emplace_back(osm_node_idx_t{b_idx});
    });

    auto keys = std::vector<std::uint64_t>(osm_ids.size());
    utl::parallel_for_run(
        way_osm_nodes_.size(),
        [&](std::size_t const way) {
          for (auto const [osm_node_idx, pos] :
               utl::zip(way_osm_nodes_[way_idx_t{way}],
                        way_polylines_[way_idx_t{way}])) {
            if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
              auto const i = static_cast<std::size_t>(
                  std::lower_bound(begin(osm_ids), end(osm_ids), osm_node_idx) -
                  begin(osm_ids));
              std::atomic_ref{keys[i]}.store(geo::morton_encode(pos),
                                             std::memory_order_relaxed);
            }
          }
        },
        pt->update_fn());

    // (key, rank by OSM id): equal keys keep the OSM id order
    auto order = std::vector<std::pair<std::uint64_t, std::uint32_t>>{};
    order.reserve(osm_ids.size());
    for (auto i = 0U; i != osm_ids.size(); ++i) {
      order.emplace_back(keys[i], i);
    }
    keys = {};
    std::sort(begin(order), end(order));

    osm_node_order_.resize(osm_ids.size());
    for (auto const& [_, rank] : order) {
      osm_node_order_[rank] = node_idx_t{node_to_osm_.size()};
      node_to_osm_.push_back(osm_ids[rank]);
    }
    r_->node_is_restricted_.resize(n_nodes());
  }

  // Build edges.
  {
    auto const n_nodes = static_cast<std::size_t>(node_to_osm_.size());
    auto const n_ways = static_cast<std::size_t>(way_osm_nodes_.size());

    pt->status("Connect ways").in_high(n_ways).out_bounds(50, 71);

    // Phase 1: Count number of ways per node (if way visits *N -> counted *N).
    // Phase 2: Reserve node_ways_ / node_in_way_idx_ and reset counts to 0.
    // Phase 3: Fill node_ways_ / node_in_way_idx_ = transpose of way_nodes_.
    //          -> count becomes a fill cursor per node
    auto count = std::vector<std::atomic<std::uint32_t>>(n_nodes);

    // At nodes visited more than kMaxWaysPerNode times, the visits of the ways
    // with the lowest OSM ids (then by position in the way) keep their slot.
    // Claiming the slots in the parallel loop below would make the graph
    // depend on thread timing and on the way order.
    utl::parallel_for_run(
        n_ways,
        [&](std::size_t const way) {
          for (auto const osm_node_idx : way_osm_nodes_[way_idx_t{way}]) {
            if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
              count[to_idx(get_node_idx(osm_node_idx))].fetch_add(
                  1U, std::memory_order_relaxed);
            }
          }
        },
        pt->update_fn());
    auto overfull = hash_set<osm_node_idx_t>{};
    for (auto n = std::size_t{0U}; n != n_nodes; ++n) {
      if (count[n].exchange(0U, std::memory_order_relaxed) > kMaxWaysPerNode) {
        overfull.emplace(node_to_osm_[node_idx_t{n}]);
      }
    }

    auto allowed = std::vector<std::pair<way_idx_t, std::uint32_t>>{};
    if (!overfull.empty()) {
      struct visit {
        osm_node_idx_t node_;
        osm_way_idx_t osm_way_;
        std::uint32_t pos_;
        way_idx_t way_;
      };
      auto visits = std::vector<visit>{};
      auto m = std::mutex{};
      utl::parallel_for_run(n_ways, [&](std::size_t const way) {
        auto const w = way_idx_t{way};
        auto local = std::vector<visit>{};
        for (auto const [pos, osm_node_idx] :
             utl::enumerate(way_osm_nodes_[w])) {
          if (overfull.contains(osm_node_idx)) {
            local.push_back({osm_node_idx, way_osm_idx_[w],
                             static_cast<std::uint32_t>(pos), w});
          }
        }
        if (!local.empty()) {
          auto const lock = std::scoped_lock{m};
          visits.insert(end(visits), begin(local), end(local));
        }
      });
      std::sort(begin(visits), end(visits), [](visit const& a, visit const& b) {
        if (a.node_ != b.node_) {
          return a.node_ < b.node_;
        }
        if (a.osm_way_ != b.osm_way_) {
          return osm_id_less(a.osm_way_, b.osm_way_);
        }
        return a.pos_ < b.pos_;
      });
      for (auto i = std::size_t{0U}; i != visits.size();) {
        auto j = i;
        while (j != visits.size() && visits[j].node_ == visits[i].node_) {
          ++j;
        }
        std::clog << "osr: node " << to_idx(visits[i].node_) << " has "
                  << (j - i) << " way visits, keeping the "
                  << static_cast<unsigned>(kMaxWaysPerNode)
                  << " with the lowest OSM ids\n";
        for (auto k = i; k != std::min(j, i + kMaxWaysPerNode); ++k) {
          allowed.emplace_back(visits[k].way_, visits[k].pos_);
        }
        i = j;
      }
      utl::sort(allowed);
    }

    struct way_edges {
      std::vector<node_idx_t> nodes_;
      std::vector<std::uint16_t> dists_;
      std::vector<routing::long_distance> long_dists_;
    };

    constexpr auto kConnectChunkSize = std::size_t{1024U};
    auto const n_chunks = (n_ways + kConnectChunkSize - 1U) / kConnectChunkSize;
    utl::parallel_ordered_collect_threadlocal<char>(
        n_chunks,
        [&](char&, std::size_t const chunk) {
          auto const chunk_begin = chunk * kConnectChunkSize;
          auto const chunk_end =
              std::min(chunk_begin + kConnectChunkSize, n_ways);
          auto out = std::vector<way_edges>{};
          out.reserve(chunk_end - chunk_begin);
          for (auto way = chunk_begin; way != chunk_end; ++way) {
            auto const way_idx = way_idx_t{way};
            auto c = way_edges{};
            auto pred_pos = std::make_optional<point>();
            auto from = node_idx_t::invalid();
            auto distance = 0.0;
            auto i = std::uint16_t{0U};
            auto visit_pos = std::uint32_t{0U};
            for (auto const [osm_node_idx, pos] :
                 utl::zip(way_osm_nodes_[way_idx], way_polylines_[way_idx])) {
              auto const this_visit = std::pair{way_idx, visit_pos++};
              if (pred_pos.has_value()) {
                distance += geo::distance(pos, *pred_pos);
              }

              if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
                if (!allowed.empty() && overfull.contains(osm_node_idx) &&
                    !std::binary_search(begin(allowed), end(allowed),
                                        this_visit)) {
                  pred_pos = pos;
                  continue;
                }
                auto const to = get_node_idx(osm_node_idx);
                auto const rank =
                    count[to_idx(to)].fetch_add(1U, std::memory_order_relaxed);
                if (rank >= kMaxWaysPerNode) {
                  pred_pos = pos;
                  continue;
                }

                c.nodes_.push_back(to);

                if (from != node_idx_t::invalid()) {
                  auto const dist =
                      static_cast<distance_t>(std::round(distance));
                  if (dist < std::numeric_limits<std::uint16_t>::max()) {
                    c.dists_.push_back(static_cast<std::uint16_t>(dist));
                  } else {
                    c.long_dists_.push_back(routing::long_distance{
                        .way_ = way_idx,
                        .node_ = static_cast<std::uint16_t>(i - 1U),
                        .distance_ = dist});
                    c.dists_.push_back(
                        std::numeric_limits<std::uint16_t>::max());
                  }
                }

                distance = 0.0;
                from = to;

                if (i == std::numeric_limits<std::uint16_t>::max()) {
                  fmt::println("error: way {} has too many nodes, truncating",
                               way_osm_idx_[way_idx]);
                  break;
                }

                ++i;
              }

              pred_pos = pos;
            }
            out.push_back(std::move(c));
          }
          return out;
        },
        [&](std::size_t, std::vector<way_edges> const& chunk_out) {
          for (auto const& c : chunk_out) {
            r_->way_nodes_.emplace_back(c.nodes_);
            r_->way_node_dist_.emplace_back(c.dists_);
            for (auto const& x : c.long_dists_) {
              r_->long_way_node_dist_.push_back(x);
            }
          }
        },
        [&](std::size_t) { pt->increment(kConnectChunkSize); });

    utl::sort(r_->long_way_node_dist_);

    pt->status("Connect ways / transpose").in_high(n_ways).out_bounds(71, 74);

    // Reserve node_ways_ / node_in_way_idx_ for each node.
    for (auto n = std::size_t{0U}; n != n_nodes; ++n) {
      auto const size =
          std::min(count[n].exchange(0U, std::memory_order_relaxed),
                   static_cast<std::uint32_t>(kMaxWaysPerNode));
      r_->node_ways_.add_back_sized(size);
      r_->node_in_way_idx_.add_back_sized(size);
    }

    // Fill node_ways_ / node_in_way_idx_ = transpose of way_nodes_.
    // Threads claims their write slot via the atomic per-node cursor.
    utl::parallel_for_run(
        n_ways,
        [&](std::size_t const way) {
          auto i = std::uint16_t{0U};
          for (auto const n : r_->way_nodes_[way_idx_t{way}]) {
            auto const pos =
                count[to_idx(n)].fetch_add(1U, std::memory_order_relaxed);
            r_->node_ways_[n][pos] = way_idx_t{way};
            r_->node_in_way_idx_[n][pos] = i++;
          }
        },
        pt->update_fn());

    // Sort node_ways_ / node_in_way_idx_ by OSM way id (then position) for
    // deterministic ordering. Routing breaks equal-cost ties in this order,
    // so it must not depend on the way numbering.
    pt->status("Connect ways / sort").in_high(n_nodes).out_bounds(74, 75);
    utl::parallel_for_run_threadlocal<
        std::vector<std::pair<way_idx_t, std::uint16_t>>>(
        n_nodes,
        [&](std::vector<std::pair<way_idx_t, std::uint16_t>>& scratch,
            std::size_t const n) {
          auto ways = r_->node_ways_[node_idx_t{n}];
          auto positions = r_->node_in_way_idx_[node_idx_t{n}];

          // Copy to scratch + sort.
          scratch.clear();
          for (auto const [w, p] : utl::zip(ways, positions)) {
            scratch.emplace_back(w, p);
          }
          utl::sort(scratch, [&](auto const& a, auto const& b) {
            return a.first == b.first ? a.second < b.second
                                      : osm_id_less(way_osm_idx_[a.first],
                                                    way_osm_idx_[b.first]);
          });

          // Copy back from scratch.
          for (auto const [i, x] : utl::enumerate(scratch)) {
            ways[i] = x.first;
            positions[i] = x.second;
          }
        },
        pt->update_fn());
  }

  compute_turn_bearings();
}

std::size_t ways::get_polyline_node_idx(
    way_idx_t const way, std::uint16_t const target_routing_idx) const {
  auto const& routing_nodes = r_->way_nodes_[way];
  auto const& polyline_osm_nodes = way_osm_nodes_[way];

  auto current_routing_idx = 0U;
  for (auto const [poly_idx, osm_node] : utl::enumerate(polyline_osm_nodes)) {
    auto const expected_osm_node =
        node_to_osm_[routing_nodes[current_routing_idx]];
    if (osm_node == expected_osm_node) {
      if (current_routing_idx == target_routing_idx) {
        return poly_idx;
      }
      ++current_routing_idx;
    }
  }

  throw utl::fail("polyline node index not found: way={} idx={}", to_idx(way),
                  target_routing_idx);
}

void ways::compute_turn_bearings() {
  for (auto i = node_idx_t{0U}; i != n_nodes(); ++i) {
    auto bearings = r_->node_turn_bearings_.add_back_sized(0U);
    for (auto const [way, node_in_way_idx] :
         utl::zip(r_->node_ways_[i], r_->node_in_way_idx_[i])) {
      auto const polyline = way_polylines_[way];
      auto const polyline_idx = get_polyline_node_idx(way, node_in_way_idx);
      bearings.push_back(
          turn_bearing{.to_prev_ = get_prev_bearing(polyline, polyline_idx),
                       .to_next_ = get_next_bearing(polyline, polyline_idx)});
    }
  }
}

void ways::sync() {
  node_to_osm_.mmap_.sync();
  osm_node_order_.mmap_.sync();
  way_osm_idx_.mmap_.sync();
  osm_way_order_.mmap_.sync();
  way_polylines_.data_.mmap_.sync();
  way_polylines_.bucket_starts_.mmap_.sync();
  way_osm_nodes_.data_.mmap_.sync();
  way_osm_nodes_.bucket_starts_.mmap_.sync();
  strings_.data_.mmap_.sync();
  strings_.bucket_starts_.mmap_.sync();
  way_names_.mmap_.sync();
}

std::optional<std::string_view> ways::get_access_restriction(
    way_idx_t const way) const {
  if (!way_has_conditional_access_no_.test(way)) {
    return std::nullopt;
  }
  auto const it = std::lower_bound(
      begin(way_conditional_access_no_), end(way_conditional_access_no_), way,
      [](auto&& a, auto&& b) { return a.first < b; });
  utl::verify(
      it != end(way_conditional_access_no_) && it->first == way,
      "access restriction for way with access restriction not found way={}",
      way_osm_idx_[way]);
  return strings_[it->second].view();
}

cista::wrapped<ways::routing> ways::routing::read(
    std::filesystem::path const& p) {
  return cista::read<ways::routing>(p / "routing.bin");
}

void ways::routing::write(std::filesystem::path const& p) const {
  return cista::write(p / "routing.bin", *this);
}

}  // namespace osr
