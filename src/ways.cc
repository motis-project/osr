#include "osr/ways.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <span>
#include <thread>
#include <tuple>

#include "date/tz.h"

#include "utl/helpers/algorithm.h"
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

bool is_impassable(ways::routing const& r,
                   component_class const cls,
                   node_idx_t const n) {
  auto const p = r.node_properties_[n];
  switch (cls) {
    case component_class::kFoot:
      return !p.is_walk_accessible() && !p.has_barrier_cost();
    case component_class::kBike:
      return !p.is_bike_accessible() && !p.has_barrier_cost();
    case component_class::kCar: return !p.is_car_accessible();
  }
  return false;
}

// Union-find elements: one per way (its first piece), then one per further
// piece of a way split at impassable nodes (appended to `pieces`).
std::vector<std::uint32_t> build_component_forest(
    ways::routing const& r,
    component_class const cls,
    std::vector<class_components::piece>& pieces) {
  auto const n_ways = static_cast<std::uint32_t>(r.way_properties_.size());
  auto first_piece = std::vector<std::uint32_t>(n_ways, kNoComponent);
  for (auto i = 0U; i != n_ways; ++i) {
    auto const way = way_idx_t{i};
    if (!is_accessible(cls, r.way_properties_[way])) {
      continue;
    }
    auto const nodes = r.way_nodes_[way];
    for (auto pos = 0U; pos != nodes.size(); ++pos) {
      if (is_impassable(r, cls, nodes[pos])) {
        if (first_piece[i] == kNoComponent) {
          first_piece[i] = static_cast<std::uint32_t>(pieces.size());
        }
        pieces.push_back({way, static_cast<std::uint16_t>(pos + 1U)});
      }
    }
  }

  auto parent =
      std::vector<std::uint32_t>(n_ways + pieces.size(), kNoComponent);
  for (auto i = 0U; i != parent.size(); ++i) {
    if (i >= n_ways || is_accessible(cls, r.way_properties_[way_idx_t{i}])) {
      parent[i] = i;
    }
  }

  // Element of the piece that contains the (passable) way node at `pos`.
  auto const element = [&](way_idx_t const way, std::uint16_t const pos) {
    auto e = to_idx(way);
    for (auto k = first_piece[to_idx(way)];
         k != pieces.size() && pieces[k].way_ == way && pieces[k].from_ <= pos;
         ++k) {
      e = n_ways + k;
    }
    return e;
  };

  for (auto n = node_idx_t{0U}; n != r.node_ways_.size(); ++n) {
    if (is_impassable(r, cls, n)) {
      continue;
    }
    auto anchor = kNoComponent;
    auto const node_ways = r.node_ways_[n];
    for (auto i = 0U; i != node_ways.size(); ++i) {
      auto const w = node_ways[i];
      if (parent[to_idx(w)] == kNoComponent) {
        continue;
      }
      auto const e = first_piece[to_idx(w)] == kNoComponent
                         ? to_idx(w)
                         : element(w, r.node_in_way_idx_[n][i]);
      auto const root = find_component_root(parent, e);
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

// A way counts once per component, however many of its pieces lie in it.
void count_split_ways_once(
    std::vector<std::uint32_t> const& comp,
    std::span<class_components::piece const> const pieces,
    std::vector<std::uint32_t>& sizes) {
  auto const n_ways = comp.size() - pieces.size();
  auto seen = std::vector<std::uint32_t>{};
  for (auto k = 0U; k != pieces.size(); ++k) {
    auto const way = pieces[k].way_;
    if (k == 0U || pieces[k - 1U].way_ != way) {
      seen.clear();
      seen.push_back(comp[to_idx(way)]);
    }
    auto const c = comp[n_ways + k];
    if (utl::find(seen, c) != end(seen)) {
      --sizes[c];
    } else {
      seen.push_back(c);
    }
  }
}

}  // namespace

class_components compute_class_components(ways::routing const& r,
                                          component_class const cls) {
  auto pieces = std::vector<class_components::piece>{};
  auto parent = build_component_forest(r, cls, pieces);
  auto sizes = number_component_roots(parent);
  count_split_ways_once(parent, pieces, sizes);
  return class_components::build(parent, sizes, pieces);
}

class_components class_components::build(
    std::span<std::uint32_t const> const elements,
    std::span<std::uint32_t const> const sizes,
    std::span<piece const> const pieces) {
  auto const comp = elements.subspan(0U, elements.size() - pieces.size());
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
  auto out = class_components{};
  for (auto i = std::size_t{0U}; i != ranked.size(); ++i) {
    auto const id = static_cast<std::uint32_t>(i + 1U);
    auto const size = sizes[ranked[i]];
    ids[ranked[i]] = id;
    if (out.size_run_size_.empty() || out.size_run_size_.back() != size) {
      out.size_run_id_.push_back(id);
      out.size_run_size_.push_back(size);
    }
  }

  out.way_component_.resize(static_cast<std::uint32_t>(comp.size()),
                            kNoComponent);
  auto exception_ways = std::uint64_t{0U};
  for (auto i = std::size_t{kException - 1U}; i < ranked.size(); ++i) {
    exception_ways += sizes[ranked[i]];
  }
  utl::verify(exception_ways <= comp.size(), "invalid component sizes");
  auto const max_exceptions =
      static_cast<std::uint32_t>(exception_ways + 2U * pieces.size());
  out.exception_way_.reserve(max_exceptions);
  out.exception_from_.reserve(max_exceptions);
  out.exception_component_.reserve(max_exceptions);
  auto const add_exception = [&](way_idx_t const way, std::uint16_t const from,
                                 std::uint32_t const id) {
    out.exception_way_.push_back(way);
    out.exception_from_.push_back(from);
    out.exception_component_.push_back(id);
  };

  auto k = std::size_t{0U};
  for (auto i = std::size_t{0U}; i != comp.size(); ++i) {
    auto const c = comp[i];
    if (c == std::numeric_limits<std::uint32_t>::max()) {
      continue;
    }
    utl::verify(c < sizes.size(), "invalid component id {}", c);
    auto const id = ids[c];
    auto const way = way_idx_t{static_cast<std::uint32_t>(i)};
    if (k != pieces.size() && pieces[k].way_ == way) {
      out.way_component_[way] = kException;
      add_exception(way, 0U, id);
      for (; k != pieces.size() && pieces[k].way_ == way; ++k) {
        add_exception(way, pieces[k].from_, ids[elements[comp.size() + k]]);
      }
    } else if (id < kException) {
      out.way_component_[way] = static_cast<std::uint8_t>(id);
    } else {
      out.way_component_[way] = kException;
      add_exception(way, 0U, id);
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
      way_osm_idx_{mm("way_osm_idx.bin")},
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
          if (x.type_ == resolved_restriction::type::kNo) {
            r_->node_restrictions_[x.via_].push_back(restriction{
                r_->get_way_pos(x.via_, x.from_),
                r_->get_way_pos(x.via_, x.to_), x.applies_to_default_,
                x.applies_to_bus_, x.applies_to_hgv_, x.condition_set_});
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

void ways::connect_ways() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  {  // Assign graph node ids to every node with >1 way.
    pt->status("Create graph nodes")
        .in_high(node_way_counter_.size())
        .out_bounds(40, 50);

    auto node_idx = node_idx_t{0U};
    node_way_counter_.for_each_multi([&](std::uint64_t const b_idx) {
      auto const i = osm_node_idx_t{b_idx};
      node_to_osm_.push_back(i);
      ++node_idx;
      pt->update(b_idx);
    });
    r_->node_is_restricted_.resize(to_idx(node_idx));
  }

  // Build edges.
  {
    auto const n_nodes = static_cast<std::size_t>(node_to_osm_.size());
    auto const n_ways = static_cast<std::size_t>(way_osm_nodes_.size());

    pt->status("Connect ways").in_high(n_ways).out_bounds(50, 71);

    // Phase 1: Count number of ways per node (if way visits *N -> counted *N).
    // Phase 2: Reserve node_ways_ / node_in_way_idx_ / node_turn_bearings_
    //          and reset counts to 0.
    // Phase 3: Fill node_ways_ / node_in_way_idx_ / node_turn_bearings_ =
    //          transpose of way_nodes_ / way_bearings.
    //          -> count becomes a fill cursor per node
    auto count = std::vector<std::atomic<std::uint32_t>>(n_nodes);
    auto way_bearings = vecvec<way_idx_t, turn_bearing>{};

    struct way_edges {
      std::vector<node_idx_t> nodes_;
      std::vector<turn_bearing> bearings_;
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
            auto const polyline = way_polylines_[way_idx];
            for (auto const [poly_idx, osm_node_idx] :
                 utl::enumerate(way_osm_nodes_[way_idx])) {
              auto const pos = polyline[poly_idx];
              if (pred_pos.has_value()) {
                distance += geo::distance(pos, *pred_pos);
              }

              if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
                auto const to = get_node_idx(osm_node_idx);
                auto const rank =
                    count[to_idx(to)].fetch_add(1U, std::memory_order_relaxed);
                if (rank >= kMaxWaysPerNode) {
                  pred_pos = pos;
                  continue;
                }

                c.nodes_.push_back(to);
                c.bearings_.push_back(
                    {.to_prev_ = get_prev_bearing(polyline, poly_idx),
                     .to_next_ = get_next_bearing(polyline, poly_idx)});

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
            way_bearings.emplace_back(c.bearings_);
            r_->way_node_dist_.emplace_back(c.dists_);
            for (auto const& x : c.long_dists_) {
              r_->long_way_node_dist_.push_back(x);
            }
          }
        },
        [&](std::size_t) { pt->increment(kConnectChunkSize); });

    utl::sort(r_->long_way_node_dist_);

    pt->status("Connect ways / transpose").in_high(n_ways).out_bounds(71, 74);

    // Reserve node_ways_ / node_in_way_idx_ / node_turn_bearings_ per node.
    for (auto n = std::size_t{0U}; n != n_nodes; ++n) {
      auto const size =
          std::min(count[n].exchange(0U, std::memory_order_relaxed),
                   static_cast<std::uint32_t>(kMaxWaysPerNode));
      r_->node_ways_.add_back_sized(size);
      r_->node_in_way_idx_.add_back_sized(size);
      r_->node_turn_bearings_.add_back_sized(size);
    }

    // Fill node_ways_ / node_in_way_idx_ / node_turn_bearings_ = transpose of
    // way_nodes_ / way_bearings.
    // Threads claims their write slot via the atomic per-node cursor.
    utl::parallel_for_run(
        n_ways,
        [&](std::size_t const way) {
          auto i = std::uint16_t{0U};
          for (auto const [n, bearing] :
               utl::zip(r_->way_nodes_[way_idx_t{way}],
                        way_bearings[way_idx_t{way}])) {
            auto const pos =
                count[to_idx(n)].fetch_add(1U, std::memory_order_relaxed);
            r_->node_ways_[n][pos] = way_idx_t{way};
            r_->node_in_way_idx_[n][pos] = i++;
            r_->node_turn_bearings_[n][pos] = bearing;
          }
        },
        pt->update_fn());
    way_bearings = {};

    // Sort node_ways_ / node_in_way_idx_ / node_turn_bearings_ by
    // (way_idx, node_in_way_idx) for deterministic ordering.
    struct node_way {
      way_idx_t way_;
      std::uint16_t in_way_idx_;
      turn_bearing bearing_;
    };
    pt->status("Connect ways / sort").in_high(n_nodes).out_bounds(74, 75);
    utl::parallel_for_run_threadlocal<std::vector<node_way>>(
        n_nodes,
        [&](std::vector<node_way>& scratch, std::size_t const n) {
          auto ways = r_->node_ways_[node_idx_t{n}];
          auto positions = r_->node_in_way_idx_[node_idx_t{n}];
          auto bearings = r_->node_turn_bearings_[node_idx_t{n}];

          // Copy to scratch + sort.
          scratch.clear();
          for (auto const [w, p, b] : utl::zip(ways, positions, bearings)) {
            scratch.push_back({w, p, b});
          }
          utl::sort(scratch, [](node_way const& a, node_way const& b) {
            return std::tie(a.way_, a.in_way_idx_) <
                   std::tie(b.way_, b.in_way_idx_);
          });

          // Copy back from scratch.
          for (auto const [i, x] : utl::enumerate(scratch)) {
            ways[i] = x.way_;
            positions[i] = x.in_way_idx_;
            bearings[i] = x.bearing_;
          }
        },
        pt->update_fn());
  }

  node_way_counter_ = {};
}

void ways::sync() {
  node_to_osm_.mmap_.sync();
  way_osm_idx_.mmap_.sync();
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
