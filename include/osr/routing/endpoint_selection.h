#pragma once

#include <cstdint>
#include <array>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <vector>

#include "utl/helpers/algorithm.h"
#include "utl/to_vec.h"

#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/ways.h"

namespace osr {

using component_key_t = std::uint64_t;

constexpr auto const kNoComponentKey =
    std::numeric_limits<component_key_t>::max();

using class_keys = std::array<component_key_t, kNumComponentClasses>;

// Ids are ranked by size and single-way keys come after all ids, so every key
// from the returned one on is tiny. The largest component (id 1) is never
// tiny: in a small extract, it may have fewer ways.
inline class_keys first_tiny_keys(ways const& w,
                                  std::uint32_t const tiny_ways) {
  auto first = class_keys{};
  for (auto const c : kAllComponentClasses) {
    first[static_cast<std::size_t>(c)] =
        tiny_ways <= 1U
            ? kNoComponentKey
            : std::max(component_key_t{2U},
                       component_key_t{
                           w.r_->get_class_components(c).first_smaller_than(
                               tiny_ways)});
  }
  return first;
}

// Returns whether any class of `P` can use the way.
template <Profile P, typename Fn>
bool for_each_component(ways const& w,
                        match_view_t const& m,
                        std::size_t const i,
                        route_end const end,
                        Fn&& fn) {
  auto const classes = P::endpoint_component_classes(end, m.state_[i]);
  auto const& props = w.r_->way_properties_[m.way_[i]];
  auto any = false;
  for (auto const c : kAllComponentClasses) {
    if (classes.contains(c) && is_accessible(c, props)) {
      any = true;
      fn(c, w.r_->get_class_components(c).key(m.way_[i]));
    }
  }
  return any;
}

template <Profile P>
bool has_several_components(ways const& w,
                            match_view_t const& m,
                            route_end const end) {
  auto first = class_keys{};
  first.fill(kNoComponentKey);
  auto several = false;
  for (auto i = std::size_t{0U}; i != m.size(); ++i) {
    for_each_component<P>(
        w, m, i, end, [&](component_class const c, component_key_t const k) {
          auto& f = first[static_cast<std::size_t>(c)];
          if (f == kNoComponentKey) {
            f = k;
          } else if (f != k) {
            several = true;
          }
        });
  }
  return several;
}

struct endpoint_component {
  component_class class_;
  component_key_t key_;
  float dist_;
  bool tiny_;
};

struct match_components {
  endpoint_component const* nearest(component_class const c) const {
    auto const it = utl::find_if(components_, [&](endpoint_component const& x) {
      return x.class_ == c;
    });
    return it == end(components_) ? nullptr : &*it;
  }

  // Foot and bike only switch to a component at most the requested matching
  // distance farther away than the closest one. Car endpoints are often far
  // from roads: cars switch to any component the lookup found.
  endpoint_component const* nearest_big(component_class const c) const {
    auto const n = nearest(c);
    auto const max_dist = c == component_class::kCar
                              ? std::numeric_limits<float>::infinity()
                              : n->dist_ + max_switch_;
    auto const it = utl::find_if(components_, [&](endpoint_component const& x) {
      return x.class_ == c && !x.tiny_ && x.dist_ <= max_dist;
    });
    return it == end(components_) ? nullptr : &*it;
  }

  std::vector<endpoint_component> components_;  // by closest candidate
  float max_switch_{std::numeric_limits<float>::infinity()};
};

template <Profile P>
match_components get_match_components(ways const& w,
                                      class_keys const& first_tiny,
                                      match_view_t const& m,
                                      route_end const end) {
  auto mc =
      match_components{.components_ = {}, .max_switch_ = m.max_match_distance_};
  for (auto i = std::size_t{0U}; i != m.size(); ++i) {
    for_each_component<P>(
        w, m, i, end, [&](component_class const c, component_key_t const k) {
          if (utl::none_of(mc.components_, [&](endpoint_component const& x) {
                return x.class_ == c && x.key_ == k;
              })) {
            mc.components_.push_back(
                {.class_ = c,
                 .key_ = k,
                 .dist_ = m.dist_to_way_[i],
                 .tiny_ = k >= first_tiny[static_cast<std::size_t>(c)]});
          }
        });
  }
  return mc;
}

inline component_key_t choose_component(match_components const& e,
                                        match_components const& o,
                                        component_class const c) {
  auto const n = e.nearest(c);
  if (n == nullptr) {
    return kNoComponentKey;
  }
  auto const other = o.nearest(c);
  if (!n->tiny_ || (other != nullptr && other->key_ == n->key_)) {
    return n->key_;
  }
  auto const big = e.nearest_big(c);
  return big == nullptr ? n->key_ : big->key_;
}

struct selected_component {
  component_class class_;
  component_key_t key_;
};

using component_selection = std::vector<selected_component>;

inline void add_selected(component_selection& sel,
                         component_class const c,
                         component_key_t const k) {
  if (k != kNoComponentKey &&
      utl::none_of(sel, [&](selected_component const& x) {
        return x.class_ == c && x.key_ == k;
      })) {
    sel.push_back({.class_ = c, .key_ = k});
  }
}

// The components of different destinations are not connected: each
// destination is only reached from its own.
inline component_selection select_source(
    match_components const& s, std::span<match_components const> const ds) {
  auto sel = component_selection{};
  for (auto const c : kAllComponentClasses) {
    for (auto const& d : ds) {
      add_selected(sel, c, choose_component(s, d, c));
    }
  }
  return sel;
}

inline component_selection select_destination(match_components const& d,
                                              match_components const& s) {
  auto sel = component_selection{};
  for (auto const c : kAllComponentClasses) {
    add_selected(sel, c, choose_component(d, s, c));
  }
  return sel;
}

struct candidate_components {
  bool shares_component(candidate_components const& o) const {
    return utl::any_of(kAllComponentClasses, [&](component_class const c) {
      auto const ci = static_cast<std::size_t>(c);
      return selected_[ci] != kNoComponentKey &&
             selected_[ci] == o.selected_[ci];
    });
  }

  bool keep() const {
    return !classified_ || utl::any_of(selected_, [](component_key_t const k) {
      return k != kNoComponentKey;
    });
  }

  class_keys selected_;
  bool classified_;
};

template <Profile P>
candidate_components get_candidate_components(ways const& w,
                                              match_view_t const& m,
                                              std::size_t const i,
                                              route_end const end,
                                              component_selection const& sel) {
  auto out = candidate_components{};
  out.selected_.fill(kNoComponentKey);
  out.classified_ = for_each_component<P>(
      w, m, i, end, [&](component_class const c, component_key_t const k) {
        if (utl::any_of(sel, [&](selected_component const& x) {
              return x.class_ == c && x.key_ == k;
            })) {
          out.selected_[static_cast<std::size_t>(c)] = k;
        }
      });
  return out;
}

// Candidates on ways no class can use are kept. A source that keeps several
// components of a class measures the matching penalty in each one separately,
// as the route between the same two points would.
template <Profile P>
void append_selected(ways const& w,
                     match_view_t const& m,
                     route_end const match_end,
                     component_selection const& sel,
                     match_result& out) {
  auto const several =
      utl::any_of(kAllComponentClasses, [&](component_class const c) {
        return utl::count_if(sel, [&](selected_component const& x) {
                 return x.class_ == c;
               }) > 1;
      });
  auto kept = std::vector<std::size_t>{};
  auto components = std::vector<candidate_components>{};
  for (auto i = std::size_t{0U}; i != m.size(); ++i) {
    auto const cc = get_candidate_components<P>(w, m, i, match_end, sel);
    if (cc.keep()) {
      kept.push_back(i);
      components.push_back(cc);
    }
  }

  out.start(m.lvl_, m.max_match_distance_);
  for (auto k = std::size_t{0U}; k != kept.size(); ++k) {
    auto const i = kept[k];
    auto pool = std::views::iota(std::size_t{0U}, kept.size()) |
                std::views::filter([&](std::size_t const x) {
                  return !several || !components[k].classified_ ||
                         components[k].shares_component(components[x]);
                });
    auto const ground = std::ranges::find_if(pool, [&](std::size_t const x) {
      return !m.lvl_.has_level() &&
             touches_ground(w.r_->way_properties_[m.way_[kept[x]]]);
    });
    out.add(m.dist_to_way_[i], m.way_[i], m.nodes_[i], m.state_[i]);
    out.penalty_ref_.emplace_back(
        m.dist_to_way_[kept[ground != std::ranges::end(pool)
                                ? *ground
                                : *std::ranges::begin(pool)]]);
  }
  out.finish();
}

struct selected_matches {
  match_result from_;  // one match
  match_result to_;  // one match per destination
};

// nullopt: there is nothing to choose, all candidates stay.
template <Profile P>
std::optional<selected_matches> select_endpoints(
    ways const& w,
    match_view_t const& from,
    std::span<match_view_t const> const to,
    direction const dir,
    std::uint32_t const tiny_ways) {
  auto const from_end = route_end_of(dir);
  auto const to_end = route_end_of(opposite(dir));
  if (!has_several_components<P>(w, from, from_end) &&
      utl::none_of(to, [&](match_view_t const& m) {
        return has_several_components<P>(w, m, to_end);
      })) {
    return std::nullopt;
  }

  auto const first_tiny = first_tiny_keys(w, tiny_ways);
  auto const from_components =
      get_match_components<P>(w, first_tiny, from, from_end);
  auto const to_components = utl::to_vec(to, [&](match_view_t const& m) {
    return get_match_components<P>(w, first_tiny, m, to_end);
  });

  auto out = selected_matches{};
  append_selected<P>(w, from, from_end,
                     select_source(from_components, to_components), out.from_);
  for (auto i = std::size_t{0U}; i != to.size(); ++i) {
    append_selected<P>(w, to[i], to_end,
                       select_destination(to_components[i], from_components),
                       out.to_);
  }
  return out;
}

template <Profile P>
std::optional<selected_matches> select_endpoints(
    ways const& w,
    match_view_t const& from,
    match_result const& to,
    direction const dir,
    std::uint32_t const tiny_ways) {
  auto const views = utl::to_vec(
      std::views::iota(match_idx_t::value_t{0U},
                       static_cast<match_idx_t::value_t>(to.size())),
      [&](match_idx_t::value_t const i) { return to[match_idx_t{i}]; });
  return select_endpoints<P>(w, from, views, dir, tiny_ways);
}

}  // namespace osr
