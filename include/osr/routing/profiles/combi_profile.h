#pragma once

#include <variant>

#include "osr/types.h"
#include "osr/ways.h"

#include "osr/routing/profiles/transitions.h"
#include "osr/routing/profiles/car.h"

template <class T, T ... Is, class F>
auto compile_switch(T i, std::integer_sequence<T, Is...>, F f) {
  using return_type = std::common_type_t<decltype(f(std::integral_constant<T, Is>{}))...>;

  if constexpr (std::is_void_v<return_type>) {
    std::initializer_list<int> ({(i == Is ? (f(std::integral_constant<T, Is>{}), 0) : 0)...});
  } else {
    return_type ret;
    std::initializer_list<int> ({(i == Is ? (ret = f(std::integral_constant<T, Is>{})), 0 : 0)...});
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconditional-uninitialized"
    return ret; // NOLINT(clang-diagnostic-conditional-uninitialized): this will be initialized in the fold expression
    #pragma GCC diagnostic pop
  }
}

template <typename Variant, typename Visitor>
constexpr auto visit_with_index(Variant&& variant, Visitor&& visitor) {
  return compile_switch(variant.index(), std::make_index_sequence<std::variant_size_v<std::decay_t<Variant>>>{}, [&](auto i) {
    return visitor(std::get<i>(variant), i);
  });
}

template<typename T>
struct profile_types;

template<typename... Ts>
struct profile_types<std::tuple<Ts...>> {
  using profile_node = std::variant<typename Ts::node...>;
  using profile_key = std::variant<typename Ts::key...>;
  using profile_hash_tuple = std::tuple<typename Ts::hash...>;
  using profile_entry_tuple = std::tuple<typename Ts::entry...>;
};

namespace osr {

template <typename profile_tuple, typename transition_tuple>
struct combi_profile {
  static_assert(std::tuple_size_v<profile_tuple> > 0, "At least one profile must be provided.");
  static_assert(std::tuple_size_v<profile_tuple> - 1 == std::tuple_size_v<transition_tuple>, "You must provide one transition less than the number of profiles.");

  using first_profile = std::tuple_element_t<0, profile_tuple>;
  using last_profile = std::tuple_element_t<std::tuple_size_v<profile_tuple> - 1, profile_tuple>;
  using last_profile_index = std::integral_constant<size_t, std::tuple_size_v<profile_tuple> - 1>;

  using profile_node = typename profile_types<profile_tuple>::profile_node;

  using key = node_idx_t;

  struct node {
    friend bool operator==(node, node) = default;

    boost::json::object geojson_properties(ways const& w) const {
      return std::visit([&](auto const& n) { return n.geojson_properties(w); }, n_);
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return std::visit([&](auto const& n) -> std::ostream& { return n.print(out, w); }, n_);
    }

    constexpr node_idx_t get_node() const noexcept {
        return std::visit([](auto const& n) { return n.get_node(); }, n_);
    }

    constexpr key get_key() const noexcept {
      return std::visit([](auto const& n) { return n.get_node(); }, n_);
    }

    profile_node n_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(to_idx(n));
    }
  };

  struct label {
    label(node const n, cost_t const c) : n_{n}, cost_{c} {}

    constexpr node get_node() const noexcept { return n_; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(label const&, ways::routing const&, way_idx_t, node_idx_t) {}

    node n_;
    cost_t cost_;
  };


  struct entry {

    template<typename T>
    constexpr size_t get_offset(typename T::node const& n) const noexcept {
      if constexpr (requires(const T& t) { T::get_index(n); }) {
        return T::get_index(n);
      } else {
        return 0;
      }
    }

    constexpr size_t get_index(node const n) const noexcept {
      return visit_with_index(n.n_, [&](auto const& profile_node_, auto const index) -> size_t {
        return get_offset<std::tuple_element_t<index, profile_tuple>>(profile_node_);
      });
    }

    constexpr cost_t cost(node const n) const noexcept {
      return visit_with_index(n.n_, [&](auto const& profile_node_, auto const index) -> cost_t {
        auto const& entry = std::get<index>(entries_);
        return entry.cost(profile_node_);
      });
    }

    template<direction SearchDir>
    constexpr bool update(label const& l,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      return visit_with_index(n.n_, [&](auto const& profile_node_, auto const index) -> bool {
        auto& entry = std::get<index>(entries_);

        if (index == pred.n_.index()) {

          return entry.template update<SearchDir>(
            {std::get<index>(l.n_.n_), l.cost_},
            profile_node_,
            c,
            std::get<index>(pred.n_)
          );
        }


        auto updated = entry.template update<SearchDir>(
            {std::tuple_element_t<index, profile_tuple>::template get_starting_node_pred<SearchDir>(), l.cost_},
            profile_node_,
            c,
            std::tuple_element_t<index, profile_tuple>::template get_starting_node_pred<SearchDir>()
          );

        if (updated) {
          auto const transition_index = (index - 1) * 32 + get_offset<std::tuple_element_t<index, profile_tuple>>(profile_node_);
          pred_[transition_index] = pred;
        }

        return updated;
      });
    }

    constexpr std::optional<node> pred(node const n, direction const dir) const noexcept {

      return visit_with_index(n.n_, [&](auto const& profile_node_, auto const index) -> std::optional<node> {

        auto const& entry = std::get<index>(entries_);

        if (auto const& p = entry.pred(profile_node_, dir)) {
          return node{profile_node{std::in_place_index<index>, *p}};
        }

        if ((dir == direction::kForward && index == 0) || (dir == direction::kBackward && index == last_profile_index::value)) {
          return std::nullopt;
        } else {
          auto const transition_index = (index - 1) * 32 + get_offset<std::tuple_element_t<index, profile_tuple>>(profile_node_);
          return pred_[transition_index];
        }
      });
    }

    void write(node, path&) const {}

    using profile_entry_tuple = typename profile_types<profile_tuple>::profile_entry_tuple;
    profile_entry_tuple entries_ = profile_entry_tuple{};
    std::array<node, (std::tuple_size_v<profile_tuple> - 1) * 32> pred_;
  };

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t const level,
                                 direction const dir,
                                 Fn&& f) {
    if (dir == direction::kForward) {
      first_profile::resolve_start_node(w, way, n, level, dir, [&](typename first_profile::node first_profile_node) {
        f(node{profile_node{std::in_place_index<0>, first_profile_node}});
      });
    } else {
      last_profile::resolve_start_node(w, way, n, level, dir, [&](typename last_profile::node last_profile_node) {
        f(node{profile_node{std::in_place_index<last_profile_index::value>, last_profile_node}});
      });
    }
  }

  template<direction SearchDir>
  static constexpr node get_starting_node_pred() noexcept {
    if (SearchDir == direction::kForward) {
      return node{profile_node{std::in_place_index<0>, first_profile::template get_starting_node_pred<SearchDir>()}};
    } else {
      return node{profile_node{std::in_place_index<last_profile_index::value>, last_profile::template get_starting_node_pred<SearchDir>()}};
    }
  }

  template <typename Fn>
  static void resolve_all(const ways::routing& w,
                          node_idx_t n,
                          level_t level,
                          Fn&& fn) {
    [&]<std::size_t... I>(std::index_sequence<I...>) {
      (std::tuple_element_t<I, profile_tuple>::resolve_all(w, n, level, [&](auto const& profile_node_) {
        fn(node{profile_node{std::in_place_index<I>, std::forward<decltype(profile_node_)>(profile_node_)}});
      }), ...);
    }(std::make_index_sequence<std::tuple_size_v<profile_tuple>>{});


  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
   static void adjacent(ways::routing const& w,
                        node const n,
                        bitvec<node_idx_t> const* blocked,
                        Fn&& fn) {
    visit_with_index(n.n_, [&](auto const& profile_node_, auto const index) {
      using NodeType = std::decay_t<decltype(profile_node_)>;
      using Profile = std::tuple_element_t<index, profile_tuple>;
      Profile::template adjacent<SearchDir, WithBlocked>(w, profile_node_, blocked, [&](
        NodeType adjacent_profile_node,
        std::uint32_t const cost,
        distance_t const dist,
        way_idx_t const way,
        std::uint16_t const from,
        std::uint16_t const to
      ) {
        fn(node{profile_node{std::in_place_index<index>, adjacent_profile_node}}, cost, dist, way, from, to);
      });

      if constexpr (SearchDir == direction::kBackward && index > 0) {
        using transition = std::tuple_element_t<index - 1, transition_tuple>;

        if (transition::operator()(w, n.get_node(), blocked)) {
          using NextProfile = std::tuple_element_t<index - 1, profile_tuple>;
          using NextProfileNode = typename NextProfile::node;

          if (NextProfile::node_cost(w.node_properties_[n.get_node()]) == kInfeasible) {
            return;
          }

          NextProfile::template resolve_all(w, n.get_node(), w.node_properties_[n.get_node()].from_level(), [&](NextProfileNode const& next_profile_node) {
            NextProfile::template adjacent<SearchDir, WithBlocked>(w, next_profile_node, blocked, [&](
              NextProfileNode adjacent_profile_node,
              std::uint32_t const cost,
              distance_t const dist,
              way_idx_t const way,
              std::uint16_t const from,
              std::uint16_t const to
            ) {
              fn(node{profile_node{std::in_place_index<index - 1>, adjacent_profile_node}}, cost, dist, way, from, to);
            });
          });
        }
      } else if constexpr (SearchDir == direction::kForward && index < last_profile_index::value) {
        using transition = std::tuple_element_t<index, transition_tuple>;

        if (transition::operator()(w, n.get_node(), blocked)) {
          using NextProfile = std::tuple_element_t<index + 1, profile_tuple>;
          using NextProfileNode = typename NextProfile::node;

          if (NextProfile::node_cost(w.node_properties_[n.get_node()]) == kInfeasible) {
            return;
          }

          NextProfile::template resolve_all(w, n.get_node(), w.node_properties_[n.get_node()].from_level(), [&](NextProfileNode const& next_profile_node) {
            NextProfile::template adjacent<SearchDir, WithBlocked>(w, next_profile_node, blocked, [&](
              NextProfileNode adjacent_profile_node,
              std::uint32_t const cost,
              distance_t const dist,
              way_idx_t const way,
              std::uint16_t const from,
              std::uint16_t const to
            ) {
              fn(node{profile_node{std::in_place_index<index + 1>, adjacent_profile_node}}, cost, dist, way, from, to);
            });
          });
        }
      }

    });
  }

  static bool is_dest_reachable(ways::routing const& w,
                              node const n,
                              way_idx_t const way,
                              direction const way_dir,
                              direction const search_dir) {
    if (search_dir == direction::kForward) {
      if (n.n_.index() > 0) {
        return false;
      }

      return first_profile::is_dest_reachable(w, std::get<0>(n.n_), way, way_dir, search_dir);
    }

    if (n.n_.index() < last_profile_index::value) {
      return false;
    }

    return last_profile::is_dest_reachable(w, std::get<last_profile_index::value>(n.n_), way, way_dir, search_dir);
  }


  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   direction const search_dir,
                                   std::uint16_t const dist) {
    if (search_dir == direction::kForward) {
      auto l = first_profile::way_cost(e, dir, search_dir, dist);
      return l;
    }

    return last_profile::way_cost(e, dir, search_dir, dist);
  }
};

using combi_foot_car_foot_profile = combi_profile<std::tuple<foot<false>, car, foot<false>>, std::tuple<transitions::is_parking, transitions::is_parking>>;
}