#pragma once

#include <cstdint>

#include <concepts>
#include <functional>
#include <ostream>
#include <string_view>

#include "osr/elevation_storage.h"
#include "osr/preprocessing/contraction_hierarchy/shortcut_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <typename Parameters, typename Profile>
concept IsParameters =
    std::is_default_constructible_v<Parameters> &&
    std::is_same_v<typename Parameters::profile_t, Profile> &&
    std::is_same_v<Parameters, typename Profile::parameters>;

template <typename Node, typename NodeKey>
concept IsNode = requires {
  { Node::invalid() } -> std::same_as<Node>;
} && requires(Node const& node, std::ostream& out, ways const& w) {
  { node.get_key() } -> std::same_as<NodeKey>;
  { node.get_node() } -> std::same_as<node_idx_t>;
  { node.get_mode() } -> std::same_as<mode>;
  { node == node } -> std::same_as<bool>;
  { node.print(out, w) } -> std::same_as<decltype(out)>;
};

template <typename Label, typename Node>
concept IsLabel =
    requires {
      { Label(std::declval<Node>(), std::declval<cost_t>()) };
    } &&
    requires(Label const& label) {
      { label.get_node() } -> std::same_as<Node>;
      { label.cost() } -> std::same_as<cost_t>;
    } &&
    requires(Label label,
             Label const& l,
             ways::routing const r,
             way_idx_t const w,
             node_idx_t const n) {
      { label.track(l, r, w, n, std::declval<bool>()) } -> std::same_as<void>;
    };

template <typename Entry, typename Node, typename Label, typename NodeKey>
concept IsEntry =
    requires(Entry const& entry, Node const node, path& p) {
      { entry.pred(node) } -> std::same_as<std::optional<Node>>;
      { entry.way(node) } -> std::same_as<way_idx_t>;
      { entry.cost(node) } -> std::same_as<cost_t>;
      { entry.write(node, p) } -> std::same_as<void>;
    } &&
    requires(Entry entry,
             Node const node,
             Label const& label,
             cost_t const cost,
             way_idx_t const way,
             path& p) {
      { entry.update(label, node, cost, node, way) } -> std::same_as<bool>;
    } &&
    requires(Entry entry,
             NodeKey key,
             std::function<void(Node node, cost_t cost)> fn) {
      { entry.for_each(key, fn) } -> std::same_as<void>;
    };

template <typename Hash, typename NodeKey>
concept IsHash = requires(Hash const& h, NodeKey key) {
  { h(key) };
};

template <typename P>
concept Profile =
    IsParameters<typename P::parameters, P> &&
    IsNode<typename P::node, typename P::key> &&
    IsLabel<typename P::label, typename P::node> &&
    IsEntry<typename P::entry,
            typename P::node,
            typename P::label,
            typename P::key> &&
    IsHash<typename P::hash, typename P::key> &&
    requires(typename P::node const node,
             ways::routing const& r,
             way_idx_t const w,
             node_idx_t const node_idx,
             level_t const lvl,
             direction const dir,
             std::function<void(typename P::node const)>&& f) {
      {
        P::resolve_start_node(r, w, node_idx, lvl, dir, f)
      } -> std::same_as<void>;
      { P::resolve_all(r, node_idx, lvl, f) } -> std::same_as<void>;
    } &&
    requires(typename P::parameters const& params,
             typename P::node const node,
             ways::routing const& r,
             way_idx_t const w,
             direction const dir,
             node_properties const n_props,
             way_properties const w_props,
             double const dist) {
      {
        P::is_dest_reachable(params, r, node, w, dir, dir)
      } -> std::same_as<bool>;
      {
        P::way_cost(params, w_props, dir, std::declval<std::uint16_t>())
      } -> std::same_as<cost_t>;
      { P::node_cost(n_props) } -> std::same_as<cost_t>;
      { P::heuristic(params, dist) } -> std::same_as<double>;
      { P::get_reverse(node) } -> std::same_as<typename P::node>;
    } &&
    requires(typename P::parameters const& params,
             typename P::node const n,
             ways::routing const& r,
             bitvec<node_idx_t> const* blocked,
             sharing_data const* sharing,
             elevation_storage const* elevation,
             shortcut_storage<typename P::node> const* sc_stor,
             node_idx_t min_order,
             std::function<void(typename P::node const,
                                std::uint32_t const,
                                distance_t,
                                way_idx_t const,
                                std::uint16_t,
                                std::uint16_t,
                                elevation_storage::elevation,
                                bool const,
                                typename P::node const,
                                cost_t)> f) {
      {
        P::template adjacent<direction::kBackward, adj_conf::kNone>(
            params, r, n, blocked, sharing, elevation, sc_stor, min_order, f)
      } -> std::same_as<void>;
    };

template <typename Parameters>
concept ProfileParameters = Profile<typename Parameters::profile_t>;

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kBikeFast,
  kBikeElevationLow,
  kBikeElevationHigh,
  kCar,
  kCarDropOff,
  kCarDropOffWheelchair,
  kCarParking,
  kCarParkingWheelchair,
  kBikeSharing,
  kCarSharing,
};

constexpr auto const kNumProfiles =
    static_cast<std::underlying_type_t<search_profile>>(10U);

search_profile to_profile(std::string_view);

std::string_view to_str(search_profile);

bool is_rental_profile(search_profile);

}  // namespace osr
