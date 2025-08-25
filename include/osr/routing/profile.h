#pragma once

#include <cinttypes>
#include <concepts>
#include <string_view>

#include "osr/elevation_storage.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <typename Node, typename NodeValue>
concept IsNodeBase = requires(Node node, NodeValue value) {
  { node.get_node() } -> std::same_as<NodeValue>;
};

template <typename Node, typename NodeKey>
concept IsNode = IsNodeBase<Node, osr::node_idx_t> && requires(Node node) {
  { node.get_key() } -> std::same_as<NodeKey>;
};

template <typename Label, typename Node>
concept IsLabel = IsNodeBase<Label, Node> && requires(Label label) {
  { label.cost() } -> std::same_as<osr::cost_t>;
};

template <typename Entry, typename Node, typename Label>
concept IsEntry =
    requires(Entry entry, Node node, Label label, osr::cost_t cost) {
      { entry.pred(node) } -> std::same_as<std::optional<Node>>;
      { entry.cost(node) } -> std::same_as<osr::cost_t>;
      { entry.update(label, node, cost, node) } -> std::same_as<bool>;
    };

template <typename Profile, typename F = decltype([] {})>
concept IsProfile =
    IsNode<typename Profile::node, typename Profile::key> &&
    IsLabel<typename Profile::label, typename Profile::node> &&
    IsEntry<typename Profile::entry,
            typename Profile::node,
            typename Profile::label> &&
    std::invocable<decltype(Profile::template resolve_start_node<F>),
                   ways::routing,
                   way_idx_t,
                   node_idx_t,
                   level_t,
                   direction,
                   F> &&
    std::invocable<decltype(Profile::template resolve_all<F>),
                   ways::routing,
                   node_idx_t,
                   level_t,
                   F> &&
    // std::invocable<decltype(T::way_cost)> &&
    std::invocable<
        decltype(Profile::
                     template adjacent<osr::direction::kBackward, true, F>),
        osr::ways::routing const,
        typename Profile::node const,
        bitvec<node_idx_t> const*,
        sharing_data const*,
        elevation_storage const*,
        F,
        routing_parameters> &&
    requires(Profile p) {
      {
        Profile::way_cost(std::declval<way_properties>(),
                          std::declval<direction>(), std::uint16_t(),
                          std::declval<routing_parameters>())
      } -> std::same_as<cost_t>;
    };
// TODO: is_dest_reachable, heuristic

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
