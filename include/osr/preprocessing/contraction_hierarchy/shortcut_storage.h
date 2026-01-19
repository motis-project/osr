#pragma once

#include <string>

#include "osr/preprocessing/contraction_hierarchy/node_order.h"
#include "osr/types.h"

namespace osr {

enum class search_profile : std::uint8_t;
std::string_view to_str(search_profile);

template <typename Node>
struct shortcut_storage final {
  struct shortcut final {
    Node from_;
    Node to_;
  };
  struct shortcut_reconstruct final {
    Node via_in_;
    Node via_out_;
    way_idx_t from_way_;
    way_idx_t to_way_;
    cost_t from_way_cost_;
    cost_t to_way_cost_;
    cost_t loop_cost_;
  };
  struct neighbor final {
    node_idx_t idx_;
    way_idx_t sc_idx_;
  };
  using data_t = cista::paged<
      cista::offset::vector<neighbor>,
      typename cista::offset::vector<neighbor>::size_type,
      uint32_t,
      std::max(
          std::size_t{2U},
          cista::next_power_of_two(
              sizeof(cista::page<
                     typename cista::offset::vector<neighbor>::size_type,
                     uint32_t>) /
              sizeof(typename cista::offset::vector<neighbor>::value_type))),
      (1U << 31U)>;
  using idx_t = cista::offset::vector<typename data_t::page_t>;

  static constexpr std::string_view kFilenamePrefix = "shortcuts_";

  cista::offset::vector_map<way_idx_t, shortcut> shortcuts_;
  cista::offset::vector_map<way_idx_t, shortcut_reconstruct>
      shortcuts_reconstruct_;
  cista::offset::vector_map<way_idx_t, cost_t> costs_;
  cista::paged_vecvec<idx_t, data_t, node_idx_t> neighbors_fwd_;
  cista::paged_vecvec<idx_t, data_t, node_idx_t> neighbors_bwd_;
  node_order order_;

  static constexpr std::string get_filename(
      search_profile const profile) noexcept {
    return (std::string(kFilenamePrefix) + std::string(to_str(profile)) +
            ".bin");
  }

  static cista::wrapped<shortcut_storage const> read(
      std::filesystem::path const& p, search_profile const profile) {
    return cista::read<shortcut_storage const>(p / get_filename(profile));
  }

  void write(std::filesystem::path const& p,
             search_profile const profile) const {
    return cista::write(p / get_filename(profile), *this);
  }
};

template <typename Node>
struct path_label final {
  Node node_;
  cost_t cost_;
  way_idx_t way_;
  bool operator==(path_label const&) const = default;
  path_label() = default;
  path_label(Node const n, cost_t const c, way_idx_t const w)
      : node_{std::move(n)}, cost_{c}, way_{w} {}
};

template <typename Node>
cista::wrapped<shortcut_storage<Node> const>& get_shortcut_storage() {
  static auto sc_stor = cista::wrapped<shortcut_storage<Node> const>{};
  return sc_stor;
}

}  // namespace osr
