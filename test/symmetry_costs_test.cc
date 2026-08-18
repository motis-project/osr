#include <filesystem>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

namespace osr {

TEST(symmetry_costs, way_aware_profiles_price_both_directions_alike) {
  auto const dir = fs::temp_directory_path() / "osr-symmetry-costs-test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  extract(false, "test/station-border.osm.pbf", dir, {});
  auto w = ways{dir, cista::mmap::protection::READ};
  auto const l = lookup{w, dir, cista::mmap::protection::READ};

  auto const from = location{48.7258, 2.25826, kNoLevel};
  auto const to = location{48.7235, 2.25462, kNoLevel};

  for (auto const profile :
       {search_profile::kBus, search_profile::kHgv, search_profile::kRailway}) {
    auto const params = get_parameters(profile);
    auto const fwd = route(params, w, l, profile, from, to, cost_t{100'000U},
                           direction::kForward, 250.0);
    auto const bwd = route(params, w, l, profile, to, from, cost_t{100'000U},
                           direction::kBackward, 250.0);
    ASSERT_TRUE(fwd.has_value()) << to_str(profile);
    ASSERT_TRUE(bwd.has_value()) << to_str(profile);
    EXPECT_EQ(fwd->cost_, bwd->cost_) << to_str(profile);
    EXPECT_EQ(fwd->duration_, bwd->duration_) << to_str(profile);
  }
}

TEST(symmetry_costs, node_penalties_are_charged_in_both_directions) {
  struct testcase {
    std::string_view fixture_;
    search_profile profile_;
    location from_, to_;
  };
  auto const cases = std::vector<testcase>{
      // walks up to an elevator node, no turn costs anywhere
      {"test/station-border.osm.pbf", search_profile::kFoot,
       location{48.7265456, 2.259178, kNoLevel},
       location{48.7252129, 2.2595882, kNoLevel}},
      // ends next to a gate node that charges `private_gate_penalty_`
      {"test/karlsruhe-kit-nord.osm.pbf", search_profile::kBus,
       location{49.1027884, 8.4348879, kNoLevel},
       location{49.1011031, 8.4367592, kNoLevel}}};

  for (auto const& c : cases) {
    auto const dir = fs::temp_directory_path() / "osr-node-penalty-test";
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    extract(false, std::string{c.fixture_}, dir, {});
    auto w = ways{dir, cista::mmap::protection::READ};
    auto const l = lookup{w, dir, cista::mmap::protection::READ};
    auto const params = get_parameters(c.profile_);

    auto const fwd = route(params, w, l, c.profile_, c.from_, c.to_,
                           cost_t{100'000U}, direction::kForward, 250.0);
    auto const bwd = route(params, w, l, c.profile_, c.to_, c.from_,
                           cost_t{100'000U}, direction::kBackward, 250.0);
    ASSERT_TRUE(fwd.has_value()) << c.fixture_;
    ASSERT_TRUE(bwd.has_value()) << c.fixture_;
    EXPECT_EQ(fwd->cost_, bwd->cost_) << c.fixture_;
    EXPECT_EQ(fwd->duration_, bwd->duration_) << c.fixture_;
  }
}

TEST(symmetry_levels, elevator_endpoint_is_direction_independent) {
  auto const dir = fs::temp_directory_path() / "osr-symmetry-levels-test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  extract(false, "test/station-border.osm.pbf", dir, {});
  auto w = ways{dir, cista::mmap::protection::READ};
  auto const l = lookup{w, dir, cista::mmap::protection::READ};
  auto const params = get_parameters(search_profile::kFoot);

  auto const from = location{48.7265456, 2.259178, kNoLevel};
  auto const to = location{48.7263761, 2.2576106, kNoLevel};

  auto const fwd = route(params, w, l, search_profile::kFoot, from, to,
                         cost_t{100'000U}, direction::kForward, 250.0);
  auto const bwd = route(params, w, l, search_profile::kFoot, to, from,
                         cost_t{100'000U}, direction::kBackward, 250.0);
  ASSERT_TRUE(fwd.has_value());
  ASSERT_TRUE(bwd.has_value());
  EXPECT_EQ(fwd->cost_, bwd->cost_);
  EXPECT_EQ(fwd->duration_, bwd->duration_);
}

TEST(symmetry_modes, car_parking_transition_is_direction_independent) {
  struct testcase {
    std::string_view fixture_;
    search_profile profile_;
    location from_, to_;
  };
  auto const cases = std::vector<testcase>{
      {"test/karlsruhe-kirchfeld.osm.pbf", search_profile::kCarParking,
       location{49.0446865, 8.3896766, kNoLevel},
       location{49.0516714, 8.387649, kNoLevel}},
      {"test/station-border.osm.pbf", search_profile::kCarParking,
       location{48.7234757, 2.2545168, kNoLevel},
       location{48.7253219, 2.2613838, kNoLevel}},
      {"test/karlsruhe-kirchfeld.osm.pbf", search_profile::kCarDropOff,
       location{49.0395005, 8.3939066, kNoLevel},
       location{49.0451469, 8.3936645, kNoLevel}}};

  for (auto const& c : cases) {
    auto const dir =
        fs::temp_directory_path() / "osr-car-parking-symmetry-test";
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    extract(false, std::string{c.fixture_}, dir, {});
    auto w = ways{dir, cista::mmap::protection::READ};
    auto const l = lookup{w, dir, cista::mmap::protection::READ};
    auto const params = get_parameters(c.profile_);

    auto const fwd = route(params, w, l, c.profile_, c.from_, c.to_,
                           cost_t{100'000U}, direction::kForward, 250.0);
    auto const bwd = route(params, w, l, c.profile_, c.to_, c.from_,
                           cost_t{100'000U}, direction::kBackward, 250.0);
    ASSERT_TRUE(fwd.has_value()) << c.fixture_ << " " << to_str(c.profile_);
    ASSERT_TRUE(bwd.has_value()) << c.fixture_ << " " << to_str(c.profile_);
    EXPECT_EQ(fwd->cost_, bwd->cost_)
        << c.fixture_ << " " << to_str(c.profile_);
    EXPECT_EQ(fwd->duration_, bwd->duration_)
        << c.fixture_ << " " << to_str(c.profile_);
  }
}

// Vehicles spread over the graph, either returnable anywhere (free floating) or
// only at their station.
struct sharing_fixture {
  sharing_fixture(ways const& w,
                  unsigned const n_vehicles,
                  bool const free_floating) {
    auto const n_nodes = w.n_nodes();
    auto const size =
        static_cast<bitvec<node_idx_t>::size_type>(n_nodes + n_vehicles);
    start_allowed_.resize(size);
    end_allowed_.resize(size);
    through_allowed_.resize(size);
    if (free_floating) {
      end_allowed_.one_out();
    }
    through_allowed_.one_out();
    for (auto i = 0U; i != n_vehicles; ++i) {
      auto const graph_node =
          node_idx_t{(i + 1U) * (n_nodes / (n_vehicles + 1U))};
      auto const additional = node_idx_t{n_nodes + i};
      start_allowed_.set(additional, true);
      if (!free_floating) {
        end_allowed_.set(additional, true);
      }
      additional_node_coordinates_.push_back(
          w.get_node_pos(graph_node).as_latlng());
      additional_edges_[graph_node].push_back(
          additional_edge{.to_ = additional, .distance_ = 0U});
      additional_edges_[additional].push_back(
          additional_edge{.to_ = graph_node, .distance_ = 0U});
    }
  }

  sharing_data view(ways const& w) const {
    verify_additional_edge_count(additional_edges_);
    return {.start_allowed_ = &start_allowed_,
            .end_allowed_ = &end_allowed_,
            .through_allowed_ = &through_allowed_,
            .additional_node_offset_ = w.n_nodes(),
            .additional_node_coordinates_ = additional_node_coordinates_,
            .additional_edges_ = additional_edges_};
  }

  bitvec<node_idx_t> start_allowed_{}, end_allowed_{}, through_allowed_{};
  std::vector<geo::latlng> additional_node_coordinates_{};
  hash_map<node_idx_t, std::vector<additional_edge>> additional_edges_{};
};

TEST(symmetry_modes, sharing_transitions_are_direction_independent) {
  struct testcase {
    std::string_view fixture_;
    search_profile profile_;
    bool free_floating_;
    location from_, to_;
  };
  auto const cases = std::vector<testcase>{
      {"test/karlsruhe-kirchfeld.osm.pbf", search_profile::kCarSharing, false,
       location{49.0432396, 8.3976147, kNoLevel},
       location{49.0608279, 8.3970761, kNoLevel}},
      {"test/karlsruhe-kirchfeld.osm.pbf", search_profile::kBikeSharing, true,
       location{49.0554595, 8.3995238, kNoLevel},
       location{49.0520846, 8.3883652, kNoLevel}},
      {"test/da_hbf_2.osm.pbf", search_profile::kBikeSharing, true,
       location{49.8696469, 8.6360084, kNoLevel},
       location{49.8740786, 8.6322856, kNoLevel}}};

  for (auto const& c : cases) {
    auto const dir = fs::temp_directory_path() / "osr-sharing-symmetry-test";
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    extract(false, std::string{c.fixture_}, dir, {});
    auto w = ways{dir, cista::mmap::protection::READ};
    auto const l = lookup{w, dir, cista::mmap::protection::READ};
    auto const data = sharing_fixture{w, 8U, c.free_floating_};
    auto const sharing = data.view(w);
    auto const params = get_parameters(c.profile_);

    auto const fwd =
        route(params, w, l, c.profile_, c.from_, c.to_, cost_t{3600U},
              direction::kForward, 100.0, nullptr, &sharing);
    auto const bwd =
        route(params, w, l, c.profile_, c.to_, c.from_, cost_t{3600U},
              direction::kBackward, 100.0, nullptr, &sharing);
    ASSERT_TRUE(fwd.has_value()) << c.fixture_ << " " << to_str(c.profile_);
    ASSERT_TRUE(bwd.has_value()) << c.fixture_ << " " << to_str(c.profile_);
    EXPECT_EQ(fwd->cost_, bwd->cost_)
        << c.fixture_ << " " << to_str(c.profile_);
    EXPECT_EQ(fwd->duration_, bwd->duration_)
        << c.fixture_ << " " << to_str(c.profile_);
  }
}

}  // namespace osr
