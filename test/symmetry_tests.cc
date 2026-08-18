#include <filesystem>
#include <iostream>
#include <random>
#include <sstream>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

constexpr auto kSymmetryMaxCost = osr::cost_t{100'000U};

struct symmetry_case {
  std::string_view path_;
  osr::location from_;
  osr::location to_;
};

struct result_t {
  bool has_path_;
  osr::cost_t cost_;
  osr::duration_t duration_;
};

bool supports_bidirectional(osr::search_profile const profile) {
  switch (profile) {
    case osr::search_profile::kBikeSharing:
    case osr::search_profile::kCarSharing:
    case osr::search_profile::kCarParking:
    case osr::search_profile::kCarParkingWheelchair:
    case osr::search_profile::kCarDropOff:
    case osr::search_profile::kCarDropOffWheelchair:
    case osr::search_profile::kHgv: return false;
    default: return true;
  }
}

template <typename... Args>
result_t route_sym(osr::profile_parameters const& params,
                   osr::search_profile const profile,
                   osr::ways const& w,
                   osr::lookup const& l,
                   osr::location const& from,
                   osr::location const& to,
                   osr::direction const dir,
                   osr::routing_algorithm const algo,
                   Args&&... args) {
  auto const p = osr::route(params, w, l, profile, from, to, kSymmetryMaxCost,
                            dir, 250.0, nullptr, nullptr, nullptr, algo,
                            std::nullopt, std::forward<Args>(args)...);
  return p.has_value() ? result_t{true, p->cost_, p->duration_}
                       : result_t{false, 0U, osr::duration_t{0}};
}

TEST(symmetry, multilevel_foot_dijkstra_is_direction_independent) {
  auto const cases = std::vector<std::pair<osr::location, osr::location>>{
      {osr::location{48.725480463902784, 2.2588322597458728, osr::level_t{0.F}},
       osr::location{48.723831, 2.261355, osr::level_t{0.F}}},
      {osr::location{48.725296645530705, 2.2612587304760723, osr::level_t{0.F}},
       osr::location{48.725480463902784, 2.2588322597458728,
                     osr::level_t{0.F}}},
  };
  auto const dir =
      fs::temp_directory_path() / "osr-multilevel-foot-symmetry-test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  osr::extract(false, "test/station-border.osm.pbf", dir, {});
  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto l = osr::lookup{w, dir, cista::mmap::protection::READ};
  auto const params = osr::profile_parameters{
      osr::foot<false, osr::elevator_tracking>::parameters{}};

  for (auto const& [from, to] : cases) {
    auto const fwd =
        route_sym(params, osr::search_profile::kFoot, w, l, from, to,
                  osr::direction::kForward, osr::routing_algorithm::kDijkstra);
    auto const bwd =
        route_sym(params, osr::search_profile::kFoot, w, l, to, from,
                  osr::direction::kBackward, osr::routing_algorithm::kDijkstra);
    ASSERT_TRUE(fwd.has_path_);
    ASSERT_TRUE(bwd.has_path_);
    EXPECT_EQ(fwd.cost_, bwd.cost_);
    EXPECT_EQ(fwd.duration_, bwd.duration_);
  }
}

TEST(symmetry, forward_backward_cost_equivalence) {
  auto const cases = std::vector<symmetry_case>{
      // station-border.osm.pbf has multi-level / elevator geometry
      {.path_ = "test/station-border.osm.pbf",
       .from_ = osr::location{48.725480463902784, 2.2588322597458728,
                              osr::level_t{0.F}},
       .to_ = osr::location{48.723831, 2.261355, osr::level_t{0.F}}},
      {.path_ = "test/station-border.osm.pbf",
       .from_ = osr::location{48.725480463902784, 2.2588322597458728,
                              osr::level_t{0.F}},
       .to_ = osr::location{48.7266, 2.2600, osr::level_t{1.F}}},
      {.path_ = "test/station-border.osm.pbf",
       .from_ = osr::location{48.725296645530705, 2.2612587304760723,
                              osr::level_t{0.F}},
       .to_ = osr::location{48.725480463902784, 2.2588322597458728,
                            osr::level_t{0.F}}},
      {.path_ = "test/luisenplatz-darmstadt.osm.pbf",
       .from_ = osr::location{49.872715, 8.651534, osr::level_t{0.F}},
       .to_ = osr::location{49.873023, 8.651523, osr::level_t{0.F}}},
      {.path_ = "test/london-northern-line.osm.pbf",
       .from_ = osr::location{51.5555, -0.1078, osr::kNoLevel},
       .to_ = osr::location{51.5559, -0.1073, osr::kNoLevel}},
      {.path_ = "test/da_hbf_2.osm.pbf",
       .from_ = osr::location{49.8786, 8.6107, osr::kNoLevel},
       .to_ = osr::location{49.8789, 8.6114, osr::kNoLevel}},
  };

  auto const profiles =
      std::vector<std::pair<osr::profile_parameters, osr::search_profile>>{
          {osr::foot<false, osr::elevator_tracking>::parameters{},
           osr::search_profile::kFoot},
          {osr::get_parameters(osr::search_profile::kBike),
           osr::search_profile::kBike},
          {osr::get_parameters(osr::search_profile::kCar),
           osr::search_profile::kCar},
      };

  for (auto const& c : cases) {
    auto const dir = fs::temp_directory_path() / c.path_;
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    osr::extract(false, c.path_, dir, {});
    auto w = osr::ways{dir, cista::mmap::protection::READ};
    auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

    for (auto const& [params, profile] : profiles) {
      for (auto const algo : {osr::routing_algorithm::kDijkstra,
                              osr::routing_algorithm::kAStarBi}) {
        auto const algo_str =
            algo == osr::routing_algorithm::kDijkstra ? "dijkstra" : "astarbi";
        auto const fwd = route_sym(params, profile, w, l, c.from_, c.to_,
                                   osr::direction::kForward, algo);
        auto const bwd = route_sym(params, profile, w, l, c.to_, c.from_,
                                   osr::direction::kBackward, algo);

        // MOTIS flow: 1:n backward dijkstra D->S (offset) vs bidirectional
        // forward reconstruction S->D.
        if (algo == osr::routing_algorithm::kDijkstra && bwd.has_path_) {
          auto const bi = route_sym(params, profile, w, l, c.from_, c.to_,
                                    osr::direction::kForward,
                                    osr::routing_algorithm::kAStarBi);
          if (bi.has_path_) {
            EXPECT_EQ(bwd.cost_, bi.cost_)
                << c.path_ << " " << osr::to_str(profile)
                << " dijkstra-offset vs bidirectional-reconstruction cost "
                   "mismatch";
            EXPECT_EQ(bwd.duration_, bi.duration_)
                << c.path_ << " " << osr::to_str(profile)
                << " dijkstra-offset vs bidirectional-reconstruction duration "
                   "mismatch";
          }
          // same query, different algorithm -> isolates endpoint resolution /
          // algorithm exactness
          EXPECT_EQ(fwd.cost_, bi.cost_)
              << c.path_ << " " << osr::to_str(profile)
              << " dijkstra vs bidirectional same-direction cost mismatch";
          EXPECT_EQ(fwd.duration_, bi.duration_)
              << c.path_ << " " << osr::to_str(profile)
              << " dijkstra vs bidirectional same-direction duration mismatch";
        }
        if (fwd.has_path_ && bwd.has_path_ && fwd.cost_ != bwd.cost_) {
          std::cout << "MISMATCH " << c.path_ << " " << osr::to_str(profile)
                    << " " << algo_str << " fwd=" << fwd.cost_
                    << " bwd=" << bwd.cost_ << "\n";
        }
        EXPECT_EQ(fwd.has_path_, bwd.has_path_)
            << c.path_ << " " << osr::to_str(profile) << " " << algo_str
            << " reachability mismatch";
        if (fwd.has_path_ && bwd.has_path_) {
          EXPECT_EQ(fwd.cost_, bwd.cost_)
              << c.path_ << " " << osr::to_str(profile) << " " << algo_str
              << " cost mismatch";
          EXPECT_EQ(fwd.duration_, bwd.duration_)
              << c.path_ << " " << osr::to_str(profile) << " " << algo_str
              << " duration mismatch";
        }
      }
    }
  }
}

// Routes many random location pairs in both directions and with both
// algorithms. Forward and backward search have to agree on cost and duration:
// MOTIS computes public transport offsets with a 1:n backward search and then
// reconstructs the selected leg with a forward search - if the two disagree,
// the resulting itinerary is broken.
//
TEST(symmetry, random_pairs_forward_backward_equivalence) {
  auto const maps = std::vector<std::string_view>{
      "test/station-border.osm.pbf",
      "test/luisenplatz-darmstadt.osm.pbf",
      "test/da_hbf_2.osm.pbf",
      "test/karlsruhe-kirchfeld.osm.pbf",
      "test/tram-junction.osm.pbf",
      "test/miraustr.osm.pbf",
      "test/london-northern-line.osm.pbf",
      "test/darmstadt-bismarckstr.osm.pbf",
  };
  auto const profiles =
      std::vector<std::pair<osr::profile_parameters, osr::search_profile>>{
          {osr::profile_parameters{
               osr::foot<false, osr::elevator_tracking>::parameters{}},
           osr::search_profile::kFoot},
          {osr::get_parameters(osr::search_profile::kWheelchair),
           osr::search_profile::kWheelchair},
          {osr::get_parameters(osr::search_profile::kBike),
           osr::search_profile::kBike},
          {osr::get_parameters(osr::search_profile::kCar),
           osr::search_profile::kCar},
          {osr::get_parameters(osr::search_profile::kBus),
           osr::search_profile::kBus},
          {osr::get_parameters(osr::search_profile::kHgv),
           osr::search_profile::kHgv},
          {osr::get_parameters(osr::search_profile::kRailway),
           osr::search_profile::kRailway},
          {osr::get_parameters(osr::search_profile::kCarParking),
           osr::search_profile::kCarParking},
          {osr::get_parameters(osr::search_profile::kCarDropOff),
           osr::search_profile::kCarDropOff},
      };

  for (auto const& map : maps) {
    auto const dir = fs::temp_directory_path() / "osr-symmetry-random" / map;
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    osr::extract(false, std::string{map}, dir, {});
    auto w = osr::ways{dir, cista::mmap::protection::READ};
    auto l = osr::lookup{w, dir, cista::mmap::protection::READ};
    ASSERT_GT(w.n_nodes(), 4U);

    auto rng = std::mt19937{42U};
    auto node_dist = std::uniform_int_distribution<osr::node_idx_t::value_t>{
        0U, w.n_nodes() - 1U};
    auto jitter = std::uniform_real_distribution<double>{-0.00015, 0.00015};
    auto lvl_dist = std::uniform_int_distribution<int>{0, 2};
    auto const rnd_loc = [&]() {
      auto const p =
          w.get_node_pos(osr::node_idx_t{node_dist(rng)}).as_latlng();
      auto const choice = lvl_dist(rng);
      auto const lvl = choice == 0   ? osr::kNoLevel
                       : choice == 1 ? osr::level_t{0.F}
                                     : osr::level_t{-1.F};
      return osr::location{{p.lat() + jitter(rng), p.lng() + jitter(rng)}, lvl};
    };

    for (auto i = 0U; i != 60U; ++i) {
      auto const from = rnd_loc();
      auto const to = rnd_loc();
      for (auto const& [params, profile] : profiles) {
        auto const ref =
            route_sym(params, profile, w, l, from, to, osr::direction::kForward,
                      osr::routing_algorithm::kDijkstra);
        auto results = std::vector<std::pair<std::string_view, result_t>>{
            {"dijkstra backward",
             route_sym(params, profile, w, l, to, from,
                       osr::direction::kBackward,
                       osr::routing_algorithm::kDijkstra)}};
        if (supports_bidirectional(profile)) {
          results.emplace_back("bidirectional forward",
                               route_sym(params, profile, w, l, from, to,
                                         osr::direction::kForward,
                                         osr::routing_algorithm::kAStarBi));
          results.emplace_back("bidirectional backward",
                               route_sym(params, profile, w, l, to, from,
                                         osr::direction::kBackward,
                                         osr::routing_algorithm::kAStarBi));
        }
        for (auto const& [name, r] : results) {
          auto const ctx = [&]() {
            auto ss = std::stringstream{};
            ss << map << " " << osr::to_str(profile) << " " << name
               << " from=" << from.pos_ << "@" << from.lvl_ << " to=" << to.pos_
               << "@" << to.lvl_;
            return ss.str();
          };
          ASSERT_EQ(ref.has_path_, r.has_path_) << ctx();
          if (ref.has_path_) {
            EXPECT_EQ(ref.cost_, r.cost_) << ctx();
            EXPECT_EQ(ref.duration_, r.duration_) << ctx();
          }
        }
      }
    }
  }
}

// Rental vehicles at nodes spread over the map, with combinations of
// station-based / free-floating return, restricted `through_allowed_` and
// free-floating pickup.
struct spread_sharing_data {
  spread_sharing_data(osr::ways const& w,
                      unsigned const n_vehicles,
                      bool const free_floating,
                      bool const restrict_through,
                      bool const pickup_anywhere) {
    auto const n_nodes = w.n_nodes();
    auto const size = static_cast<osr::bitvec<osr::node_idx_t>::size_type>(
        n_nodes + n_vehicles);
    start_allowed_.resize(size);
    end_allowed_.resize(size);
    through_allowed_.resize(size);
    if (free_floating) {
      end_allowed_.one_out();
    }
    through_allowed_.one_out();
    if (restrict_through) {
      for (auto i = osr::node_idx_t::value_t{0U}; i < n_nodes; i += 7U) {
        through_allowed_.set(osr::node_idx_t{i}, false);
      }
    }
    if (pickup_anywhere) {
      for (auto i = osr::node_idx_t::value_t{0U}; i < n_nodes; i += 5U) {
        start_allowed_.set(osr::node_idx_t{i}, true);
      }
    }
    for (auto i = 0U; i != n_vehicles; ++i) {
      auto const graph_node =
          osr::node_idx_t{(i + 1U) * (n_nodes / (n_vehicles + 1U))};
      auto const additional = osr::node_idx_t{n_nodes + i};
      start_allowed_.set(additional, true);
      if (!free_floating) {
        end_allowed_.set(additional, true);
      }
      additional_node_coordinates_.push_back(
          w.get_node_pos(graph_node).as_latlng());
      additional_edges_[graph_node].push_back(
          osr::additional_edge{.to_ = additional, .distance_ = 0U});
      additional_edges_[additional].push_back(
          osr::additional_edge{.to_ = graph_node, .distance_ = 0U});
    }
  }

  osr::sharing_data view(osr::ways const& w) const {
    osr::verify_additional_edge_count(additional_edges_, w.n_nodes());
    return {.start_allowed_ = &start_allowed_,
            .end_allowed_ = &end_allowed_,
            .through_allowed_ = &through_allowed_,
            .additional_node_offset_ = w.n_nodes(),
            .additional_node_coordinates_ = additional_node_coordinates_,
            .additional_edges_ = additional_edges_};
  }

  osr::bitvec<osr::node_idx_t> start_allowed_{}, end_allowed_{},
      through_allowed_{};
  std::vector<geo::latlng> additional_node_coordinates_{};
  osr::hash_map<osr::node_idx_t, std::vector<osr::additional_edge>>
      additional_edges_{};
};

// Forward and backward search have to agree on cost and duration: MOTIS
// computes public transport offsets with a 1:n backward search and then
// reconstructs the selected leg with a forward search, so any disagreement
// produces a broken itinerary.
TEST(symmetry, sharing_random_pairs_forward_backward_equivalence) {
  auto const maps = std::vector<std::string_view>{
      "test/da_hbf_2.osm.pbf",
      "test/karlsruhe-kirchfeld.osm.pbf",
      "test/luisenplatz-darmstadt.osm.pbf",
  };
  auto const profiles =
      std::vector<std::pair<osr::profile_parameters, osr::search_profile>>{
          {osr::get_parameters(osr::search_profile::kBikeSharing),
           osr::search_profile::kBikeSharing},
          {osr::get_parameters(osr::search_profile::kCarSharing),
           osr::search_profile::kCarSharing}};

  for (auto const& map : maps) {
    auto const map_dir =
        fs::temp_directory_path() / "osr-symmetry-sharing" / map;
    auto ec = std::error_code{};
    fs::remove_all(map_dir, ec);
    fs::create_directories(map_dir, ec);
    osr::extract(false, std::string{map}, map_dir, {});
    auto w = osr::ways{map_dir, cista::mmap::protection::READ};
    auto l = osr::lookup{w, map_dir, cista::mmap::protection::READ};

    for (auto const cfg : {0, 1, 2, 3}) {
      auto const data =
          spread_sharing_data{w, 8U, cfg != 1, cfg == 2, cfg == 3};
      auto const sharing = data.view(w);
      auto rng = std::mt19937{42U};
      auto nd = std::uniform_int_distribution<osr::node_idx_t::value_t>{
          0U, w.n_nodes() - 1U};
      auto jit = std::uniform_real_distribution<double>{-0.00015, 0.00015};
      auto const rnd_loc = [&]() {
        auto const p = w.get_node_pos(osr::node_idx_t{nd(rng)}).as_latlng();
        return osr::location{{p.lat() + jit(rng), p.lng() + jit(rng)},
                             osr::kNoLevel};
      };

      for (auto i = 0U; i != 30U; ++i) {
        auto const from = rnd_loc();
        auto const to = rnd_loc();
        for (auto const& [params, profile] : profiles) {
          auto const fwd =
              osr::route(params, w, l, profile, from, to, osr::cost_t{3600U},
                         osr::direction::kForward, 100.0, nullptr, &sharing);
          auto const bwd =
              osr::route(params, w, l, profile, to, from, osr::cost_t{3600U},
                         osr::direction::kBackward, 100.0, nullptr, &sharing);
          auto const ctx = [&]() {
            auto ss = std::stringstream{};
            ss << map << " " << osr::to_str(profile) << " cfg" << cfg
               << " from=" << from.pos_ << " to=" << to.pos_;
            return ss.str();
          };
          ASSERT_EQ(fwd.has_value(), bwd.has_value()) << ctx();
          if (fwd.has_value()) {
            EXPECT_EQ(fwd->cost_, bwd->cost_) << ctx();
            EXPECT_EQ(fwd->duration_, bwd->duration_) << ctx();
          }
        }
      }
    }
  }
}

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
    auto const data =
        spread_sharing_data{w, 8U, c.free_floating_, false, false};
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
