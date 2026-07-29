#include <filesystem>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/tracking.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

struct sharing_routing_test : public ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-sharing-routing-test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::osm_to_pbf("test/sharing-routing.osm"), dir_, {});
    ways_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    lookup_ =
        std::make_unique<lookup>(*ways_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    lookup_.reset();
    ways_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static inline fs::path dir_{};
  static inline std::unique_ptr<ways> ways_{};
  static inline std::unique_ptr<lookup> lookup_{};
};

struct test_sharing_data {
  explicit test_sharing_data(
      ways const& w, osm_node_idx_t const start_osm_node = osm_node_idx_t{1U}) {
    auto const start_node = w.find_node_idx(start_osm_node).value();
    auto const additional_node = node_idx_t{w.n_nodes()};
    auto const size =
        static_cast<bitvec<node_idx_t>::size_type>(w.n_nodes() + 1U);
    start_allowed_.resize(size);
    end_allowed_.resize(size);
    through_allowed_.resize(size);
    start_allowed_.set(additional_node, true);
    end_allowed_.one_out();
    through_allowed_.one_out();
    additional_node_coordinates_.push_back(
        w.get_node_pos(start_node).as_latlng());
    additional_edges_[start_node].push_back(
        additional_edge{.to_ = additional_node, .distance_ = 0U});
    additional_edges_[additional_node].push_back(
        additional_edge{.to_ = start_node, .distance_ = 0U});
  }

  sharing_data view(ways const& w) const {
    return {.start_allowed_ = &start_allowed_,
            .end_allowed_ = &end_allowed_,
            .through_allowed_ = &through_allowed_,
            .additional_node_offset_ = w.n_nodes(),
            .additional_node_coordinates_ = additional_node_coordinates_,
            .additional_edges_ = additional_edges_};
  }

  bitvec<node_idx_t> start_allowed_{};
  bitvec<node_idx_t> end_allowed_{};
  bitvec<node_idx_t> through_allowed_{};
  std::vector<geo::latlng> additional_node_coordinates_{};
  hash_map<node_idx_t, std::vector<additional_edge>> additional_edges_{};
};

TEST_F(sharing_routing_test, bike_does_not_switch_at_bike_inaccessible_node) {
  auto const& w = *ways_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const params = bike_sharing::parameters{};
  auto const node = w.find_node_idx(osm_node_idx_t{82U});
  ASSERT_TRUE(node.has_value());
  ASSERT_FALSE(w.r_->node_properties_[*node].is_bike_accessible());

  // make sure that we don't switch to bike at a node that is not bike
  // accessible in bwd search, because in fwd search the equivalent foot -> bike
  // switch also isn't allowed - needs to be consistent (although whether the
  // switch should be allowed in either direction is debatable)

  auto switched_to_bike = false;
  bike_sharing::adjacent<direction::kBackward, false>(
      params, *w.r_, timezone_cache_t{},
      bike_sharing::node{.n_ = *node,
                         .type_ = bike_sharing::node_type::kTrailingFoot,
                         .lvl_ = kNoLevel},
      duration_t{0U}, std::nullopt, nullptr, &sharing, nullptr,
      [&](bike_sharing::node const neighbor, auto...) {
        switched_to_bike |= neighbor.n_ == *node && neighbor.is_bike_node();
      });
  EXPECT_FALSE(switched_to_bike);
}

}  // namespace
}  // namespace osr
