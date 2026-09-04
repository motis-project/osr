#include <filesystem>
#include <limits>
#include <numeric>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/railway.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace osr {
namespace {

struct endpoint_routing : testing::Test {
  void SetUp() override {
    std::filesystem::create_directories(dir_);
    extract(false, test::write_osm_pbf("osr-endpoint-routing", R"(
<osm version="0.6">
  <node id="1" lat="49" lon="8"/>
  <node id="2" lat="49" lon="8.001"/>
  <node id="3" lat="49.001" lon="8.001"/>
  <node id="4" lat="49.1" lon="8"/>
  <node id="5" lat="49.1" lon="8.02"/>
  <node id="6" lat="49.099" lon="8"/>
  <node id="7" lat="49.101" lon="8.02"/>
  <way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
  <way id="2"><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
  <way id="3"><nd ref="4"/><nd ref="5"/><tag k="highway" v="footway"/></way>
  <way id="4"><nd ref="4"/><nd ref="6"/><tag k="highway" v="footway"/></way>
  <way id="5"><nd ref="5"/><nd ref="7"/><tag k="highway" v="footway"/></way>
</osm>)"),
            dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  std::filesystem::path dir_{std::filesystem::temp_directory_path() /
                             "osr-endpoint-routing"};
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
};

TEST_F(endpoint_routing, direct_without_affordable_graph_root) {
  auto const params = get_parameters(search_profile::kFoot);
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    auto const from = location{49.1, 8.01};
    auto const to = location{49.1, 8.01001};
    auto const single = route(params, *w_, *l_, search_profile::kFoot, from, to,
                              100U, dir, 2.0);
    auto const many = route(params, *w_, *l_, search_profile::kFoot, from,
                            std::vector<location>{to}, 100U, dir, 2.0);
    ASSERT_TRUE(single.has_value());
    ASSERT_EQ(many.size(), 1U);
    ASSERT_TRUE(many.front().has_value());
    EXPECT_EQ(single->cost_, many.front()->cost_);
    EXPECT_EQ(single->duration_, many.front()->duration_);
  }
}

TEST_F(endpoint_routing, unaffordable_direct_does_not_hide_graph_route) {
  auto const params = get_parameters(search_profile::kFoot);
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    for (auto const algo :
         {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
      for (auto const max : {30U, 60U}) {
        auto const from = location{49., 8.00099};
        auto const to = location{49.00001, 8.001};
        auto const p = route(params, *w_, *l_, search_profile::kFoot, from, to,
                             max, dir, 2.0, nullptr, nullptr, nullptr, algo);
        ASSERT_TRUE(p.has_value());
        EXPECT_LT(p->cost_, max);
        auto const a = route_astar(params, *w_, *l_, search_profile::kFoot,
                                   from, to, max, dir, 2.0);
        ASSERT_TRUE(a.has_value());
        EXPECT_LT(a->cost_, max);
      }
    }
  }
}

TEST_F(endpoint_routing, meeting_turn_is_in_segment_totals) {
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    auto const from = location{49., 8.0005};
    auto const to = location{49.0005, 8.001};
    auto const bwd = dir == direction::kBackward;
    auto const p =
        route(get_parameters(search_profile::kBus), *w_, *l_,
              search_profile::kBus, bwd ? to : from, bwd ? from : to, 3600U,
              dir, 2.0, nullptr, nullptr, nullptr, routing_algorithm::kAStarBi);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->cost_, 35U);
    EXPECT_EQ(
        p->cost_,
        std::accumulate(begin(p->segments_), end(p->segments_), cost_t{0U},
                        [](auto sum, auto const& s) { return sum + s.cost_; }));
    EXPECT_EQ(p->duration_,
              std::accumulate(
                  begin(p->segments_), end(p->segments_), duration_t{0U},
                  [](auto sum, auto const& s) { return sum + s.duration_; }));
  }
}

TEST_F(endpoint_routing, geometry_follows_physical_travel) {
  auto const params = get_parameters(search_profile::kFoot);
  for (auto const& [from, to] :
       {std::pair{location{49., 8.0005}, location{49.0005, 8.001}},
        std::pair{location{49.0999, 8.}, location{49.1001, 8.02}},
        std::pair{location{49.1, 8.01}, location{49.1, 8.01001}}}) {
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      auto const bwd = dir == direction::kBackward;
      for (auto const algo :
           {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
        auto const p = route(params, *w_, *l_, search_profile::kFoot,
                             bwd ? to : from, bwd ? from : to, 100'000U, dir,
                             2.0, nullptr, nullptr, nullptr, algo);
        ASSERT_TRUE(p.has_value());
        ASSERT_FALSE(p->segments_.empty());
        EXPECT_LT(
            geo::distance(from.pos_, p->segments_.front().polyline_.front()),
            0.01);
        EXPECT_LT(geo::distance(to.pos_, p->segments_.back().polyline_.back()),
                  0.01);
        for (auto i = std::size_t{1U}; i < p->segments_.size(); ++i) {
          auto const& prev = p->segments_[i - 1U];
          auto const& next = p->segments_[i];
          EXPECT_EQ(prev.to_, next.from_);
          EXPECT_LT(
              geo::distance(prev.polyline_.back(), next.polyline_.front()),
              0.01);
        }
        auto const many = route(params, *w_, *l_, search_profile::kFoot, to,
                                std::vector<location>{from}, 100'000U,
                                direction::kBackward, 2.0, nullptr, nullptr,
                                nullptr, [](path const&) { return true; });
        ASSERT_TRUE(many.front().has_value());
        EXPECT_EQ(p->cost_, many.front()->cost_);
        EXPECT_EQ(p->duration_, many.front()->duration_);
      }
    }
  }
}

TEST_F(endpoint_routing, bidirectional_meets_at_additional_node) {
  auto const check = [&]<typename P>() {
    auto const params = typename P::parameters{};
    auto const n = node_idx_t{w_->n_nodes()};
    auto const coordinates = std::vector<geo::latlng>{{49., 8.001}};
    auto const edges = hash_map<node_idx_t, std::vector<additional_edge>>{
        {n,
         {{.to_ = n, .underlying_way_ = way_idx_t{0U}},
          {.to_ = n, .underlying_way_ = way_idx_t{1U}}}}};
    auto const sharing =
        sharing_data{.additional_node_offset_ = w_->n_nodes(),
                     .additional_node_coordinates_ = coordinates,
                     .additional_edges_ = edges};
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      auto b = bidirectional<P>{};
      auto const pos = location{coordinates.front()};
      b.reset({.profile_ = params,
               .w_ = w_.get(),
               .max_ = 3600U,
               .dir_ = dir,
               .sharing_ = &sharing,
               .start_loc_ = pos,
               .end_loc_ = pos});
      b.add_start(typename P::label{{n, 0U, direction::kForward}, 7U},
                  duration_t{3U});
      b.add_end(typename P::label{{n, 1U, direction::kForward}, 11U},
                duration_t{5U});
      b.run();
      EXPECT_EQ(b.best_cost_, 18U);
      EXPECT_EQ(b.best_duration_, duration_t{8U});
      EXPECT_EQ(b.meet_point_1_.get_node(), n);
      EXPECT_EQ(b.meet_point_2_.get_node(), n);
    }
  };
  check.template operator()<car>();
  check.template operator()<bus>();
  check.template operator()<railway>();
}

TEST_F(endpoint_routing, rejects_invalid_matching_penalty_factors) {
  auto const params = get_parameters(search_profile::kFoot);
  auto const from = location{49., 8.00099};
  auto const to = location{49.00001, 8.001};
  for (auto const factor : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
    auto const options = route_options{.matching_penalty_factor_ = factor};
    for (auto const algo :
         {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
      EXPECT_THROW(route(params, *w_, *l_, search_profile::kFoot, from, to,
                         3600U, direction::kForward, 2.0, nullptr, nullptr,
                         nullptr, algo, std::nullopt, options),
                   std::exception);
    }
    EXPECT_THROW(route_astar(params, *w_, *l_, search_profile::kFoot, from, to,
                             3600U, direction::kForward, 2.0, nullptr, nullptr,
                             nullptr, std::nullopt, options),
                 std::exception);
    EXPECT_THROW(route(
                     params, *w_, *l_, search_profile::kFoot, from,
                     std::vector<location>{to}, 3600U, direction::kForward, 2.0,
                     nullptr, nullptr, nullptr,
                     [](path const&) { return false; }, std::nullopt, options),
                 std::exception);
  }
}

TEST_F(endpoint_routing, matching_penalty_saturates_before_integer_conversion) {
  using foot_t = foot<false, elevator_tracking>;
  auto const params = foot_t::parameters{};
  auto const from = location{49., 8.0005};
  auto const to = location{49.0005, 8.001};
  auto from_matches = match_result{};
  auto to_matches = match_result{};
  l_->match<foot_t>(params, from, false, direction::kForward, 2.0, nullptr,
                    from_matches);
  l_->match<foot_t>(params, to, true, direction::kForward, 2.0, nullptr,
                    to_matches);
  auto const fm = from_matches[match_idx_t{0U}];
  ASSERT_FALSE(fm.empty());
  auto penalized = match_result{};
  penalized.start(fm.lvl_);
  penalized.add(0.0F, fm.way_.front(), {});  // Unusable nearest candidate.
  for (auto i = std::size_t{0U}; i != fm.size(); ++i) {
    penalized.add(10.0F, fm.way_[i], fm.nodes_[i]);
  }
  penalized.finish();
  for (auto const factor : {0.0, 1.0e10, std::numeric_limits<double>::max()}) {
    auto const p =
        route(params, *w_, *l_, search_profile::kFoot, from, to,
              penalized[match_idx_t{0U}], to_matches[match_idx_t{0U}], 3600U,
              direction::kForward, nullptr, nullptr, nullptr,
              routing_algorithm::kDijkstra, std::nullopt,
              route_options{.matching_penalty_factor_ = factor});
    EXPECT_EQ(p.has_value(), factor == 0.0);
  }
}

TEST_F(endpoint_routing, explicit_bidirectional_uses_supported_profile_policy) {
  auto const from = location{49., 8.0005};
  auto const to = location{49.0005, 8.001};
  auto const coordinates = std::vector<geo::latlng>{};
  auto const edges = hash_map<node_idx_t, std::vector<additional_edge>>{};
  auto const sharing = sharing_data{.additional_node_offset_ = w_->n_nodes(),
                                    .additional_node_coordinates_ = coordinates,
                                    .additional_edges_ = edges};
  for (auto const profile :
       {search_profile::kCarDropOff, search_profile::kCarDropOffWheelchair,
        search_profile::kCarParking, search_profile::kCarParkingWheelchair,
        search_profile::kBikeSharing, search_profile::kCarSharing,
        search_profile::kHgv}) {
    auto const params = get_parameters(profile);
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      SCOPED_TRACE(to_str(profile));
      auto const d = route_dijkstra(params, *w_, *l_, profile, from, to, 3600U,
                                    dir, 2.0, nullptr, &sharing);
      auto const b = route_bidirectional(params, *w_, *l_, profile, from, to,
                                         3600U, dir, 2.0, nullptr, &sharing);
      ASSERT_EQ(d.has_value(), b.has_value());
      if (d.has_value()) {
        EXPECT_EQ(d->cost_, b->cost_);
        EXPECT_EQ(d->duration_, b->duration_);
      }
    }
  }
}

}  // namespace
}  // namespace osr
