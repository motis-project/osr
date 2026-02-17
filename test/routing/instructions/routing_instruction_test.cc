#include <filesystem>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "fmt/format.h"

#include "test_util.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/instructions/instruction_annotator.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

void test_instructions(
    std::string_view pbf_name,
    osr::location const& from,
    osr::location const& to,
    osr::search_profile const sp,
    std::vector<osr::instruction_action> const& expected_actions) {
  auto const pbf_path =
      fmt::format("test/routing/instructions/data/{}", pbf_name);
  auto const dir = fmt::format("/tmp/osr_test/{}", pbf_name);

  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, pbf_path, dir, {});

  const auto w = osr::ways{dir, cista::mmap::protection::READ};
  const auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto p = route(w, l, from, to, sp);
  ASSERT_TRUE(p.has_value());
  osr::instruction_annotator annotator(w);
  annotator.annotate(p.value());

  std::vector<osr::instruction_action> actions;
  actions.reserve(p->segments_.size());
  for (auto const& s : p->segments_) {
    actions.push_back(s.instruction_annotation_);
  }

  EXPECT_EQ(expected_actions, actions);
}

TEST(routing_instructions, intersection_right_turn) {
  test_instructions("komponistenviertel-darmstadt-pbf.osm.pbf",
                    {49.883621, 8.677029, osr::level_t{0.F}},
                    {49.885453, 8.678910, osr::level_t{0.F}},
                    osr::search_profile::kFoot,
                    {osr::instruction_action::kNone,
                     osr::instruction_action::kTurnRight,
                     osr::instruction_action::kNone,
                     osr::instruction_action::kDestination});
}

TEST(routing_instructions, intersection_left_turn) {
  test_instructions("komponistenviertel-darmstadt-pbf.osm.pbf",
                    {49.884520, 8.675994, osr::level_t{0.F}},
                    {49.884308, 8.674567, osr::level_t{0.F}},
                    osr::search_profile::kFoot,
                    {osr::instruction_action::kNone,
                     osr::instruction_action::kTurnLeft,
                     osr::instruction_action::kNone,
                     osr::instruction_action::kDestination});
}

TEST(routing_instructions, intersection_continue) {
  test_instructions("komponistenviertel-darmstadt-pbf.osm.pbf",
                    {49.883621, 8.677029, osr::level_t{0.F}},
                    {49.884755, 8.675667, osr::level_t{0.F}},
                    osr::search_profile::kFoot,
                    {osr::instruction_action::kNone,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kNone,
                     osr::instruction_action::kDestination});
}

TEST(routing_instructions, intersection_composition_1) {
  test_instructions("komponistenviertel-darmstadt-pbf.osm.pbf",
                    {49.882061, 8.679368, osr::level_t{0.F}},
                    {49.883765, 8.679143, osr::level_t{0.F}},
                    osr::search_profile::kCar,
                    {osr::instruction_action::kNone,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kTurnLeft,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kTurnLeft,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kTurnLeft,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kNone,
                     osr::instruction_action::kDestination});
}

TEST(routing_instructions, intersection_composition_2) {
  test_instructions("komponistenviertel-darmstadt-pbf.osm.pbf",
                    {49.880222, 8.673360, osr::level_t{0.F}},
                    {49.886988, 8.677882, osr::level_t{0.F}},
                    osr::search_profile::kCar,
                    {osr::instruction_action::kNone,
                     osr::instruction_action::kTurnLeft,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kTurnRight,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kTurnLeft,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kContinue,
                     osr::instruction_action::kNone,
                     osr::instruction_action::kDestination});
}