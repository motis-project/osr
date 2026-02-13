#include "gtest/gtest.h"
#include "osr/location.h"
#include "test_util.h"
#include "osr/extract/extract.h"
#include "osr/routing/instructions/instruction_annotator.h"

namespace fs = std::filesystem;

TEST(routing_instructions, intersection_right_turn) {
  auto const from =
  osr::location{49.883621, 8.677029, osr::level_t{0.F}};
  auto const to =
      osr::location{49.885453, 8.678910, osr::level_t{0.F}};
  auto const path = "test/komponistenviertel-darmstadt-pbf.osm.pbf";
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  const auto w = osr::ways{dir, cista::mmap::protection::READ};
  const auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto p = route(w, l, from, to).value();
  osr::instruction_annotator annotator(w);
  annotator.annotate(p);

  std::vector<osr::instruction_action> actions;
  actions.reserve(p.segments_.size());
  for (auto const& s : p.segments_) {
    actions.push_back(s.instruction_annotation_);
  }

  std::vector expected = {
    osr::instruction_action::kNone,
    osr::instruction_action::kTurnRight,
    osr::instruction_action::kNone,
    osr::instruction_action::kDestination
  };
  EXPECT_EQ(expected, actions);
}

TEST(routing_instructions, intersection_left_turn) {
  auto const from =
  osr::location{49.884520, 8.675994, osr::level_t{0.F}};
  auto const to =
      osr::location{49.884308, 8.674567, osr::level_t{0.F}};
  auto const path = "test/komponistenviertel-darmstadt-pbf.osm.pbf";
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  const auto w = osr::ways{dir, cista::mmap::protection::READ};
  const auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto p = route(w, l, from, to).value();
  osr::instruction_annotator annotator(w);
  annotator.annotate(p);

  std::vector<osr::instruction_action> actions;
  actions.reserve(p.segments_.size());
  for (auto const& s : p.segments_) {
    actions.push_back(s.instruction_annotation_);
  }

  std::vector expected = {
    osr::instruction_action::kNone,
    osr::instruction_action::kTurnLeft,
    osr::instruction_action::kNone,
    osr::instruction_action::kDestination
  };
  EXPECT_EQ(expected, actions);
}

TEST(routing_instructions, intersection_continue) {
  auto const from =
  osr::location{49.883621, 8.677029, osr::level_t{0.F}};
  auto const to =
      osr::location{49.884755, 8.675667, osr::level_t{0.F}};
  auto const path = "test/komponistenviertel-darmstadt-pbf.osm.pbf";
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  const auto w = osr::ways{dir, cista::mmap::protection::READ};
  const auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto p = route(w, l, from, to).value();
  osr::instruction_annotator annotator(w);
  annotator.annotate(p);

  std::vector<osr::instruction_action> actions;
  actions.reserve(p.segments_.size());
  for (auto const& s : p.segments_) {
    actions.push_back(s.instruction_annotation_);
  }

  std::vector expected = {
    osr::instruction_action::kNone,
    osr::instruction_action::kContinue,
    osr::instruction_action::kNone,
    osr::instruction_action::kDestination
  };
  EXPECT_EQ(expected, actions);
}
