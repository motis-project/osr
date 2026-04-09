#include "osr/routing/instructions/routing_instructions_test_fixture.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/instructions/instruction_annotator.h"
#include "osr/routing/routing_test_util.h"

namespace fs = std::filesystem;

namespace osr::test {

void test_instructions(
    std::string_view pbf_name,
    location const& from,
    location const& to,
    search_profile const sp,
    std::vector<instruction_action> const& expected_actions) {
  auto const pbf_path =
      fmt::format("test/routing/instructions/data/{}", pbf_name);
  auto const dir = fmt::format("/tmp/osr_test/{}", pbf_name);

  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  extract(false, pbf_path, dir, {});

  const auto w = ways{dir, cista::mmap::protection::READ};
  const auto l = lookup{w, dir, cista::mmap::protection::READ};

  auto p = route(w, l, from, to, osr::get_parameters(sp), sp);
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

TEST_P(routing_instructions_test_fixture, ValidatesInstructions) {
  const auto& test_params = GetParam();

  test_instructions(test_params.osm_file, test_params.start, test_params.end,
                    test_params.profile, test_params.expected_actions);
}

}

