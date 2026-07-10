#include "osr/routing/instructions/routing_instructions_test_fixture.h"

#include <memory>
#include <mutex>
#include <unordered_map>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/instructions/instruction_annotator.h"
#include "osr/routing/routing_test_util.h"

namespace fs = std::filesystem;

namespace osr::test {

struct cached_dataset {
  fs::path dir_;
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
};

static std::mutex g_cache_mutex;
static std::unordered_map<std::string, std::shared_ptr<cached_dataset>> g_dataset_cache;

std::shared_ptr<cached_dataset> get_dataset(fs::path const& pbf_path) {
  std::lock_guard<std::mutex> lock(g_cache_mutex);
  auto const key = pbf_path.string();
  auto it = g_dataset_cache.find(key);
  if (it == g_dataset_cache.end()) {
    auto const dir = fmt::format("/tmp/osr_test/{}", pbf_path.filename().string());
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    extract(false, pbf_path, dir, {});

    auto dataset = std::make_shared<cached_dataset>();
    dataset->dir_ = dir;
    dataset->w_ = std::make_unique<ways>(dir, cista::mmap::protection::READ);
    dataset->l_ = std::make_unique<lookup>(*dataset->w_, dir, cista::mmap::protection::READ);
    g_dataset_cache[key] = dataset;
    return dataset;
  }
  return it->second;
}

void test_instructions(
    fs::path const& pbf_path,
    location const& from,
    location const& to,
    search_profile const sp,
    routing_algorithm const algo,
    direction const direction,
    std::vector<boost::json::value> const& expected_actions_json) {
  try {
    auto const dataset = get_dataset(pbf_path);
    const auto& w = *dataset->w_;
    const auto& l = *dataset->l_;

    auto p = route(w, l, from, to, osr::get_parameters(sp), sp, direction, algo);
    ASSERT_TRUE(p.has_value());
    instruction_annotator annotator(w);
    annotator.annotate(p.value());

    std::vector<routing_instruction> expected_actions;
    expected_actions.reserve(expected_actions_json.size());
    for (auto const& val : expected_actions_json) {
      expected_actions.push_back(from_json(val, w));
    }

    std::vector<routing_instruction> actions;
    actions.reserve(p->segments_.size());
    for (auto const& s : p->segments_) {
      actions.push_back(s.instruction_annotation_);
    }

    EXPECT_EQ(expected_actions, actions);
  } catch (...) {
    GTEST_SKIP();
  }
}

TEST_P(routing_instructions_test_fixture, ValidatesInstructions) {
  const auto& test_params = GetParam();

  test_instructions(test_params.osm_file, test_params.start, test_params.end,
                    test_params.profile, test_params.routing_algo,
                    test_params.direction, test_params.expected_actions);
}

}

