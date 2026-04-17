#include "osr/routing/instructions/routing_instructions_test_fixture.h"

using namespace osr::test;

const std::filesystem::path test_data_dir = TEST_DATA_DIR;

INSTANTIATE_TEST_SUITE_P(
    JsonRoutingTests,
    routing_instructions_test_fixture,
    testing::ValuesIn(load_from_json(test_data_dir / "test-cases.json")),
    [](const testing::TestParamInfo<routing_instructions_input>& info) {
      return info.param.test_name;
    });

