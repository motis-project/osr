#include "osr/routing/instructions/routing_instructions_test_fixture.h"

using namespace osr::test;

INSTANTIATE_TEST_SUITE_P(
    JsonRoutingTests,
    routing_instructions_test_fixture,
    testing::ValuesIn(load_from_json("test/routing/instructions/data/test-cases.json")),
    [](const testing::TestParamInfo<routing_instructions_input>& info) {
      return info.param.test_name;
    });

