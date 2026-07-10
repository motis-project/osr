#pragma once

#include "gtest/gtest.h"
#include "osr/routing/instructions/routing_instructions_input.h"

namespace osr::test {

class routing_instructions_test_fixture
    : public testing::TestWithParam<routing_instructions_input> {};

}  // namespace osr
