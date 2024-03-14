#include "gtest/gtest.h"

#include "osr/paged.h"

using namespace osr;

TEST(osr, paged) {
  auto p = mmm<osm_node_idx_t>{"/tmp/osr_paged_test.bin"};
  p.put(osm_node_idx_t{101}, std::string_view("Hello World!"));
  EXPECT_EQ(std::string_view{"Hello World!"}, p.get(osm_node_idx_t{101}));
}