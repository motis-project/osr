#include "osr/routing/instructions/instruction_util.h"

namespace osr {

bool ways_have_same_name(way_idx_t const w1, way_idx_t const w2, ways const& w) {
  const string_idx_t s1 = w.way_names_[w1];
  if (s1 == string_idx_t::invalid()) {
    return false;
  }

  const string_idx_t s2 = w.way_names_[w2];
  if (s2 == string_idx_t::invalid()) {
    return false;
  }

  return s1 == s2;
}

} // namespace osr