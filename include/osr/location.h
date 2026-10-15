#pragma once

#include <ostream>

#include "geo/latlng.h"

#include "cista/reflection/comparable.h"

#include "osr/types.h"

namespace osr {

struct location {
  CISTA_FRIEND_COMPARABLE(location)

  friend std::ostream& operator<<(std::ostream& out, location const& l) {
    return out << "{ pos=" << l.pos_ << ", lvl=" << l.lvl_.to_float() << " }";
  }

  geo::latlng pos_;
  level_t lvl_;
};

}  // namespace osr