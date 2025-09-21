#include "osr/routing/profile.h"
#include "osr/routing/parameters.h"

template <typename... Ts>
constexpr bool is_parameters_variant(std::variant<Ts...> const&) {
  static_assert((osr::ProfileParameters<Ts> && ...));
  return (osr::ProfileParameters<Ts> && ...);
}

static_assert(is_parameters_variant(osr::profile_parameters{}));
