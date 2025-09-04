#include "osr/routing/profile.h"
#include "osr/routing/parameters.h"

template <typename... Ts>
constexpr bool is_parameters_variant(std::variant<Ts...> const&) {
  static_assert((osr::IsProfileParameters<Ts> && ...));
  return (osr::IsProfileParameters<Ts> && ...);
}

static_assert(is_parameters_variant(osr::profile_parameters{}));
