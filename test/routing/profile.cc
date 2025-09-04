#include "osr/routing/profile.h"
#include "osr/routing/parameters.h"

constexpr auto const kProfileCount =
    std::variant_size_v<osr::profile_parameters>;

template <std::size_t N>
constexpr bool is_profile() {
  return osr::IsProfileParameters<
      std::variant_alternative_t<N, osr::profile_parameters>>;
}

template <std::size_t N>
constexpr bool is_profile_helper(bool valid) {
  if constexpr (N == 0U) {
    return valid && is_profile<N>();
  } else {
    return is_profile_helper<N - 1>(valid && is_profile<N>());
  }
}

constexpr auto const is_parameters_variant =
    is_profile_helper<kProfileCount - 1>(true);

static_assert(kProfileCount > 0U);
static_assert(is_parameters_variant);
