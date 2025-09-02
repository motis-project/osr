#include "osr/routing/profile.h"

#include "osr/routing/parameters.h"

template <std::size_t N>
constexpr bool is_profile_rec() {
  using profile_t =
      std::variant_alternative_t<N, osr::profile_parameters>::profile_t;
  if constexpr (N > 0U) {
    return osr::IsProfile<profile_t> && is_profile_rec<N - 1>();
  } else {
    return osr::IsProfile<profile_t>;
  }
}

constexpr auto const contains_only_profiles =
    is_profile_rec<std::variant_size_v<osr::profile_parameters> - 1U>();

static_assert(contains_only_profiles);
