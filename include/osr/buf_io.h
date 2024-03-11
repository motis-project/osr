#pragma once

#include <cstring>
#include <span>
#include <type_traits>
#include <vector>

#include "cista/type_hash/type_name.h"

#include "fmt/core.h"
#include "fmt/ostream.h"
#include "utl/verify.h"

namespace osr {

template <typename... T, typename Vec>
void write_buf(std::vector<std::uint8_t>& buf, Vec&& data, T&&... t) {
  static_assert((std::is_trivially_copyable_v<std::decay_t<T>> && ...));
  static_assert(std::is_trivially_copyable_v<
                std::decay_t<typename std::decay_t<Vec>::value_type>>);

  buf.resize((sizeof(std::decay_t<T>) + ...) +
             data.size() *
                 sizeof(std::decay_t<typename std::decay_t<Vec>::value_type>));

  auto offset = std::size_t{0U};
  auto const write = [&](auto&& x) {
    constexpr auto const size = sizeof(std::decay_t<decltype(x)>);
    std::memcpy(&buf[offset], &x, size);
    offset += size;
  };

  (write(t), ...);
  for (auto const& x : data) {
    write(x);
  }
}

template <typename... T, typename Vec>
void read_buf(std::span<std::uint8_t const> buf, Vec& data, T&... t) {
  static_assert((std::is_trivially_copyable_v<T> && ...));
  static_assert(
      std::is_trivially_copyable_v<typename std::decay_t<Vec>::value_type>);
  
  constexpr auto const required_size = (sizeof(std::decay_t<T>) + ...);
  utl::verify(buf.size() >= required_size, "buffer size {} < required size {}",
              buf.size(), required_size);

  auto offset = std::size_t{0U};
  auto const read = [&](auto& x) {
    constexpr auto const size = sizeof(std::decay_t<decltype(x)>);
    std::memcpy(&x, &buf[offset], size);
    offset += size;
  };

  data.clear();

  (read(t), ...);
  while (offset <= buf.size() - sizeof(typename Vec::value_type)) {
    read(data.emplace_back());
  }
}

inline std::string_view view(std::vector<std::uint8_t> const& v) {
  return {reinterpret_cast<char const*>(v.data()), v.size()};
}

inline std::span<std::uint8_t const> span(std::string_view s) {
  return {reinterpret_cast<std::uint8_t const*>(s.data()), s.size()};
}

}  // namespace osr