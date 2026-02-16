#pragma once

#include <cmath>

#include <iterator>
#include <type_traits>

#include "utl/verify.h"

namespace osr {

template <std::size_t Radius, typename It>
struct window {
  window(It begin, It end, It focus, std::ptrdiff_t const index, std::ptrdiff_t const size)
      : begin_{begin}, end_{end}, focus_{focus}, index_{index}, size_{size} {}

  It focus() const { return focus_; }

  bool has(int const offset) const {
    if (std::abs(offset) > static_cast<int>(Radius)) {
      return false;
    }
    auto const target_idx = index_ + offset;
    if (target_idx < 0 || target_idx >= size_) {
      return false;
    }

    return true;
  }

  It at(int const offset) const {
    if (std::abs(offset) > static_cast<int>(Radius)) {
      return end_;
    }
    auto const target_idx = index_ + offset;
    if (target_idx < 0 || target_idx >= size_) {
      return end_;
    }

    // Optimization for Random Access Iterators
    if constexpr (std::is_base_of_v<
                      std::random_access_iterator_tag,
                      typename std::iterator_traits<It>::iterator_category>) {
      return begin_ + target_idx;
    } else {
      // Fallback for other iterators
      if (offset == 0) {
        return focus_;
      }
      if (offset > 0) {
        return std::next(focus_, offset);
      }
      return std::prev(focus_, -offset);
    }
  }

  It operator[](int const offset) const { return at(offset); }

  It begin_;
  It end_;
  It focus_;
  std::ptrdiff_t index_;
  std::ptrdiff_t size_;
};

template <std::size_t Radius, typename It>
struct sliding_window_iterator {
  using iterator_category = std::forward_iterator_tag;
  using value_type = window<Radius, It>;
  using difference_type = std::ptrdiff_t;
  using pointer = window<Radius, It>;
  using reference = window<Radius, It>;

  sliding_window_iterator(It begin, It end, It focus, std::ptrdiff_t index,
                          std::ptrdiff_t size)
      : begin_{begin}, end_{end}, focus_{focus}, index_{index}, size_{size} {}

  reference operator*() const { return {begin_, end_, focus_, index_, size_}; }

  sliding_window_iterator& operator++() {
    ++focus_;
    ++index_;
    return *this;
  }

  sliding_window_iterator operator++(int) {
    auto tmp = *this;
    ++(*this);
    return tmp;
  }

  friend bool operator==(sliding_window_iterator const& a,
                         sliding_window_iterator const& b) {
    return a.focus_ == b.focus_;
  }

  friend bool operator!=(sliding_window_iterator const& a,
                         sliding_window_iterator const& b) {
    return a.focus_ != b.focus_;
  }

  It begin_;
  It end_;
  It focus_;
  std::ptrdiff_t index_;
  std::ptrdiff_t size_;
};

template <std::size_t Radius, typename It>
struct sliding_window_range {
  using iterator = sliding_window_iterator<Radius, It>;

  sliding_window_range(It begin, It end, std::ptrdiff_t size)
      : begin_{begin}, end_{end}, size_{size} {}

  iterator begin() const { return iterator{begin_, end_, begin_, 0, size_}; }
  iterator end() const { return iterator{begin_, end_, end_, size_, size_}; }

  friend iterator begin(sliding_window_range const& r) { return r.begin(); }
  friend iterator end(sliding_window_range const& r) { return r.end(); }

  It begin_;
  It end_;
  std::ptrdiff_t size_;
};

template <std::size_t Radius, typename Container>
auto sliding_window(Container&& c) {
  using std::begin;
  using std::end;
  using std::size;
  return sliding_window_range<Radius, decltype(begin(c))>{
      begin(c), end(c), static_cast<std::ptrdiff_t>(size(c))};
}

}  // namespace osr
