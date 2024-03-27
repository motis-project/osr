#pragma once

#include <iterator>

namespace osr {

template <typename It>
struct reverse_iterator {
  using difference_type = typename It::difference_type;
  using value_type = typename It::value_type;
  using pointer = typename It::pointer;
  using reference = typename It::reference;
  using iterator_category = typename It::iterator_category;

  reverse_iterator(It it, bool const is_reverse)
      : it_{it},
        rit_{std::make_reverse_iterator(it)},
        is_reverse_{is_reverse} {}

  reverse_iterator& operator++() {
    if (is_reverse_) {
      ++rit_;
    } else {
      ++it_;
    }
    return *this;
  }
  reference operator*() { return is_reverse_ ? *rit_ : *it_; }
  pointer operator->() { return is_reverse_ ? rit_ : it_; }

  friend bool operator==(reverse_iterator const& a, reverse_iterator const& b) {
    assert(a.is_reverse_ == b.is_reverse_);
    return a.is_reverse_ ? a.rit_ == b.rit_ : a.it_ == b.it_;
  }

  It it_;
  std::reverse_iterator<It> rit_;
  bool is_reverse_;
};

template <typename It>
struct reverse_range {
  reverse_range(It from, It to, bool const is_reverse)
      : begin_{is_reverse ? to : from, is_reverse},
        end_{is_reverse ? from : to, is_reverse} {}

  reverse_iterator<It> begin() { return begin_; }
  reverse_iterator<It> end() { return end_; }

  friend reverse_iterator<It> begin(reverse_range const& r) {
    return r.begin();
  }

  friend reverse_iterator<It> end(reverse_range const& r) { return r.end(); }

  reverse_iterator<It> begin_;
  reverse_iterator<It> end_;
};

template <typename It>
reverse_range(It, It, bool) -> reverse_range<It>;

template <typename Collection>
auto reverse(Collection&& c, bool const is_reverse) {
  using std::begin;
  using std::end;
  return reverse_range{begin(c), end(c), is_reverse};
}

}  // namespace osr