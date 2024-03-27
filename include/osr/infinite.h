#pragma once

#include <cassert>

namespace osr {

template <typename BeginIt, typename EndIt>
struct infinite_iterator {
  using difference_type = typename BeginIt::difference_type;
  using value_type = typename BeginIt::value_type;
  using pointer = typename BeginIt::pointer;
  using reference = typename BeginIt::reference;
  using iterator_category = typename BeginIt::iterator_category;

  infinite_iterator(BeginIt from, EndIt to, bool const is_infinite)
      : it_{from}, begin_{from}, end_{to}, is_infinite_{is_infinite} {}

  infinite_iterator& operator++() {
    if (it_ == end_ && is_infinite_) {
      it_ = begin_;
    } else {
      ++it_;
    }
    return *this;
  }
  reference operator*() { return *it_; }
  pointer operator->() { return it_; }
  friend bool operator==(infinite_iterator const& a,
                         infinite_iterator const& b) {
    assert(a.is_infinite_ == b.is_infinite_);
    return !a.is_infinite_ && a.it_ == b.it_;
  }

  BeginIt it_;
  BeginIt begin_;
  EndIt end_;
  bool is_infinite_;
};

template <typename BeginIt, typename EndIt>
struct infinite_range {
  infinite_range(BeginIt from, EndIt to, bool const is_infinite)
      : begin_{from, to, is_infinite}, end_{to, to, is_infinite} {}

  infinite_iterator<BeginIt, EndIt> begin() const { return begin_; }
  infinite_iterator<BeginIt, EndIt> end() const { return end_; }

  friend infinite_iterator<BeginIt, EndIt> begin(infinite_range const& r) {
    return r.begin();
  }

  friend infinite_iterator<BeginIt, EndIt> end(infinite_range const& r) {
    return r.end();
  }

  infinite_iterator<BeginIt, EndIt> begin_;
  infinite_iterator<BeginIt, EndIt> end_;
};

template <typename BeginIt, typename EndIt>
infinite_range(BeginIt, EndIt, bool) -> infinite_range<BeginIt, EndIt>;

template <typename Collection>
auto infinite(Collection&& c, bool const is_reverse) {
  using std::begin;
  using std::end;
  return infinite_range{begin(c), end(c), is_reverse};
}

}  // namespace osr