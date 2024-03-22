#pragma once

#include <cassert>
#include <algorithm>
#include <vector>

namespace osr {

template <typename T,
          typename GetBucketFn /* GetBucketFn(T) -> size_t <= MaxBucket */>
struct dial {
  using dist_t =
      std::decay_t<decltype(std::declval<GetBucketFn>()(std::declval<T>()))>;

  dial() = default;

  explicit dial(GetBucketFn get_bucket = GetBucketFn())
      : get_bucket_(std::forward<GetBucketFn>(get_bucket)) {}

  template <typename El>
  void push(El&& el) {
    auto const dist = get_bucket_(el);
    assert(dist < buckets_.size());

    buckets_[dist].emplace_back(std::forward<El>(el));
    current_bucket_ = std::min(current_bucket_, dist);
    ++size_;
  }

  T const& top() {
    assert(!empty());
    current_bucket_ = get_next_bucket();
    assert(!buckets_[current_bucket_].empty());
    return buckets_[current_bucket_].back();
  }

  void pop() {
    assert(!empty());
    current_bucket_ = get_next_bucket();
    buckets_[current_bucket_].pop_back();
    --size_;
  }

  std::size_t size() const { return size_; }

  bool empty() const { return size_ == 0; }

  void clear() {
    current_bucket_ = 0U;
    size_ = 0U;
    for (auto& b : buckets_) {
      b.clear();
    }
  }

  void n_buckets(dist_t const n) { buckets_.resize(n); }

  dist_t n_buckets() const { return static_cast<dist_t>(buckets_.size()); }

private:
  dist_t get_next_bucket() const {
    assert(size_ != 0);
    auto bucket = current_bucket_;
    while (bucket < buckets_.size() && buckets_[bucket].empty()) {
      ++bucket;
    }
    return bucket;
  }

  GetBucketFn get_bucket_;
  dist_t current_bucket_;
  std::size_t size_;
  std::vector<std::vector<T>> buckets_;
};

}  // namespace osr