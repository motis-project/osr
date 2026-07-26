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
    max_bucket_ = std::max(max_bucket_, dist);
    ++size_;
  }

  T pop() {
    assert(!empty());
    current_bucket_ = get_next_bucket();
    assert(!buckets_[current_bucket_].empty());
    auto item = buckets_[current_bucket_].back();
    buckets_[current_bucket_].pop_back();
    --size_;
    return item;
  }

  std::size_t size() const { return size_; }

  bool empty() const { return size_ == 0; }

  void clear() {
    current_bucket_ = 0U;
    size_ = 0U;
    // Only buckets up to the highest one ever pushed can hold anything.
    // Clearing all of them dominates short searches, which is what the
    // one-to-many offset queries do (one search per rental provider).
    auto const end = std::min(static_cast<std::size_t>(max_bucket_) + 1U,
                              buckets_.size());
    for (auto i = std::size_t{0U}; i != end; ++i) {
      buckets_[i].clear();
    }
    max_bucket_ = 0U;
  }

  void n_buckets(dist_t const n) { buckets_.resize(n); }

  dist_t n_buckets() const { return static_cast<dist_t>(buckets_.size()); }

public:
  dist_t get_next_bucket() const {
    assert(size_ != 0);
    auto bucket = current_bucket_;
    while (bucket < buckets_.size() && buckets_[bucket].empty()) {
      ++bucket;
    }
    return bucket;
  }

  GetBucketFn get_bucket_;
  dist_t current_bucket_{};
  dist_t max_bucket_{};
  std::size_t size_{};
  std::vector<std::vector<T>> buckets_;
};

}  // namespace osr
