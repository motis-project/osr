#pragma once

#include "osr/mmap_vec.h"

#include "cista/memory_holder.h"

#include "osr/types.h"
#include "utl/verify.h"

namespace osr {

// memory mapped map
// some key -> byte buffer
template <typename Key,
          typename SizeType = std::uint64_t,
          SizeType MinPageSize = 16U,
          SizeType MaxPageSize = 4096U>
struct mmm {
  using size_type = SizeType;

  struct page {
    page(size_type const size,
         size_type const capacity,
         size_type const start) {
      size_ = size;
      capacity_ = capacity;
      start_ = start;
    }

    page() {
      size_ = 0U;
      capacity_ = 0U;
      start_ = 0U;
    }

    friend std::ostream& operator<<(std::ostream& out, page const& p) {
      return out << "(start=" << p.start_ << ", capacity=" << p.capacity_
                 << ", size=" << p.size_ << ")";
    }

    bool valid() const { return capacity_ != 0U; }

    std::string_view view(mmm const& m) const {
      return {reinterpret_cast<char const*>(data(m)), size_};
    }
    std::uint8_t* data(mmm& m) const { return &m.data_[start_]; }
    std::uint8_t const* data(mmm const& m) const { return &m.data_[start_]; }

    void write(mmm& m, std::string_view d) const {
      std::memcpy(data(m), d.data(), d.size());
    }

    union {
      size_type next_;
      size_type size_;
    };
    size_type capacity_, start_;
  };

  static constexpr size_type free_list_index(size_type const capacity) {
    return std::countr_zero(capacity) - std::countr_zero(MinPageSize);
  }

  explicit mmm(std::filesystem::path const& p)
      : data_{cista::mmap{p.c_str()}},
        map_{cista::raw::make_unique<hash_map<Key, page>>()} {}

  std::string_view get(Key const i) const {
    auto const it = map_->find(i);
    utl::verify(it != end(*map_), "key {} not found", i);
    //    std::cerr << "  GET " << i << " => " << it->second << "\n";
    return it->second.view(*this);
  }

  void put(Key const i, std::string_view data) {
    auto const p = map_->find(i);
    if (p == end(*map_)) {
      auto const new_page = create_page(data.size());
      std::memcpy(new_page.data(*this), data.data(), data.size());
      map_->emplace_hint(p, i, new_page);
      //      std::cerr << "    => " << new_page << "\n";
    } else {
      p->second = update_page(p->second, data);
      //      std::cerr << "    => " << p->second << "\n";
    }
  }

  void erase(Key const i) {
    auto const p = map_->find(i);
    if (p != end(*map_)) {
      free_page(p->second);
      map_->erase(p);
    }
  }

  void size() const { return map_->size(); }

  page update_page(page const& p, std::string_view data) {
    if (data.size() <= p.capacity_) {
      p.write(*this, data);
      return {data.size(), p.capacity_, p.start_};
    } else {
      free_page(p);
      auto const new_page = create_page(data.size());
      //      std::cerr << "  UPDATE PAGE: " << p << " => " << new_page << ",
      //      writing "
      //                << data.size() << " bytes\n";
      new_page.write(*this, data);
      return new_page;
    }
  }

  page create_page(std::size_t const size) {
    auto const capacity = cista::next_power_of_two(std::max(MinPageSize, size));
    auto const i = free_list_index(capacity);
    if (!free_list_[i].empty()) {
      auto start = free_list_[i].pop(*this);
      //      std::cerr << "  USING FREE LIST [capacity=" << capacity << "]: "
      //      << start
      //                << "\n";
      return page{size, capacity, start};
    } else {
      //      std::cerr << "  CREATING NEW size=" << size << ", capacity=" <<
      //      capacity
      //                << " [RESIZE " << data_.size() << " -> "
      //                << (data_.size() + capacity) << "], START=" <<
      //                data_.size()
      //                << "\n";
      auto const start = data_.size();
      data_.resize(data_.size() + capacity);
      return page(size, capacity, start);
    }
    return page{};
  }

  void free_page(page p) {
    //    std::cerr << "    FREE PAGE: " << p << "\n";
    free_list_[free_list_index(p.capacity_)].push(*this, p.start_);
  }

  template <typename T>
  T read(size_type const offset) {
    static_assert(std::is_trivially_copyable_v<T>);
    auto x = T{};
    std::memcpy(&x, &data_[offset], sizeof(x));
    return x;
  }

  template <typename T>
  void write(size_type const offset, T const& x) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::memcpy(&data_[offset], &x, sizeof(T));
  }

  struct node {
    bool empty() const {
      return next_ == std::numeric_limits<size_type>::max();
    }

    void push(mmm& m, size_type const x) {
      //      std::cerr << "  PUSH curr_next=" << next_ << ", x=" << x << "\n";
      m.write(x, next_);
      next_ = x;
    }

    size_type pop(mmm& m) {
      auto const next_next = m.read<size_type>(next_);
      //      std::cerr << "  POP curr_next=" << next_ << ", next_next=" <<
      //      next_next
      //                << "\n";
      auto start = next_;
      next_ = next_next;
      return start;
    }

    size_type next_{std::numeric_limits<size_type>::max()};
  };

  mmap_vec<std::uint8_t> data_;
  cista::wrapped<hash_map<Key, page>> map_;
  std::array<node, free_list_index(MaxPageSize) + 1U> free_list_{};
};

}  // namespace osr