#pragma once

#include "osr/types.h"
#include "osr/ways.h"

namespace osr::ch {

struct OrderStrategy {
  virtual ~OrderStrategy() = default;

  virtual void compute_order(
    ways const& w
  ) = 0;
  virtual bool has_next() = 0;
  virtual node_idx_t next_node() = 0;
  virtual std::vector<node_idx_t> get_node_order() = 0;
};

struct RandomOrderStrategy : public OrderStrategy {
  explicit RandomOrderStrategy(int const seed) : seed_(seed) {
    if (seed == -1) {
      seed_ = time(nullptr);
    }
    fmt::println("seeded node order with: {}", seed_);
  }
  int seed_;
  int count_{0};

  std::vector<node_idx_t> order_;

  void compute_order(ways const& w) override {
    int const n = w.n_nodes();
    std::vector<node_idx_t> result;
    for (node_idx_t idx{0U}; idx < n; ++idx) {
      result.push_back(idx);
    }
    order_ = result;
  }

  bool has_next() override {
    return !order_.empty();
  }

  node_idx_t next_node() override {
    ++count_;
    node_idx_t const node = rnd_vec_elem(order_);
    return node;
  }

  node_idx_t rnd_vec_elem(std::vector<node_idx_t>& v) {
    int n = v.size();
    srand(seed_);
    int idx = rand() % n;
    node_idx_t const result = v[idx];

    std::swap(v[idx], v[n - 1]);
    v.pop_back();
    return result;
  }
};

} // namespace osr::ch