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
};

struct node_importance_order_strategy : public OrderStrategy {
  explicit node_importance_order_strategy(int const seed) : seed_(seed) {
    if (seed == -1) {
      seed_ = time(nullptr);
    }
  }
  int seed_;
  std::unordered_map<size_t, std::vector<node_idx_t>> order_;
  int current_node_importance = 0;
  void compute_order(ways const& w) override {
    int const n = w.n_nodes();
    for (node_idx_t idx{0U}; idx < n; ++idx) {
      order_[w.r_->node_properties_[idx].importance_].push_back(idx);
    }
  }

  bool has_next() override {
    if (order_[current_node_importance].empty()) {
      ++current_node_importance;
    }
    return !order_[current_node_importance].empty();
  }

  node_idx_t next_node() override {
    node_idx_t const node = rnd_vec_elem(order_[current_node_importance]);
    return node;
  }

  node_idx_t rnd_vec_elem(std::vector<node_idx_t>& v) const {
    auto const n = v.size();
    srand(seed_);
    auto const idx = rand() % n;
    node_idx_t const result = v[idx];

    std::swap(v[idx], v[n - 1]);
    v.pop_back();
    return result;
  }
};

struct RandomOrderStrategy : public OrderStrategy {
  explicit RandomOrderStrategy(int const seed) : seed_(seed) {
    if (seed == -1) {
      seed_ = time(nullptr);
    }
    fmt::println("seeded node order with: {}", seed_);
  }
  int seed_;
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