#pragma once

#include "osr/types.h"
#include "osr/ways.h"

namespace osr::ch {

struct order_strategy {
  virtual ~order_strategy() = default;

  virtual void compute_order(ways const& w) = 0;
  virtual bool has_next() = 0;
  virtual node_idx_t next_node() = 0;
};

struct node_importance_order_strategy final : public order_strategy {
  explicit node_importance_order_strategy(int const seed) : seed_(seed) {
    if (seed == -1) {
      seed_ = time(nullptr);
    }
    fmt::println("seeded node order with: {}", seed_);
  }
  int seed_;
  std::vector<std::vector<node_idx_t>> order_;
  size_t max_importance_ = 0;
  size_t current_node_importance = 0;
  void compute_order(ways const& w) override {
    int const n = w.n_nodes();
    for (node_idx_t idx{0U}; idx < n; ++idx) {
      auto const importance = w.r_->node_properties_[idx].importance_;
      if (importance > max_importance_) {
        max_importance_ = importance;
      }
      if (importance >= order_.size()) {
        order_.resize(importance + 1);
      }
      order_[importance].push_back(idx);
    }
  }

  bool has_next() override {
    while (order_[current_node_importance].empty()) {
      if (current_node_importance == max_importance_) {
        break;
      }
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

struct random_order_strategy final : public order_strategy {
  explicit random_order_strategy(int const seed) : seed_(seed) {
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

  bool has_next() override { return !order_.empty(); }

  node_idx_t next_node() override {
    node_idx_t const node = rnd_vec_elem(order_);
    return node;
  }

  node_idx_t rnd_vec_elem(std::vector<node_idx_t>& v) const {
    auto const n = v.size();
    srand(seed_);
    size_t const idx = rand() % n;
    node_idx_t const result = v[idx];

    std::swap(v[idx], v[n - 1]);
    v.pop_back();
    return result;
  }
};

}  // namespace osr::ch