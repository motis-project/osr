#include "osr/preprocessing/contraction_hierarchies/preprocessor.h"
#include <chrono>
#include <iostream>
#include "fmt/core.h"
#include "fmt/std.h"
#include "osr/lookup.h"
#include "osr/preprocessing/contraction_hierarchies/contractor.h"

#include "osr/preprocessing/contraction_hierarchies/node_order_strategy.h"
#include "osr/routing/route.h"
#include "osr/ways.h"
using namespace std::chrono;
namespace osr::ch {

void process_all_nodes(
  ways& w,
  ways const& w_without,
  bitvec<node_idx_t>* blocked,
  shortcut_storage& shortcuts,
  contractor& c,
  std::unique_ptr<OrderStrategy> const& order_strategy,
  osr::ch::node_order* order
  ) {
  int processed = 0;
  auto pt = utl::get_active_progress_tracker_or_activate("contraction-hierarchy");
  auto const total_nodes = w.n_nodes();
  pt->status("Contracting nodes").in_high(total_nodes).out_bounds(0, 100);
  while (order_strategy->has_next()) {
    node_idx_t node = order_strategy->next_node();
    order->add_node(node);
    c.contract_node(w,w_without, blocked, &shortcuts, node);
    pt->update(++processed);
  }
}

void process_ch(std::filesystem::path const& in,
                std::filesystem::path const& original_in,
                std::unique_ptr<OrderStrategy>& order_strategy) {
  fmt::println("Begin Setup for Contraction Hirachies");

  auto w = ways{in, cista::mmap::protection::READ};
  auto const w_without = ways{original_in, cista::mmap::protection::READ};
  auto node_order = osr::ch::node_order{in, cista::mmap::protection::WRITE};

  node_order.init(w.n_nodes());

  fmt::println("loaded extraction [file={}] with {} nodes and {} ways", in, w.n_nodes(), w.n_ways());

  order_strategy->compute_order(w);

  fmt::println("node order computed result has {} nodes", order_strategy->get_node_order().size());

  fmt::println("End Setup for Contraction Hirachies");
  fmt::println("Begin Contraction Hirachies Processing\n");

  auto start = high_resolution_clock::now();
  auto c = contractor();
  auto shortcuts = shortcut_storage{};

  shortcuts.set_max_way_idx(way_idx_t{w.n_ways() - 1});

  c.calculate_neighbors(w);
  bitvec<node_idx_t> blocked(w.n_nodes());
  process_all_nodes(w, w_without, &blocked, shortcuts, c, order_strategy, &node_order);

  fmt::println("Found {} shortcuts", shortcuts.shortcuts_.size());
  fmt::println("End Contraction Hirachies Processing");
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<seconds>(stop - start);
  fmt::println("Computation Time: {} s", duration.count());

  shortcuts.save(in);
  shortcuts.save(in);
  w.r_->write(in);

  fmt::println("Shortcuts saved to [file={}]", in);
}
} // namespace osr::ch