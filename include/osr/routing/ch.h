#pragma once

#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include "osr/routing/profiles/foot.h"
#include "osr/types.h"
#include "osr/ways.h"

// Minimal Contraction Hierarchy for osr's foot graph (isochrone / large-radius
// one-to-all acceleration). Built in stages, each gated on matching plain
// Dijkstra exactly.
//
// Stage 1 (this file): materialize the foot routing graph as an explicit CSR
// adjacency + a plain Dijkstra on it, so we can (a) verify the materialization
// against osr's own Dijkstra and (b) use it as the base graph for contraction.
//
// v1 simplification: vertices are node indices (levels collapsed). Levels only
// matter at a handful of multi-level structures; a level-aware version is a
// follow-up.

namespace osr {

struct ch_graph {
  std::size_t n_vertices() const noexcept {
    return first_out_.empty() ? 0U : first_out_.size() - 1U;
  }
  std::size_t n_edges() const noexcept { return edge_target_.size(); }

  // CSR adjacency
  std::vector<std::uint32_t> first_out_;    // size n+1
  std::vector<std::uint32_t> edge_target_;  // target vertex (= node idx value)
  std::vector<cost_t> edge_cost_;           // edge weight (seconds)
};

// Materialize the foot (pedestrian) routing graph into an explicit CSR graph.
inline ch_graph materialize_foot(ways const& w) {
  using P = foot<false>;
  auto const& r = *w.r_;
  auto const n = static_cast<std::uint32_t>(w.n_nodes());
  auto const params = typename P::parameters{};

  auto first_out = std::vector<std::uint32_t>(n + 1U, 0U);
  for (auto from = 0U; from != n; ++from) {
    P::template adjacent<direction::kForward, false>(
        params, r, typename P::node{node_idx_t{from}, kNoLevel}, nullptr,
        nullptr, nullptr,
        [&](typename P::node const, std::uint32_t const, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool) { ++first_out[from + 1U]; });
  }
  for (auto i = 0U; i != n; ++i) {
    first_out[i + 1U] += first_out[i];
  }

  auto const m = first_out.back();
  auto edge_target = std::vector<std::uint32_t>(m);
  auto edge_cost = std::vector<cost_t>(m);
  auto pos = first_out;  // insertion cursor
  for (auto from = 0U; from != n; ++from) {
    P::template adjacent<direction::kForward, false>(
        params, r, typename P::node{node_idx_t{from}, kNoLevel}, nullptr,
        nullptr, nullptr,
        [&](typename P::node const nb, std::uint32_t const cost, distance_t,
            way_idx_t const, std::uint16_t, std::uint16_t,
            elevation_storage::elevation, bool) {
          auto const p = pos[from]++;
          edge_target[p] = to_idx(nb.get_node());
          edge_cost[p] = static_cast<cost_t>(cost);
        });
  }

  return ch_graph{.first_out_ = std::move(first_out),
                  .edge_target_ = std::move(edge_target),
                  .edge_cost_ = std::move(edge_cost)};
}

// Plain Dijkstra one-to-all on the CSR graph (baseline + verification oracle).
// Returns cost to every vertex (kInfeasible if unreachable / beyond max).
inline std::vector<cost_t> dijkstra_all(ch_graph const& g,
                                        std::uint32_t const source,
                                        cost_t const max) {
  auto dist = std::vector<cost_t>(g.n_vertices(), kInfeasible);
  using qe = std::pair<cost_t, std::uint32_t>;  // (cost, vertex)
  auto pq = std::priority_queue<qe, std::vector<qe>, std::greater<>>{};
  dist[source] = 0U;
  pq.push({0U, source});
  while (!pq.empty()) {
    auto const [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) {
      continue;
    }
    for (auto e = g.first_out_[u]; e != g.first_out_[u + 1U]; ++e) {
      auto const v = g.edge_target_[e];
      auto const nd = static_cast<std::uint64_t>(d) + g.edge_cost_[e];
      if (nd < max && nd < dist[v]) {
        dist[v] = static_cast<cost_t>(nd);
        pq.push({dist[v], v});
      }
    }
  }
  return dist;
}

// Reverse (transpose) the CSR graph. dijkstra_all(transpose(g), d) then gives
// the cost from every vertex TO d in g -- i.e. the last-mile (many-to-one).
inline ch_graph transpose(ch_graph const& g) {
  auto const n = static_cast<std::uint32_t>(g.n_vertices());
  auto t = ch_graph{};
  t.first_out_.assign(n + 1U, 0U);
  for (auto const v : g.edge_target_) {
    ++t.first_out_[v + 1U];
  }
  for (auto i = 0U; i != n; ++i) {
    t.first_out_[i + 1U] += t.first_out_[i];
  }
  t.edge_target_.resize(g.edge_target_.size());
  t.edge_cost_.resize(g.edge_cost_.size());
  auto pos = t.first_out_;
  for (auto u = 0U; u != n; ++u) {
    for (auto e = g.first_out_[u]; e != g.first_out_[u + 1U]; ++e) {
      auto const p = pos[g.edge_target_[e]]++;
      t.edge_target_[p] = u;
      t.edge_cost_[p] = g.edge_cost_[e];
    }
  }
  return t;
}

// ---- Contraction Hierarchy (node-based) ----------------------------------
struct ch {
  std::size_t n() const noexcept { return rank_.size(); }

  std::vector<std::uint32_t> rank_;   // contraction rank per vertex
  std::vector<std::uint32_t> order_;  // vertices in increasing rank

  // up-graph: edges toward higher rank (CSR)
  std::vector<std::uint32_t> up_fo_, up_tgt_;
  std::vector<cost_t> up_cost_;
  // down-graph: for vertex u, edges (u -> lower-rank d) (CSR), used by PHAST
  std::vector<std::uint32_t> dn_fo_, dn_tgt_;
  std::vector<cost_t> dn_cost_;
};

// Bounded witness search: is there a path src->tgt with cost <= limit that
// avoids `avoid`, using current (uncontracted) out-adjacency? Conservative:
// bounded node budget; returns false if not found within budget (=> shortcut).
inline bool witness(std::vector<std::vector<std::pair<std::uint32_t, cost_t>>> const& out,
                    std::vector<char> const& contracted,
                    std::uint32_t const src, std::uint32_t const tgt,
                    std::uint32_t const avoid, cost_t const limit,
                    std::vector<cost_t>& dist, std::vector<std::uint32_t>& touched,
                    std::uint32_t const budget) {
  using qe = std::pair<cost_t, std::uint32_t>;
  std::priority_queue<qe, std::vector<qe>, std::greater<>> pq;
  dist[src] = 0U;
  touched.push_back(src);
  pq.push({0U, src});
  auto settled = 0U;
  while (!pq.empty()) {
    auto const [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) continue;
    if (u == tgt) return true;
    if (++settled > budget) return false;
    for (auto const& [v, c] : out[u]) {
      if (contracted[v] || v == avoid) continue;
      auto const nd = static_cast<std::uint64_t>(d) + c;
      if (nd <= limit && nd < dist[v]) {
        dist[v] = static_cast<cost_t>(nd);
        touched.push_back(v);
        pq.push({dist[v], v});
      }
    }
  }
  return dist[tgt] <= limit;
}

inline ch build_ch(ch_graph const& g, std::uint32_t const witness_budget = 60U) {
  auto const n = static_cast<std::uint32_t>(g.n_vertices());
  // mutable in/out adjacency (grows with shortcuts)
  auto out = std::vector<std::vector<std::pair<std::uint32_t, cost_t>>>(n);
  auto in = std::vector<std::vector<std::pair<std::uint32_t, cost_t>>>(n);
  for (auto u = 0U; u != n; ++u) {
    for (auto e = g.first_out_[u]; e != g.first_out_[u + 1U]; ++e) {
      auto const v = g.edge_target_[e];
      auto const c = g.edge_cost_[e];
      out[u].emplace_back(v, c);
      in[v].emplace_back(u, c);
    }
  }

  auto contracted = std::vector<char>(n, 0);
  auto rank = std::vector<std::uint32_t>(n, 0U);
  auto order = std::vector<std::uint32_t>{};
  order.reserve(n);
  auto wdist = std::vector<cost_t>(n, kInfeasible);
  auto touched = std::vector<std::uint32_t>{};

  auto const add_edge = [&](std::uint32_t const a, std::uint32_t const b,
                            cost_t const c) {
    for (auto& [t, w2] : out[a]) {
      if (t == b) { if (c < w2) w2 = c; return; }
    }
    out[a].emplace_back(b, c);
    in[b].emplace_back(a, c);
  };

  // count shortcuts that contracting v would need (for edge-difference)
  auto const simulate = [&](std::uint32_t const v) -> int {
    auto added = 0;
    for (auto const& [u, cu] : in[v]) {
      if (contracted[u]) continue;
      for (auto const& [w, cw] : out[v]) {
        if (contracted[w] || w == u) continue;
        auto const limit = static_cast<cost_t>(
            std::min<std::uint64_t>(static_cast<std::uint64_t>(cu) + cw, kInfeasible));
        for (auto const t : touched) wdist[t] = kInfeasible;
        touched.clear();
        if (!witness(out, contracted, u, w, v, limit, wdist, touched,
                     witness_budget)) {
          ++added;
        }
      }
    }
    auto removed = 0;
    for (auto const& [u, c] : in[v]) if (!contracted[u]) ++removed;
    for (auto const& [w, c] : out[v]) if (!contracted[w]) ++removed;
    return added - removed;
  };

  using pqe = std::pair<int, std::uint32_t>;  // (priority, vertex)
  std::priority_queue<pqe, std::vector<pqe>, std::greater<>> pq;
  for (auto v = 0U; v != n; ++v) pq.push({simulate(v), v});

  while (!pq.empty()) {
    auto const [p, v] = pq.top();
    pq.pop();
    if (contracted[v]) continue;
    auto const cur = simulate(v);  // lazy update
    if (!pq.empty() && cur > pq.top().first) {
      pq.push({cur, v});
      continue;
    }
    // contract v: insert needed shortcuts
    for (auto const& [u, cu] : in[v]) {
      if (contracted[u]) continue;
      for (auto const& [w, cw] : out[v]) {
        if (contracted[w] || w == u) continue;
        auto const sc = static_cast<cost_t>(std::min<std::uint64_t>(
            static_cast<std::uint64_t>(cu) + cw, kInfeasible));
        for (auto const t : touched) wdist[t] = kInfeasible;
        touched.clear();
        if (!witness(out, contracted, u, w, v, sc, wdist, touched,
                     witness_budget)) {
          add_edge(u, w, sc);
        }
      }
    }
    contracted[v] = 1;
    rank[v] = static_cast<std::uint32_t>(order.size());
    order.push_back(v);
  }

  // build up/down CSR from final adjacency, split by rank
  auto res = ch{};
  res.rank_ = rank;
  res.order_ = order;
  res.up_fo_.assign(n + 1U, 0U);
  res.dn_fo_.assign(n + 1U, 0U);
  for (auto u = 0U; u != n; ++u) {
    for (auto const& [v, c] : out[u]) {
      if (rank[v] > rank[u]) ++res.up_fo_[u + 1U];
      else ++res.dn_fo_[u + 1U];
    }
  }
  for (auto i = 0U; i != n; ++i) {
    res.up_fo_[i + 1U] += res.up_fo_[i];
    res.dn_fo_[i + 1U] += res.dn_fo_[i];
  }
  res.up_tgt_.resize(res.up_fo_.back());
  res.up_cost_.resize(res.up_fo_.back());
  res.dn_tgt_.resize(res.dn_fo_.back());
  res.dn_cost_.resize(res.dn_fo_.back());
  auto up_pos = res.up_fo_;
  auto dn_pos = res.dn_fo_;
  for (auto u = 0U; u != n; ++u) {
    for (auto const& [v, c] : out[u]) {
      if (rank[v] > rank[u]) {
        auto const p = up_pos[u]++;
        res.up_tgt_[p] = v;
        res.up_cost_[p] = c;
      } else {
        auto const p = dn_pos[u]++;
        res.dn_tgt_[p] = v;
        res.dn_cost_[p] = c;
      }
    }
  }
  return res;
}

// PHAST one-to-all: upward Dijkstra from source, then downward sweep in
// decreasing rank. Returns exact cost to every vertex (bounded by max).
inline std::vector<cost_t> phast_all(ch const& h, std::uint32_t const source,
                                     cost_t const max) {
  auto const n = static_cast<std::uint32_t>(h.n());
  auto dist = std::vector<cost_t>(n, kInfeasible);
  using qe = std::pair<cost_t, std::uint32_t>;
  std::priority_queue<qe, std::vector<qe>, std::greater<>> pq;
  dist[source] = 0U;
  pq.push({0U, source});
  while (!pq.empty()) {  // upward phase
    auto const [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) continue;
    for (auto e = h.up_fo_[u]; e != h.up_fo_[u + 1U]; ++e) {
      auto const v = h.up_tgt_[e];
      auto const nd = static_cast<std::uint64_t>(d) + h.up_cost_[e];
      if (nd < max && nd < dist[v]) {
        dist[v] = static_cast<cost_t>(nd);
        pq.push({dist[v], v});
      }
    }
  }
  // downward sweep: process vertices in decreasing rank
  for (auto it = h.order_.rbegin(); it != h.order_.rend(); ++it) {
    auto const u = *it;
    if (dist[u] == kInfeasible) continue;
    for (auto e = h.dn_fo_[u]; e != h.dn_fo_[u + 1U]; ++e) {
      auto const v = h.dn_tgt_[e];
      auto const nd = static_cast<std::uint64_t>(dist[u]) + h.dn_cost_[e];
      if (nd < max && nd < dist[v]) {
        dist[v] = static_cast<cost_t>(nd);
      }
    }
  }
  return dist;
}

}  // namespace osr
