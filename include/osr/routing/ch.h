#pragma once

#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
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

// Materialize osr's EXACT (node, level) foot routing graph into a CSR graph.
// Vertices [0, n_nodes) are the base (node, level-0/kNoLevel) states; extra
// (node, level>0) states (elevators / multi-level structures) get appended ids
// as they are discovered through the same adjacent() osr routes with -- so the
// CH is built on osr's exact graph, not a level-collapsed approximation.
inline ch_graph materialize_foot(ways const& w) {
  using P = foot<false>;
  using fnode = typename P::node;
  auto const& r = *w.r_;
  auto const n = static_cast<std::uint32_t>(w.n_nodes());
  auto const params = typename P::parameters{};

  auto const is_base = [](fnode const nd) {
    return nd.lvl_ == kNoLevel || to_idx(nd.lvl_) == 0U;
  };
  auto extra = std::unordered_map<std::uint64_t, std::uint32_t>{};
  auto vert = std::vector<fnode>{};
  vert.reserve(n);
  for (auto i = 0U; i != n; ++i) {
    vert.push_back(fnode{node_idx_t{i}, kNoLevel});
  }
  auto next_id = n;
  auto const get_id = [&](fnode const nd) -> std::uint32_t {
    if (is_base(nd)) {
      return to_idx(nd.get_node());
    }
    auto const key = (static_cast<std::uint64_t>(to_idx(nd.get_node())) << 8) |
                     to_idx(nd.lvl_);
    auto const it = extra.find(key);
    if (it != extra.end()) {
      return it->second;
    }
    auto const id = next_id++;
    extra.emplace(key, id);
    vert.push_back(nd);
    return id;
  };

  // discover all edges (vert grows as extra (node,level) states appear)
  auto src_of = std::vector<std::uint32_t>{};
  auto edge_target = std::vector<std::uint32_t>{};
  auto edge_cost = std::vector<cost_t>{};
  for (auto id = 0U; id < vert.size(); ++id) {
    auto const src = vert[id];  // copy: get_id may reallocate vert
    P::template adjacent<direction::kForward, false>(
        params, r, src, nullptr, nullptr, nullptr,
        [&](fnode const nb, std::uint32_t const cost, distance_t, way_idx_t,
            std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
          src_of.push_back(id);
          edge_target.push_back(get_id(nb));
          edge_cost.push_back(static_cast<cost_t>(cost));
        });
  }

  // build CSR over V = n_nodes + #extra vertices
  auto const V = static_cast<std::uint32_t>(vert.size());
  auto first_out = std::vector<std::uint32_t>(V + 1U, 0U);
  for (auto const s : src_of) {
    ++first_out[s + 1U];
  }
  for (auto i = 0U; i != V; ++i) {
    first_out[i + 1U] += first_out[i];
  }
  auto tgt = std::vector<std::uint32_t>(edge_target.size());
  auto cst = std::vector<cost_t>(edge_cost.size());
  auto pos = first_out;
  for (auto e = 0U; e != src_of.size(); ++e) {
    auto const p = pos[src_of[e]]++;
    tgt[p] = edge_target[e];
    cst[p] = edge_cost[e];
  }
  return ch_graph{.first_out_ = std::move(first_out),
                  .edge_target_ = std::move(tgt),
                  .edge_cost_ = std::move(cst)};
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
