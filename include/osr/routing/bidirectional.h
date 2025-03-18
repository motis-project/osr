#pragma once

#include "osr/types.h"
#include "osr/routing/a_star.h"

namespace osr {

struct sharing_data;

template <typename Profile>
struct bidirectional{
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using node_h = typename a_star<Profile>::node_h;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(node_h const& n) { return n.cost + n.heuristic; }
  };

  void add(ways const& w,
           label const l,
           location const& loc,
           cost_map& cost_map,
           dial<node_h, get_bucket>& d){
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(), node::invalid())){
        d.push(node_h{l, l.cost(), heuristic(w, l, loc)});
    }
  }

  void add_start(ways const& w, label const l){
    add(w, l, end_loc_, cost1_, pq1_);
  }

  void add_end(ways const& w, label const l){
    add(w, l, start_loc_, cost2_, pq2_);
  }

  void clear_mp() {meet_point = meet_point.invalid();}

  void reset(cost_t max, location const& start_loc, location const& end_loc){
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);

    meet_point = meet_point.invalid();
    cost1_.clear();
    cost2_.clear();
    expanded_start_.clear();
    expanded_end_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;

    auto const start_coord = geo::latlng_to_merc(start_loc.pos_);
    auto const end_coord = geo::latlng_to_merc(end_loc.pos_);

    auto const dx = start_coord.x_ - end_coord.x_;
    auto const dy = start_coord.y_ - end_coord.y_;
    auto const dist = std::sqrt(dx * dx + dy * dy);

    PI = Profile::heuristic(dist) / 2;
  }

  cost_t heuristic(ways const& w, label const l, location const& loc){
    auto const node_coord =
        geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const loc_coord = geo::latlng_to_merc(loc.pos_);

    auto const dx = node_coord.x_ - loc_coord.x_;
    auto const dy = node_coord.y_ - loc_coord.y_;

    auto const dist = std::sqrt(dx * dx + dy * dy);

    auto const other_loc = loc == start_loc_ ? end_loc_ : start_loc_;
    auto const other_coord = geo::latlng_to_merc(other_loc.pos_);

    auto const other_dx = node_coord.x_ - other_coord.x_;
    auto const other_dy = node_coord.y_ - other_coord.y_;

    auto const other_dist =
        std::sqrt(other_dx * other_dx + other_dy * other_dy);
    return 0.5 * (Profile::heuristic(dist) - Profile::heuristic(other_dist)) + PI*2;
  }

  cost_t get_cost_from_start(node const n) const {
    auto const it = cost1_.find(n.get_key());
    return it == cost1_.end() ? kInfeasible : it->second.cost(n);
  }

  cost_t get_cost_from_end(node const n) const {
    auto const it = cost2_.find(n.get_key());
    return it == cost2_.end() ? kInfeasible : it->second.cost(n);
  }

  cost_t get_cost_to_mp(node const n) const {
    auto cost1 = get_cost_from_start(n);
    if (cost1 == kInfeasible){
      auto rev_node = Profile::get_reverse(n);
      cost1 = get_cost_from_start(rev_node);
    }
    auto cost2 = get_cost_from_end(n);
    if (cost2 == kInfeasible){
      auto rev_node = Profile::get_reverse(n);
      cost2 = get_cost_from_end(rev_node);
    }
    if (cost1 == kInfeasible || cost2 == kInfeasible) {
      std::cout << "\nOne of the costs is infeasible: " << cost1 << " + " << cost2 << std::endl;
      return kInfeasible;
    }
    return cost1 + cost2;
  }

  template <direction SearchDir, bool WithBlocked, typename fn>
  std::optional<node> run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           dial<node_h, get_bucket>& d,
           cost_map& cost_map,
           fn get_cost,
           location& loc){

    auto curr_node_h = d.pop();

    auto l = curr_node_h.l;
    auto curr_node = l.get_node();

    if(get_cost(curr_node) < l.cost()) {
      return std::nullopt;
    }

    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
      r, curr_node, blocked, sharing,
      [&](node const neighbor, std::uint32_t const cost, distance_t,
          way_idx_t const way, std::uint16_t, std::uint16_t) {
          if (l.cost() > max - cost) {
            std::cout << "Overflow " << std::endl;
            return;
          }
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          auto const total = l.cost() + cost;
          if(total < max && cost_map[neighbor.get_key()].update(
            l, neighbor, static_cast<cost_t>(total), curr_node)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h = node_h{next, next.cost_, heuristic(w, next, loc)};
            if (next_h.cost + next_h.heuristic < max) {
              d.push(std::move(next_h));
              if constexpr (kDebug) {
                std::cout << " -> PUSH\n";
              }
            }
          } else {
            if constexpr (kDebug) {
                std::cout << " -> DOMINATED\n";
            }
          }
      });
    return curr_node;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing){
    auto best_cost = kInfeasible - PI*2;

    // next_item is are top heap values (forward and reverse)
    cost_t top_f; //cost + heuristic forward top
    cost_t top_r; //cost + heuristic reverse top

    while (!pq1_.empty() && !pq2_.empty()){
      top_f = pq1_.buckets_[pq1_.get_next_bucket()].back().priority();
      top_r = pq2_.buckets_[pq2_.get_next_bucket()].back().priority();
      if (top_f + top_r >= best_cost + PI * 2){
        break;
      }
      auto curr1 = run<SearchDir, WithBlocked>(w, r, max, blocked, sharing, pq1_, cost1_, [this](auto curr){return get_cost_from_start(curr);}, end_loc_);
      auto curr2 = run<opposite(SearchDir), WithBlocked>(w, r, max, blocked, sharing, pq2_, cost2_, [this](auto curr){return get_cost_from_end(curr);}, start_loc_);
      //A meeting point is identified, when a node is already expanded by the other search
      if (curr1 != std::nullopt){
        if (expanded_end_.contains(curr1.value().n_)) {
          if constexpr (kDebug) {
            std::cout << "  potential MEETPOINT found by start ";
            curr1.value().print(std::cout, w);
          }
          if (cost2_.find(curr1.value().get_key()) != cost2_.end()) {
            // This node has been expanded from both directions and has entries in both cost maps
            if (get_cost_to_mp(curr1.value()) < best_cost) {
              meet_point = curr1.value();
              best_cost = get_cost_to_mp(curr1.value());
              if constexpr (kDebug) {
                std::cout << " -> ACCEPTED\n";
              }
            } else if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          } else if constexpr (kDebug){
              std::cout << " -> DOMINATED\n";
          }

        }
        expanded_start_.emplace(curr1.value().n_);
      }
      if (curr2 != std::nullopt) {
        if (expanded_start_.contains(curr2.value().n_)) {
          if constexpr (kDebug) {
            std::cout << "  potential MEETPOINT found by end ";
            curr2.value().print(std::cout, w);
          }
          if (cost1_.find(curr2.value().get_key()) != cost1_.end()) {
            if (get_cost_to_mp(curr2.value()) < best_cost) {
              meet_point = curr2.value();
              best_cost = get_cost_to_mp(curr2.value());
              if constexpr (kDebug) {
                std::cout << " -> ACCEPTED\n";
              }
            } else if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          } else if constexpr (kDebug) {
            std::cout << " -> DOMINATED\n";
          }
        }
        expanded_end_.emplace(curr2.value().n_);
      }
    }
  }

  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           direction const dir) {
    if (blocked == nullptr) {
      dir == direction::kForward
          ? run<direction::kForward, false>(w, r, max, blocked, sharing)
          : run<direction::kBackward, false>(w, r, max, blocked, sharing);
    } else {
      dir == direction::kForward
          ? run<direction::kForward, true>(w, r, max, blocked, sharing)
          : run<direction::kBackward, true>(w, r, max, blocked, sharing);
    }
  }

  dial<node_h, get_bucket> pq1_{get_bucket{}};
  dial<node_h, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  hash_set<node_idx_t> expanded_start_;
  hash_set<node_idx_t> expanded_end_;
  node meet_point;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  cost_t PI;
};

}  // namespace osr
