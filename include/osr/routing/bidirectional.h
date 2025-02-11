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

  // "get_bucket" function for the dial priority queue
  struct get_bucket {
    cost_t operator()(node_h const& n) { return n.cost + n.heuristic; }
  };

  // Add a label to the cost:map and the dial f it improved
  void add(ways const& w,
           label const l,
           location const& loc,
           cost_map& cost_map,
           dial<node_h, get_bucket>& d){
    if (cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(), node::invalid())){
        std::cout << "in the add" << std::endl;
        d.push(node_h{l, l.cost(), heuristic(w, l, loc)});
    }
  }

  void add_start(ways const& w, label const l){
    add(w, l, end_loc_, cost1_, pq1_);
    std::cout << l.get_node().n_ << " added as start" << std::endl;
  }

  void add_end(ways const& w, label const l){
    add(w, l, start_loc_, cost2_, pq2_);
    std::cout << l.get_node().n_ << " added as end" << std::endl;
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
    expanded_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
  }

  cost_t heuristic(ways const& w, label const l, location const& loc){
    auto const node_coord = geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const loc_coord = geo::latlng_to_merc(loc.pos_);

    auto const dx = node_coord.x_ - loc_coord.x_;
    auto const dy = node_coord.y_ - loc_coord.y_;

    auto const dist = std::sqrt(dx * dx + dy * dy);

    return Profile::heuristic(dist);
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
    auto const cost1 = get_cost_from_start(n);
    auto const cost2 = get_cost_from_end(n);
    if (cost1 == kInfeasible || cost2 == kInfeasible) {
      return kInfeasible;
    }
    return cost1 + cost2;
  }


  // Single-step expansion function
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

    Profile::template adjacent<SearchDir, WithBlocked>(
      r, curr_node, blocked, sharing,
      [&](node const neighbor, std::uint32_t const cost, distance_t,
          way_idx_t const way, std::uint16_t, std::uint16_t) {
        auto const total = l.cost() + cost;
        if(total < max && cost_map[neighbor.get_key()].update(
          l, neighbor, static_cast<cost_t>(total), curr_node)){
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h = node_h{next, next.cost_, heuristic(w, next, loc)};
            if(next_h.cost + next_h.heuristic < max){
              d.push(std::move(next_h));
            }
        }
      });
    return curr_node;
  }

  // Bidirectional run loop
  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing){
    auto best_cost = kInfeasible;

    // update top priorities on every loop iteration

    // next_item is are top heap values (forward and reverse)
    auto next_item_f = pq1_.buckets_[pq1_.get_next_bucket()].back(); //forward
    auto next_item_r = pq2_.buckets_[pq2_.get_next_bucket()].back(); //reverse
    auto top_f = next_item_f.priority(); //cost + heuristic forward top
    auto top_r = next_item_r.priority(); //cost + heuristic reverse top

    while (!pq1_.empty() && !pq2_.empty()){

      if ((top_f + top_r > best_cost) && best_cost != kInfeasible)
        break;

      auto curr1 = run<SearchDir, WithBlocked>(w, r, max, blocked, sharing, pq1_, cost1_, [this](auto curr){return get_cost_from_start(curr);}, end_loc_);
      auto curr2 = run<opposite(SearchDir), WithBlocked>(w, r, max, blocked, sharing, pq2_, cost2_, [this](auto curr){return get_cost_from_end(curr);}, start_loc_);

      // When a node is found that is already expanded from the other search, we have a meeting point
      if (curr1 != std::nullopt){
        if  (!expanded_.contains(curr1.value().n_)){
          std::cout << "pq1 expands " << static_cast<uint32_t>(curr1.value().n_) << std::endl;
          expanded_.emplace(curr1.value().n_);
        } else if (get_cost_to_mp(curr1.value()) < best_cost) {
          meet_point = curr1.value();
         // std::cout << "-----meet point has been found by pq1 " << static_cast<uint32_t>(meet_point.get_node()) << std::endl;
          best_cost = get_cost_to_mp(curr1.value());
        }
      }
      if (curr2 != std::nullopt) {
        if (!expanded_.contains(curr2.value().n_)) {
          std::cout << "pq2 expands " << static_cast<uint32_t>(curr2.value().n_) << std::endl;
          expanded_.emplace(curr2.value().n_);
        } else if (get_cost_to_mp(curr2.value()) < best_cost) {
          meet_point = curr2.value();
          //std::cout << "-----meet point has been found by pq2 " << static_cast<uint32_t>(meet_point.get_node()) << std::endl;
          best_cost = get_cost_to_mp(curr2.value());
        }
      }
      //std::cout << "Infisieable " << static_cast<uint32_t>(kInfeasible) << std::endl;
      // Debugging code
      auto next_bucket_idx = pq1_.get_next_bucket();
      /*if(pq1_.buckets_[next_bucket_idx].empty()){
        std::cout << "pq1 bucket empty, breaking out" << std::endl;
        break;
      }*/
      top_f = pq1_.buckets_[next_bucket_idx].back().priority();
      //std::cout << "top_f - " << static_cast<uint32_t>(top_f) << std::endl;
      top_r = pq2_.buckets_[pq2_.get_next_bucket()].back().priority();
      //std::cout << "top_r - " << static_cast<uint32_t>(top_r) << std::endl;
    }
  }

  // a wrapper to select whether blocked edges should be considered
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
  hash_set<node_idx_t> expanded_;
  node meet_point;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;

};

}  // namespace osr
