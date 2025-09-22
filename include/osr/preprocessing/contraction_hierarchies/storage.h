#pragma once
#include <stack>
#include "osr/types.h"
#include "osr/ways.h"

namespace osr::ch {

struct node_order {
  mm_vec_map<node_idx_t, uint64_t> order_to_store_;
  uint64_t order_count_{0U};
  node_order(std::filesystem::path const& p, cista::mmap::protection const mode)
      : order_to_store_(cista::mmap{
            (p / "node_order.bin").generic_string().c_str(), mode}) {}

  void init(size_t const& n) {
    order_count_ = 0U;
    for (size_t idx{0U}; idx < n; ++idx) {
      order_to_store_.push_back(0U);
    }
  }

  void add_node(node_idx_t const n, bool const stall = false) {
    order_to_store_[n] =
        stall ? order_count_ : order_count_++;  // store the current count and
                                                // increment it afterwards
  }

  uint64_t get_order(node_idx_t const n) { return order_to_store_[n]; }
};

struct ShortcutSegment {
  way_idx_t w;
  node_idx_t from;
  node_idx_t to;
  distance_t distance;
  std::uint32_t cost;
};

struct shortcut_data {
  const node_idx_t from;
  const node_idx_t to;
  const uint32_t cost;
  const distance_t distance;
  const node_idx_t via_node;  // node that was contracted
  const way_idx_t upward_way;  // from -> via
  const direction upward_dir;  // direction of upward_way
  const distance_t upward_distance;  // distance of upward_way
  const uint32_t upward_cost;  // cost of upward_way
  const direction downward_dir;  // direction of downward_way
  const way_idx_t downward_way;  // via -> to
  const distance_t downward_distance;  // distance of downward_way
  const uint32_t downward_cost;  // cost of downward_way
  const way_idx_t
      selfloop_way_idx;  // special shortcut for selfloop (used to contract a 3
                         // way pair where one is a self loop)
  vec<ShortcutSegment> loop_segments{};
};
struct way_and_dir {
  way_idx_t way;
  direction dir;
  way_pos_t pos;
};
struct shortcut_storage {

  vec<shortcut_data> shortcuts_;

  std::unique_ptr<node_order> node_order_;
  std::vector<way_pos_t> shortcut_way_poses_at_to_node;
  std::vector<way_pos_t> shortcut_way_poses_at_from_node;
  std::vector<bool> shortcut_has_restricted_nodes;
  way_idx_t max_way_idx_;  // This will contain the last non shortcut way_idx_t
                           // so if there are 10 ways it will be 9

  std::vector<way_and_dir> first_way_on_shortcut;
  std::vector<way_and_dir> last_way_on_shortcut;

  void set_max_way_idx(way_idx_t const max_way_idx) {
    max_way_idx_ = max_way_idx;
  }

  bool shortcut_has_restricted_node(way_idx_t const way) {
    if (!is_shortcut(way)) {
      return false;
    }
    return shortcut_has_restricted_nodes[to_shortcut_idx(way)];
  }

  [[nodiscard]] way_pos_t get_way_pos_at_to_node(
      way_idx_t const way_idx) const {
    if (!is_shortcut(way_idx)) {
      throw std::invalid_argument("way index is no shortcut");
    }
    return shortcut_way_poses_at_to_node[to_shortcut_idx(way_idx)];
  }

  [[nodiscard]] way_pos_t get_way_pos_at_from_node(
      way_idx_t const way_idx) const {
    if (!is_shortcut(way_idx)) {
      throw std::invalid_argument("way index is no shortcut");
    }
    return shortcut_way_poses_at_from_node[to_shortcut_idx(way_idx)];
  }

  [[nodiscard]] uint64_t get_node_order(node_idx_t const n) const {
    return node_order_->get_order(n);
  }

  [[nodiscard]] bool is_shortcut(way_idx_t const idx) const {
    return idx > max_way_idx_;
  }

  [[nodiscard]] shortcut_data const* get_shortcut(way_idx_t const idx) const {
    if (!is_shortcut(idx)) {
      return nullptr;
    }
    return &shortcuts_[to_shortcut_idx(idx)];
  }

  [[nodiscard]] unsigned int to_shortcut_idx(way_idx_t const idx) const {
    return cista::to_idx(idx) - cista::to_idx(max_way_idx_) - 1;
  }

  [[nodiscard]] bool shortcut_is_selfloop(way_idx_t const idx) const {
    if (!is_shortcut(idx)) {
      return false;
    }
    auto const& s = get_shortcut(idx);
    return s->from == s->to;
  }

  void save(std::filesystem::path const& p) const;

  void load(std::filesystem::path const& p);

  [[nodiscard]] way_idx_t resolve_first_way(way_idx_t const way) const {
    if (!is_shortcut(way)) return way;
    return first_way_on_shortcut[to_shortcut_idx(way)].way;
  }

  [[nodiscard]] way_and_dir resolve_first_way_and_dir(
      way_idx_t const way,
      direction const dir,
      way_pos_t const way_pos = way_pos_t{0U}) const {
    if (!is_shortcut(way)) return way_and_dir{way, dir, way_pos};
    return first_way_on_shortcut[to_shortcut_idx(way)];
  }

  [[nodiscard]] way_and_dir resolve_last_way_and_dir(
      way_idx_t const way,
      direction const dir,
      way_pos_t const way_pos = way_pos_t{0U}) const {
    if (!is_shortcut(way)) return way_and_dir{way, dir, way_pos};
    return last_way_on_shortcut[to_shortcut_idx(way)];
  }
  struct SegmentFrame {
    way_idx_t w;
    shortcut_data const* s;
    bool is_left_child;
  };
  [[nodiscard]] std::vector<ShortcutSegment> get_shortcut_segments(
      way_idx_t const way) const {
    if (way == way_idx_t::invalid() || !is_shortcut(way)) {
      return {};
    }
    std::vector<ShortcutSegment> out = {};
    std::stack<SegmentFrame> st;
    auto const sh = get_shortcut(way);
    if (!sh->loop_segments.empty()) {
      for (auto const& s : sh->loop_segments) {
        out.push_back(s);
      }
      return out;
    }
    st.push({sh->downward_way, sh, false});
    if (cista::to_idx(sh->selfloop_way_idx) !=
        std::numeric_limits<std::uint32_t>::max()) {
      auto const sh_selfloop = get_shortcut(sh->selfloop_way_idx);
      st.push({sh->selfloop_way_idx, sh_selfloop,
               false});  // is_left_child is not important here
    }
    st.push({sh->upward_way, sh, true});

    while (!st.empty()) {
      const auto& [w, s, is_left_child] = st.top();
      st.pop();

      if (!is_shortcut(w)) {
        if (is_left_child) {
          out.push_back(ShortcutSegment{w, s->from, s->via_node,
                                        s->upward_distance, s->upward_cost});
        } else {
          out.push_back(ShortcutSegment{
              w, s->via_node, s->to, s->downward_distance, s->downward_cost});
        }
        continue;
      }
      if (auto const& sh_data = get_shortcut(w);
          !sh_data->loop_segments.empty()) {
        for (auto const& seg : sh_data->loop_segments) {
          out.push_back(seg);
        }
      } else {
        st.push({sh_data->downward_way, sh_data, false});
        if (cista::to_idx(sh_data->selfloop_way_idx) !=
            std::numeric_limits<std::uint32_t>::max()) {
          auto const sh_selfloop = get_shortcut(sh_data->selfloop_way_idx);
          st.push({sh_data->selfloop_way_idx, sh_selfloop,
                   false});  // is_left_child is not important here
        }
        st.push({sh_data->upward_way, sh_data, true});
      }
    }
    return out;
  }

  way_idx_t add_shortcut(ways& w, shortcut_data const& shortcut);

  void construct_way_on_shortcut_arrays(ways const& ways,
                                        way_idx_t const shortcut_way,
                                        shortcut_data const& shortcut) {
    if (to_shortcut_idx(shortcut_way) != first_way_on_shortcut.size()) {
      throw std::runtime_error("shortcut construction failed");
    }
    shortcut_way_poses_at_to_node.push_back(
        ways.r_->get_way_pos(shortcut.to, shortcut_way, 1));
    shortcut_way_poses_at_from_node.push_back(
        ways.r_->get_way_pos(shortcut.from, shortcut_way, 0));
    shortcut_has_restricted_nodes.push_back(
        ways.r_->node_is_restricted_[shortcut.from] ||
        ways.r_->node_is_restricted_[shortcut.via_node] ||
        ways.r_->node_is_restricted_[shortcut.to] ||
        shortcut_has_restricted_node(shortcut.upward_way) ||
        shortcut_has_restricted_node(shortcut.downward_way));
    if (is_shortcut(shortcut.upward_way)) {
      first_way_on_shortcut.push_back(
          first_way_on_shortcut[to_shortcut_idx(shortcut.upward_way)]);
    } else {
      first_way_on_shortcut.push_back(way_and_dir{
          shortcut.upward_way, shortcut.upward_dir,
          ways.r_->get_way_pos(shortcut.from, shortcut.upward_way)});
    }
    if (is_shortcut(shortcut.downward_way)) {
      last_way_on_shortcut.push_back(
          last_way_on_shortcut[to_shortcut_idx(shortcut.downward_way)]);
    } else {
      last_way_on_shortcut.push_back(way_and_dir{
          shortcut.downward_way, shortcut.downward_dir,
          ways.r_->get_way_pos(shortcut.to, shortcut.downward_way)});
    }
  }

  static void add_shortcut_to_graph(osr::ways& w,
                                    shortcut_data const& shortcut,
                                    way_idx_t way_idx,
                                    bool add_to_nodes);

  void load_all_shortcuts_in_graph(osr::ways& w);
};

}  // namespace osr::ch