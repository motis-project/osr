#include "osr/preprocessing/contraction_hierarchies/storage.h"

namespace osr::ch {

void shortcut_storage::save(std::filesystem::path const& path) const {
  struct serialized_data {
    vec<shortcut_data> shortcuts_;
    way_idx_t max_way_idx_;
  };
  auto data = serialized_data{shortcuts_, max_way_idx_};
  cista::write(path / "shortcuts.bin", data);
}

void shortcut_storage::load(std::filesystem::path const& path) {
  struct serialized_data {
    vec<shortcut_data> shortcuts_;
    way_idx_t max_way_idx_;
  };
  auto loaded = cista::read<serialized_data>(path / "shortcuts.bin");
  fmt::print("load {} shortcuts\n", loaded->shortcuts_.size());
  for (auto& shortcut : loaded->shortcuts_) {
    if (shortcut.loop_segments.empty()) {
      shortcuts_.push_back(shortcut);
    }else {
      vec<ShortcutSegment> loop_segments;
      for (auto const& s : shortcut.loop_segments) {
        loop_segments.push_back(ShortcutSegment{s.w,s.from,s.to,s.distance,s.cost});
      }
      shortcuts_.push_back(shortcut_data{
        shortcut.from,
            shortcut.to,
            shortcut.cost,
            shortcut.distance,
            shortcut.via_node,
            shortcut.upward_way,
            shortcut.upward_dir,
            shortcut.upward_distance,
            shortcut.upward_cost,
            shortcut.downward_dir,
            shortcut.downward_way,
            shortcut.downward_distance,
            shortcut.downward_cost,
            shortcut.selfloop_way_idx,
        loop_segments

      });
    }

  }
  max_way_idx_ = loaded->max_way_idx_;
}
way_idx_t shortcut_storage::add_shortcut(ways& w,
                                         shortcut_data const& shortcut) {
  shortcuts_.push_back(shortcut);

  way_idx_t shortcut_way = way_idx_t{w.r_->way_nodes_.size()};

  // neue Way hinzufügen
  add_shortcut_to_graph(w,shortcut,shortcut_way, true);

  construct_way_on_shortcut_arrays(w,shortcut_way, shortcut);
  return shortcut_way;
}
void shortcut_storage::add_shortcut_to_graph(ways& w,
                                                  shortcut_data const& shortcut,
                                                  way_idx_t way_idx,
                                                  bool add_to_nodes) {
  auto way_nodes = w.r_->way_nodes_.add_back_sized(0U);
  way_nodes.push_back(shortcut.from);
  way_nodes.push_back(shortcut.to);

  auto way_dists = w.r_->way_node_dist_.add_back_sized(0U);
  way_dists.push_back(shortcut.distance);

  w.r_->way_properties_.push_back(way_properties{ false, false, true, false,
                                                 true, false, false, false, 3,
                                                 0, 0, 0, false, false, false, false, false, false, 3});

  // neue Way in node hinzufügen
  if (add_to_nodes) {
    w.r_->node_ways_[shortcut.from].push_back(way_idx);
    w.r_->node_in_way_idx_[shortcut.from].push_back(0U);
    w.r_->node_ways_[shortcut.to].push_back(way_idx);
    w.r_->node_in_way_idx_[shortcut.to].push_back(1U);
  }
  //w.way_osm_idx_.push_back(osm_way_idx_t{0U});
}

void add_shortcut_restrictions(osr::ways& w,
                              const shortcut_data& shortcut, way_idx_t shortcut_way) {
  auto from = shortcut.from;
  auto to = shortcut.to;
  if (w.r_->node_is_restricted_[from]) {
    auto shortcut_pos = w.r_->get_way_pos(from, shortcut_way);
    for (auto const& r : w.r_->node_restrictions_[from]) {

      if (r.to_ == w.r_->get_way_pos(from, shortcut.upward_way) &&
          r.to_ != r.from_) {
        w.r_->node_restrictions_[from].push_back(
            restriction{r.from_, shortcut_pos});
          }
    }
  }
  if (w.r_->node_is_restricted_[to]) {
    auto shortcut_pos = w.r_->get_way_pos(to, shortcut_way);
    for (auto const& r : w.r_->node_restrictions_[to]) {
      if (r.from_ == w.r_->get_way_pos(to, shortcut.downward_way) &&
          r.to_ != r.from_) {
        w.r_->node_restrictions_[to].push_back(
            restriction{shortcut_pos, r.to_});
      }
    }
  }
}

void shortcut_storage::load_all_shortcuts_in_graph(osr::ways& w) {
  for (auto i = 0U; i < shortcuts_.size(); ++i) {
    auto const& shortcut = shortcuts_[i];
    way_idx_t shortcut_way = way_idx_t{max_way_idx_ + i + 1U};
    //add_shortcut_to_graph(w, shortcut, shortcut_way, true);
    construct_way_on_shortcut_arrays(w,shortcut_way, shortcut);
    //add_shortcut_restrictions(w, shortcut, shortcut_way);
  }
}
} // nnamespace osr::ch