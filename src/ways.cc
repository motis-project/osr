#include "osr/ways.h"

#include <algorithm>

#include "utl/parallel_for.h"

#include "cista/io.h"

namespace osr {

ways::ways(std::filesystem::path p, cista::mmap::protection const mode)
    : p_{std::move(p)},
      mode_{mode},
      r_{mode == cista::mmap::protection::READ
             ? routing::read(p_)
             : cista::wrapped<routing>{cista::raw::make_unique<routing>()}},
      node_to_osm_{mm("node_to_osm.bin")},
      way_osm_idx_{mm("way_osm_idx.bin")},
      way_polylines_{mm_vec<point>{mm("way_polylines_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_polylines_index.bin")}},
      way_osm_nodes_{mm_vec<osm_node_idx_t>{mm("way_osm_nodes_data.bin")},
                     mm_vec<std::uint64_t>{mm("way_osm_nodes_index.bin")}},
      strings_{mm_vec<char>(mm("strings_data.bin")),
               mm_vec<std::uint64_t>(mm("strings_idx.bin"))},
      way_names_{mm("way_names.bin")},
      way_has_conditional_access_no_{
          mm_vec<std::uint64_t>(mm("way_has_conditional_access_no"))},
      way_conditional_access_no_{mm("way_conditional_access_no")},
      areas_ {},
      area_indices_{},
      area_nodes_{},
      area_node_positions_{} {}

void ways::build_components() {
  auto q = hash_set<way_idx_t>{};
  auto flood_fill = [&](way_idx_t const way_idx, component_idx_t const c) {
    assert(q.empty());
    q.insert(way_idx);
    while (!q.empty()) {
      auto const next = *q.begin();
      q.erase(q.begin());
      for (auto const n : r_->way_nodes_[next]) {
        for (auto const w : r_->node_ways_[n]) {
          auto& wc = r_->way_component_[w];
          if (wc == component_idx_t::invalid()) {
            wc = c;
            q.insert(w);
          }
        }
      }
    }
  };

  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Build components").in_high(n_ways()).out_bounds(75, 90);

  auto next_component_idx = component_idx_t{0U};
  r_->way_component_.resize(n_ways(), component_idx_t::invalid());
  for (auto i = 0U; i != n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto& c = r_->way_component_[way_idx];
    if (c != component_idx_t::invalid()) {
      continue;
    }
    c = next_component_idx++;
    flood_fill(way_idx, c);
    pt->increment();
  }
}

void ways::add_restriction(std::vector<resolved_restriction>& rs) {
  using it_t = std::vector<resolved_restriction>::iterator;
  utl::sort(rs, [](auto&& a, auto&& b) { return a.via_ < b.via_; });
  utl::equal_ranges_linear(
      begin(rs), end(rs), [](auto&& a, auto&& b) { return a.via_ == b.via_; },
      [&](it_t const& lb, it_t const& ub) {
        auto const range = std::span{lb, ub};
        r_->node_restrictions_.resize(to_idx(range.front().via_) + 1U);
        r_->node_is_restricted_.set(range.front().via_, true);

        for (auto const& x : range) {
          if (x.type_ == resolved_restriction::type::kNo) {
            r_->node_restrictions_[x.via_].push_back(
                restriction{r_->get_way_pos(x.via_, x.from_),
                            r_->get_way_pos(x.via_, x.to_)});
          } else /* kOnly */ {
            for (auto const [i, from] :
                 utl::enumerate(r_->node_ways_[x.via_])) {
              for (auto const [j, to] :
                   utl::enumerate(r_->node_ways_[x.via_])) {
                if (x.from_ == from && x.to_ != to) {
                  r_->node_restrictions_[x.via_].push_back(restriction{
                      static_cast<way_pos_t>(i), static_cast<way_pos_t>(j)});
                }
              }
            }
          }
        }
      });
  r_->node_restrictions_.resize(node_to_osm_.size());
}

void ways::compute_big_street_neighbors() {
  struct state {
    hash_set<way_idx_t> done_;
  };

  auto pt = utl::get_active_progress_tracker();

  auto is_orig_big_street = std::vector<bool>(n_ways());
  for (auto const [i, p] : utl::enumerate(r_->way_properties_)) {
    is_orig_big_street[i] = p.is_big_street();
  }

  utl::parallel_for_run_threadlocal<state>(
      n_ways(), [&](state& s, std::size_t const i) {
        auto const way = way_idx_t{i};

        if (is_orig_big_street[to_idx(way)]) {
          pt->update_monotonic(i);
          return;
        }

        s.done_.clear();

        auto const expand = [&](way_idx_t const x, bool const go_further,
                                auto&& recurse) {
          for (auto const& n : r_->way_nodes_[x]) {
            for (auto const& w : r_->node_ways_[n]) {
              if (is_orig_big_street[to_idx(w)]) {
                r_->way_properties_[way].is_big_street_ = true;
                return true;
              }

              if (s.done_.emplace(w).second && go_further) {
                if (recurse(x, false, recurse)) {
                  return true;
                }
              }
            }
          }
          return false;
        };

        s.done_.emplace(way);
        expand(way, true, expand);
        pt->update_monotonic(i);
      });
}

void ways::connect_ways() {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  {  // Assign graph node ids to every node with >1 way.
    pt->status("Create graph nodes")
        .in_high(node_way_counter_.size())
        .out_bounds(40, 50);

    auto node_idx = node_idx_t{0U};
    node_way_counter_.multi_.for_each_set_bit([&](std::uint64_t const b_idx) {
      auto const i = osm_node_idx_t{b_idx};
      node_to_osm_.push_back(i);
      ++node_idx;
      pt->update(b_idx);
    });
    r_->node_is_restricted_.resize(to_idx(node_idx));
  }

  generate_area_ways();

  // Build edges.
  {
    pt->status("Connect ways")
        .in_high(way_osm_nodes_.size())
        .out_bounds(50, 75);
    auto node_ways = mm_paged_vecvec<node_idx_t, way_idx_t>{
        cista::paged<mm_vec32<way_idx_t>>{
            mm_vec32<way_idx_t>{mm("tmp_node_ways_data.bin")}},
        mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
            mm("tmp_node_ways_index.bin")}};
    auto node_in_way_idx = mm_paged_vecvec<node_idx_t, std::uint16_t>{
        cista::paged<mm_vec32<std::uint16_t>>{
            mm_vec32<std::uint16_t>{mm("tmp_node_in_way_idx_data.bin")}},
        mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
            mm("tmp_node_in_way_idx_index.bin")}};
    node_ways.resize(node_to_osm_.size());
    node_in_way_idx.resize(node_to_osm_.size());
    for (auto const [osm_way_idx, osm_nodes, polyline] :
         utl::zip(way_osm_idx_, way_osm_nodes_, way_polylines_)) {
      auto pred_pos = std::make_optional<point>();
      auto from = node_idx_t::invalid();
      auto distance = 0.0;
      auto i = std::uint16_t{0U};
      auto way_idx = way_idx_t{r_->way_nodes_.size()};
      auto dists = r_->way_node_dist_.add_back_sized(0U);
      auto nodes = r_->way_nodes_.add_back_sized(0U);
      for (auto const [osm_node_idx, pos] : utl::zip(osm_nodes, polyline)) {
        if (pred_pos.has_value()) {
          distance += geo::distance(pos, *pred_pos);
        }

        if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
          auto const to = get_node_idx(osm_node_idx);
          node_ways[to].push_back(way_idx);
          node_in_way_idx[to].push_back(i);
          nodes.push_back(to);

          if (from != node_idx_t::invalid()) {
            auto const dist = static_cast<distance_t>(std::round(distance));
            if (dist < std::numeric_limits<std::uint16_t>::max()) {
              dists.push_back(static_cast<std::uint16_t>(dist));
            } else {
              r_->long_way_node_dist_.push_back(routing::long_distance{
                  .way_ = way_idx,
                  .node_ = static_cast<std::uint16_t>(i - 1U),
                  .distance_ = dist});
              dists.push_back(std::numeric_limits<std::uint16_t>::max());
            }
          }

          distance = 0.0;
          from = to;

          if (i == std::numeric_limits<std::uint16_t>::max()) {
            fmt::println("error: way with {} nodes", osm_way_idx);
          }

          ++i;
        }

        pred_pos = pos;
      }
      pt->increment();
    }

    std::sort(begin(r_->long_way_node_dist_), end(r_->long_way_node_dist_));

    for (auto const x : node_ways) {
      r_->node_ways_.emplace_back(x);
    }
    for (auto const x : node_in_way_idx) {
      r_->node_in_way_idx_.emplace_back(x);
    }
  }

  auto e = std::error_code{};
  std::filesystem::remove(p_ / "tmp_node_ways_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_ways_index.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_data.bin", e);
  std::filesystem::remove(p_ / "tmp_node_in_way_idx_index.bin", e);
}

void ways::sync() {
  node_to_osm_.mmap_.sync();
  way_osm_idx_.mmap_.sync();
  way_polylines_.data_.mmap_.sync();
  way_polylines_.bucket_starts_.mmap_.sync();
  way_osm_nodes_.data_.mmap_.sync();
  way_osm_nodes_.bucket_starts_.mmap_.sync();
  strings_.data_.mmap_.sync();
  strings_.bucket_starts_.mmap_.sync();
  way_names_.mmap_.sync();
}

void ways::build_area_ways_mapping(const osmium::Way& way) {

  //Gather IDs and Positions for all Nodes in Way.
  vec<osm_node_idx_t> nv;
  vec<point> pv;
  areas_.push_back(osm_way_idx_t{way.positive_id()});
  for (auto n : way.nodes()) {
    nv.push_back(osm_node_idx_t{n.positive_ref()});
    pv.push_back(point::from_location(n.location()));
  }
  area_nodes_.emplace_back(nv);
  area_node_positions_.emplace_back(pv);
}

void ways::generate_area_ways() {
  //Generate Visibility Graph for each Area.
  for (int i = 0; i < (int)areas_.size(); i++) {

    build_visibility_graph(i);
  }
}

void ways::build_visibility_graph(uint64_t index) {

  int n = (int)area_nodes_[index].size();
  vecvec<double, double> distances;

  
  //Prepare distance Matrix
  for (int i = 0; i < n; i++) {
    vec<double> new_column;
    for (int j = 0; j < n; j++) {
      new_column.push_back(INFINITY);
    }
    new_column[i] = 0;
    distances.emplace_back(new_column);
  }

auto positions_of_area = area_node_positions_[index];

  //Set adjacend distances
  //TODO-J: Set m to size of outer nodes later
  int m = n;
  for (int i = 0; i < m - 1; i++) {
    
    point p0 = positions_of_area[i];
    point p1 = positions_of_area[i+1];

    double d = geo::distance(p0, p1);

    distances[i + 1][i] = d;
    distances[i][i + 1] = d;
  }

  //TODO-J:
  //Add Inner Ring Handling

  
  for (int i = 0; i < n; i++) {
    calc_visibility(index, distances, i, i+1);
  }

  vecvec<int, int> next;
  // Prepare next Matrix
  for (int i = 0; i < n; i++) {
    vec<int> new_column;
    for (int j = 0; j < n; j++) {
      new_column.push_back(std::numeric_limits<int>::max());
    }
    next.emplace_back(new_column);
  }
  //Simplify vis-graph
  reduce_visibility_graph(index, distances, next, 0);
  
  //Transfer vis-graph to routing-graph.
  extract_reduced_visibility_graph(index, distances, next);

}

void ways::calc_visibility(uint64_t index,
                           vecvec<double, double>& distances,
                           int i,
                           int start) {
  int n = (int)area_nodes_[index].size();
  bool visibility = false;
  auto positions_of_area = area_node_positions_[index];
  for (int j = start; j < n; j++) {
    
    if (i == j || (area_nodes_[index][i] == area_nodes_[index][j])) {
      continue;
    }

    vec<point> seg = {area_node_positions_[index][i],
                      area_node_positions_[index][j]};

    // TODO-J: Fill with data from outerRing only instead of all data. m instead of n
    vec<point> ring;
    for (int k = 0; k < n-1; k++) {
      ring.push_back(area_node_positions_[index][k]);
    }

    if (check_for_intersection(seg, ring)) {
      continue;
    }
    visibility = true;
    //TODO-J:
    // For each inner Ring:
    // Fill ring
    // if(check_for_intersection(seg, ring)){
    // visibility = false;
    // break;
    //}

    if (visibility) {
      point p0 = positions_of_area[i];
      point p1 = positions_of_area[j];
      double d = geo::distance(p0, p1);
      distances[i][j] = d;
      distances[j][i] = d;
    }
  }

}

void ways::add_virtual_way(uint64_t area_index, vec<osm_node_idx_t> nodes_on_way) {

  max_osm_way_idx_++;
  way_osm_idx_.push_back(osm_way_idx_t{max_osm_way_idx_}); 

  osm_way_idx_t w_id = areas_[(int)area_index];
  auto opt_way = find_way(w_id);
  way_idx_t way_id = *opt_way;
  way_properties prop = r_->way_properties_[way_id];
  vec<point> pointVec;
  auto area_nodes = area_nodes_[area_index];
  auto area_positions = area_node_positions_[area_index];
  int next_index = 0;
  for (auto n : nodes_on_way) {
    for (int i = 0; i < area_nodes.size(); i++) {
      if (area_nodes[i] == n) {
        next_index = i;
        break;
      }
    }
    pointVec.push_back(area_positions[next_index]);
  }

  way_polylines_.emplace_back(pointVec);
  way_osm_nodes_.emplace_back(nodes_on_way);
  r_->way_properties_.emplace_back(prop);
}

bool ways::check_for_intersection(vec<point> line, vec<point> ring) {

  point prev_point;
  bool first = true;
  for (auto r : ring) {
    if (first) {
      first = false;
      prev_point = r;
      continue;
    }

    vec<point> current_ring_segment = {prev_point, r};

    if (do_lines_cross(line, current_ring_segment)) {
      return true;
    }
    prev_point = r;
  }

  //If no crossings were found, sample line-mid-point,
  // to determine if the line is inside the area-way.
  // If there is a part of the way with a concave indentation, the corner nodes on both sides
  // of the cavity can see each other without crossing any edges.
  //           
  //        inside of area
  //      |   o-------o      /
  //      |   |       |     /
  //      |   |       |    /
  //      o___o- - - -o---o
  //
  //        outside of area
  //
  //This leads to connections outside of the area (dashed line: - - - -).
  //By checking if the line center is inside or outside, we can discard lines
  // that leave the area-way without crossing any other edges.
  //

  point a = line[0];
  point b = line[1];

  point sample_position;
  sample_position.lat_ = (a.lat_ + b.lat_) / 2;
  sample_position.lng_ = (a.lng_ + b.lng_) / 2;
  return !(is_point_in_polygon(sample_position, ring));



}

bool ways::do_lines_cross(vec<point> lineA, vec<point> lineB) { 
  //Works for line-segments (vec<point> with exactly 2 elements). Returns true iff lines cross each other.
  //Based on https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line

  double x1 = lineA[0].lat_;
  double y1 = lineA[0].lng_;
  double x2 = lineA[1].lat_;
  double y2 = lineA[1].lng_;
  double x3 = lineB[0].lat_;
  double y3 = lineB[0].lng_;
  double x4 = lineB[1].lat_;
  double y4 = lineB[1].lng_;
 
  //Parallel lines can never "cross" each other. Avoids Div-by-0.
  double div = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

  if (div == 0.0) {
    return false;
  }

  double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / div;

  double u = -(((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / div);
  
  return (0.0 < t && t < 1.0 && 0.0 < u && u < 1.0);
}

bool ways::is_point_in_polygon(point p, vec<point> polygon) { 

  //Based on https://en.wikipedia.org/wiki/Point_in_polygon
  //Used to disqualify potential edges between nodes that pass outside of the way-area.
  //Returns true iff the point is contained inside the polygon.
  int tracer_length = 100000;
  point a_;
  point b_;
  a_ = point(p.lat_ + tracer_length, p.lng_);
  b_ = p;
  vec<point> trace = {a_, b_};

  //Count Crossings
  int crossings = 0;
  for (int j = 0; j < (int)polygon.size() - 1; j++) {
    vec<point> seg = {polygon[j], polygon[j + 1]};
    if (do_lines_cross(trace, seg)) {
      crossings++;
    }
  }

  //Even number of crossings indicates the point is outside of the polygon.
  return !(crossings % 2 == 0);
}

void ways::reduce_visibility_graph(
    uint64_t index, vecvec<double, double>& distances,
                                   vecvec<int, int>& next,
                                   int start) {

  int n = (int)area_nodes_[index].size();
  //Copy shortest direct Paths from distances to next.
  for (int i = 0; i < n; i++) {
    for (int j = start; j < n - 1; j++) {
      if (distances[i][j] != INFINITY && distances[i][j] != 0) {
        next[i][j] = j;
        next[j][i] = i;
      }
    }
  }

  //Create Paths in next
  for (int k = 0; k < n; k++) {
    for (int i = 0; i < n; i++) {
      for (int j = start; j < n; j++) {
        if (distances[i][k] == INFINITY || distances[k][j] == INFINITY) {
          continue;
        }
        if (distances[i][j] > distances[i][k] + distances[k][j]) {
          distances[i][j] = distances[i][k] + distances[k][j];
          distances[j][i] = distances[i][k] + distances[k][j];
          next[i][j] = next[i][k];
          next[j][i] = next[j][k];
        }
      }
    }
  }


}
void ways::extract_reduced_visibility_graph(uint64_t index,
                                            vecvec<double, double>& distances,
                                            vecvec<int, int>& next) {

  //Only Create Paths from access nodes.
  auto way_nodes = area_nodes_[index];
  vec<osm_node_idx_t> access_nodes;

  for (auto n : way_nodes) {
    if (node_way_counter_.is_multi(to_idx(n))) {
      access_nodes.push_back(n);
    }
  }

  //Add Paths between all access nodes.
  for (int i = 0; i < (int)access_nodes.size(); i++) {
    for (int j = i + 1; j < (int)access_nodes.size(); j++) {
      vec<osm_node_idx_t> nodes_on_way = trace_visibility_graph(index, next, access_nodes[i], access_nodes[j]);
      add_virtual_way(index, nodes_on_way);
    }
  }

}
vec<osm_node_idx_t> ways::trace_visibility_graph(uint64_t index,
                                                 vecvec<int, int>& next,
                                                 osm_node_idx_t start,
                                                 osm_node_idx_t end) {

  vec<osm_node_idx_t> path;
  path.push_back(start);
  osm_node_idx_t current_node = start;
  int current_ind = osm_to_area_index(index, current_node);
  int final_ind = osm_to_area_index(index, end);
  
  while (current_node != end) {
    current_ind = next[current_ind][final_ind];
    current_node = area_nodes_[index][current_ind];
    path.push_back(current_node);
  }
  return path;
}
int ways::osm_to_area_index(uint64_t index, osm_node_idx_t node_id) {

    auto relevant_nodes = area_nodes_[index];
  int counter = 0;
    while (node_id != relevant_nodes[counter] && counter < relevant_nodes.size()) {
    counter++;
    }
    return counter;

}
template <typename T>
void ways::debug_print_matrix(vecvec<T, T> matrix, int size) {
  int n = size;
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < n; i++) {
      std::cout << matrix[i][j] << " | ";
    }

  std::cout << "\n";
  for (int h = 0; h < n * 3 + n * 5; h++) {
    std::cout << "-";
  }
  std::cout << "\n";
  }

}

std::optional<std::string_view> ways::get_access_restriction(
    way_idx_t const way) const {
  if (!way_has_conditional_access_no_.test(way)) {
    return std::nullopt;
  }
  auto const it = std::lower_bound(
      begin(way_conditional_access_no_), end(way_conditional_access_no_), way,
      [](auto&& a, auto&& b) { return a.first < b; });
  utl::verify(
      it != end(way_conditional_access_no_) && it->first == way,
      "access restriction for way with access restriction not found way={}",
      way_osm_idx_[way]);
  return strings_[it->second].view();
}

cista::wrapped<ways::routing> ways::routing::read(
    std::filesystem::path const& p) {
  return cista::read<ways::routing>(p / "routing.bin");
}

void ways::routing::write(std::filesystem::path const& p) const {
  return cista::write(p / "routing.bin", *this);
}

}  // namespace osr
