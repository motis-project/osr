#include "osr/ways.h"

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
  std::cout << "Entering Area Ways!\n";
  std::cout << "Entering Area Ways!\n";
  generate_area_ways();
  std::cout << "Exiting Area Ways!\n";
  std::cout << "Exiting Area Ways!\n";

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
            dists.push_back(static_cast<std::uint16_t>(std::round(distance)));
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
  /*
  //Gather IDs and Positions for all Nodes in Way.
  vec<osm_node_idx_t> nv;
  vec<point> pv;
  //first element is vector of outer nodes, further elements are vectors of inner nodes.
  //Way-Area only has single loop.
  vec<vec<osm_node_idx_t>> nodes;
  vec<vec<point>> positions;

  //Save area way IDs.
  pair<osm_way_idx_t, vec<osm_way_idx_t>> outer_inner_pair;
  outer_inner_pair.first = osm_way_idx_t{way.positive_id()};
  outer_inner_pair.second = vec<osm_way_idx_t>{};
  area_polygons_.push_back(outer_inner_pair);

  //Gather Node IDs and Positions.
  for (auto n : way.nodes()) {
    nv.push_back(osm_node_idx_t{n.positive_ref()});
    pv.push_back(point::from_location(n.location()));
  }

  nodes.emplace_back(nv);
  area_polygon_nodes_.emplace_back(nodes);
  positions.emplace_back(pv);
  area_polygon_node_positions_.emplace_back(positions);
  */

    pair<vec<pair<osm_way_idx_t, bool>>, vec<vec<pair<osm_way_idx_t, bool>>>> area_description;

    pair<osm_way_idx_t, bool> onlyOuterWay = {(osm_way_idx_t)way.positive_id(), true};

    vec<pair<osm_way_idx_t, bool>> outerLoop;

    outerLoop.push_back(onlyOuterWay);

    vec<vec<pair<osm_way_idx_t, bool>>> innerLoops = {};

    area_description.first = outerLoop;
    area_description.second = innerLoops;

    final_area_vector_.push_back(area_description);

    vec<osm_node_idx_t> nv;

    // Gather Node IDs and Positions.
    for (auto n : way.nodes()) {
      nv.push_back(osm_node_idx_t{n.positive_ref()});
      final_area_node_positions_.insert({(osm_node_idx_t)n.positive_ref(), point::from_location(n.location())});
    }
    final_area_nodes_.insert({onlyOuterWay.first, nv});


}

void ways::generate_area_ways() {
  //Generate Visibility Graph for each Area.
  //An Area is a single Multipolygon.
  //It may consist of one or multiple outer Ways and
  //none or multiple inner Ways.
  //One Multipolygon Relation may contain multiple Multipolygons.
  //This is the case when a single Relation contains multiple closed sets
  //of outer Ways that describe multiple seperate Areas.

  //For each area, single closed outer ring, optional with inner rings
  for (uint32_t i = 0; i < final_area_vector_.size(); i++) {
    build_visibility_graph(i);
  }
}

void ways::build_visibility_graph(uint32_t area_index) {

  //Size of Matrix.
  uint32_t n = get_area_number_of_nodes(area_index);
  
  //Size of outer Way.
  uint32_t m = get_area_number_of_outer_nodes(area_index);
 
  vecvec<double, double> distances;

  //Prepare distance Matrix
  for (uint32_t i = 0; i < n; i++) {
    vec<double> new_column;
    for (uint32_t j = 0; j < n; j++) {
      new_column.push_back(INFINITY);
    }
    new_column[i] = 0;
    distances.emplace_back(new_column);
  }

  auto positions_of_outer_loop = get_area_loop_locations(get_area_outer_loop(area_index));

  //Set adjacend distances
  for (uint32_t j = 0; j < m - 1; j++) {

    point p0 = positions_of_outer_loop[j];
    point p1 = positions_of_outer_loop[j + 1];

    double d = geo::distance(p0, p1);

    distances[j][j + 1] = d;
    distances[j + 1][j] = d;
  }

  uint32_t i = m;
  vec<vec<osm_node_idx_t>> inner_loops = get_area_inner_loops(area_index);
  for (vec<osm_node_idx_t> l : inner_loops) {
    m = l.size();
    vec<point> locations = get_area_loop_locations(l);
    for (uint32_t j = 0; j < m - 1; j++) {
      point p0 = locations[j];
      point p1 = locations[j + 1];
      double d = geo::distance(p0, p1);
      distances[i][i + 1] = d;
      distances[i + 1][i] = d;
      i++;
    }
    //TODO-J: Check if distance from last to first is calculated later on of if it need manual work here.
  }
  nodes_of_current_area_ = get_area_nodes(area_index);
  positions_of_current_area_ = get_area_locations(area_index);
  
  for (uint32_t i = 0; i < n; i++) {
    calc_visibility(area_index, distances, i, i + 1, n, positions_of_outer_loop,
                    inner_loops);
  }
  
  vecvec<uint32_t, uint32_t> next;
  // Prepare next Matrix
  for (uint32_t i = 0; i < n; i++) {
    vec<uint32_t> new_column;
    for (uint32_t j = 0; j < n; j++) {
      new_column.push_back(std::numeric_limits<uint32_t>::max());
    }
    next.emplace_back(new_column);
  }

  //Simplify vis-graph
  reduce_visibility_graph(area_index, distances, next, 0, n);

  //Transfer vis-graph to routing-graph.
  extract_reduced_visibility_graph(area_index, distances, next);
}

void ways::calc_visibility(uint32_t area_index,
                           vecvec<double, double>& distances,
                           uint32_t i,
                           uint32_t start,
                           uint32_t matrix_size,
                           vec<point> outer_loop,
                           vec<vec<osm_node_idx_t>> inner_loops) {
  uint32_t n = matrix_size;
  bool visibility = false;

  for (uint32_t j = start; j < n; j++) {

    if (i == j || (nodes_of_current_area_[i] == nodes_of_current_area_[j])) {
      continue;
    }
    vec<point> seg = {positions_of_current_area_[i], positions_of_current_area_[j]};

    if (check_for_intersection(seg, outer_loop)) {
      continue;
    }

    visibility = true;

    for (vec<osm_node_idx_t> l : inner_loops) {
        if (check_for_intersection(seg, get_area_loop_locations(l))) {
            visibility = false;
            break;
        }
    }

    if (visibility) {

      point p0 = positions_of_current_area_[i];
      point p1 = positions_of_current_area_[j];
      double d = geo::distance(p0, p1);
      distances[i][j] = d;
      distances[j][i] = d;
    }

  }

}

void ways::add_virtual_way(uint32_t area_index,
                           vec<osm_node_idx_t> nodes_on_way) {

  max_osm_way_idx_++;
  way_osm_idx_.push_back(osm_way_idx_t{max_osm_way_idx_}); 

  //Copy properties from first way in outer loop.
  osm_way_idx_t w_id = final_area_vector_[area_index].first[0].first;
  auto opt_way = find_way(w_id);
  way_idx_t way_id = *opt_way;
  way_properties prop = r_->way_properties_[way_id];


  vec<point> pointVec;
  
  uint32_t next_index = 0;
  for (osm_node_idx_t n : nodes_on_way) {
    for (uint32_t i = 0; i < nodes_of_current_area_.size(); i++) {
      if (nodes_of_current_area_[i] == n) {
        next_index = i;
        break;
      }
    }
    pointVec.push_back(positions_of_current_area_[next_index]);
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
      prev_point = ring.back();
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
  bool result = !(is_point_in_polygon(sample_position, ring));
  
  
  return result;
}

bool ways::do_lines_cross(vec<point> lineA, vec<point> lineB) { 
  //Works for line-segments (vec<point> with exactly 2 elements). Returns true iff lines cross each other.
  //Based on https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
  /*
  double x1 = lineA[0].lat_;
  double y1 = lineA[0].lng_;
  double x2 = lineA[1].lat_;
  double y2 = lineA[1].lng_;
  double x3 = lineB[0].lat_;
  double y3 = lineB[0].lng_;
  double x4 = lineB[1].lat_;
  double y4 = lineB[1].lng_;
  */

  //Project lines to origin relative to end of first line.
  //Avoids potential precision issues at large latlng values.
  double x1 = (int)lineA[0].lat_ - (int)lineA[1].lat_;
  double y1 = (int)lineA[0].lng_ - (int)lineA[1].lng_;
  double x2 = 0;
  double y2 = 0;
  double x3 = (int)lineB[0].lat_ - (int)lineA[1].lat_;
  double y3 = (int)lineB[0].lng_ - (int)lineA[1].lng_;
  double x4 = (int)lineB[1].lat_ - (int)lineA[1].lat_;
  double y4 = (int)lineB[1].lng_ - (int)lineA[1].lng_;



 
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
  
  point a_;
  point b_;
  //Reasonably far-away trace with slight angle.
  a_ = point(p.lat_ + 100000, p.lng_ - 250);
  b_ = p;
  vec<point> trace = {a_, b_};
  
  //Count Crossings
  uint32_t crossings = 0;
  for (uint32_t j = 0; j < polygon.size() - 1; j++) {
    vec<point> seg = {polygon[j], polygon[j + 1]};
    if (do_lines_cross(trace, seg)) {
      crossings++;
    }
  }
  vec<point> seg = {polygon[polygon.size()-1], polygon[0]};
  if (do_lines_cross(trace, seg)) {
    crossings++;
  }

  // Even number of crossings indicates the point is outside of the polygon.
  return !(crossings % 2 == 0);
}

void ways::reduce_visibility_graph(
    uint32_t area_index, vecvec<double, double>& distances,
                                   vecvec<uint32_t, uint32_t>& next,
                                   uint32_t start,
                                   uint32_t n) {

  //Copy shortest direct Paths from distances to next.
  for (uint32_t i = 0; i < n; i++) {
    for (uint32_t j = start; j < n - 1; j++) {
      if (distances[i][j] != INFINITY && distances[i][j] != 0) {
        next[i][j] = j;
        next[j][i] = i;
      }
    }
  }

  //Create Paths in next
  for (uint32_t k = 0; k < n; k++) {
    for (uint32_t i = 0; i < n; i++) {
      for (uint32_t j = start; j < n; j++) {
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
void ways::extract_reduced_visibility_graph(uint32_t area_index,
                                            vecvec<double, double>& distances,
                                            vecvec<uint32_t, uint32_t>& next) {

  //Only Create Paths from access nodes.
  vec<osm_node_idx_t> access_nodes;

  for (auto n : nodes_of_current_area_) {
    if (node_way_counter_.is_multi(to_idx(n))) {
      access_nodes.push_back(n);
    }
  }

  //Add Paths between all access nodes.
  for (uint32_t i = 0; i < access_nodes.size(); i++) {
    for (uint32_t j = i + 1; j < access_nodes.size(); j++) {
      vec<osm_node_idx_t> nodes_on_way = trace_visibility_graph(area_index, next, access_nodes[i], access_nodes[j]);
      add_virtual_way(area_index, nodes_on_way);
    }
  }

}
vec<osm_node_idx_t> ways::trace_visibility_graph(uint32_t area_index,
                                                 vecvec<uint32_t, uint32_t>& next,
                                                 osm_node_idx_t start, osm_node_idx_t end) {

  vec<osm_node_idx_t> path;
  path.push_back(start);
  osm_node_idx_t current_node = start;
  uint32_t current_ind = osm_to_area_index(nodes_of_current_area_, current_node);
  uint32_t final_ind = osm_to_area_index(nodes_of_current_area_, end);
  

  while (current_node != end) {
    current_ind = next[current_ind][final_ind];
    current_node = nodes_of_current_area_[current_ind];
    path.push_back(current_node);
  }
  return path;
}
uint32_t ways::osm_to_area_index(vec<osm_node_idx_t> relevant_nodes, osm_node_idx_t node_id) {

  uint32_t counter = 0;
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

uint32_t ways::get_area_number_of_nodes(uint32_t area_index) { 
    
    return get_area_number_of_outer_nodes(area_index) + get_area_number_of_inner_nodes(area_index);
}

uint32_t ways::get_area_number_of_outer_nodes(uint32_t area_index) { 

  vec<pair<osm_way_idx_t, bool>> outer_loops = final_area_vector_[area_index].first;

  uint32_t outer_loop_node_counter = 0;

  for (pair<osm_way_idx_t, bool> e : outer_loops) {
    outer_loop_node_counter += final_area_nodes_[e.first].size() - 1;
  }

  return outer_loop_node_counter;
}

uint32_t ways::get_area_number_of_inner_nodes(uint32_t area_index) {

  vec<vec<pair<osm_way_idx_t, bool>>> inner_loops = final_area_vector_[area_index].second;

  uint32_t inner_loop_node_counter = 0;

  for (uint32_t i = 0; i < inner_loops.size(); i++) {
    for (uint32_t j = 0; j < inner_loops[i].size(); j++) {
      osm_way_idx_t current_way = inner_loops[i][j].first;
      inner_loop_node_counter += final_area_nodes_[current_way].size() - 1;
    }
  }

  return inner_loop_node_counter;
}

vec<osm_node_idx_t> ways::get_area_outer_loop(uint32_t area_index) {

  vec<osm_node_idx_t> outer_loop;
  vec<pair<osm_way_idx_t, bool>> outer_loop_data = final_area_vector_[area_index].first;

  ordered_area_loop_vector(outer_loop, outer_loop_data);

  return outer_loop;
}
vec<vec<osm_node_idx_t>> ways::get_area_inner_loops(uint32_t area_index) {

  vec<vec<osm_node_idx_t>> inner_loops;
  vec<vec<pair<osm_way_idx_t, bool>>> inner_loops_data = final_area_vector_[area_index].second;

  for (uint32_t i = 0; i < inner_loops_data.size(); i++) {
    vec<pair<osm_way_idx_t, bool>> current_inner_loop_data = inner_loops_data[i];
    vec<osm_node_idx_t> current_inner_loop;
    ordered_area_loop_vector(current_inner_loop, current_inner_loop_data);
    inner_loops.emplace_back(current_inner_loop);
  }

  return inner_loops;
}

vec<osm_node_idx_t> ways::get_area_nodes(uint32_t area_index) {

  vec<osm_node_idx_t> nodes_of_area = get_area_outer_loop(area_index);

  vec<vec<osm_node_idx_t>> inner_nodes_of_area = get_area_inner_loops(area_index);

  for (vec<osm_node_idx_t> l : inner_nodes_of_area) {

    for (osm_node_idx_t n : l) {
      nodes_of_area.emplace_back(n);
    }
  }

  return nodes_of_area;
}

vec<vec<osm_node_idx_t>> ways::get_area_nodes_by_loops(uint32_t area_index) {

  vec<vec<osm_node_idx_t>> nodes_of_area;
  nodes_of_area.emplace_back(get_area_outer_loop(area_index));

  for (vec<osm_node_idx_t> l : get_area_inner_loops(area_index)) {
    nodes_of_area.emplace_back(l);
  }

  return nodes_of_area;
}

void ways::ordered_area_loop_vector(vec<osm_node_idx_t>& loop, vec<pair<osm_way_idx_t, bool>> loop_data) {

    for (uint32_t i = 0; i < loop_data.size(); i++) {

    osm_way_idx_t current_way = loop_data[i].first;
    bool direction = loop_data[i].second;
    vec<osm_node_idx_t> current_way_nodes = final_area_nodes_[current_way];

    if (!direction) {
      for (uint32_t j = current_way_nodes.size() - 1; j > 0; j--) {
        loop.push_back(current_way_nodes[j-1]);
      }
    } else {
      for (uint32_t j = 0; j < current_way_nodes.size() - 1; j++) {
        loop.push_back(current_way_nodes[j]);
      }
    }
  }

}

vec<point> ways::get_area_loop_locations(vec<osm_node_idx_t> nodes_of_loop) {

  vec<point> loop_locations;

  for (osm_node_idx_t n : nodes_of_loop) {
    loop_locations.push_back(final_area_node_positions_[n]);
  }

  return loop_locations;
}

vec<vec<point>> ways::get_area_inner_loop_locations(vec<vec<osm_node_idx_t>> nodes_of_inner_loops) {
  vec<vec<point>> inner_loops_locations;

  for (uint32_t i = 0; i < nodes_of_inner_loops.size(); i++) {
    vec<osm_node_idx_t> current_inner_loop = nodes_of_inner_loops[i];
    inner_loops_locations.push_back(get_area_loop_locations(current_inner_loop));
  }

  return inner_loops_locations;
}

vec<point> ways::get_area_locations(uint32_t area_index) {
  vec<osm_node_idx_t> nodes_of_area = get_area_nodes(area_index);
  return get_area_loop_locations(nodes_of_area);
}

void ways::debugPrintArea(uint32_t area_index) {
  std::cout << "Area Debug Print of area " << area_index << ": \n ";

  std::cout << "Area Outer WayIDs:\n";

  for (pair<osm_way_idx_t, bool> w : final_area_vector_[area_index].first) {
    std::cout << w.first << " in direction " << w.second << ",\n";

    std::cout << "    Nodes of way: \n";
    vec<osm_node_idx_t> nodes_of_way = final_area_nodes_[w.first];
    for (osm_node_idx_t n : nodes_of_way) {
      std::cout << "  " << n << " with position "
                << final_area_node_positions_[n] << ",\n";
    }
  }
}

cista::wrapped<ways::routing> ways::routing::read(
    std::filesystem::path const& p) {
  return cista::read<ways::routing>(p / "routing.bin");
}

void ways::routing::write(std::filesystem::path const& p) const {
  return cista::write(p / "routing.bin", *this);
}

}  // namespace osr