#include "osr/ways.h"


#include "utl/parallel_for.h"

#include "cista/io.h"


#include "boost/polygon/voronoi.hpp"
using namespace boost::polygon;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;


// Selects the area routing mode to use.
// 0: Default Behaviour - Follows ways along the outline of an area.
// 1: Visibility Graph -  Computes Routes between all access points of an area,
// taking the optimal path. 
// 2: Voronoi Segment Graph - Computes a Voronoi Segment for each area,
// resulting in more realistic paths. (only 90% complete- Polylines have to be computed and inserted via method add_virtual_way().
// Only works if allow_area_routing == true in extract.cc!
constexpr uint8_t area_routing_mode = 0;


//Voronoi Structures Begin



namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
  typedef point_concept type;
};

template <>
struct point_traits<Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(const Point& point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <>
struct geometry_concept<Segment> {
  typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef Point point_type;

  static inline point_type get(const Segment& segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};
} 
} 
// Voronoi Structures End

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
      way_conditional_access_no_{mm("way_conditional_access_no")} {}

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
    float upper_bound = 50;
    if constexpr (area_routing_mode > 0) {
      upper_bound = 45;
    }

    pt->status("Create graph nodes")
        .in_high(node_way_counter_.size())
        .out_bounds(40, upper_bound);

    auto node_idx = node_idx_t{0U};
    node_way_counter_.multi_.for_each_set_bit([&](std::uint64_t const b_idx) {
      auto const i = osm_node_idx_t{b_idx};
      node_to_osm_.push_back(i);
      ++node_idx;
      pt->update(b_idx);
    });
    r_->node_is_restricted_.resize(to_idx(node_idx));
  }
  

  if constexpr (area_routing_mode == 1) {
    pt->status("Generating Visibility Graph Routing Data")
        .in_high(internal_area_vector_.size())
        .out_bounds(45, 60);
    // Generate Visibility Graph-Areas
    generate_visibility_graph_areas();
  } else if constexpr (area_routing_mode == 2) {
    pt->status("Generating Voronoi Segment Graph Routing Data")
        .in_high(internal_area_vector_.size())
        .out_bounds(45, 60);
    //Generate Voronoi Segment Graph-Areas
    generate_voronoi_segment_areas();
  }

  

  // Build edges.
  {

    float lower_bound = 50;
    if constexpr (area_routing_mode > 0) {
      lower_bound = 60;
    }

    pt->status("Connect ways")
        .in_high(way_osm_nodes_.size())
        .out_bounds(lower_bound, 75);
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


void ways::generate_visibility_graph_areas() {
  //Generate Visibility Graph for each Area.
  //An Area consists of 1 outer ring and multiple optional inner rings.
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  uint64_t progress_counter = 0;
  for (auto area: internal_area_vector_) {

    build_visibility_graph(area.first, area.second.first, area.second.second);
    progress_counter++;
    pt->update(progress_counter);
  }
}


void ways::build_visibility_graph(uint64_t area_index, uint64_t original_id, bool id_is_from_way) {


  //Size of Matrix.
  uint32_t n = get_area_number_of_nodes(area_index) - get_area_number_of_rings(area_index);
  
  //Size of outer Way.
  uint32_t m = get_area_number_of_outer_nodes(area_index) - 1;
 
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

  //Cache outer loop data for reuse.
  auto positions_of_outer_loop = get_area_ring_locations(get_area_outer_ring(area_index));

  //Set adjacend distances for outer and inner loops.
  for (uint32_t j = 0; j < m - 1; j++) {

    point p0 = positions_of_outer_loop[j];
    point p1 = positions_of_outer_loop[j + 1];

    double d = geo::distance(p0, p1);

    distances[j][j + 1] = d;
    distances[j + 1][j] = d;
  }

  uint32_t i = m;
  vec<vec<osm_node_idx_t>> inner_loops = get_area_inner_rings(area_index);
  for (vec<osm_node_idx_t> l : inner_loops) {
    m = l.size();
    vec<point> locations = get_area_ring_locations(l);
    for (uint32_t j = 0; j < m - 1; j++) {
      point p0 = locations[j];
      point p1 = locations[j + 1];
      double d = geo::distance(p0, p1);
      distances[i][i + 1] = d;
      distances[i + 1][i] = d;
      i++;
    }
  }

  //Cache area data for reuse.
  nodes_of_current_area_ = get_area_nodes(area_index);
  positions_of_current_area_ = get_area_locations(area_index);

  //Calculate Line-of-sight between all nodes.
  for (uint32_t i = 0; i < n; i++) {
    calc_visibility(distances, i, i + 1, n, positions_of_outer_loop, inner_loops);
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
  reduce_visibility_graph(distances, next, 0, n);


  //Transfer vis-graph to routing-graph.
  extract_reduced_visibility_graph(area_index, distances, next, original_id, id_is_from_way);
}


void ways::calc_visibility(vecvec<double, double>& distances, uint32_t i, uint32_t start, uint32_t matrix_size, vec<point> outer_loop, vec<vec<osm_node_idx_t>> inner_loops) {

  uint32_t n = matrix_size;
  bool visibility = false;

  point p0 = positions_of_current_area_[i];
  
  for (uint32_t j = start; j < n; j++) {
    if (i == j || (nodes_of_current_area_[i] == nodes_of_current_area_[j])) {
      continue;
    }

    vec<point> seg = {positions_of_current_area_[i], positions_of_current_area_[j]};
    


    if (check_for_intersection(seg, outer_loop, false)) {
      continue;
    }
    
    visibility = true;

    

    for (vec<osm_node_idx_t> l : inner_loops) {
        if (check_for_intersection(seg, get_area_ring_locations(l), true)) {
          visibility = false;
          break;
        }
    }

    
    if (visibility) {
      point p1 = positions_of_current_area_[j];
      double d = geo::distance(p0, p1);
      distances[i][j] = d;
      distances[j][i] = d;
    }
  }
}


void ways::add_virtual_way(uint64_t area_index, uint64_t original_id, bool id_is_from_way, vec<osm_node_idx_t> nodes_on_way) {

  max_osm_way_idx_++;
  way_osm_idx_.push_back(osm_way_idx_t{max_osm_way_idx_}); 

  way_properties way_prop = internal_area_properties_[area_index];

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

  r_->way_properties_.emplace_back(way_prop);

}


bool ways::check_for_intersection(vec<point> line, vec<point> ring, bool ring_is_inner) {

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
  bool result = !(is_point_in_polygon(sample_position, ring));
  
  //Flip Result if comparing a gainst inner ring.
  //Here we WANT the point to be outside the ring.
  if (ring_is_inner) {
    result = !result;
  }
  
  return result;
}


bool ways::do_lines_cross(vec<point> lineA, vec<point> lineB) { 
  //Works for line-segments (vec<point> with exactly 2 elements). Returns true iff lines cross each other.
  //Based on https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line

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

  // Even number of crossings indicates the point is outside of the polygon.
  return !(crossings % 2 == 0);
}


void ways::reduce_visibility_graph(vecvec<double, double>& distances, vecvec<uint32_t, uint32_t>& next, uint32_t start, uint32_t n) {

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


void ways::extract_reduced_visibility_graph(uint64_t area_index, vecvec<double, double>& distances, vecvec<uint32_t, uint32_t>& next, uint64_t original_id, bool id_is_from_way) {
  
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

      vec<osm_node_idx_t> nodes_on_way = trace_visibility_graph(next, access_nodes[i], access_nodes[j]);

      add_virtual_way(area_index, original_id, id_is_from_way, nodes_on_way);

    }
  }

}


vec<osm_node_idx_t> ways::trace_visibility_graph(vecvec<uint32_t, uint32_t>& next, osm_node_idx_t start, osm_node_idx_t end) {
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

void ways::generate_voronoi_segment_areas() {


    auto pt = utl::get_active_progress_tracker_or_activate("osr");
    uint64_t progress_counter = 0;
    for (auto area : internal_area_vector_) {

      
        build_voronoi_graph(area.first, area.second.first, area.second.second);
        progress_counter++;
        pt->update(progress_counter);

    }

}

void ways::build_voronoi_graph(uint64_t area_index,
                               uint64_t original_id,
                               bool id_is_from_way) {




  vec<osm_node_idx_t> relevant_outer_nodes = get_area_outer_ring(area_index);
  vec<vec<osm_node_idx_t>> relevant_inner_ring_nodes =
      get_area_inner_rings(area_index);

  // Construct complete list of relevant nodes for area by ring. Start-End
  // Duplicate for each ring.
  vec<vec<osm_node_idx_t>> relevant_nodes_by_ring;
  relevant_nodes_by_ring.push_back(relevant_outer_nodes);
  for (auto inner_ring : relevant_inner_ring_nodes) {
    relevant_nodes_by_ring.push_back(inner_ring);
  }

  // Construct flat vector that contains each node only once and does not
  // separate by ring.
  vec<osm_node_idx_t> relevant_unique_nodes_of_area;
  for (auto ring : relevant_nodes_by_ring) {
    for (uint32_t i = 0; i < ring.size() - 1; i++) {
      relevant_unique_nodes_of_area.push_back(ring[i]);
    }
  }

  // Generates Input segments for voronoi diagram from rings.
  std::vector<Segment> area_segments = extract_area_segments(
      area_index, relevant_outer_nodes, relevant_inner_ring_nodes);

  // Generates Voronoi Data
  voronoi_diagram<double> vd;
  construct_voronoi(area_segments.begin(), area_segments.end(), &vd);



  // Filters out infinite edges and those inside of inner rings.
  std::vector<boost::polygon::voronoi_edge<double>> vor_edg =
      get_relevant_voronoi_edges(vd.edges(), area_index);

  // Get all Access Nodes of area.
  std::set<osm_node_idx_t> access_nodes;
  std::set<osm_node_idx_t> irrelevant_nodes;
  for (uint32_t i = 0; i < relevant_unique_nodes_of_area.size(); i++) {
    bool is_access =
        node_way_counter_.multi_[(uint64_t)relevant_unique_nodes_of_area[i]];
    if (is_access) {
      access_nodes.insert(relevant_unique_nodes_of_area[i]);
    } else {
      irrelevant_nodes.insert(relevant_unique_nodes_of_area[i]);
    }
  }



  // Remove all edges that connect to irrelevant nodes.
  vor_edg = remove_irrelevant_edges(vor_edg, irrelevant_nodes);


  vec<osm_node_idx_t> access_node_vector(access_nodes.begin(),
                                         access_nodes.end());

  // Find all valid node locations created by the voronoi diagram
  std::vector<osr::point> additional_Locations =
      find_additional_nodes(vor_edg, access_node_vector, relevant_outer_nodes,
                            relevant_inner_ring_nodes);

  vec<osm_node_idx_t> voronoi_nodes;
  add_virtual_voronoi_nodes(area_index, original_id, id_is_from_way,
                            voronoi_nodes, additional_Locations);

  nodes_of_current_area_ = get_area_nodes(area_index);
  for (auto n : voronoi_nodes) {
    nodes_of_current_area_.push_back(n);
  }
  positions_of_current_area_ = get_area_locations(area_index);
  for (auto n : voronoi_nodes) {
    positions_of_current_area_.push_back(internal_area_node_positions_[n]);
  }

  // BUILD CUSTOM GRAPH

  // Vertices
  // Contains all Nodes in der Graph
  std::vector<osm_node_idx_t> graph_nodes;
  // Maps a nodes ID into an index in graph_nodes offset by 1
  std::map<osm_node_idx_t, uint32_t> graph_node_index;
  uint32_t index_counter = 0;
  for (auto n : access_nodes) {
    graph_nodes.push_back(n);
    graph_node_index.insert({n, index_counter});
    index_counter++;
  }
  for (auto n : voronoi_nodes) {
    graph_nodes.push_back(n);
    graph_node_index.insert({n, index_counter});
    index_counter++;
  }

  std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>> edges =
      voronoi_build_edges(vor_edg, graph_nodes);


  //TODO: Get Pathfinding working on the graph and compute the poly-lines between each of the access points of the area.
  //Insert dijkstra paths as polylines via method add_virtual_way().

  //Disabled Test- Path not correct. Out of Time.
  //std::vector<osm_node_idx_t> test_path;

  //voronoi_dijkstra(graph_nodes, graph_node_index, edges, access_node_vector[2], access_node_vector[4], test_path);

}

std::vector<Segment> ways::extract_area_segments(
    uint64_t area_index,
    vec<osm_node_idx_t> outer_nodes,
    vec<vec<osm_node_idx_t>> inner_rings) {

    std::vector<Segment> segments;

    
    area_ring_to_segments(segments, outer_nodes);

    for (vec<osm_node_idx_t> inner_ring : inner_rings) {
      area_ring_to_segments(segments, inner_ring);
    }
    
    return segments;
}



std::vector<boost::polygon::voronoi_edge<double>> ways::remove_irrelevant_edges(
    std::vector<boost::polygon::voronoi_edge<double>> edges,
    std::set<osm_node_idx_t> irrelevant_nodes) {

    std::vector<boost::polygon::voronoi_edge<double>> relevant_edges;

    //Check for each edge if vert0 or vert1 match an irrelevant Node.
    //If yes, discard edge.

    for (auto e : edges) {
        
      bool matched_irrelevant = false;
      std::set<osm_node_idx_t>::iterator itr;
      for (itr = irrelevant_nodes.begin(); itr != irrelevant_nodes.end(); itr++) {
        
        if (does_voronoi_vertex_match_node(e.vertex0(), *itr)) {
          matched_irrelevant = true;
          break;
        } else if (does_voronoi_vertex_match_node(e.vertex1(), *itr)) {
          matched_irrelevant = true;
          break;
        }
      }

      if (!matched_irrelevant) {
        relevant_edges.push_back(e);
      }
    }


  return relevant_edges;
}



std::vector<osr::point> ways::find_additional_nodes(
    std::vector<boost::polygon::voronoi_edge<double>> edges,
    vec<osm_node_idx_t> access_area_nodes,
    vec<osm_node_idx_t> outer_ring,
    vec<vec<osm_node_idx_t>> inner_rings) {

    std::vector<osr::point> additional_Locations;

    //All locations used by edges must now either be access nodes or new locations used in the voronoi diagram.

    //Go over all edges and collect all locations, that have not already been collected and that do not match any node of the area.
    for (voronoi_edge<double> e : edges) {
      
      //Does vert0 match any access point?
      bool vert0_is_new = true;
      for (auto n : access_area_nodes) {
        if (does_voronoi_vertex_match_node(e.vertex0(), n)) {
          vert0_is_new = false;
          break;
        }
      }

      // Does vert1 match any access point?
      bool vert1_is_new = true;
      for (auto n : access_area_nodes) {
        if (does_voronoi_vertex_match_node(e.vertex1(), n)) {
          vert1_is_new = false;
          break;
        }
      }

      //Store those points that did not match any access point.
      if (vert0_is_new) {
        additional_Locations.push_back(voronoi_position_to_osr_position(e.vertex0()->x(), e.vertex0()->y()));
      }

      if (vert1_is_new) {
        additional_Locations.push_back(voronoi_position_to_osr_position(e.vertex1()->x(), e.vertex1()->y()));
      }
      

    }



    //For each of them, check if they lie within the outer ring but not in any inner ring.
    std::vector<osr::point> valid_additional_Locations;
    auto outer_locations = get_area_ring_locations(outer_ring);
    for (osr::point p : additional_Locations) {
      bool valid_location = false;
      //Check Point Candidate against outer Ring.
      if (!is_point_in_polygon(p, outer_locations)) {
        continue;
      }
      valid_location = true;
      // Check Point Candidate against each inner Ring.
      for (auto inner_ring : inner_rings) {
        auto inner_locations = get_area_ring_locations(inner_ring);
        if (is_point_in_polygon(p, inner_locations)) {
          valid_location = false;
          break;
        }
      }

      if (!valid_location) {
        continue;
      }

      //Location was inside the outer and outside of all inner rings -> Valid.
      valid_additional_Locations.push_back(p);
    }

  //Then return these as Points.
  return valid_additional_Locations;
}



void ways::area_ring_to_segments(std::vector<Segment>& segments, vec<osm_node_idx_t> ring) {

  auto nodes = ring;

  osm_node_idx_t node_id0 = nodes[0];
  osr::point location0 = internal_area_node_positions_[node_id0];
  auto x0 = location0.as_location().x();
  auto y0 = location0.as_location().y();
  Point prev_point = Point(x0, y0);

    for (uint32_t i = 1; i < nodes.size(); i++) {
    //std::cout << "Node " << i << " is " << nodes[i] << "\n";
    osm_node_idx_t node_id1 = nodes[i];
    osr::point location1 = internal_area_node_positions_[node_id1];
    int x1 = location1.as_location().x();
    int y1 = location1.as_location().y();
    Point p1 = Point(x1, y1);

    Segment seg = Segment(prev_point, p1);

    segments.push_back(seg);

    prev_point = p1;
  }

}


std::vector<boost::polygon::voronoi_edge<double>> ways::get_relevant_voronoi_edges(std::vector<boost::polygon::voronoi_edge<double>> all_edges, uint64_t area_index) {


  std::cout << "We have " << all_edges.size() << " initial edges.\n";

  //We are only interested in those edges, that are finite.
  std::vector<voronoi_edge<double>> finite_edges;
  for (voronoi_edge<double> e : all_edges) {
    if (e.is_finite() && e.is_primary()) {
      finite_edges.push_back(e);
    }
  }

  std::cout << finite_edges.size() << " of these are finite and primary.\n";

  //We do not want double edges, filter out (b, a) for each (a, b)
  std::vector<voronoi_edge<double>> unique_edges;
  std::vector<voronoi_edge<double>*> duplicate_edges;
  for (auto edge : finite_edges) {
  
    //Check if edge is duplicate itself->continue with next
    bool is_unique = true;
    for (voronoi_edge<double>* dup : duplicate_edges) {
      if (dup->cell() == edge.twin()->cell()) {
        if (dup == edge.twin()) {
          is_unique = false;
          break;
        }
      }
    }

    if (!is_unique) {
      continue;
    }

    //Mark edge as unique and remember self-pointer as duplicate.
    unique_edges.push_back(edge);
    duplicate_edges.push_back(edge.twin()->twin());
  }


  std::cout << "After removing duplicate twins, we still have " << unique_edges.size() << " unique edges.\n";
  

  //We ignore all edges that are inside an inner ring.
  std::vector<voronoi_edge<double>> relevant_edges;
  vec<vec<osm_node_idx_t>> inner_rings = get_area_inner_rings(area_index);

  for (voronoi_edge<double> edge : unique_edges) {
    bool is_valid = true;
    auto vert_0 = edge.vertex0();
    auto vert_1 = edge.vertex1();
    double x0 = vert_0->x();
    double y0 = vert_0->y();
    double x1 = vert_1->x();
    double y1 = vert_1->y();

    osr::point a = voronoi_position_to_osr_position(x0, y0);
    osr::point b = voronoi_position_to_osr_position(x1, y1);

    point sample_position;
    sample_position.lat_ = (a.lat_ + b.lat_) / 2;
    sample_position.lng_ = (a.lng_ + b.lng_) / 2;

    //Check if edge midpoint is outside of all inner rings.
    for (auto r : inner_rings) {
      auto ring_locations = get_area_ring_locations(r);
      if (is_point_in_polygon(sample_position, ring_locations)) {
        is_valid = false;
        break;
      }
    }

    if (is_valid) {
      relevant_edges.push_back(edge);
    }
  }

  std::cout << relevant_edges.size() << " of those were outside the inner rings.\n";

  return relevant_edges;
}


osr::point ways::voronoi_position_to_osr_position(double x, double y) {

  double x_coord = x;
  double y_coord = y;

  // Divide by osmium::Location precision
  double manual_scaled_x = x_coord / 10000000;
  double manual_scaled_y = y_coord / 10000000;

  geo::latlng lalo = geo::latlng(manual_scaled_y, manual_scaled_x);

  osr::point p = osr::point().from_latlng(lalo);

  return p;
}


bool ways::does_voronoi_vertex_match_node(boost::polygon::voronoi_vertex<double> *vert, osm_node_idx_t node) {
  
  auto vert_loc = voronoi_position_to_osr_position(vert->x(), vert->y()).as_location();
  
  return (vert_loc == internal_area_node_positions_[node].as_location());
}


void ways::add_virtual_voronoi_nodes(uint64_t area_index,
                                     uint64_t original_id,
                                     bool id_is_from_way,
    vec<osm_node_idx_t>& voronoi_nodes,
    std::vector<osr::point> additional_Locations) {






    for (osr::point p : additional_Locations) {
    
      //Create new ID for node
      max_osm_node_idx_ = max_osm_node_idx_ + 1;
      osm_node_idx_t new_node_id = (osm_node_idx_t)max_osm_node_idx_;

      //Store information for remaining area processing
      voronoi_nodes.push_back(new_node_id);
      internal_area_node_positions_.insert({new_node_id, p});

    }
}

bool ways::voronoi_dijkstra(std::vector<osm_node_idx_t> graph_nodes,
    std::map<osm_node_idx_t, uint32_t> graph_node_index,
    std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>> edges,
                            osm_node_idx_t source,
    osm_node_idx_t target,
    std::vector<osm_node_idx_t>& path) {

  //Current node
  osm_node_idx_t u;
  // Init

  std::vector<double> dist;
  std::set<osm_node_idx_t> Q;

  for (auto n : graph_nodes) {
    dist.push_back(INFINITY);
    path.push_back((osm_node_idx_t)0);
    Q.insert(n);
  }

  dist[graph_node_index[source]] = 0;
  //End of Init


  while (!Q.empty()) {
    //Step to next Node

    u = get_closest_neighbor(Q, dist, graph_node_index);

    if (u == target) {
      return true;
    }

    Q.erase(u);



    double alt;
    std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>> connected_edges = get_connected_edges(u, edges);


    for (auto e : connected_edges) {
      alt = dist[graph_node_index[e.second.first]] + e.first;
      if (alt < dist[graph_node_index[e.second.second]]) {
        dist[graph_node_index[e.second.second]] = alt;
        path[graph_node_index[e.second.second]] = u;

      } 
    }
  }

  return (u == target);

}



std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>> ways::voronoi_build_edges(std::vector<boost::polygon::voronoi_edge<double>> voronoi_edges, std::vector<osm_node_idx_t> graph_nodes) {

    std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>> edges;
    //For each vor_kannte, finde node for vert0 und vert1
    for (auto vor_edge : voronoi_edges) {
      
      bool found_vert_0_node = false;
      bool found_vert_1_node = false;
      osm_node_idx_t node0;
      osm_node_idx_t node1;

      //Find Node for verts
      auto vert0 = vor_edge.vertex0();
      auto vert1 = vor_edge.vertex1();
      for (osm_node_idx_t n : graph_nodes) {
          
        if (!found_vert_0_node && (!found_vert_1_node || (found_vert_1_node && node1 != n))) {
          if (does_voronoi_vertex_match_node(vert0, n)) {

            found_vert_0_node = true;
            node0 = n;
          }
        }
        if (!found_vert_1_node && (!found_vert_0_node || (found_vert_0_node && node0 != n))) {
          if (does_voronoi_vertex_match_node(vert1, n)) {

            found_vert_1_node = true;
            node1 = n;
          }
        }

        if (found_vert_0_node && found_vert_1_node) {
          break;
        }

      }

      if (found_vert_0_node && found_vert_1_node && node0 != node1) {
        
        double distance = geo::distance(internal_area_node_positions_[node0].as_latlng(), internal_area_node_positions_[node1].as_latlng());

        if (distance <= 0.00001) {
          continue;
        }

        pair<double, pair<osm_node_idx_t, osm_node_idx_t>> e0 = {
            distance, {node0, node1}};
        pair<double, pair<osm_node_idx_t, osm_node_idx_t>> e1 = {
            distance, {node1, node0}};

        edges.push_back(e0);
        edges.push_back(e1);
      }
    }
    
    return edges;



  return std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>>();
}

double ways::voronoi_dijkstra_get_distance(
    osm_node_idx_t from,
    osm_node_idx_t to,
    std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>>
        connected_edges) {

    double found_distance = INFINITY;

    for (auto c : connected_edges) {
      if (c.second.first == from && c.second.second == to) {
        found_distance = c.first;
      }
    }
  return found_distance;
}

std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>>
ways::get_connected_edges(
    osm_node_idx_t from,
    std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>> edges) {
  
    std::vector<pair<double, pair<osm_node_idx_t, osm_node_idx_t>>>
      connected_edges;

    for (auto e : edges) {
    
      if (from == e.second.first) {
        connected_edges.push_back(e);
      }
    }

    return connected_edges;

}

osm_node_idx_t ways::get_closest_neighbor(
    std::set<osm_node_idx_t> Q,
    std::vector<double> dist,
    std::map<osm_node_idx_t, uint32_t> graph_node_index) {
  osm_node_idx_t min_dist_node = (osm_node_idx_t)0;
  double min_dist = INFINITY;

  for (auto q : Q) {
    if (dist[graph_node_index[q]] <= min_dist) {
      min_dist = dist[graph_node_index[q]];
      min_dist_node = q;
    }
  }
  return min_dist_node;
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

uint32_t ways::get_area_number_of_nodes(uint64_t area_index) { 
    
  auto all_rings = internal_area_nodes_[area_index];

  uint32_t loop_node_counter = 0;

  for (auto ring : all_rings) {
    loop_node_counter += ring.size();
  }
  return loop_node_counter;
}

uint32_t ways::get_area_number_of_outer_nodes(uint64_t area_index) { 

  return internal_area_nodes_[area_index][0].size();
}

uint32_t ways::get_area_number_of_inner_nodes(uint64_t area_index) {

  uint32_t inner_loop_node_counter = get_area_number_of_nodes(area_index);

  inner_loop_node_counter -= get_area_number_of_outer_nodes(area_index);

  return inner_loop_node_counter;
}

uint32_t ways::get_area_number_of_rings(uint64_t area_index) { 

    return internal_area_nodes_[area_index].size();
}

uint32_t ways::get_area_number_of_inner_rings(uint64_t area_index) {

  return get_area_number_of_rings(area_index) - 1;
}

vec<osm_node_idx_t> ways::get_area_outer_ring(uint64_t area_index) {

  return internal_area_nodes_[area_index][0];
}

vec<vec<osm_node_idx_t>> ways::get_area_inner_rings(uint64_t area_index) {

  vec<vec<osm_node_idx_t>> inner_loops;

  for (uint32_t i = 1; i < internal_area_nodes_[area_index].size(); i++) {
    vec<osm_node_idx_t> current_inner_loop;
    inner_loops.emplace_back(internal_area_nodes_[area_index][i]);
  }

  return inner_loops;
}

vec<osm_node_idx_t> ways::get_area_nodes(uint64_t area_index) {

  vec<osm_node_idx_t> nodes_of_area;

  for (auto ring : internal_area_nodes_[area_index]) {
    for (uint32_t i = 0; i < ring.size() - 1; i++) {
      nodes_of_area.emplace_back(ring[i]);
    }
  }

  return nodes_of_area;
}


vec<point> ways::get_area_ring_locations(vec<osm_node_idx_t> nodes_of_ring) {

  vec<point> ring_locations;

  for (auto n : nodes_of_ring) {
    ring_locations.push_back(internal_area_node_positions_[n]);
  }

  return ring_locations;
}

vec<point> ways::get_area_locations(uint64_t area_index) {

  vec<point> locations;
  
  for (vec<osm_node_idx_t> r : internal_area_nodes_[area_index]) {
    auto node_locations = get_area_ring_locations(r);
    for (uint32_t i = 0; i < r.size() - 1; i++) {
      locations.emplace_back(node_locations[i]);
    }
    
  }

  return locations;
}



cista::wrapped<ways::routing> ways::routing::read(
    std::filesystem::path const& p) {
  return cista::read<ways::routing>(p / "routing.bin");
}

void ways::routing::write(std::filesystem::path const& p) const {
  return cista::write(p / "routing.bin", *this);
}

}  // namespace osr