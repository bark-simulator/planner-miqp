// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "convexified_map.hpp"
#include <boost/geometry/algorithms/difference.hpp>
#include <boost/geometry/iterators/segment_iterator.hpp>
#include <deque>
#include <string>

namespace miqp {
namespace common {
namespace map {

namespace bg = boost::geometry;
namespace mg = miqp::common::geometry;
using bark::commons::ParamsPtr;
using bark::geometry::Polygon;

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::models::dynamic::StateDefinition;

using mg::BoostMultiPolygon;
using mg::BoostPolygon;
using mg::PointXY;
using mg::PolygonMap;

ConvexifiedMap::ConvexifiedMap(const ParamsPtr& params,
                               const Polygon& map_polygon,
                               const double buffer_radius,
                               const double max_simplify_dist,
                               const double buffer_reference,
                               const double buffer_for_merging_tolerance)
    : bark::commons::BaseType(params),
      nonconvex_input_polygon_(map_polygon),
      convex_output_polygon_map_(),
      envelope_offset_(100),
      decomposed_(false),
      merging_tolerance_(5e-1),
      buffer_radius_(buffer_radius),
      buffer_reference_(buffer_reference),
      buffer_for_merging_tolerance_(buffer_for_merging_tolerance),
      max_simplify_dist_(max_simplify_dist),
      postprocess_simplification_dist_(0.01),
      num_merging_runs_(4) {
  assert(buffer_radius >= 0);  // by definition
  assert(max_simplify_dist >=
         1e-6);  // to make sure that at least overlapping points will be
                 // removed that may be introduced by bufferung
}

void ConvexifiedMap::SetMapPolygon(const bark::geometry::Polygon& map_polygon) {
  nonconvex_input_polygon_ = map_polygon;
}

bool ConvexifiedMap::Convert() {
  if (!nonconvex_input_polygon_.Valid()) {
    LOG(WARNING) << "Environment map could note be converted! Polygon is: "
                 << nonconvex_input_polygon_.ShapeToString() << std::endl;
    return false;
  } else {
    // Simplify and buffer
    Polygon buffered_simplified_input_polygon = PreprocessPolygon();

    // Construct Voronoi diagram
    std::vector<point_data_t> input_points;
    boost_voronoi_diagram vd;
    ConstructVoronoiDiagram(buffered_simplified_input_polygon, input_points,
                            vd);
    BoostMultiPolygon voronoi_cells = VoronoiCellsToBoostMultiPolygon(vd);

    // Intersecting Voronoi cells with map polygon
    BoostMultiPolygon cells_inside_map;
    for (auto it = voronoi_cells.begin(); it != voronoi_cells.end(); ++it) {
      if (bg::within(*it, buffered_simplified_input_polygon.obj_)) {
        // polygon is fully inside map
        cells_inside_map.push_back(*it);
      } else if (bg::intersects(*it, buffered_simplified_input_polygon.obj_)) {
        // polygon is only partially inside map
        std::deque<BoostPolygon> output;
        bg::intersection(*it, buffered_simplified_input_polygon.obj_, output);
        for (auto ito = output.begin(); ito != output.end(); ++ito) {
          cells_inside_map.push_back(*ito);
        }
      }
    }

    // Decompose-non convex polygons again:
    BoostMultiPolygon boost_polygon_convex_out;
    TriangulateCells(input_points, cells_inside_map, boost_polygon_convex_out);

    LOG(INFO) << "decomposition yields " << boost_polygon_convex_out.size()
              << " cells " << std::endl;

    mg::BufferPolygonCollection(boost_polygon_convex_out, 1e-5);

    // polygon to be used for collision checking with merged polygons
    bark::geometry::Polygon buffered_env_for_merging;
    bark::geometry::BufferPolygon(
        nonconvex_input_polygon_,
        -buffer_radius_ + buffer_for_merging_tolerance_,
        &buffered_env_for_merging);
    for (int i = 0; i < num_merging_runs_; ++i) {
      mg::MergeToConvexPolygons(boost_polygon_convex_out, merging_tolerance_,
                                buffered_env_for_merging);
      LOG(INFO) << "Merging Run #" << i << ", merged to "
                << boost_polygon_convex_out.size() << " cells " << std::endl;
      if (boost_polygon_convex_out.size() == 1) {
        break;
      }
    }

    // Generate Bark Polygons
    convex_output_polygon_map_ =
        mg::BoostMultiPolygonToBarkPolygonMap(boost_polygon_convex_out);

    // Avoid short segments
    PostprocessPolygonVector();

    decomposed_ = CheckDecomposition(buffered_simplified_input_polygon,
                                     boost_polygon_convex_out);
    return decomposed_;
  }
}

void ConvexifiedMap::ConstructVoronoiDiagram(
    const Polygon& input_polygon, std::vector<point_data_t>& input_points,
    boost_voronoi_diagram& vd) const {
  // Converting polygon to input data for Voronoi Diagram
  for (auto it = bg::segments_begin(input_polygon.obj_);
       it != bg::segments_end(input_polygon.obj_); ++it) {
    point_data_t p0(bg::get<0, 0>(*it), bg::get<0, 1>(*it));
    input_points.push_back(p0);
  }

  // Constructing bounding box around polygon so that voronoi edges don't go
  // to infinity
  boost::geometry::model::box<point_data_t> box;
  boost::geometry::envelope(input_polygon.obj_, box);

  input_points.push_back(point_data_t(box.min_corner().x() - envelope_offset_,
                                      box.min_corner().y() - envelope_offset_));
  input_points.push_back(point_data_t(box.min_corner().x() - envelope_offset_,
                                      box.max_corner().y() + envelope_offset_));
  input_points.push_back(point_data_t(box.max_corner().x() + envelope_offset_,
                                      box.min_corner().y() - envelope_offset_));
  input_points.push_back(point_data_t(box.max_corner().x() + envelope_offset_,
                                      box.max_corner().y() + envelope_offset_));

  // Constructing voronoi diagram
  construct_voronoi(input_points.begin(), input_points.end(), &vd);
}

void ConvexifiedMap::TriangulateCells(
    const std::vector<point_data_t>& input_points,
    BoostMultiPolygon& cells_inside_map,
    BoostMultiPolygon& boost_polygon_convex_out) {
  for (auto& p : cells_inside_map) {
    if (!mg::IsConvex(p)) {
      // if polygon is not convex, we need to triangulate it
      BoostPolygon hull;
      boost::geometry::convex_hull(p, hull);

      // find the one input point within the hull
      PointXY inner_pt;
      int cnt = 0;
      for (auto& pt : input_points) {
        // see boost docu: If a point is located exactly on the border of a
        // geometry, the result depends on the strategy. The default strategy
        // (Winding (coordinate system agnostic)) returns false in that case.
        if (boost::geometry::within(pt, hull)) {
          inner_pt.set<0>(pt.x());
          inner_pt.set<1>(pt.y());
          ++cnt;
        }
      }
      // there cannot be more than one point within convex hull of polygon
      assert(cnt == 1);

      // split the along this point
      int nr_points = p.outer().size();
      for (int pt_idx = 0; pt_idx < nr_points - 1; ++pt_idx) {
        BoostPolygon bp;
        // create triangles from polygon by rotating around inner support
        // point
        bool valid = CreateTriangle(inner_pt, p.outer().at(pt_idx),
                                    p.outer().at(pt_idx + 1), bp);
        if (valid) {
          boost_polygon_convex_out.push_back(bp);
        }
      }
      BoostPolygon bp;
      // create last triangle
      bool valid = CreateTriangle(inner_pt, p.outer().at(nr_points - 1),
                                  p.outer().at(0), bp);
      if (valid) {
        boost_polygon_convex_out.push_back(bp);
      }
    } else {
      // if polygon is convex, we can just add it to our list
      boost_polygon_convex_out.push_back(p);
    }
  }
}

bool ConvexifiedMap::CreateTriangle(const PointXY& p1, const PointXY& p2,
                                    const PointXY& p3,
                                    BoostPolygon& triangle) const {
  std::vector<PointXY> points_vec;
  points_vec.push_back(p1);
  points_vec.push_back(p2);
  points_vec.push_back(p3);
  bg::assign_points(triangle, points_vec);
  bg::correct(triangle);
  std::string reason;
  bool ok = boost::geometry::is_valid(triangle, reason);
  bool valid = (triangle.outer().size() > 2);
  return ok && valid;
}

Polygon ConvexifiedMap::PreprocessPolygon() const {
  bark::geometry::Polygon simplified_input_polygon;
  boost::geometry::simplify(nonconvex_input_polygon_.obj_,
                            simplified_input_polygon.obj_, max_simplify_dist_);

  bark::geometry::Polygon buffered_simplified_input_polygon;
  bark::geometry::BufferPolygon(simplified_input_polygon, -buffer_radius_,
                                &buffered_simplified_input_polygon);

  // This step will remove unnecessary points on a straight line
  bark::geometry::Polygon preprocessed_polygon;
  boost::geometry::simplify(buffered_simplified_input_polygon.obj_,
                            preprocessed_polygon.obj_, max_simplify_dist_);
  return preprocessed_polygon;
}

PolygonMap ConvexifiedMap::GetIntersectingConvexPolygons(
    const bark::models::dynamic::Trajectory& referenceTraj) const {
  // making sure convertion was done before
  assert(decomposed_ == true);

  Line line;
  for (int i = 0; i < referenceTraj.rows(); ++i) {
    line.AddPoint(Point2d(referenceTraj(i, StateDefinition::X_POSITION),
                          referenceTraj(i, StateDefinition::Y_POSITION)));
  }
  // Declare strategies
  const double buffer_distance = buffer_reference_;
  const int points_per_circle = 36;
  typedef double coordinate_type;
  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
      distance_strategy(buffer_distance);
  boost::geometry::strategy::buffer::join_round join_strategy(
      points_per_circle);
  boost::geometry::strategy::buffer::end_flat end_strategy;
  boost::geometry::strategy::buffer::point_circle circle_strategy(
      points_per_circle);
  boost::geometry::strategy::buffer::side_straight side_strategy;

  // Declare output
  BoostMultiPolygon result;
  Polygon buffered_reference;

  // Create the buffer of a linestring
  boost::geometry::buffer(line.obj_, result, distance_strategy, side_strategy,
                          join_strategy, end_strategy, circle_strategy);

  bg::correct(result);
  // std::cout << "result.size() " << result.size() << std::endl;
  assert(result.size() == 1);
  for (auto const& point :
       boost::make_iterator_range(bg::exterior_ring(result.front()))) {
    buffered_reference.AddPoint(point);
  }
  if (!buffered_reference.Valid()) {
    LOG(INFO) << "Buffered reference is not valid.";
  }

  PolygonMap intersecting_polygons;
  for (auto it = convex_output_polygon_map_.begin();
       it != convex_output_polygon_map_.end(); ++it) {
    if (Collide(it->second, buffered_reference)) {
      intersecting_polygons.insert(*it);
    }
  }
  return intersecting_polygons;
}

bool ConvexifiedMap::CheckDecomposition(
    const bark::geometry::Polygon& shrinked_input_polygon,
    const BoostMultiPolygon& convex_output_polygons) const {
  // making sure that all polygons are convex
  for (auto& p : convex_output_polygons) {
    if (!mg::IsConvex(p)) {
      LOG(ERROR) << "Non convex polygon: " << std::endl;
      mg::PrintPolygon(p);
      return false;
    }
  }

  // making sure decomposition did not loose anything
  const double max_tolerance =
      merging_tolerance_ * convex_output_polygons.size();
  // with the parameter doing the simplification
  double a_initial = bg::area(shrinked_input_polygon.obj_);
  double a_voronoi = bg::area(convex_output_polygons);
  if (a_initial <= a_voronoi + merging_tolerance_) {
    return true;
  } else {
    LOG(ERROR) << "convexified polygon map is not complete \n areas: "
               << a_initial << " vs " << a_voronoi << std::endl;
    return false;
  }
}

BoostMultiPolygon ConvexifiedMap::VoronoiCellsToBoostMultiPolygon(
    const boost_voronoi_diagram& vd) {
  BoostMultiPolygon cells;
  for (boost_voronoi_diagram::const_cell_iterator it = vd.cells().begin();
       it != vd.cells().end(); ++it) {
    const boost_voronoi_diagram::cell_type& cell = *it;
    const boost_voronoi_diagram::edge_type* edge = cell.incident_edge();
    std::vector<PointXY> points_vec;

    // convert cell to polygon by walking over all edges
    do {
      edge = edge->next();
      if (edge->is_primary()) {
        auto v = edge->vertex0();
        if (v != nullptr) {
          double x = static_cast<double>(v->x());
          double y = static_cast<double>(v->y());
          PointXY pt(x, y);
          points_vec.push_back(pt);
        }
      }
    } while (edge != cell.incident_edge());

    BoostPolygon bp;
    bg::assign_points(bp, points_vec);
    bg::correct(bp);
    cells.push_back(bp);
  }
  return cells;
}

void ConvexifiedMap::PostprocessPolygonVector() {
  const double dist = postprocess_simplification_dist_;
  for (auto it = convex_output_polygon_map_.begin();
       it != convex_output_polygon_map_.end(); ++it) {
    bark::geometry::Polygon polyin = it->second;
    bark::geometry::Polygon polysimplified;
    // remove short segments
    boost::geometry::simplify(polyin.obj_, polysimplified.obj_, dist);
    bark::geometry::Polygon polybuffered;
    // avoid holes
    bark::geometry::BufferPolygon(polysimplified, dist, &polybuffered);
    bark::geometry::Polygon polyout;
    // remove unnecessary points on straight lines
    boost::geometry::simplify(polybuffered.obj_, polyout.obj_, dist);
    it->second = polyout;
  }
}

}  // namespace map
}  // namespace common
}  // namespace miqp