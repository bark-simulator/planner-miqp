// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_MAP_CONVEXIFIED_MAP_HPP_
#define MIQP_COMMON_MAP_CONVEXIFIED_MAP_HPP_

#include <vector>

#include <boost/geometry/geometries/geometries.hpp>
#include <boost/polygon/voronoi.hpp>
#include "bark/commons/base_type.hpp"
#include "bark/commons/util/util.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "common/geometry/geometry.hpp"

namespace miqp {
namespace common {
namespace map {

typedef boost::polygon::voronoi_diagram<double> boost_voronoi_diagram;
typedef boost::polygon::point_data<double> point_data_t;

class ConvexifiedMap : public bark::commons::BaseType {
 public:
  explicit ConvexifiedMap(const bark::commons::ParamsPtr& params,
                          const bark::geometry::Polygon& map_polygon,
                          const double buffer_radius,
                          const double max_simplify_dist,
                          const double buffer_reference,
                          const double buffer_for_merging_tolerance);

  virtual ~ConvexifiedMap() {}

  /**
   * @brief shrinks and converts nonconvex polygon to multiple convex ones
   *
   * @return true ... if conversion was successfull
   */
  bool Convert();

  /**
   * @brief Get the Map Non Convex Polygon object
   *
   * @return bark::geometry::Polygon
   */
  bark::geometry::Polygon GetMapNonConvexPolygon() const {
    return nonconvex_input_polygon_;
  }

  /**
   * @brief Get the Map Convex Polygons object
   *
   * @return miqp::common::geometry::PolygonMap
   */
  miqp::common::geometry::PolygonMap GetMapConvexPolygons() const {
    return convex_output_polygon_map_;
  }

  /**
   * @brief returns convex polygons of street that intersect with reference.
   * reference is bufferd to make sure that enough environment polygons are
   * available for a maneuver.
   *
   * @param referenceTraj ... reference trajectory
   * @return PolygonMap ... convex polygons
   */
  miqp::common::geometry::PolygonMap GetIntersectingConvexPolygons(
      const bark::models::dynamic::Trajectory& referenceTraj) const;

  /**
   * @brief checks if input polygon is valid
   *
   * @return true if polygon is valid
   * @return false if polygon is not valid
   */
  bool HasValidPolygon() { return nonconvex_input_polygon_.Valid(); }

  /**
   * @brief Set the Map Polygon object
   *
   * @param map_polygon
   */
  void SetMapPolygon(const bark::geometry::Polygon& map_polygon);

 private:
  /**
   * @brief function checks if decomposition of initial polygon was successfull
   * or not
   *
   * @param shrinked_input_polygon ... shrinked nonconvex input polygon
   * @param convex_output_polygons ... shrinked decomposed output polygons
   * @return true ... if decomposition successfull
   */
  bool CheckDecomposition(const bark::geometry::Polygon& shrinked_input_polygon,
                          const miqp::common::geometry::BoostMultiPolygon&
                              convex_output_polygons) const;

  /**
   * @brief converts cells of voronoi diagram to multipolygon
   *
   * @param vd ... voronoi diagram
   * @return BoostMultiPolygon ... cells from voronoi diagram
   */
  miqp::common::geometry::BoostMultiPolygon VoronoiCellsToBoostMultiPolygon(
      const boost::polygon::voronoi_diagram<double>& vd);

  /**
   * @brief construct voronoi diagram from input polygon
   *
   * @param input_polygon ... input geometry as polygon
   * @param input_points ... input geometry as points vector
   * @param vd ... voronoi diagram
   */
  void ConstructVoronoiDiagram(
      const bark::geometry::Polygon& buffered_simplified_input_polygon,
      std::vector<point_data_t>& input_points, boost_voronoi_diagram& vd) const;

  /**
   * @brief converts possibly nonconvex cells to triangles
   *
   * @param input_points ... points vector of input geometry
   * @param cells_inside_map ... possibly nonconvex cells
   * @param boost_polygon_convex_out ... triangles
   */
  void TriangulateCells(
      const std::vector<point_data_t>& input_points,
      miqp::common::geometry::BoostMultiPolygon& cells_inside_map,
      miqp::common::geometry::BoostMultiPolygon& boost_polygon_convex_out);

  /**
   * @brief create polygon from three points
   *
   * @param p1 ... input point
   * @param p2 ... input point
   * @param p3 ... input point
   * @param triangle ... output polygon
   * @return true ... if polygon could be constructed
   */
  bool CreateTriangle(const miqp::common::geometry::PointXY& p1,
                      const miqp::common::geometry::PointXY& p2,
                      const miqp::common::geometry::PointXY& p3,
                      miqp::common::geometry::BoostPolygon& triangle) const;

  /**
   * @brief simplifying and buffering input polygon
   *
   * @return Polygon ... simplified and buffered polygon
   */
  bark::geometry::Polygon PreprocessPolygon() const;

  /**
   * @brief thakes the convex_output_polygon_map_ member and simplifies the
   * linestrings in the polygons again to avoid too short segments. Afterwards
   * inflate the polygons again to avoid holes in between.
   */
  void PostprocessPolygonVector();

  bark::geometry::Polygon nonconvex_input_polygon_;
  miqp::common::geometry::PolygonMap convex_output_polygon_map_;
  const double envelope_offset_;
  bool decomposed_;
  double merging_tolerance_;
  const double buffer_radius_;
  const double buffer_reference_;
  const double buffer_for_merging_tolerance_;
  const double max_simplify_dist_;
  const double postprocess_simplification_dist_;
  const int num_merging_runs_;
};

}  // namespace map
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_MAP_CONVEXIFIED_MAP_HPP_
