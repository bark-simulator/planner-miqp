// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_GEOMETRY_HPP_
#define MIQP_COMMON_GEOMETRY_HPP_

#include <Eigen/Core>
#include <vector>

#include <boost/geometry/geometries/adapted/boost_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include "bark/geometry/polygon.hpp"

namespace miqp {
namespace common {
namespace geometry {

namespace bg = boost::geometry;

using PointXY = bark::geometry::Point2d;
using BoostPolygon = bg::model::polygon<PointXY>;
using BoostMultiPolygon = bg::model::multi_polygon<BoostPolygon>;
using BoostLinestring = bg::model::linestring<PointXY>;

typedef unsigned int PolygonId;
typedef std::map<PolygonId, bark::geometry::Polygon> PolygonMap;

/**
 * @brief prints points of polygon
 *
 * @param poly polygon
 */
void PrintPolygon(BoostPolygon const& poly);

bark::geometry::Polygon BoostPolygonToBarkPolygon(BoostPolygon const& poly);

/**
 * @brief conveerts Boost linestring to BARK line type
 *
 * @param line
 * @return bark::geometry::Line
 */
bark::geometry::Line BoostLinestringToBarkLine(BoostLinestring const& line);

/**
 * @brief converts BARK's polygon to Boost polygon type
 *
 * @param poly BARK polygon to be converted
 * @return BoostPolygon converted polygon of type BoostPolygon
 */
BoostPolygon BarkPolygonToBoostPolygon(bark::geometry::Polygon const& poly);

bool IsConvex(BoostPolygon const& poly);

/**
 * @brief buffer all elements in a boost MultiPolygone, overrides the entries!
 * @param polynoms_in ... the multipoly
 * @param buffer_radius .. the buffer distance
 */
void BufferPolygonCollection(BoostMultiPolygon& polygons_in,
                             const double buffer_radius);

/**
 * @brief merging convex polygons to still convex ones. suboptimal algorithm,
 * that heavily depends on the sequence of the input polygons
 *
 * @param polygons_in ... polygons that will be merged
 * @param tolerance ... tolerance for merging
 * @param buffered_env_for_merging ... polygon, which all merged cells must in
 * within
 *
 * @note to account for numerical issues with the boost geometry union_ function
 * we buffer the input polygons by a small radius. Atm we only observe this
 * problems with points of doubles. see the unit test convexified_map_test.cc
 * TEST(convexified_map_test, polymerge) for a demo.
 *
 */
void MergeToConvexPolygons(BoostMultiPolygon& polygons_in,
                           const float tolerance,
                           bark::geometry::Polygon& buffered_env_for_merging);

/**
 * @brief convert BARK polygon to definition that is used by MIQP planner
 *
 * @param poly polygon to be converted
 * @return Eigen::MatrixXd converted polygon
 */
Eigen::MatrixXd Polygon2MiqpPolygonDefinition(
    const bark::geometry::Polygon& poly);

/**
 * @brief creates hash code from polygon
 *
 * @param polygon polygon used for hash calculation
 * @return std::size_t hash code
 */
std::size_t GetHash(const bark::geometry::Polygon& polygon);

/**
 * @brief convert array to BARK line type
 *
 * @param ref_in points array
 * @param ref_size size of points array
 * @param simplification_distance distance used for simplification of line
 * @param line_out converted line
 */
void ConvertToBarkLine(double ref_in[], const int ref_size,
                       double simplification_distance,
                       bark::geometry::Line& line_out);

/**
 * @brief converter from multipolygon to polygon map
 *
 * @param polygons_in ... input as multipolygon
 * @return PolygonMap ... output as polygon map
 */
PolygonMap BoostMultiPolygonToBarkPolygonMap(
    miqp::common::geometry::BoostMultiPolygon& polygons_in);

}  // namespace geometry
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_GEOMETRY_HPP_
