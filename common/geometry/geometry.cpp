// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "geometry.hpp"
#include <boost/functional/hash.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/num_geometries.hpp>
#include <boost/geometry/algorithms/num_points.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace miqp {
namespace common {
namespace geometry {

void PrintPolygon(BoostPolygon const& poly) {
  std::cout << "poly = [";
  std::vector<PointXY> points = poly.outer();
  for (std::vector<PointXY>::size_type i = 0; i < points.size(); ++i) {
    std::cout << bg::get<0>(points[i]) << ", " << bg::get<1>(points[i]) << "; ";
  }
  std::cout << "]" << std::endl;
}

bark::geometry::Polygon BoostPolygonToBarkPolygon(BoostPolygon const& poly) {
  using bark::geometry::Point2d;
  bark::geometry::Polygon barkpoly;
  std::vector<PointXY> points = poly.outer();
  for (std::vector<PointXY>::size_type i = 0; i < points.size(); ++i) {
    barkpoly.AddPoint(Point2d(bg::get<0>(points[i]), bg::get<1>(points[i])));
  }
  return barkpoly;
}

bark::geometry::Line BoostLinestringToBarkLine(BoostLinestring const& line) {
  using bark::geometry::Point2d;
  bark::geometry::Line barkline;
  for (auto& pt : line) {
    barkline.AddPoint(Point2d(bg::get<0>(pt), bg::get<1>(pt)));
  }
  return barkline;
}

BoostPolygon BarkPolygonToBoostPolygon(bark::geometry::Polygon const& poly) {
  return poly.obj_;
}

bool IsConvex(BoostPolygon const& poly) {
  BoostPolygon hull;
  boost::geometry::convex_hull(poly, hull);
  //! boost geometry equals sometimes returns a very small value, even if the
  //! polygon points are identical. Use area check
  const double diff = bg::area(hull) - bg::area(poly);
  const double numeric_tolerance = 1e-10;
  // std::cout << "diff is = " << diff << " and bg::equals is: "
  // << bg::equals(poly, hull) << std::endl;
  return (abs(diff) < numeric_tolerance);
  // return bg::equals(poly, hull);
}

void BufferPolygonCollection(BoostMultiPolygon& polygons_in,
                             const double buffer_radius) {
  for (auto it = polygons_in.begin(); it != polygons_in.end(); ++it) {
    bark::geometry::Polygon tmp;
    BufferPolygon(BoostPolygonToBarkPolygon(*it), buffer_radius, &tmp);
    *it = BarkPolygonToBoostPolygon(tmp);
  }
}

void MergeToConvexPolygons(BoostMultiPolygon& polygons_in,
                           const float tolerance,
                           bark::geometry::Polygon& buffered_env_for_merging) {
  BoostMultiPolygon merged_polygons_out;
  std::vector<int> merged_idxs;
  for (size_t i = 0; i < polygons_in.size(); ++i) {
    bool idx_i_already_merged =
        std::find(merged_idxs.begin(), merged_idxs.end(), i) !=
        merged_idxs.end();
    if (!idx_i_already_merged) {
      BoostPolygon merged_polygon_idx_i = polygons_in.at(i);

      for (size_t j = i + 1; j < polygons_in.size(); ++j) {
        bool idx_j_already_merged =
            std::find(merged_idxs.begin(), merged_idxs.end(), j) !=
            merged_idxs.end();
        if (!idx_j_already_merged &&
            boost::geometry::intersects(merged_polygon_idx_i,
                                        polygons_in.at(j))) {
          BoostMultiPolygon merged_polygon;
          boost::geometry::correct(merged_polygon_idx_i);
          boost::geometry::correct(polygons_in.at(j));
          boost::geometry::union_(merged_polygon_idx_i, polygons_in.at(j),
                                  merged_polygon);
          boost::geometry::correct(merged_polygon);
          if (merged_polygon.size() > 0) {
            BoostPolygon hull;
            boost::geometry::convex_hull(merged_polygon, hull);
            boost::geometry::correct(hull);
            double diff = bg::area(hull) - bg::area(merged_polygon);
            const double numeric_tol = 1e-3;
            if (fabs(diff) < tolerance + numeric_tol) {
              // we use some error measure, so that we can merge two polygons
              // even if they yield non-convex polygon.
              merged_polygon_idx_i = hull;
              merged_idxs.push_back(j);
            } else if (boost::geometry::within(hull,
                                               buffered_env_for_merging.obj_)) {
              LOG(INFO) << "Merged polygon is not convex, but hull can be used";
              merged_polygon_idx_i = hull;
              merged_idxs.push_back(j);
            }
          }
        }
      }
      merged_polygons_out.push_back(merged_polygon_idx_i);
    }
  }

  polygons_in = merged_polygons_out;
}

Eigen::MatrixXd Polygon2MiqpPolygonDefinition(
    const bark::geometry::Polygon& poly) {
  // Does two things: 1. reverse order of the points, 2. removes first=last
  // doubled point
  using bark::geometry::Point2d;
  std::vector<Point2d> points = poly.obj_.outer();
  const size_t nrPoints = points.size() - 1;
  Eigen::MatrixXd outpoly(nrPoints, 2);
  for (std::vector<Point2d>::size_type i = 0; i < nrPoints; ++i) {
    outpoly(nrPoints - 1 - i, 0) = bg::get<0>(points[i]);
    outpoly(nrPoints - 1 - i, 1) = bg::get<1>(points[i]);
  }
  return outpoly;
}

std::size_t GetHash(const bark::geometry::Polygon& polygon) {
  std::size_t polygon_hash = 0;
  std::size_t num_geometries = bg::num_geometries(polygon.obj_);
  std::size_t num_geom_hash = boost::hash_value(num_geometries);
  boost::hash_combine(polygon_hash, num_geom_hash);

  std::size_t num_pts = bg::num_points(polygon.obj_);
  std::size_t num_pts_hash = boost::hash_value(num_pts);
  boost::hash_combine(polygon_hash, num_pts_hash);

  double area = bg::area(polygon.obj_);
  std::size_t area_hash = boost::hash_value(area);
  boost::hash_combine(polygon_hash, area_hash);
  return polygon_hash;
}

void ConvertToBarkLine(double ref_in[], const int ref_size,
                       double simplification_distance,
                       bark::geometry::Line& line_out) {
  std::vector<bark::geometry::Point2d> pts;
  for (int i = 0; i < ref_size * 2; i += 2) {
    pts.emplace_back(ref_in[i], ref_in[i + 1]);
  }
  bark::geometry::Line ref;
  ref.AddPoints(pts);
  line_out = Simplify(ref, simplification_distance);
}

PolygonMap BoostMultiPolygonToBarkPolygonMap(BoostMultiPolygon& polygons_in) {
  PolygonMap map_out;
  PolygonId id = 0;
  for (auto it = polygons_in.begin(); it != polygons_in.end(); ++it) {
    bark::geometry::Polygon pconv =
        miqp::common::geometry::BoostPolygonToBarkPolygon(*it);
    map_out.insert(std::pair<PolygonId, bark::geometry::Polygon>(id, pconv));
    ++id;
  }
  return map_out;
}

}  // namespace geometry
}  // namespace common
}  // namespace miqp