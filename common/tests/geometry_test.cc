// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"

#include "bark/geometry/commons.hpp"
#include "bark/geometry/polygon.hpp"
#include "common/geometry/geometry.hpp"

TEST(polygon, convex_test) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon poly;
  poly.AddPoint(Point2d(0, 0));
  poly.AddPoint(Point2d(0, 2));
  poly.AddPoint(Point2d(2, 4));
  poly.AddPoint(Point2d(4, 0));
  poly.AddPoint(Point2d(0, 0));

  auto bp = miqp::common::geometry::BarkPolygonToBoostPolygon(poly);
  miqp::common::geometry::PrintPolygon(bp);

  bool c = miqp::common::geometry::IsConvex(bp);

  EXPECT_TRUE(c);
}

TEST(polygon, non_convex_test) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon poly;
  poly.AddPoint(Point2d(0, 0));
  poly.AddPoint(Point2d(0, 2));
  poly.AddPoint(Point2d(1, 1));
  poly.AddPoint(Point2d(2, 4));
  poly.AddPoint(Point2d(4, 0));
  poly.AddPoint(Point2d(0, 0));

  auto bp = miqp::common::geometry::BarkPolygonToBoostPolygon(poly);
  miqp::common::geometry::PrintPolygon(bp);

  bool c = miqp::common::geometry::IsConvex(bp);

  EXPECT_FALSE(c);
}

TEST(polygon, merge_polygon_test) {
  // two triangles being merged together
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  miqp::common::geometry::BoostMultiPolygon boost_multi_polygon;

  Polygon poly1;
  poly1.AddPoint(Point2d(0, 0));
  poly1.AddPoint(Point2d(2, 0));
  poly1.AddPoint(Point2d(4, 3));
  poly1.AddPoint(Point2d(0, 0));

  auto bp1 = miqp::common::geometry::BarkPolygonToBoostPolygon(poly1);
  //miqp::common::geometry::PrintPolygon(bp1);
  boost_multi_polygon.push_back(bp1);

  Polygon poly2;
  poly2.AddPoint(Point2d(2, 0));
  poly2.AddPoint(Point2d(4, 0));
  poly2.AddPoint(Point2d(4, 3));
  poly2.AddPoint(Point2d(2, 0));

  Polygon polyAll;
  poly1.AddPoint(Point2d(0, 0));
  poly2.AddPoint(Point2d(4, 0));
  poly2.AddPoint(Point2d(4, 3));
  poly1.AddPoint(Point2d(0, 0));

  auto bp2 = miqp::common::geometry::BarkPolygonToBoostPolygon(poly2);
  //miqp::common::geometry::PrintPolygon(bp2);
  boost_multi_polygon.push_back(bp2);

  miqp::common::geometry::MergeToConvexPolygons(boost_multi_polygon, 0.0, polyAll);

  EXPECT_EQ(boost_multi_polygon.size(), 1);
}

TEST(polygon, merge_polygon_test_false) {
  // two triangles being merged together; if merged with tolerance 0, this
  // yields a nonconvex polygon, and is thus not possible
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  miqp::common::geometry::BoostMultiPolygon boost_multi_polygon;

  Polygon poly1;
  poly1.AddPoint(Point2d(0, 0));
  poly1.AddPoint(Point2d(2, 0.2));
  poly1.AddPoint(Point2d(4, 3));
  poly1.AddPoint(Point2d(0, 0));

  auto bp1 = miqp::common::geometry::BarkPolygonToBoostPolygon(poly1);
  miqp::common::geometry::PrintPolygon(bp1);
  boost_multi_polygon.push_back(bp1);

  Polygon poly2;
  poly2.AddPoint(Point2d(2, 0.2));
  poly2.AddPoint(Point2d(4, 0));
  poly2.AddPoint(Point2d(4, 3));
  poly2.AddPoint(Point2d(2, 0.2));

  Polygon polyAll;
  poly1.AddPoint(Point2d(0, 0));
  poly2.AddPoint(Point2d(2, 0.2));
  poly2.AddPoint(Point2d(4, 0));
  poly2.AddPoint(Point2d(4, 3));
  poly1.AddPoint(Point2d(0, 0));

  auto bp2 = miqp::common::geometry::BarkPolygonToBoostPolygon(poly2);
  miqp::common::geometry::PrintPolygon(bp2);
  boost_multi_polygon.push_back(bp2);

  miqp::common::geometry::MergeToConvexPolygons(boost_multi_polygon, 0.0, polyAll);

  EXPECT_TRUE(boost_multi_polygon.size()==2);
}

TEST(polygon, merge_polygon_test_with_tol) {
  // two triangles being merged together; if merged with tolerance 0, this
  // yields a nonconvex polygon, and is thus not possible
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  miqp::common::geometry::BoostMultiPolygon boost_multi_polygon;

  Polygon poly1;
  poly1.AddPoint(Point2d(0, 0));
  poly1.AddPoint(Point2d(2, 0.2));
  poly1.AddPoint(Point2d(4, 3));
  poly1.AddPoint(Point2d(0, 0));

  auto bp1 = miqp::common::geometry::BarkPolygonToBoostPolygon(poly1);
  miqp::common::geometry::PrintPolygon(bp1);
  boost_multi_polygon.push_back(bp1);

  Polygon poly2;
  poly2.AddPoint(Point2d(2, 0.2));
  poly2.AddPoint(Point2d(4, 0));
  poly2.AddPoint(Point2d(4, 3));
  poly2.AddPoint(Point2d(2, 0.2));

  Polygon polyAll;
  poly1.AddPoint(Point2d(0, 0));
  poly2.AddPoint(Point2d(2, 0.2));
  poly2.AddPoint(Point2d(4, 0));
  poly2.AddPoint(Point2d(4, 3));
  poly1.AddPoint(Point2d(0, 0));

  auto bp2 = miqp::common::geometry::BarkPolygonToBoostPolygon(poly2);
  miqp::common::geometry::PrintPolygon(bp2);
  boost_multi_polygon.push_back(bp2);
  
  double area_tol = 2*0.2; // area of small nonconvex piece
  miqp::common::geometry::MergeToConvexPolygons(boost_multi_polygon, area_tol, polyAll);

  EXPECT_TRUE(boost_multi_polygon.size()==1);
}

TEST(polygon, conversion) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon poly;
  poly.AddPoint(Point2d(0, 0));
  poly.AddPoint(Point2d(0, 2));
  poly.AddPoint(Point2d(1, 1));
  poly.AddPoint(Point2d(2, 4));
  poly.AddPoint(Point2d(4, 0));
  poly.AddPoint(Point2d(0, 0));

  auto bp = miqp::common::geometry::BarkPolygonToBoostPolygon(poly);
  miqp::common::geometry::PrintPolygon(bp);
  auto poly2 = miqp::common::geometry::BoostPolygonToBarkPolygon(bp);

  EXPECT_TRUE(Equals(poly, poly2));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
