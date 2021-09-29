// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "common/map/convexified_map.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/polygon.hpp"
#include "common/geometry/geometry.hpp"
#include "gtest/gtest.h"

#include <boost/geometry/algorithms/convex_hull.hpp>

using miqp::common::map::ConvexifiedMap;

using bark::commons::SetterParams;
using bark::geometry::Point2d;
using bark::geometry::Polygon;

TEST(convexified_map_test, non_convex_polygon1) {
  Polygon poly;
  poly.AddPoint(Point2d(0, 0));
  poly.AddPoint(Point2d(0, 2));
  poly.AddPoint(Point2d(4, 2));
  poly.AddPoint(Point2d(4, 0));
  poly.AddPoint(Point2d(0, 0));

  EXPECT_TRUE(poly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, poly, 0.1, 1e-6, 1.0, 0.1);

  EXPECT_TRUE(conv_map.Convert());
}

TEST(convexified_map_test, non_convex_polygon2) {
  Polygon poly;
  poly.AddPoint(Point2d(0, 0));
  poly.AddPoint(Point2d(0, 2));
  poly.AddPoint(Point2d(1, 1));
  poly.AddPoint(Point2d(4, 2));
  poly.AddPoint(Point2d(4, 0));
  poly.AddPoint(Point2d(3, 1));
  poly.AddPoint(Point2d(0, 0));

  EXPECT_TRUE(poly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, poly, 0.1, 1e-6, 1.0, 0.1);

  EXPECT_TRUE(conv_map.Convert());
}

TEST(convexified_map_test, broken_polygon) {
  Polygon poly;
  poly.AddPoint(Point2d(0, 0));
  poly.AddPoint(Point2d(0, 2));
  poly.AddPoint(Point2d(4, 2));
  poly.AddPoint(Point2d(4, 0));
  poly.AddPoint(Point2d(2, 5));
  poly.AddPoint(Point2d(0, 0));

  EXPECT_FALSE(poly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, poly, 0.1, 1e-6, 1.0, 0.1);

  EXPECT_FALSE(conv_map.Convert());
}

TEST(convexified_map_test, box_poly) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, envPoly, 1.0, 0.2, 1.0, 0.1);

  EXPECT_TRUE(conv_map.Convert());
}

TEST(convexified_map_test, nonconvex) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-100, -50));
  envPoly.AddPoint(Point2d(-100, 50));
  envPoly.AddPoint(Point2d(0, 40));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-100, -50));
  ASSERT_TRUE(envPoly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, envPoly, 1.0, 0.2, 1.0, 0.1);

  EXPECT_TRUE(conv_map.Convert());

  miqp::common::geometry::PolygonMap mp = conv_map.GetMapConvexPolygons();
  EXPECT_TRUE(mp.size() > 1) << mp.size();
}

TEST(convexified_map_test, nonconvex_miqppoly) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-100, -50));
  envPoly.AddPoint(Point2d(-100, 50));
  envPoly.AddPoint(Point2d(0, 40));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-100, -50));
  ASSERT_TRUE(envPoly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, envPoly, 1.0, 0.2, 1.0, 0.1);
  conv_map.Convert();
  miqp::common::geometry::PolygonMap mp = conv_map.GetMapConvexPolygons();
  for (auto& p : mp) {
    Eigen::MatrixXd in = p.second.ToArray().cast<double>();
    Eigen::MatrixXd out =
        miqp::common::geometry::Polygon2MiqpPolygonDefinition(p.second);
    EXPECT_EQ(out(out.rows() - 1, 0), in(0, 0)) << in << std::endl
                                                << "----" << std::endl
                                                << out << std::endl;
    EXPECT_EQ(out(out.rows() - 1, 1), in(0, 1));
  }
}

TEST(convexified_map_test, osterwald1) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-243.496, -498.317));
  envPoly.AddPoint(Point2d(-222.71, -462.154));
  envPoly.AddPoint(Point2d(-200.444, -425.045));
  envPoly.AddPoint(Point2d(-166.54, -366.63));
  envPoly.AddPoint(Point2d(-163.513, -368.387));
  envPoly.AddPoint(Point2d(-197.443, -426.846));
  envPoly.AddPoint(Point2d(-219.676, -463.898));
  envPoly.AddPoint(Point2d(-240.461, -500.061));
  envPoly.AddPoint(Point2d(-243.496, -498.317));
  ASSERT_TRUE(envPoly.Valid());

  auto params = std::make_shared<SetterParams>();
  ConvexifiedMap conv_map(params, envPoly, 1.0, 0.2, 1.0, 0.1);
  bool succ = conv_map.Convert();
  EXPECT_TRUE(succ);
}

TEST(convexified_map_test, polymerge) {
  using namespace miqp::common::geometry;

  Polygon p1, p2;
  p1.AddPoint(Point2d(907.01381456027263539, 1006.7302426712110446));
  p1.AddPoint(Point2d(884.3842400840172786, 1008.499989792169572));
  p1.AddPoint(Point2d(892.55218939901942576, 1008.4999813652518696));
  p1.AddPoint(Point2d(907.05285734565381972, 1007.3809557608964269));
  p1.AddPoint(Point2d(907.01381456027263539, 1006.7302426712110446));

  p2.AddPoint(Point2d(932.47195950210220872, 1005.4193001706015593));
  p2.AddPoint(Point2d(907.0138140794423407, 1006.7302346573733303));
  p2.AddPoint(Point2d(907.05285619349717763, 1007.3809365582861801));
  p2.AddPoint(Point2d(932.47195950210220872, 1005.4193001706015593));

  // p1.SetPrecision(5);
  // p2.SetPrecision(5);

  {
    Polygon p1_buf, p2_buf;
    const double bufferdist = 1e-5;
    BufferPolygon(p1, bufferdist, &p1_buf);
    BufferPolygon(p2, bufferdist, &p2_buf);
    BoostPolygon bp1 = BarkPolygonToBoostPolygon(p1_buf);
    BoostPolygon bp2 = BarkPolygonToBoostPolygon(p2_buf);

    BoostMultiPolygon bmp;
    boost::geometry::correct(bp1);
    boost::geometry::correct(bp2);
    boost::geometry::union_(bp1, bp2, bmp);
    boost::geometry::correct(bmp);

    EXPECT_EQ(bmp.size(), 1);
    EXPECT_FALSE(boost::geometry::equals(bmp.at(0), bp1));
    EXPECT_FALSE(boost::geometry::equals(bmp.at(0), bp2));
    EXPECT_TRUE(boost::geometry::intersects(bp1, bp2));
  }

  {
    BoostPolygon bp1 = BarkPolygonToBoostPolygon(p1);
    BoostPolygon bp2 = BarkPolygonToBoostPolygon(p2);

    Polygon poly_tol;
    BoostMultiPolygon bmp;
    bmp.push_back(bp1);
    bmp.push_back(bp2);
    // to account for numerical issues with the boost geometry union_ function
    // we buffer the input polygons by a small radius. Atm we only observe this
    // problems with points of doubles.
    miqp::common::geometry::BufferPolygonCollection(bmp, 1e-5);
    MergeToConvexPolygons(bmp, 5e-1, poly_tol);
    EXPECT_EQ(bmp.size(), 1);
    EXPECT_FALSE(boost::geometry::equals(bmp.at(0), bp1));
    EXPECT_FALSE(boost::geometry::equals(bmp.at(0), bp2));
    EXPECT_TRUE(boost::geometry::intersects(bp1, bp2));
  }
}
