// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "common/parameter/regions.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "common/parameter/linearization_parameters.hpp"
#include "common/parameter/parameter_preparer.hpp"
#include "gtest/gtest.h"

using miqp::common::parameter::ParameterPreparer;

using bark::commons::SetterParams;
using namespace miqp::common::parameter;

TEST(region_test, calculate_region_test_single_car) {
  int nr_regions = 32;
  float fitting_speed = 20;
  float min_fitting_speed = 2;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer pp(nullptr, nr_regions, fitting_speed, min_fitting_speed, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);

  FractionParameters fraction_parameters = pp.GetFractionParameters();

  Regions region_indices;
  region_indices = CalculateRegionIdx(fraction_parameters, 0.1, 0.01);
  EXPECT_TRUE(region_indices.size() == 1);
  for (auto r : region_indices) {
    EXPECT_TRUE(r.second == 0);
  }

  region_indices = CalculateRegionIdx(fraction_parameters, 0.1951, 0.9808);
  EXPECT_TRUE(region_indices.size() == 2);
  for (auto r : region_indices) {
    EXPECT_TRUE(r.second == 6 || r.second == 7);
  }

  region_indices = CalculateRegionIdx(fraction_parameters, 0.1, 0.9);
  EXPECT_TRUE(region_indices.size() == 1);
  for (auto r : region_indices) {
    EXPECT_TRUE(r.second == 7);
  }

  region_indices = CalculateRegionIdx(fraction_parameters, -0.1, -0.9);
  EXPECT_TRUE(region_indices.size() == 1);
  for (auto r : region_indices) {
    EXPECT_TRUE(r.second == 23);
  }

  region_indices = CalculateRegionIdx(fraction_parameters, 0.1, -0.01);
  EXPECT_TRUE(region_indices.size() == 1);
  for (auto r : region_indices) {
    EXPECT_TRUE(r.second == 31);
  }
}

TEST(region_test, combine_region_vectors) {
  Regions regions_car1 = {{false, 0}, {false, 1}};
  Regions regions_car2 = {{false, 5}};

  std::vector<Regions> region_vec_cars = {regions_car1, regions_car2};
  std::vector<std::vector<RegionIdx>> regions_combinations;
  std::vector<RegionIdx> accumulated;
  CalculateRegionCombinations(region_vec_cars, accumulated,
                              regions_combinations);

  std::vector<std::vector<RegionIdx>> regions_combinations_expected;
  regions_combinations_expected.push_back({0, 5});
  regions_combinations_expected.push_back({1, 5});

  EXPECT_EQ(regions_combinations_expected, regions_combinations);
}

TEST(region_test, combine_region_vectors_ordered) {
  Regions regions_car1 = {{false, 0}, {false, 1}};
  SetInitialRegion(regions_car1, 1);
  Regions regions_car2 = {{false, 5}};

  std::vector<Regions> region_vec_cars = {regions_car1, regions_car2};
  std::vector<std::vector<RegionIdx>> regions_combinations;
  std::vector<RegionIdx> accumulated;
  CalculateRegionCombinations(region_vec_cars, accumulated,
                              regions_combinations);

  std::vector<std::vector<RegionIdx>> regions_combinations_expected;
  regions_combinations_expected.push_back({1, 5});
  regions_combinations_expected.push_back({0, 5});

  EXPECT_EQ(regions_combinations_expected, regions_combinations);
}

TEST(region_test, calculate_possible_regions) {
  Eigen::VectorXd thetaRef;
  thetaRef.resize(3);
  thetaRef << 0.1, 0.1, 0.1;

  int nr_regions = 32;
  FractionParameters fraction_parameters;
  fraction_parameters.resize(nr_regions, 4);
  fraction_parameters << 20, 0, 19.6157, 3.90181, 19.6157, 3.90181, 18.4776,
      7.65367, 18.4776, 7.65367, 16.6294, 11.1114, 16.6294, 11.1114, 14.1421,
      14.1421, 14.1421, 14.1421, 11.1114, 16.6294, 11.1114, 16.6294, 7.65367,
      18.4776, 7.65367, 18.4776, 3.90181, 19.6157, 3.90181, 19.6157, 1.2247e-15,
      20, 1.2247e-15, 20, -3.90181, 19.6157, -3.90181, 19.6157, -7.65367,
      18.4776, -7.65367, 18.4776, -11.1114, 16.6294, -11.1114, 16.6294,
      -14.1421, 14.1421, -14.1421, 14.1421, -16.6294, 11.1114, -16.6294,
      11.1114, -18.4776, 7.65367, -18.4776, 7.65367, -19.6157, 3.90181,
      -19.6157, 3.90181, -20, 2.4493e-15, -20, 2.4493e-15, -19.6157, -3.90181,
      -19.6157, -3.90181, -18.4776, -7.65367, -18.4776, -7.65367, -16.6294,
      -11.1114, -16.6294, -11.1114, -14.1421, -14.1421, -14.1421, -14.1421,
      -11.1114, -16.6294, -11.1114, -16.6294, -7.65367, -18.4776, -7.65367,
      -18.4776, -3.90181, -19.6157, -3.90181, -19.6157, -3.6739e-15, -20,
      -3.6739e-15, -20, 3.90181, -19.6157, 3.90181, -19.6157, 7.65367, -18.4776,
      7.65367, -18.4776, 11.1114, -16.6294, 11.1114, -16.6294, 14.1421,
      -14.1421, 14.1421, -14.1421, 16.6294, -11.1114, 16.6294, -11.1114,
      18.4776, -7.65367, 18.4776, -7.65367, 19.6157, -3.90181, 19.6157,
      -3.90181, 20, 0;

  std::set<RegionIdx> regions =
      CalculatePossibleRegions(fraction_parameters, thetaRef);
  EXPECT_TRUE(regions.count(0) == 1);
}

TEST(region_test, calculate_possible_regions2) {
  Eigen::VectorXd thetaRef;
  thetaRef.resize(3);
  thetaRef << -0.1, -0.1, -0.1;

  int nr_regions = 32;
  FractionParameters fraction_parameters;
  fraction_parameters.resize(nr_regions, 4);
  fraction_parameters << 20, 0, 19.6157, 3.90181, 19.6157, 3.90181, 18.4776,
      7.65367, 18.4776, 7.65367, 16.6294, 11.1114, 16.6294, 11.1114, 14.1421,
      14.1421, 14.1421, 14.1421, 11.1114, 16.6294, 11.1114, 16.6294, 7.65367,
      18.4776, 7.65367, 18.4776, 3.90181, 19.6157, 3.90181, 19.6157, 1.2247e-15,
      20, 1.2247e-15, 20, -3.90181, 19.6157, -3.90181, 19.6157, -7.65367,
      18.4776, -7.65367, 18.4776, -11.1114, 16.6294, -11.1114, 16.6294,
      -14.1421, 14.1421, -14.1421, 14.1421, -16.6294, 11.1114, -16.6294,
      11.1114, -18.4776, 7.65367, -18.4776, 7.65367, -19.6157, 3.90181,
      -19.6157, 3.90181, -20, 2.4493e-15, -20, 2.4493e-15, -19.6157, -3.90181,
      -19.6157, -3.90181, -18.4776, -7.65367, -18.4776, -7.65367, -16.6294,
      -11.1114, -16.6294, -11.1114, -14.1421, -14.1421, -14.1421, -14.1421,
      -11.1114, -16.6294, -11.1114, -16.6294, -7.65367, -18.4776, -7.65367,
      -18.4776, -3.90181, -19.6157, -3.90181, -19.6157, -3.6739e-15, -20,
      -3.6739e-15, -20, 3.90181, -19.6157, 3.90181, -19.6157, 7.65367, -18.4776,
      7.65367, -18.4776, 11.1114, -16.6294, 11.1114, -16.6294, 14.1421,
      -14.1421, 14.1421, -14.1421, 16.6294, -11.1114, 16.6294, -11.1114,
      18.4776, -7.65367, 18.4776, -7.65367, 19.6157, -3.90181, 19.6157,
      -3.90181, 20, 0;

  std::set<RegionIdx> regions =
      CalculatePossibleRegions(fraction_parameters, thetaRef);
  EXPECT_TRUE(regions.count(31) == 1);
}

TEST(region_test, calculate_possible_regions3) {
  Eigen::VectorXd thetaRef;
  thetaRef.resize(3);
  thetaRef << 0.3, 0.5, 0.7;

  int nr_regions = 32;
  FractionParameters fraction_parameters;
  fraction_parameters.resize(nr_regions, 4);
  fraction_parameters << 20, 0, 19.6157, 3.90181, 19.6157, 3.90181, 18.4776,
      7.65367, 18.4776, 7.65367, 16.6294, 11.1114, 16.6294, 11.1114, 14.1421,
      14.1421, 14.1421, 14.1421, 11.1114, 16.6294, 11.1114, 16.6294, 7.65367,
      18.4776, 7.65367, 18.4776, 3.90181, 19.6157, 3.90181, 19.6157, 1.2247e-15,
      20, 1.2247e-15, 20, -3.90181, 19.6157, -3.90181, 19.6157, -7.65367,
      18.4776, -7.65367, 18.4776, -11.1114, 16.6294, -11.1114, 16.6294,
      -14.1421, 14.1421, -14.1421, 14.1421, -16.6294, 11.1114, -16.6294,
      11.1114, -18.4776, 7.65367, -18.4776, 7.65367, -19.6157, 3.90181,
      -19.6157, 3.90181, -20, 2.4493e-15, -20, 2.4493e-15, -19.6157, -3.90181,
      -19.6157, -3.90181, -18.4776, -7.65367, -18.4776, -7.65367, -16.6294,
      -11.1114, -16.6294, -11.1114, -14.1421, -14.1421, -14.1421, -14.1421,
      -11.1114, -16.6294, -11.1114, -16.6294, -7.65367, -18.4776, -7.65367,
      -18.4776, -3.90181, -19.6157, -3.90181, -19.6157, -3.6739e-15, -20,
      -3.6739e-15, -20, 3.90181, -19.6157, 3.90181, -19.6157, 7.65367, -18.4776,
      7.65367, -18.4776, 11.1114, -16.6294, 11.1114, -16.6294, 14.1421,
      -14.1421, 14.1421, -14.1421, 16.6294, -11.1114, 16.6294, -11.1114,
      18.4776, -7.65367, 18.4776, -7.65367, 19.6157, -3.90181, 19.6157,
      -3.90181, 20, 0;

  std::set<RegionIdx> regions =
      CalculatePossibleRegions(fraction_parameters, thetaRef);
  EXPECT_TRUE(regions.count(0) == 0);
  EXPECT_TRUE(regions.count(1) == 1);
  EXPECT_TRUE(regions.count(2) == 1);
  EXPECT_TRUE(regions.count(3) == 1);
}

TEST(region_test, calc_region_0) {
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer pp(nullptr, 32, 20, 2, accLonMaxLimit, accLonMinLimit,
                       jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);

  Regions region_indices =
      CalculateRegionIdx(pp.GetFractionParameters(), 0.1, 0.01);
  EXPECT_TRUE(region_indices.size() == 1);
  for (auto r : region_indices) {
    EXPECT_EQ(0, r.second);
  }
}

TEST(region_test, ReserveNeighborRegions_1) {
  Eigen::MatrixXi r(2, 32);
  r.setZero();
  r(0, 2) = 1;
  r(0, 3) = 1;
  r(1, 4) = 1;
  r(1, 5) = 1;

  bool suc;
  suc = ReserveNeighborRegions(r, 0, 1);
  EXPECT_TRUE(suc);
  EXPECT_EQ(1, r(0, 1)) << r;
  EXPECT_EQ(1, r(0, 2)) << r;
  EXPECT_EQ(1, r(0, 3)) << r;
  EXPECT_EQ(1, r(0, 4)) << r;
  EXPECT_EQ(0, r(0, 5)) << r;
  EXPECT_EQ(0, r(0, 0)) << r;

  suc = ReserveNeighborRegions(r, 1, 1);
  EXPECT_TRUE(suc);
  EXPECT_EQ(0, r(1, 2)) << r;
  EXPECT_EQ(1, r(1, 3)) << r;
  EXPECT_EQ(1, r(1, 4)) << r;
  EXPECT_EQ(1, r(1, 5)) << r;
  EXPECT_EQ(1, r(1, 6)) << r;
  EXPECT_EQ(0, r(1, 7)) << r;
}

TEST(region_test, ReserveNeighborRegions_wrap) {
  Eigen::MatrixXi r(2, 32);
  r.setZero();
  r(0, 0) = 1;
  r(0, 1) = 1;
  r(1, 30) = 1;
  r(1, 31) = 1;

  bool suc;
  suc = ReserveNeighborRegions(r, 0, 1);
  EXPECT_TRUE(suc);
  EXPECT_EQ(0, r(0, 30)) << r;
  EXPECT_EQ(1, r(0, 31)) << r;
  EXPECT_EQ(1, r(0, 0)) << r;
  EXPECT_EQ(1, r(0, 1)) << r;
  EXPECT_EQ(1, r(0, 2)) << r;
  EXPECT_EQ(0, r(0, 3)) << r;

  suc = ReserveNeighborRegions(r, 1, 1);
  EXPECT_TRUE(suc);
  EXPECT_EQ(0, r(1, 28)) << r;
  EXPECT_EQ(1, r(1, 29)) << r;
  EXPECT_EQ(1, r(1, 30)) << r;
  EXPECT_EQ(1, r(1, 31)) << r;
  EXPECT_EQ(1, r(1, 0)) << r;
  EXPECT_EQ(0, r(1, 1)) << r;
}

TEST(region_test, ReserveNeighborRegions_expand2) {
  Eigen::MatrixXi r(2, 32);
  r.setZero();
  r(0, 0) = 1;
  r(0, 1) = 1;

  bool suc;
  suc = ReserveNeighborRegions(r, 0, 2);
  EXPECT_TRUE(suc);
  EXPECT_EQ(0, r(0, 29)) << r;
  EXPECT_EQ(1, r(0, 30)) << r;
  EXPECT_EQ(1, r(0, 31)) << r;
  EXPECT_EQ(1, r(0, 0)) << r;
  EXPECT_EQ(1, r(0, 1)) << r;
  EXPECT_EQ(1, r(0, 2)) << r;
  EXPECT_EQ(1, r(0, 3)) << r;
  EXPECT_EQ(0, r(0, 4)) << r;
}
