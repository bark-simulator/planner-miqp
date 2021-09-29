// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "common/parameter/parameter_preparer.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "gtest/gtest.h"

using miqp::common::parameter::ParameterPreparer;

using bark::commons::SetterParams;
using namespace miqp::common::parameter;

TEST(parameter_preparer_test, mean_angle_vector) {
  auto params = std::make_shared<SetterParams>();
  int nrRegions = 32;
  float fittingVelocity = 20;
  float minFittingVelocity = 2;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer preparer(params, nrRegions, fittingVelocity, minFittingVelocity, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);

  std::vector<double> mean_angle = preparer.GetMeanAngleVector();
  std::vector<double> angle_expected = std::vector<double>{
      0.0982, 0.2945, 0.4909, 0.6872, 0.8836, 1.0799, 1.2763, 1.4726,
      1.6690, 1.8653, 2.0617, 2.2580, 2.4544, 2.6507, 2.8471, 3.0434,
      3.2398, 3.4361, 3.6325, 3.8288, 4.0252, 4.2215, 4.4179, 4.6142,
      4.8106, 5.0069, 5.2033, 5.3996, 5.5960, 5.7923, 5.9887, 6.1850};

  for (size_t i = 0; i < mean_angle.size(); ++i) {
    EXPECT_NEAR(mean_angle[i], angle_expected[i], 0.001)
        << "Vectors differ at index " << i;
  }
}

TEST(parameter_preparer_test, jerk_limits_test) {
  int NumCars = 1;
  auto params = std::make_shared<SetterParams>();
  int nrRegions = 32;
  float fittingVelocity = 20;
  float minFittingVelocity = 2;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer preparer(params, nrRegions, fittingVelocity, minFittingVelocity, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);

  LimitPerRegionParameters jerk_limits = preparer.CalculateJerkLimitsPerCar();

  auto jerk_limits_expected = LimitPerRegionParameters(NumCars, nrRegions);
  jerk_limits_expected.max_x << 3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489,
      2.2106, 1.6873, 1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057, 3.2772,
      3.1228, 3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489, 2.2106, 1.6873,
      1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057, 3.2772, 3.1228;
  jerk_limits_expected.min_x << -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873, -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228, -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873, -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228;
  jerk_limits_expected.max_y << 1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057,
      3.2772, 3.1228, 3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489, 2.2106,
      1.6873, 1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057, 3.2772, 3.1228,
      3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489, 2.2106, 1.6873;
  jerk_limits_expected.min_y << -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228, -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873, -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228, -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873;
  for (int i = 0; i < nrRegions; ++i) {
    EXPECT_NEAR(jerk_limits.max_x(0, i), jerk_limits_expected.max_x(0, i),
                0.001)
        << "Limits differ at index " << i;
    EXPECT_NEAR(jerk_limits.min_x(0, i), jerk_limits_expected.min_x(0, i),
                0.001)
        << "Limits differ at index " << i;
    EXPECT_NEAR(jerk_limits.max_y(0, i), jerk_limits_expected.max_y(0, i),
                0.001)
        << "Limits differ at index " << i;
    EXPECT_NEAR(jerk_limits.min_y(0, i), jerk_limits_expected.min_y(0, i),
                0.001)
        << "Limits differ at index " << i;
  }
}

TEST(parameter_preparer_test, acc_limits_test) {
  int nr_regions = 32;
  int NumCars = 1;
  auto params = std::make_shared<SetterParams>();
  float fittingVelocity = 20;
  float minFittingVelocity = 2;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer preparer(params, nr_regions, fittingVelocity, minFittingVelocity, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);

  LimitPerRegionParameters acc_limits = preparer.CalculateAccLimitsPerCar();

  auto acc_limits_expected = LimitPerRegionParameters(NumCars, nr_regions);
  acc_limits_expected.max_x << 2.1472, 2.3783, 2.5181, 2.5611, 2.5056, 2.3539,
      2.1117, 1.7883, 1.9844, 2.6922, 3.2967, 3.7744, 4.1071, 4.2819, 4.2922,
      4.1376, 4.1376, 4.2922, 4.2819, 4.1071, 3.7744, 3.2967, 2.6922, 1.9844,
      1.7883, 2.1117, 2.3539, 2.5056, 2.5611, 2.5181, 2.3783, 2.1472;
  acc_limits_expected.min_x << -4.1376, -4.2922, -4.2819, -4.1071, -3.7744,
      -3.2967, -2.6922, -1.9844, -1.7883, -2.1117, -2.3539, -2.5056, -2.5611,
      -2.5181, -2.3783, -2.1472, -2.1472, -2.3783, -2.5181, -2.5611, -2.5056,
      -2.3539, -2.1117, -1.7883, -1.9844, -2.6922, -3.2967, -3.7744, -4.1071,
      -4.2819, -4.2922, -4.1376;
  acc_limits_expected.max_y << 1.7883, 2.1117, 2.3539, 2.5056, 2.5611, 2.5181,
      2.3783, 2.1472, 2.1472, 2.3783, 2.5181, 2.5611, 2.5056, 2.3539, 2.1117,
      1.7883, 1.9844, 2.6922, 3.2967, 3.7744, 4.1071, 4.2819, 4.2922, 4.1376,
      4.1376, 4.2922, 4.2819, 4.1071, 3.7744, 3.2967, 2.6922, 1.9844;
  acc_limits_expected.min_y << -1.9844, -2.6922, -3.2967, -3.7744, -4.1071,
      -4.2819, -4.2922, -4.1376, -4.1376, -4.2922, -4.2819, -4.1071, -3.7744,
      -3.2967, -2.6922, -1.9844, -1.7883, -2.1117, -2.3539, -2.5056, -2.5611,
      -2.5181, -2.3783, -2.1472, -2.1472, -2.3783, -2.5181, -2.5611, -2.5056,
      -2.3539, -2.1117, -1.7883;
  for (int i = 0; i < nr_regions; ++i) {
    EXPECT_NEAR(acc_limits.max_x(0, i), acc_limits_expected.max_x(0, i), 0.001)
        << "Limits differ at index " << i;
    EXPECT_NEAR(acc_limits.min_x(0, i), acc_limits_expected.min_x(0, i), 0.001)
        << "Limits differ at index " << i;
    EXPECT_NEAR(acc_limits.max_y(0, i), acc_limits_expected.max_y(0, i), 0.001)
        << "Limits differ at index " << i;
    EXPECT_NEAR(acc_limits.min_y(0, i), acc_limits_expected.min_y(0, i), 0.001)
        << "Limits differ at index " << i;
  }
}

TEST(parameter_preparer_test, fraction_parameters_32) {
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

  auto params = std::make_shared<SetterParams>();
  int nrRegions = 32;
  float fittingVelocity = 20;
  float minFittingVelocity = 2;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer preparer(params, nrRegions, fittingVelocity, minFittingVelocity, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);
  FractionParameters fp = preparer.GetFractionParameters();

  auto diff = fp - fraction_parameters;
  ASSERT_TRUE(diff.norm() < 1e-3)
      << "diff is: " << std::endl
      << diff << std::endl
      << "diff norm is: " << diff.norm() << std::endl;

  // Numerical differences between matlab and c++: frobenius norm is not
  // equal...
  //   ASSERT_TRUE(fp.isApprox(fraction_parameters))
  //       << "should: " << std::endl
  //       << fraction_parameters << std::endl
  //       << std::endl
  //       << "is: " << std::endl
  //       << fp << std::endl;
}

TEST(parameter_preparer_test, fraction_parameters_16) {
  int nr_regions = 16;
  FractionParameters fraction_parameters;
  fraction_parameters.resize(nr_regions, 4);
  fraction_parameters << 20.0, 0.0, 18.477590650225736, 7.653668647301796,
      18.477590650225736, 7.653668647301796, 14.142135623730951,
      14.14213562373095, 14.142135623730951, 14.14213562373095,
      7.653668647301797, 18.477590650225736, 7.653668647301797,
      18.477590650225736, 1.2246467991473533e-15, 20.0, 1.2246467991473533e-15,
      20.0, -7.653668647301794, 18.477590650225736, -7.653668647301794,
      18.477590650225736, -14.14213562373095, 14.142135623730951,
      -14.14213562373095, 14.142135623730951, -18.477590650225736,
      7.653668647301798, -18.477590650225736, 7.653668647301798, -20.0,
      2.4492935982947065e-15, -20.0, 2.4492935982947065e-15,
      -18.477590650225736, -7.653668647301793, -18.477590650225736,
      -7.653668647301793, -14.142135623730955, -14.14213562373095,
      -14.142135623730955, -14.14213562373095, -7.6536686473018065,
      -18.47759065022573, -7.6536686473018065, -18.47759065022573,
      -3.673940397442059e-15, -20.0, -3.673940397442059e-15, -20.0,
      7.6536686473018, -18.477590650225732, 7.6536686473018,
      -18.477590650225732, 14.142135623730947, -14.142135623730955,
      14.142135623730947, -14.142135623730955, 18.47759065022573,
      -7.653668647301808, 18.47759065022573, -7.653668647301808, 20.0, 0.0;

  auto params = std::make_shared<SetterParams>();
  float fittingVelocity = 20;
  float minFittingVelocity = 2;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  ParameterPreparer preparer(params, nr_regions, fittingVelocity, minFittingVelocity, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);
  FractionParameters fp = preparer.GetFractionParameters();

  auto diff = fp - fraction_parameters;
  ASSERT_TRUE(diff.norm() < 1e-30)
      << "diff is: " << std::endl
      << diff << std::endl
      << "diff norm is: " << diff.norm() << std::endl;
}

TEST(parameter_preparer_test, fitting_10mps) {
  int nr_regions = 16;
  float fittingVelocity = 10.0;
  float minFittingVelocity = 2.0;
  float accLonMaxLimit = 2;
  float accLonMinLimit = -4;
  float jerkLonMaxLimit = 3;
  float accLatMinMaxLimit = 1.6;
  float jerkLatMinMaxLimit = 1.4;
  auto params = std::make_shared<SetterParams>();
  ParameterPreparer preparer(params, nr_regions, fittingVelocity,
                             minFittingVelocity, accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit, accLatMinMaxLimit, jerkLatMinMaxLimit);
  Eigen::MatrixXd m =
      preparer.GetFittingPolynomialParameters().GetPOLY_COSS_LB();
  EXPECT_EQ(0.9317320781172977, m(0, 0));
  EXPECT_EQ(0.024320547171933854, m(15, 2));
}