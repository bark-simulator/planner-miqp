// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "common/parameter/fitting_polynomial_parameters.hpp"
#include "gtest/gtest.h"

using namespace miqp::common::parameter;

TEST(fitting, construction) {
  FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
}

TEST(fitting, invalid_param) {
  try {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(33, 20, 2);
  } catch (std::invalid_argument const& err) {
    EXPECT_EQ(err.what(),
              std::string("Invalid number of regions or velocity!"));
  }
}

TEST(fitting, POLY_SINT_UB_32) {
  Eigen::MatrixXd POLY_SINT_UB_32;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    POLY_SINT_UB_32 = fpp.GetPOLY_SINT_UB();
  }
  EXPECT_EQ(0.18825, POLY_SINT_UB_32(0, 0));
  EXPECT_EQ(0.049029, POLY_SINT_UB_32(31, 2));
}

TEST(fitting, POLY_SINT_UB_32_exact) {
  Eigen::MatrixXd POLY_SINT_UB_32;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    POLY_SINT_UB_32 = fpp.GetPOLY_SINT_UB();
  }
  Eigen::MatrixXd should(32, 3);
  should << 0.18825, -0.0091624, 0.05868, 0.38235, -0.019062, 0.050001, 0.55699,
      -0.023663, 0.036849, 0.71125, -0.023834, 0.024695, 0.83632, -0.019361,
      0.013151, 0.93174, -0.019049, 0.0079673, 0.98244, -0.015382, 0.0037045,
      1.0047, -0.0055247, 0.00032035, 1.0047, 0.0055247, 0.00032035, 0.98244,
      0.015382, 0.0037045, 0.93174, 0.019049, 0.0079673, 0.83632, 0.019361,
      0.013151, 0.71125, 0.023834, 0.024695, 0.55699, 0.023663, 0.036849,
      0.38235, 0.019062, 0.050001, 0.19075, 0.0092873, 0.056807, 0.005,
      -4.4368e-15, 0.049029, -0.19739, -0.010959, 0.048523, -0.36666, -0.014006,
      0.034881, -0.5513, -0.016681, 0.024221, -0.70225, -0.019131, 0.018986,
      -0.82809, -0.019184, 0.012443, -0.91473, -0.013675, 0.0057777, -0.97651,
      -0.0055499, 0.00092463, -0.97651, 0.0055499, 0.00092463, -0.91473,
      0.013675, 0.0057777, -0.82809, 0.019184, 0.012443, -0.70225, 0.019131,
      0.018986, -0.5513, 0.016681, 0.024221, -0.36666, 0.014006, 0.034881,
      -0.19739, 0.010959, 0.048523, 0.005, 4.4432e-15, 0.049029;
  EXPECT_TRUE(should.isApprox(POLY_SINT_UB_32));
}

TEST(fitting, POLY_SINT_UB_16_exact) {
  Eigen::MatrixXd POLY_SINT_UB_16;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(16, 20, 2);
    POLY_SINT_UB_16 = fpp.GetPOLY_SINT_UB();
  }
  double data[] = {
      0.348508875688441,      0.7094052440899766,    0.9270720936717022,
      1.0037509353218053,     1.0037509353218055,    0.9270720936717006,
      0.7094052440899767,     0.35032998508199337,   0.005000000000002643,
      -0.38855374876443693,   -0.7027110650646589,   -0.9267320781144252,
      -0.9267320781144309,    -0.7027110650646587,   -0.38855374876443804,
      0.005000000000002289,   -0.017175443784421922, -0.027790982264217154,
      -0.02004864177828229,   -0.010717945748169198, 0.010717945748168643,
      0.020048641778282666,   0.02779098226421724,   0.017266499254099534,
      -3.278786758990503e-15, -0.017892652407872217, -0.019404275281907045,
      -0.012160273585670024,  0.012160273585668891,  0.019404275281907052,
      0.017892652407872415,   3.302698248604938e-15, 0.05687950979389347,
      0.03049251936078975,    0.009300376154024742,  0.0012490646782200609,
      0.0012490646782199128,  0.009300376154025208,  0.03049251936078988,
      0.05648493609195732,    0.04559607525876264,   0.03357030402897734,
      0.018799991403792633,   0.003413396094278538,  0.003413396094278246,
      0.018799991403792644,   0.03357030402897744,   0.04559607525876267};
  Eigen::Map<Eigen::MatrixXd> should(data, 16, 3);
  EXPECT_TRUE(should.isApprox(POLY_SINT_UB_16)) << POLY_SINT_UB_16 << std::endl
                                                << std::endl
                                                << should << std::endl;
}

TEST(fitting, POLY_SINT_LB_32) {
  Eigen::MatrixXd m;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    m = fpp.GetPOLY_SINT_LB();
  }
  EXPECT_EQ(-0.005, m(0, 0));
  EXPECT_EQ(0.056807, m(31, 2));
}

TEST(fitting, POLY_COSS_UB_32) {
  Eigen::MatrixXd m;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    m = fpp.GetPOLY_COSS_UB();
  }
  EXPECT_EQ(1.005, m(0, 0));
  EXPECT_EQ(0.0055247, m(31, 2));
}

TEST(fitting, POLY_COSS_LB_32) {
  Eigen::MatrixXd m;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    m = fpp.GetPOLY_COSS_LB();
  }
  EXPECT_EQ(0.97651, m(0, 0));
  EXPECT_EQ(0.0055499, m(31, 2));
}

TEST(fitting, POLY_KAPPA_AX_MAX_32) {
  Eigen::MatrixXd m;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    m = fpp.GetPOLY_KAPPA_AX_MAX();
  }
  EXPECT_EQ(-1.0225, m(0, 0));
  EXPECT_EQ(-0.1005, m(31, 2));
}

TEST(fitting, POLY_KAPPA_AX_MIN_32) {
  Eigen::MatrixXd m;
  {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
    m = fpp.GetPOLY_KAPPA_AX_MIN();
  }
  EXPECT_EQ(1.7056, m(0, 0));
  EXPECT_EQ(0.12325, m(31, 2));
}

TEST(fitting, valid_param_pair) {
  try {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 20, 2);
  } catch (std::invalid_argument const& err) {
    EXPECT_TRUE(false);
  }
}

TEST(fitting, invalid_param_pair) {
  try {
    FittingPolynomialParameters fpp = FittingPolynomialParameters(32, 10, 2);
  } catch (std::invalid_argument const& err) {
    EXPECT_EQ(err.what(),
              std::string("Invalid number of regions or velocity!"));
  }
}

TEST(fitting, POLY_KAPPA_AX_MIN_16_10) {
  FittingPolynomialParameters fpp = FittingPolynomialParameters(16, 10, 2);
  Eigen::MatrixXd m = fpp.GetPOLY_KAPPA_AX_MIN();
  EXPECT_NEAR(1.72762, m(0, 0), 0.0001);
  EXPECT_NEAR(0.249466, m(15, 2), 0.0001);
}

TEST(fitting, POLY_COSS_LB_16_10) {
  FittingPolynomialParameters fpp = FittingPolynomialParameters(16, 10, 2);
  Eigen::MatrixXd m = fpp.GetPOLY_COSS_LB();
  EXPECT_EQ(0.9317320781172977, m(0, 0));
  EXPECT_EQ(0.024320547171933854, m(15, 2));
}