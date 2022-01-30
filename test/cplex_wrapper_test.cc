// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/cplex_wrapper.hpp"
#include "gtest/gtest.h"

#include <boost/filesystem.hpp>

using namespace miqp::planner::cplex;
using namespace miqp::planner;
using namespace miqp::common::parameter;

// set to zero to pass all tests, to something >> 0 to let all tests fail (and
// see the outputs)
#define LET_TEST_FAIL 0

// Test data for one vehicle driving straight in one environment poly, avoiding
// one static object
std::tuple<ModelParameters, RawResults> generateTestDataHelper() {
  ModelParameters mp;  // = ModelParameters();

  int NumSteps = 20;
  int nr_environments = 1;
  int nr_regions = 32;
  int nr_obstacles = 1;
  int max_lines_obstacles = 4;
  int NumCars = 1;
  int NumStates = 6;
  int NumCar2CarCollisions = NumCars - 1;

  // TODO: put these parameter in constructor of ModelParameters, as resizing
  // depends on them
  mp.NumSteps = NumSteps;
  mp.nr_environments = nr_environments;
  mp.nr_regions = nr_regions;
  mp.nr_obstacles = nr_obstacles;
  mp.max_lines_obstacles = max_lines_obstacles;
  mp.NumCars = NumCars;

  float eps = 0.000001;
  mp.max_solution_time = 60.0;
  mp.relative_mip_gap_tolerance = 0.1;
  mp.mipdisplay = 2;
  mp.mipemphasis = 0;
  mp.relobjdif = 0.0;
  mp.cutpass = 0;
  mp.probe = 0;
  mp.repairtries = 0;
  mp.rinsheur = 0;
  mp.varsel = 0;
  mp.mircuts = 0;
  mp.parallelmode = 0;

  mp.ts = 0.2;
  mp.min_vel_x_y = -20.0 - eps;
  mp.max_vel_x_y = 20.0 + eps;
  mp.total_min_acc = -4.2922 - eps;
  mp.total_max_acc = 4.2922 + eps;
  mp.total_min_jerk = -3.3057 - eps;
  mp.total_max_jerk = 3.3057 + eps;

  mp.agent_safety_distance.resize(NumSteps);
  mp.agent_safety_distance << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0;
  mp.agent_safety_distance_slack.resize(NumSteps);
  mp.agent_safety_distance_slack << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
      3, 3, 3, 3, 3;
  mp.maximum_slack = 3;

  mp.WEIGHTS_POS_X.resize(NumCars);
  mp.WEIGHTS_POS_X << 1;
  mp.WEIGHTS_VEL_X.resize(NumCars);
  mp.WEIGHTS_VEL_X << 0;
  mp.WEIGHTS_ACC_X.resize(NumCars);
  mp.WEIGHTS_ACC_X << 0;
  mp.WEIGHTS_JERK_X.resize(NumCars);
  mp.WEIGHTS_JERK_X << 0.5;
  mp.WEIGHTS_POS_Y.resize(NumCars);
  mp.WEIGHTS_POS_Y << 1;
  mp.WEIGHTS_VEL_Y.resize(NumCars);
  mp.WEIGHTS_VEL_Y << 0;
  mp.WEIGHTS_ACC_Y.resize(NumCars);
  mp.WEIGHTS_ACC_Y << 0;
  mp.WEIGHTS_JERK_Y.resize(NumCars);
  mp.WEIGHTS_JERK_Y << 0.5;
  mp.WEIGHTS_SLACK = 30;
  mp.WEIGHTS_SLACK_OBSTACLE = 2000;
  mp.WheelBase.resize(NumCars);
  mp.WheelBase << 2.8;
  mp.CollisionRadius.resize(NumCars);
  mp.CollisionRadius << 1.0;
  mp.IntitialState.resize(NumCars, NumStates);
  mp.IntitialState << 0, 5, 0, 0, 0.1, 0;
  mp.x_ref.resize(NumCars, NumSteps);
  mp.x_ref << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
      19;
  mp.vx_ref.resize(NumCars, NumSteps);
  mp.vx_ref << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  mp.y_ref.resize(NumCars, NumSteps);
  mp.y_ref << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  mp.vy_ref.resize(NumCars, NumSteps);
  mp.vy_ref << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  mp.acc_limit_params = LimitPerRegionParameters(NumCars, nr_regions);
  mp.acc_limit_params.min_x << -4.1376, -4.2922, -4.2819, -4.1071, -3.7744,
      -3.2967, -2.6922, -1.9844, -1.7883, -2.1117, -2.3539, -2.5056, -2.5611,
      -2.5181, -2.3783, -2.1472, -2.1472, -2.3783, -2.5181, -2.5611, -2.5056,
      -2.3539, -2.1117, -1.7883, -1.9844, -2.6922, -3.2967, -3.7744, -4.1071,
      -4.2819, -4.2922, -4.1376;
  mp.acc_limit_params.max_x << 2.1472, 2.3783, 2.5181, 2.5611, 2.5056, 2.3539,
      2.1117, 1.7883, 1.9844, 2.6922, 3.2967, 3.7744, 4.1071, 4.2819, 4.2922,
      4.1376, 4.1376, 4.2922, 4.2819, 4.1071, 3.7744, 3.2967, 2.6922, 1.9844,
      1.7883, 2.1117, 2.3539, 2.5056, 2.5611, 2.5181, 2.3783, 2.1472;
  mp.acc_limit_params.min_y << -1.9844, -2.6922, -3.2967, -3.7744, -4.1071,
      -4.2819, -4.2922, -4.1376, -4.1376, -4.2922, -4.2819, -4.1071, -3.7744,
      -3.2967, -2.6922, -1.9844, -1.7883, -2.1117, -2.3539, -2.5056, -2.5611,
      -2.5181, -2.3783, -2.1472, -2.1472, -2.3783, -2.5181, -2.5611, -2.5056,
      -2.3539, -2.1117, -1.7883;
  mp.acc_limit_params.max_y << 1.7883, 2.1117, 2.3539, 2.5056, 2.5611, 2.5181,
      2.3783, 2.1472, 2.1472, 2.3783, 2.5181, 2.5611, 2.5056, 2.3539, 2.1117,
      1.7883, 1.9844, 2.6922, 3.2967, 3.7744, 4.1071, 4.2819, 4.2922, 4.1376,
      4.1376, 4.2922, 4.2819, 4.1071, 3.7744, 3.2967, 2.6922, 1.9844;
  mp.acc_limit_params = mp.acc_limit_params;

  mp.jerk_limit_params = LimitPerRegionParameters(NumCars, nr_regions);
  mp.jerk_limit_params.min_x << -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873, -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228, -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873, -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228;
  mp.jerk_limit_params.max_x << 3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489,
      2.2106, 1.6873, 1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057, 3.2772,
      3.1228, 3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489, 2.2106, 1.6873,
      1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057, 3.2772, 3.1228;
  mp.jerk_limit_params.min_y << -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228, -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873, -1.6873, -2.2106, -2.6489, -2.9854, -3.2072,
      -3.3057, -3.2772, -3.1228, -3.1228, -3.2772, -3.3057, -3.2072, -2.9854,
      -2.6489, -2.2106, -1.6873;
  mp.jerk_limit_params.max_y << 1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057,
      3.2772, 3.1228, 3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489, 2.2106,
      1.6873, 1.6873, 2.2106, 2.6489, 2.9854, 3.2072, 3.3057, 3.2772, 3.1228,
      3.1228, 3.2772, 3.3057, 3.2072, 2.9854, 2.6489, 2.2106, 1.6873;
  mp.initial_region.resize(NumCars);
  mp.initial_region << 1;
  mp.possible_region.resize(NumCars, nr_regions);
  mp.possible_region << 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
  mp.fraction_parameters.resize(nr_regions, 4);
  mp.fraction_parameters << 20, 0, 19.6157, 3.90181, 19.6157, 3.90181, 18.4776,
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
  mp.minimum_region_change_speed = 2;
  mp.poly_orientation_params.POLY_SINT_UB.resize(nr_regions, 3);
  mp.poly_orientation_params.POLY_SINT_UB << 0.18825, -0.0091624, 0.05868,
      0.38235, -0.019062, 0.050001, 0.55699, -0.023663, 0.036849, 0.71125,
      -0.023834, 0.024695, 0.83632, -0.019361, 0.013151, 0.93174, -0.019049,
      0.0079673, 0.98244, -0.015382, 0.0037045, 1.0047, -0.0055247, 0.00032035,
      1.0047, 0.0055247, 0.00032035, 0.98244, 0.015382, 0.0037045, 0.93174,
      0.019049, 0.0079673, 0.83632, 0.019361, 0.013151, 0.71125, 0.023834,
      0.024695, 0.55699, 0.023663, 0.036849, 0.38235, 0.019062, 0.050001,
      0.19075, 0.0092873, 0.056807, 0.005, -4.4368e-15, 0.049029, -0.19739,
      -0.010959, 0.048523, -0.36666, -0.014006, 0.034881, -0.5513, -0.016681,
      0.024221, -0.70225, -0.019131, 0.018986, -0.82809, -0.019184, 0.012443,
      -0.91473, -0.013675, 0.0057777, -0.97651, -0.0055499, 0.00092463,
      -0.97651, 0.0055499, 0.00092463, -0.91473, 0.013675, 0.0057777, -0.82809,
      0.019184, 0.012443, -0.70225, 0.019131, 0.018986, -0.5513, 0.016681,
      0.024221, -0.36666, 0.014006, 0.034881, -0.19739, 0.010959, 0.048523,
      0.005, 4.4432e-15, 0.049029;
  mp.poly_orientation_params.POLY_SINT_LB.resize(nr_regions, 3);
  mp.poly_orientation_params.POLY_SINT_LB << -0.005, -3.1165e-11, 0.049029,
      0.19739, -0.010959, 0.048523, 0.36666, -0.014007, 0.034882, 0.5513,
      -0.016681, 0.024221, 0.70225, -0.019131, 0.018986, 0.82809, -0.019184,
      0.012443, 0.91473, -0.013675, 0.0057777, 0.97651, -0.0055499, 0.00092463,
      0.97651, 0.0055499, 0.00092463, 0.91473, 0.013675, 0.0057777, 0.82809,
      0.019184, 0.012443, 0.70225, 0.019131, 0.018986, 0.5513, 0.016681,
      0.024221, 0.36666, 0.014007, 0.034882, 0.19739, 0.010959, 0.048523,
      -0.005, 3.7954e-11, 0.049029, -0.19075, -0.0092873, 0.056807, -0.38235,
      -0.019062, 0.050001, -0.55699, -0.023663, 0.036849, -0.71125, -0.023834,
      0.024695, -0.83632, -0.019361, 0.013151, -0.93174, -0.019049, 0.0079673,
      -0.98244, -0.015382, 0.0037045, -1.0047, -0.0055247, 0.00032035, -1.0047,
      0.0055247, 0.00032035, -0.98244, 0.015382, 0.0037045, -0.93174, 0.019049,
      0.0079673, -0.83632, 0.019361, 0.013151, -0.71125, 0.023834, 0.024695,
      -0.55699, 0.023663, 0.036849, -0.38235, 0.019062, 0.050001, -0.19075,
      0.0092873, 0.056807;
  mp.poly_orientation_params.POLY_COSS_UB.resize(nr_regions, 3);
  mp.poly_orientation_params.POLY_COSS_UB << 1.005, 0.00027187, -0.0052001,
      0.98244, 0.0037045, -0.015382, 0.93174, 0.0079673, -0.019049, 0.83632,
      0.013151, -0.019361, 0.71125, 0.024695, -0.023834, 0.55699, 0.036849,
      -0.023663, 0.38235, 0.050001, -0.019062, 0.19075, 0.056807, -0.0092873,
      0.005, 0.049029, 4.9329e-15, -0.19739, 0.048523, 0.010959, -0.36666,
      0.034881, 0.014006, -0.5513, 0.024221, 0.016681, -0.70225, 0.018986,
      0.019131, -0.82809, 0.012443, 0.019184, -0.91473, 0.0057777, 0.013675,
      -0.97651, 0.00092463, 0.0055499, -0.97651, 0.00092463, -0.0055499,
      -0.91473, 0.0057777, -0.013675, -0.82809, 0.012443, -0.019184, -0.70225,
      0.018986, -0.019131, -0.5513, 0.024221, -0.016681, -0.36666, 0.034881,
      -0.014006, -0.19739, 0.048523, -0.010959, 0.005, 0.049029, -4.9239e-15,
      0.19075, 0.056807, 0.0092873, 0.38235, 0.050001, 0.019062, 0.55699,
      0.036849, 0.023663, 0.71125, 0.024695, 0.023834, 0.83632, 0.013151,
      0.019361, 0.93174, 0.0079673, 0.019049, 0.98244, 0.0037045, 0.015382,
      1.0047, 0.00032035, 0.0055247;
  mp.poly_orientation_params.POLY_COSS_LB.resize(nr_regions, 3);
  mp.poly_orientation_params.POLY_COSS_LB << 0.97651, 0.00092463, -0.0055499,
      0.91473, 0.0057777, -0.013675, 0.82809, 0.012443, -0.019184, 0.70225,
      0.018986, -0.019131, 0.5513, 0.024221, -0.016681, 0.36666, 0.034882,
      -0.014007, 0.19739, 0.048523, -0.010959, -0.005, 0.049029, -5.94e-11,
      -0.19075, 0.056807, 0.0092873, -0.38235, 0.050001, 0.019062, -0.55699,
      0.036849, 0.023663, -0.71125, 0.024695, 0.023834, -0.83632, 0.013151,
      0.019361, -0.93174, 0.0079673, 0.019049, -0.98244, 0.0037045, 0.015382,
      -1.0047, 0.00032035, 0.0055247, -1.0047, 0.00032035, -0.0055247, -0.98244,
      0.0037045, -0.015382, -0.93174, 0.0079673, -0.019049, -0.83632, 0.013151,
      -0.019361, -0.71125, 0.024695, -0.023834, -0.55699, 0.036849, -0.023663,
      -0.38235, 0.050001, -0.019062, -0.19075, 0.056807, -0.0092873, -0.005,
      0.049029, 5.94e-11, 0.19739, 0.048523, 0.010959, 0.36666, 0.034882,
      0.014007, 0.5513, 0.024221, 0.016681, 0.70225, 0.018986, 0.019131,
      0.82809, 0.012443, 0.019184, 0.91473, 0.0057777, 0.013675, 0.97651,
      0.00092463, 0.0055499;
  mp.poly_curvature_params.POLY_KAPPA_AX_MAX.resize(nr_regions, 3);
  mp.poly_curvature_params.POLY_KAPPA_AX_MAX << -1.0225, 0.6232, 0.087427,
      -1.0717, 0.60088, 0.27229, -1.1628, 0.54125, 0.48588, -1.3405, 0.42212,
      0.75354, -1.4722, 0.15387, 1.0746, -1.4566, -0.36661, 1.4131, -1.5607,
      -1.5712, 1.9537, 0, 0, 0, 0, 0, 0, -1.5607, 1.5712, 1.9537, -1.4566,
      0.36661, 1.4131, -1.4927, -0.15288, 1.0835, -1.3348, -0.42416, 0.74821,
      -1.1628, -0.54125, 0.48588, -1.0717, -0.60088, 0.27229, -1.0342, -0.6247,
      0.1005, -1.0225, -0.6232, -0.087427, -1.0717, -0.60088, -0.27229, -1.1628,
      -0.54125, -0.48588, -1.3363, -0.42733, -0.74475, -1.4802, -0.1601,
      -1.0729, -1.4566, 0.36661, -1.4131, -1.5607, 1.5712, -1.9537, 0, 0, 0, 0,
      0, 0, -1.5607, -1.5712, -1.9537, -1.4566, -0.36661, -1.4131, -1.485,
      0.14387, -1.0874, -1.3395, 0.42037, -0.7554, -1.1628, 0.54125, -0.48588,
      -1.0717, 0.60088, -0.27229, -1.0342, 0.6247, -0.1005;
  mp.poly_curvature_params.POLY_KAPPA_AX_MIN.resize(nr_regions, 3);
  mp.poly_curvature_params.POLY_KAPPA_AX_MIN << 1.7056, -0.81996, -0.11019,
      1.7661, -0.7799, -0.37313, 1.8762, -0.69245, -0.63982, 2.0996, -0.54149,
      -0.94897, 2.2624, -0.19861, -1.3451, 2.2639, 0.45677, -1.7807, 2.4805,
      2.2343, -2.5705, 2.9872, 7.6414, -3.911, 2.9872, -7.6414, -3.911, 2.4805,
      -2.2343, -2.5705, 2.2639, -0.45677, -1.7807, 2.2686, 0.17663, -1.364,
      2.1042, 0.53029, -0.96525, 1.8762, 0.69245, -0.63982, 1.7661, 0.7799,
      -0.37313, 1.7318, 0.82348, -0.12325, 1.7056, 0.81996, 0.11019, 1.7661,
      0.7799, 0.37313, 1.8762, 0.69245, 0.63982, 2.0983, 0.53948, 0.95113,
      2.2632, 0.19709, 1.3465, 2.2639, -0.45677, 1.7807, 2.4805, -2.2343,
      2.5705, 2.9872, -7.6414, 3.911, 2.9872, 7.6414, 3.911, 2.4805, 2.2343,
      2.5705, 2.2639, 0.45677, 1.7807, 2.2685, -0.17747, 1.3633, 2.1056,
      -0.53224, 0.96322, 1.8762, -0.69245, 0.63982, 1.7661, -0.7799, 0.37313,
      1.7318, -0.82348, 0.12325;

  Eigen::MatrixXd one_obstacle_each_time(4, 2);
  one_obstacle_each_time << 23, -1, 23, 5, 17, 5, 17, -1;
  std::vector<Eigen::MatrixXd> one_obstacle;
  one_obstacle.reserve(NumSteps);
  for (int i = 0; i < NumSteps; ++i) {
    one_obstacle.push_back(one_obstacle_each_time);
  }
  mp.ObstacleConvexPolygon.reserve(1);
  mp.ObstacleConvexPolygon.push_back(one_obstacle);

  mp.obstacle_is_soft.reserve(1);
  mp.obstacle_is_soft.push_back(0);

  Eigen::MatrixXd one_environment(4, 2);
  one_environment << -10, -10, 80, -10, 80, 20, -10, 20;
  mp.MultiEnvironmentConvexPolygon.reserve(1);
  mp.MultiEnvironmentConvexPolygon.push_back(one_environment);

  RawResults wv;  // warmstartvalues

  wv.N = NumSteps;
  wv.NrCars = NumCars;
  wv.NrEnvironments = nr_environments;
  wv.NrRegions = nr_regions;
  wv.NrObstacles = nr_obstacles;
  wv.MaxLinesObstacles = max_lines_obstacles;
  wv.NrCarToCarCollisions = NumCar2CarCollisions;

  wv.pos_x.resize(NumCars, NumSteps);
  wv.pos_x.setValues(
      {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}});
  wv.vel_x.resize(NumCars, NumSteps);
  wv.vel_x.setValues(
      {{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5}});
  wv.acc_x.resize(NumCars, NumSteps);
  wv.acc_x.setValues(
      {{0,          -2.9316e-8, -4.8875e-8, -6.0434e-8, -6.5658e-8,
        -6.6088e-8, -6.3108e-8, -5.7923e-8, -5.1546e-8, -4.4798e-8,
        -3.83e-8,   -3.2491e-8, -2.7639e-8, -2.386e-8,  -2.114e-8,
        -1.9367e-8, -1.8357e-8, -1.7888e-8, -1.7737e-8, -1.7717e-8}});
  wv.pos_y.resize(NumCars, NumSteps);
  wv.pos_y.setValues(
      {{0,         0.019179, 0.033586, 0.039222, 0.032977, 0.012594, -0.023365,
        -0.075549, -0.14385, -0.22744, -0.3248,  -0.4338,  -0.55177, -0.67568,
        -0.80226,  -0.92832, -1.0511,  -1.1688,  -1.2806,  -1.3862}});
  wv.vel_y.resize(NumCars, NumSteps);
  wv.vel_y.setValues(
      {{0.1,      0.087683, 0.053065, 0.00070665, -0.065036, -0.13999, -0.22014,
        -0.30162, -0.38071, -0.4539,  -0.5179,    -0.56983,  -0.60736, -0.62894,
        -0.63416, -0.62411, -0.60239, -0.57419,   -0.54347,  -0.51209}});
  wv.acc_y.resize(NumCars, NumSteps);
  wv.acc_y.setValues(
      {{0,        -0.12317, -0.22301, -0.30057, -0.35685, -0.39267, -0.40881,
        -0.40598, -0.38498, -0.34687, -0.29317, -0.22615, -0.1491,  -0.066723,
        0.014515, 0.085985, 0.13124,  0.15069,  0.15655,  0.15729}});
  wv.u_x.resize(NumCars, NumSteps);
  wv.u_x.setValues({{-1.4658e-7, -9.7796e-8, -5.7792e-8, -2.6122e-8, -2.1514e-9,
                     1.4902e-8,  2.5927e-8,  3.1881e-8,  3.3744e-8,  3.2489e-8,
                     2.9042e-8,  2.4259e-8,  1.8898e-8,  1.3599e-8,  8.866e-9,
                     5.0519e-9,  2.3443e-9,  7.5306e-10, 9.8559e-11, 0}});
  wv.u_y.resize(NumCars, NumSteps);
  wv.u_y.setValues(
      {{-0.61586, -0.49917, -0.38784, -0.28136, -0.17914,  -0.080684, 0.014167,
        0.10499,  0.19055,  0.26848,  0.33512,  0.38525,   0.41188,   0.40619,
        0.35735,  0.22628,  0.097256, 0.029291, 0.0036965, 0}});
  wv.slackvars.resize(NumCar2CarCollisions, NumCar2CarCollisions, NumSteps, 4);
  wv.slackvars.setValues({});
  wv.pos_x_front_UB.resize(NumCars, NumSteps);
  wv.pos_x_front_UB.setValues(
      {{2.7994, 3.8165, 4.817,  5.8178, 6.8166, 7.8155, 8.8142,
        9.813,  10.812, 11.811, 12.81,  13.809, 14.808, 15.808,
        16.808, 17.808, 18.808, 19.809, 20.809, 21.81}});
  wv.pos_x_front_LB.resize(NumCars, NumSteps);
  wv.pos_x_front_LB.setValues(
      {{2.7994, 3.7458, 4.7463, 5.7472, 6.7462, 7.745,  8.7438,
        9.7425, 10.741, 11.74,  12.739, 13.738, 14.738, 15.737,
        16.737, 17.737, 18.738, 19.738, 20.739, 21.739}});
  wv.pos_y_front_UB.resize(NumCars, NumSteps);
  wv.pos_y_front_UB.setValues(
      {{0.055989, 0.43241,  0.44113,  0.43816, 0.038049, 0.0073762, -0.039585,
        -0.10296, -0.18212, -0.27575, -0.3819, -0.49802, -0.62115,  -0.74802,
        -0.87531, -1,       -1.1198,  -1.2337, -1.3412,  -1.4425}});
  wv.pos_y_front_LB.resize(NumCars, NumSteps);
  wv.pos_y_front_LB.setValues(
      {{0.055989, 0.017216, 0.026871, 0.025319, -0.38145, -0.41375, -0.46246,
        -0.5276,  -0.60849, -0.70371, -0.81125, -0.92851, -1.0525,  -1.1798,
        -1.3072,  -1.4317,  -1.551,   -1.6643,  -1.7712,  -1.8717}});
  wv.active_region.resize(NumCars, NumSteps, nr_regions);
  wv.active_region.setValues(
      {{{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}}});
  wv.region_change_not_allowed_x_positive.resize(NumCars, NumSteps);
  wv.region_change_not_allowed_x_positive.setValues(
      {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}});
  wv.region_change_not_allowed_y_positive.resize(NumCars, NumSteps);
  wv.region_change_not_allowed_y_positive.setValues(
      {{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}});
  wv.region_change_not_allowed_x_negative.resize(NumCars, NumSteps);
  wv.region_change_not_allowed_x_negative.setValues(
      {{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}});
  wv.region_change_not_allowed_y_negative.resize(NumCars, NumSteps);
  wv.region_change_not_allowed_y_negative.setValues(
      {{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}});
  wv.region_change_not_allowed_combined.resize(NumCars, NumSteps);
  wv.region_change_not_allowed_combined.setValues(
      {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}});
  wv.notWithinEnvironmentRear.resize(NumCars, nr_environments, NumSteps);
  wv.notWithinEnvironmentRear.setValues(
      {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}});
  wv.notWithinEnvironmentFrontUbUb.resize(NumCars, nr_environments, NumSteps);
  wv.notWithinEnvironmentFrontUbUb.setValues(
      {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}});
  wv.notWithinEnvironmentFrontLbUb.resize(NumCars, nr_environments, NumSteps);
  wv.notWithinEnvironmentFrontLbUb.setValues(
      {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}});
  wv.notWithinEnvironmentFrontUbLb.resize(NumCars, nr_environments, NumSteps);
  wv.notWithinEnvironmentFrontUbLb.setValues(
      {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}});
  wv.notWithinEnvironmentFrontLbLb.resize(NumCars, nr_environments, NumSteps);
  wv.notWithinEnvironmentFrontLbLb.setValues(
      {{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}});
  wv.deltacc.resize(NumCars, nr_obstacles, NumSteps, max_lines_obstacles);
  wv.deltacc.setValues(
      {{{{1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1},
         {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1},
         {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1},
         {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 0, 1},
         {1, 1, 0, 1}, {1, 1, 1, 0}, {1, 1, 1, 0}, {1, 1, 1, 0}}}});
  wv.deltacc_front.resize(NumCars, nr_obstacles, NumSteps, max_lines_obstacles,
                          4);
  wv.deltacc_front.setValues(
      {{{{{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 0, 0}, {0, 0, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {0, 0, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}, {0, 0, 1, 1}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}},
         {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0}}}}});
  wv.car2car_collision.resize(NumCar2CarCollisions, NumCar2CarCollisions,
                              NumSteps, 16);
  wv.car2car_collision.setValues({});

  return {mp, wv};
}

TEST(cplex_wrapper_test, test_hardcoded_data) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);

  CplexWrapper::ParameterSource ps = CplexWrapper::CPPINPUTS;
  const int precision = 12;
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
  cw.resetParameters(parameters);
  int status = cw.callCplex();

  EXPECT_EQ(0 + LET_TEST_FAIL, status);
}

TEST(cplex_wrapper_test, test_hardcoded_data_versus_datfile) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  parameters->relative_mip_gap_tolerance = 1e-3;

  const int precision = 12;
  CplexWrapper cw =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw.resetParameters(parameters);

  // exporting data
  cw.setDebugOutputFilePath("/tmp");
  cw.setDebugOutputFilePrefix("test_hardcoded_data_versus_datfile");
  cw.setDebugOutputPrint(true);

  int status = cw.callCplex();
  std::string tmp_str = cw.getDebugOutputParameterFilePath();
  const char* datfile = tmp_str.c_str();

  CplexWrapper cw2 =
      CplexWrapper("cplexmodel.mod", CplexWrapper::DATFILE, precision);

  cw2.setParameterDatFileAbsolute(datfile);
  int status2 = cw2.callCplex();

  EXPECT_EQ(cw.getSolutionProperties().status,
            cw2.getSolutionProperties().status);
  EXPECT_DOUBLE_EQ(cw.getSolutionProperties().objective,
                   cw2.getSolutionProperties().objective);
  EXPECT_DOUBLE_EQ(cw.getSolutionProperties().gap,
                   cw2.getSolutionProperties().gap);
}

TEST(cplex_wrapper_test, test_datfile) {
  const char* datfile = "cplexmodel_testcase.dat";

  const int precision = 12;
  CplexWrapper::ParameterSource ps = CplexWrapper::DATFILE;
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
  cw.setParameterDatFileRelative(datfile);
  int status = cw.callCplex();

  EXPECT_EQ(0 + LET_TEST_FAIL, status);
}

TEST(cplex_wrapper_test, test_warmstart) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);

  const int precision = 12;
  CplexWrapper::ParameterSource ps = CplexWrapper::CPPINPUTS;
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
  cw.resetParameters(parameters);
  cw.addRecedingHorizonWarmstart(warmstart);

  int status = cw.callCplex();

  EXPECT_EQ(0 + LET_TEST_FAIL, status);
}

TEST(cplex_wrapper_test, test_no_obstacle) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);

  parameters->nr_obstacles = 0;
  parameters->ObstacleConvexPolygon.clear();
  parameters->obstacle_is_soft.clear();

  const int precision = 12;

  CplexWrapper cw =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw.resetParameters(parameters);

  int status = cw.callCplex();

  EXPECT_EQ(0 + LET_TEST_FAIL, status);
}

TEST(cplex_wrapper_test, test_debug_outputs) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);

  const int precision = 12;

  CplexWrapper cw =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw.resetParameters(parameters);

  cw.setDebugOutputPrint(true);
  const std::string p = ".";  // "/home/kessler/MIQP_DEBUG";
  const double t = 123;
  const std::string n = "test_debug_outputs_";
  cw.setDebugOutputFilePath(p);
  cw.setDebugOutputFilePrefix(n);
  cw.callCplex(t);

  std::stringstream ss, sp;
  sp << p << "/" << n << "parameters_" << t << ".txt";
  ss << p << "/" << n << "solution_" << t << ".txt";
  std::ifstream fs(ss.str().c_str());
  std::ifstream fp(sp.str().c_str());

  EXPECT_TRUE(fs.good());
  EXPECT_TRUE(fp.good());
}

TEST(cplex_wrapper_test, test_copy_constructor) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);

  const char* datfile = "cplexmodel_testcase.dat";

  const int precision = 12;

  CplexWrapper::ParameterSource ps = CplexWrapper::DATFILE;
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
  cw.setParameterDatFileRelative(datfile);
  cw.resetParameters(parameters);

  CplexWrapper cw2 = CplexWrapper(cw);

  EXPECT_EQ(cw.getDebugOutputParameterFilePath(),
            cw2.getDebugOutputParameterFilePath());
}

TEST(cplex_wrapper_test, test_compare_sos) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);

  const int precision = 12;
  CplexWrapper cw_no_sos =
      CplexWrapper("cplexmodel.mod", CplexWrapper::DATFILE, precision);
  cw_no_sos.setParameterDatFileRelative("test_sos.dat");
  int status1 = cw_no_sos.callCplex();
  auto sol_no_sos = cw_no_sos.getSolutionProperties();

  CplexWrapper cw_sos =
      CplexWrapper("cplexmodel.mod", CplexWrapper::DATFILE, precision);
  cw_sos.setParameterDatFileRelative("test_sos.dat");
  cw_sos.setSpecialOrderedSets(true);
  cw_sos.setDebugOutputFilePath("/tmp");
  cw_sos.setDebugOutputFilePrefix("test_sos");
  cw_sos.setDebugOutputPrint(true);
  int status2 = cw_sos.callCplex();
  auto sol_sos = cw_sos.getSolutionProperties();

  EXPECT_EQ(sol_no_sos.objective, sol_sos.objective);
  EXPECT_EQ(sol_no_sos.gap, sol_sos.gap);

  //   EXPECT_GT(sol_no_sos.time, sol_sos.time)
  //       << "Time without sos = " << sol_no_sos.time
  //       << " Time with sos = " << sol_sos.time << std::endl;

  EXPECT_EQ(0, status1);
  EXPECT_EQ(0, status2);
}

TEST(cplex_wrapper_test, test_max_solution_time) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);

  CplexWrapper::ParameterSource ps = CplexWrapper::CPPINPUTS;
  const int precision = 12;
  parameters->relative_mip_gap_tolerance = 0.00001;
  double timeeps = 0.3;

  {
    parameters->max_solution_time = 1.0;
    CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
    cw.resetParameters(parameters);
    cw.callCplex();
    EXPECT_LT(cw.getSolutionProperties().time,
              parameters->max_solution_time + timeeps);
  }

  {
    parameters->max_solution_time = 0.5;
    CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
    cw.resetParameters(parameters);
    cw.callCplex();
    EXPECT_LT(cw.getSolutionProperties().time,
              parameters->max_solution_time + timeeps);
  }

  {
    parameters->max_solution_time = 100.0;
    CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
    cw.resetParameters(parameters);
    cw.callCplex();
    EXPECT_LT(cw.getSolutionProperties().time,
              parameters->max_solution_time + timeeps);
  }
}

TEST(cplex_wrapper_test, test_compare_warmstart) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);
  int precision = 12;

  CplexWrapper cw_no_ws =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw_no_ws.resetParameters(parameters);
  int status1 = cw_no_ws.callCplex();
  auto sol_no_ws = cw_no_ws.getSolutionProperties();

  CplexWrapper cw_ws =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw_ws.resetParameters(parameters);
  cw_ws.addRecedingHorizonWarmstart(warmstart);
  int status2 = cw_ws.callCplex();
  auto sol_ws = cw_ws.getSolutionProperties();

  CplexWrapper cw_ws2 =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw_ws2.resetParameters(parameters);
  cw_ws2.setLastSolutionWarmstart();
  int status3 = cw_ws2.callCplex();  // be sure to write warmstart file
  status3 = cw_ws2.callCplex();
  auto sol_ws2 = cw_ws2.getSolutionProperties();

  // Asserts are too shaky
  //   EXPECT_GT(sol_no_ws.time, sol_ws.time)
  //       << "Time no warmstart = " << sol_no_ws.time
  //       << " Time with warmstart = " << sol_ws.time << std::endl;
  //   EXPECT_GT(sol_no_ws.time, sol_ws2.time)
  //       << "Time no warmstart = " << sol_no_ws.time
  //       << " Time with old solution warmstart = " << sol_ws.time <<
  //       std::endl;

  EXPECT_EQ(0, status1);
  EXPECT_EQ(0, status2);
  EXPECT_EQ(0, status3);
}

TEST(cplex_wrapper_test, test_complete_warmstart_receding_horizon) {
  // Capture stdout:
  // https://stackoverflow.com/questions/3803465/how-to-capture-
  // stdout-stderr-with-googletest

  // This can be an ofstream as well or any other ostream
  std::stringstream buffer;

  // Save cout's buffer here
  std::streambuf* sbuf = std::cout.rdbuf();

  // Redirect cout to our stringstream buffer or any other ostream
  std::cout.rdbuf(buffer.rdbuf());

  // Actual test
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);
  int precision = 12;

  CplexWrapper cw_ws =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw_ws.resetParameters(parameters);
  cw_ws.addRecedingHorizonWarmstart(warmstart);
  cw_ws.setDebugOutputPrint(false);
  cw_ws.setDebugOutputFilePath("/tmp/");
  cw_ws.setDebugOutputFilePrefix("warmstart_");
  int status2 = cw_ws.callCplex();
  EXPECT_EQ(0, status2);

  bool did_warmstart =
      buffer.str().find(
          "MIP start 'combined' defined initial solution with objective") !=
      std::string::npos;
  EXPECT_TRUE(did_warmstart);

  // When done redirect cout to its old self
  std::cout.rdbuf(sbuf);
}

TEST(cplex_wrapper_test, test_branching_priorities) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  auto warmstart = std::make_shared<RawResults>(warmstart_obj);

  const int precision = 12;
  CplexWrapper cw_no_branching_prio =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw_no_branching_prio.resetParameters(parameters);
  int status_no_branching_prio = cw_no_branching_prio.callCplex();
  auto solution_no_braching_prio = cw_no_branching_prio.getSolutionProperties();

  CplexWrapper cw_branching_prio =
      CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, precision);
  cw_branching_prio.resetParameters(parameters);
  cw_branching_prio.setUseBranchingPriorities(true);
  cw_branching_prio.setDebugOutputFilePath("/tmp");
  cw_branching_prio.setDebugOutputFilePrefix("test_branching_prio");
  cw_branching_prio.setDebugOutputPrint(true);
  int status_branching_prio = cw_branching_prio.callCplex();
  auto solution_branching_prio = cw_branching_prio.getSolutionProperties();

  EXPECT_EQ(solution_no_braching_prio.objective,
            solution_branching_prio.objective);
  EXPECT_EQ(solution_no_braching_prio.gap, solution_branching_prio.gap);

  // No real runtime improvement in this simple example
  //   EXPECT_GT(solution_no_braching_prio.time, solution_branching_prio.time)
  //       << "Time without branching priorities = "
  //       << solution_no_braching_prio.time
  //       << " Time with branching priorities = " <<
  //       solution_branching_prio.time
  //       << std::endl;

  EXPECT_EQ(0, status_branching_prio);
  EXPECT_EQ(0, status_no_branching_prio);
}

int reproduce_result_helper(std::string datfile, std::string lastsol) {
  const int precision = 12;
  CplexWrapper::ParameterSource ps = CplexWrapper::DATFILE;
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);

  // copy warmstart file to tmp folder
  if (!lastsol.empty()) {
    std::ifstream src(lastsol.c_str(), std::ios::binary);
    std::ofstream dst(cw.getTmpWarmstartFile().c_str(), std::ios::binary);
    dst << src.rdbuf();
    cw.setLastSolutionWarmstart();
  }
  cw.setSpecialOrderedSets(false);
  cw.setUseBranchingPriorities(true);
  // cw.setBranchingPriorityValueExtent(1, 19);
  cw.setParameterDatFileAbsolute(datfile.c_str());
  std::cout << "datfile = " << datfile.c_str() << std::endl;
  cw.setDebugOutputFilePath("/tmp");
  auto sep = datfile.find_last_of("/");
  auto filename = datfile.substr(sep + 1);
  std::string solfile = std::string("reproduce_") + filename;
  cw.setDebugOutputFilePrefix(solfile.c_str());
  cw.setDebugOutputPrint(true);
  int status = cw.callCplex();

  return status;
}

TEST(cplex_wrapper_test, test_overwrite_parameters) {
  auto [parameters_obj, warmstart_obj] = generateTestDataHelper();
  auto parameters = std::make_shared<ModelParameters>(parameters_obj);
  parameters->max_solution_time = 10.0;

  CplexWrapper cw = CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, 12);
  cw.resetParameters(parameters);
  int status = cw.callCplex();
  EXPECT_EQ(0, status);
  auto sp = cw.getSolutionProperties();

  auto overwriteParameters = std::make_shared<ModelParameters>(*parameters);
  overwriteParameters->max_solution_time = sp.time * 0.7;
  cw.overrideSolverSettingsDataSource(overwriteParameters);
  int status2 = cw.callCplex();
  EXPECT_EQ(0, status2);
  auto sp2 = cw.getSolutionProperties();

  EXPECT_GE(sp.time, sp2.time);
  EXPECT_LT(sp.objective, sp2.objective);
  EXPECT_LT(sp.gap, sp2.gap);
}

TEST(cplex_wrapper_test, test_create_new_logolder) {
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", CplexWrapper::CPPINPUTS, 12);

  const char* testpath = "/tmp/asdfasdfasdfasdfasdf";

  // exporting data
  cw.setDebugOutputFilePath(testpath);
  cw.setDebugOutputFilePrefix("test");
  cw.setDebugOutputPrint(true);

  EXPECT_TRUE(boost::filesystem::exists(testpath));
}

TEST(cplex_wrapper_test, test_problem_properties) {
  const char* datfile = "cplexmodel_testcase.dat";

  const int precision = 12;
  CplexWrapper::ParameterSource ps = CplexWrapper::DATFILE;
  CplexWrapper cw = CplexWrapper("cplexmodel.mod", ps, precision);
  cw.setParameterDatFileRelative(datfile);
  int status = cw.callCplex();
  auto sp = cw.getSolutionProperties();
  EXPECT_EQ(29834, sp.NonZeroCoefficients);  // values from the oplide
  EXPECT_EQ(1240, sp.NrBinaryVariables);
  EXPECT_EQ(12361, sp.NrConstraints);
  EXPECT_LT(10000,
            sp.NrIterations);  // oplide: 52555 (deviation is ok, reproducable)
  EXPECT_EQ(340, sp.NrFloatVariables);
  EXPECT_LT(2,
            sp.NrSolutionPool);  // oplide: 15 (deviation is ok, reproducable)
  EXPECT_NEAR(9.57603, sp.objective, 1e-5);
  EXPECT_EQ(0, status);
}