// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_PLANNER_DATA_HEADER
#define MIQP_PLANNER_DATA_HEADER

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include "src/miqp_planner_settings.h"
#include "common/parameter/limits_parameters.hpp"
#include "common/parameter/linearization_parameters.hpp"

using miqp::common::parameter::FractionParameters;
using miqp::common::parameter::LimitPerRegionParameters;
using miqp::common::parameter::PolynomialCurvatureParameters;
using miqp::common::parameter::PolynomialOrientationParameters;

namespace miqp {
namespace common {
namespace map {
typedef unsigned int PolygonId;
}  // namespace map
}  // namespace common
}  // namespace miqp

namespace miqp {
namespace planner {

typedef enum InitialStateIndices : int {
  MIQP_STATE_X = 0,
  MIQP_STATE_VX = 1,
  MIQP_STATE_AX = 2,
  MIQP_STATE_Y = 3,
  MIQP_STATE_VY = 4,
  MIQP_STATE_AY = 5,
  MIQP_INITIAL_STATE_SIZE = 6
} InitialStateIndices;

// RawResults: A 1:1 copy of the cplex decision variables after solution, mapped
// into Eigen::Tensors
struct RawResults {
  Eigen::Tensor<double, 2> u_x;             //(NrCars, N);
  Eigen::Tensor<double, 2> u_y;             //(NrCars, N);
  Eigen::Tensor<double, 2> pos_x;           //(NrCars, N);
  Eigen::Tensor<double, 2> vel_x;           //(NrCars, N);
  Eigen::Tensor<double, 2> acc_x;           //(NrCars, N);
  Eigen::Tensor<double, 2> pos_y;           //(NrCars, N);
  Eigen::Tensor<double, 2> vel_y;           //(NrCars, N);
  Eigen::Tensor<double, 2> acc_y;           //(NrCars, N);
  Eigen::Tensor<double, 2> pos_x_front_UB;  //(NrCars, N);
  Eigen::Tensor<double, 2> pos_x_front_LB;  //(NrCars, N);
  Eigen::Tensor<double, 2> pos_y_front_UB;  //(NrCars, N);
  Eigen::Tensor<double, 2> pos_y_front_LB;  //(NrCars, N);
  Eigen::Tensor<int, 3>
      notWithinEnvironmentRear;  //(NrCars, NrEnvironments, N);
  Eigen::Tensor<int, 3>
      notWithinEnvironmentFrontUbUb;  //(NrCars, NrEnvironments,N);
  Eigen::Tensor<int, 3>
      notWithinEnvironmentFrontLbUb;  //(NrCars, NrEnvironments, N);
  Eigen::Tensor<int, 3>
      notWithinEnvironmentFrontUbLb;  //(NrCars, NrEnvironments,  N);
  Eigen::Tensor<int, 3>
      notWithinEnvironmentFrontLbLb;    //(NrCars, NrEnvironments, N);
  Eigen::Tensor<int, 3> active_region;  //(NrCars, N, NrRegions);
  Eigen::Tensor<int, 2> region_change_not_allowed_x_positive;  //(NrCars, N);
  Eigen::Tensor<int, 2> region_change_not_allowed_y_positive;  //(NrCars, N);
  Eigen::Tensor<int, 2> region_change_not_allowed_x_negative;  //(NrCars, N);
  Eigen::Tensor<int, 2> region_change_not_allowed_y_negative;  //(NrCars, N);
  Eigen::Tensor<int, 2> region_change_not_allowed_combined;    //(NrCars, N);
  Eigen::Tensor<int, 4>
      deltacc;  //(NrCars, NrObstacles,  N,  MaxLinesObstacles);
  Eigen::Tensor<int, 5>
      deltacc_front;  //(NrCars, NrObstacles, N, MaxLinesObstacles, 4);  // 4 :
                      // all combinations of UBUB, UBLB, LBUB, UBUB
  Eigen::Tensor<int, 3>
      car2car_collision;  //(NrCarToCarCollisions, N, 16);  // 16 Idxs:
                          // Rear/rear = 1..4, Rear/front = 5..8, Front/rear
                          //= 9..12, Front/front = 13..16
  Eigen::Tensor<int, 3>
      slackvars;  //(NrCarToCarCollisions, N, 4);  // 4 Idxs: rear x = 1, rear y
                  //= 2, front x = 3, front y = 4
  Eigen::Tensor<int, 3>
      slackvarsObstacle;  //(NrCars, NrObstacles,  N)
  Eigen::Tensor<int, 4>
      slackvarsObstacle_front;  //(NrCars, NrObstacles,  N, 4)

  int N;
  int NrEnvironments;
  int NrRegions;
  int NrObstacles;
  int MaxLinesObstacles;
  int NrCarToCarCollisions;
  int NrCars;
};

struct ModelParameters {
  // API MultiEnvironmentConvexPolygon: std::vector containing the n polygons.
  //     each polygon is represented by an Eigen::MatrixXd with:
  //     x0, x1, x2, x3, ...
  //     y0, y1, y2, y3, ...

  // Solver Parameters
  float max_solution_time; // default: 1e+75, limit maximum time, if cplex already found a valid integer solution, it return with success but not the global optimum
  float relative_mip_gap_tolerance; //default: 1e-04, opimality gap
  int mipdisplay; // default: 2, display verbosity https://www.ibm.com/support/knowledgecenter/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/MIPDisplay.html
  int mipemphasis; // default: 0, find feasible solutions fast https://www.ibm.com/support/knowledgecenter/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/MIPEmphasis.html
  float relobjdif; //default 0.0, ignore integer solutions that are not x% better than already found ones https://www.ibm.com/support/knowledgecenter/en/SSSA5P_12.10.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/RelObjDif.html
  int cutpass; //default: 0, proposed by the tuning tool, nr of cutting planes //https://www.ibm.com/support/knowledgecenter/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/CutPass.html
  int probe; //default: 0, probing:  logical implications of fixing each binary variable to 0 or 1https://www.ibm.com/support/knowledgecenter/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/Probe.html
  int repairtries; //default: 0, number of tries to repair a warmstart solution //https://www.ibm.com/support/knowledgecenter/pl/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/RepairTries.html
  int rinsheur; //default: 0, relaxation induced neighborhood search (RINS) heuristic //https://www.ibm.com/support/knowledgecenter/en/SS9UKU_12.9.0/com.ibm.cplex.zos.help/CPLEX/Parameters/topics/RINSHeur.html
  int varsel; //default: 0, select branching variable: cplex automatically takes 4 (best in our case) // https://www.ibm.com/support/knowledgecenter/SSSA5P_12.9.0/ilog.odms.cplex.help/CPLEX/Parameters/topics/VarSel.html
  int mircuts; //default: 0,  mixed integer rounding cuts
  int parallelmode; //default: 0, opportunistic (-1) or deterministic (1) parallel search

  // Model Parameters  
  int NumSteps;
  float ts;
  int nr_regions;
  int NumCars;
  int NumCar2CarCollisions;
  float min_vel_x_y;
  float max_vel_x_y;
  float total_min_acc;
  float total_max_acc;
  float total_min_jerk;
  float total_max_jerk;
  Eigen::VectorXd agent_safety_distance;
  Eigen::VectorXd agent_safety_distance_slack;
  float maximum_slack;
  Eigen::VectorXd WEIGHTS_POS_X;
  Eigen::VectorXd WEIGHTS_VEL_X;
  Eigen::VectorXd WEIGHTS_ACC_X;
  Eigen::VectorXd WEIGHTS_POS_Y;
  Eigen::VectorXd WEIGHTS_VEL_Y;
  Eigen::VectorXd WEIGHTS_ACC_Y;
  Eigen::VectorXd WEIGHTS_JERK_X;
  Eigen::VectorXd WEIGHTS_JERK_Y;
  float WEIGHTS_SLACK;
  float WEIGHTS_SLACK_OBSTACLE;
  Eigen::VectorXd WheelBase;
  Eigen::VectorXd CollisionRadius;
  Eigen::VectorXd BufferReference;
  Eigen::MatrixXd IntitialState;
  Eigen::MatrixXd x_ref;
  Eigen::MatrixXd vx_ref;
  Eigen::MatrixXd y_ref;
  Eigen::MatrixXd vy_ref;
  LimitPerRegionParameters
      acc_limit_params;  // = LimitPerRegionParameters(NumCars, nr_regions);
  LimitPerRegionParameters
      jerk_limit_params;  // = LimitPerRegionParameters(NumCars, nr_regions);
  Eigen::VectorXi initial_region;
  Eigen::MatrixXi possible_region;
  int nr_obstacles;
  std::vector<std::vector<Eigen::MatrixXd> > ObstacleConvexPolygon;
  int max_lines_obstacles;
  std::vector<int> obstacle_is_soft; // 0: hard obstacle, 1: soft obstacle
  int nr_environments;
  std::vector<Eigen::MatrixXd> MultiEnvironmentConvexPolygon;
  std::vector<miqp::common::map::PolygonId> environmentPolygonIds;
  FractionParameters fraction_parameters;
  float minimum_region_change_speed;
  PolynomialCurvatureParameters poly_curvature_params;
  PolynomialOrientationParameters poly_orientation_params;
};

// For the C API the Settings struct has to be outside of a namespace
typedef MiqpPlannerSettings Settings;

inline Settings DefaultSettings() {
  Settings s = Settings();
  s.nr_regions = 16;
  s.nr_steps = 20;
  s.nr_neighbouring_possible_regions = 1;
  s.ts = 0.25;
  s.max_solution_time = 10;
  s.relative_mip_gap_tolerance = 0.1;
  s.mipdisplay = 2; 
  s.mipemphasis = 0;
  s.relobjdif = 0.0;
  s.cutpass = 0;
  s.probe = 0;
  s.repairtries = 0;
  s.rinsheur = 0;
  s.varsel = 0;
  s.mircuts = 0;
  s.precision = 12;
  s.constant_agent_safety_distance_slack = 3;
  s.minimum_region_change_speed = 2;
  s.lambda = 0.5;
  s.wheelBase = 2.8;
  s.collisionRadius = 1;
  s.slackWeight = 30;
  s.slackWeightObstacle = 2000;
  s.jerkWeight = 1;
  s.positionWeight = 2;
  s.velocityWeight = 0;
  s.acclerationWeight = 0;
  s.accLonMaxLimit = 2;
  s.accLonMinLimit = -4;
  s.jerkLonMaxLimit = 3;
  s.accLatMinMaxLimit = 1.6;
  s.jerkLatMinMaxLimit = 1.4;
  s.simplificationDistanceMap = 0.2;
  s.simplificationDistanceReferenceLine = 0.05;
  s.bufferReference = s.collisionRadius;
  s.buffer_for_merging_tolerance = 0.1;
  s.refLineInterpInc = 0.2;
  s.additionalStepsForReferenceLongerHorizon = 4;
  strcpy(s.cplexModelpath, "cplexmodel/");
  s.useSos = false;
  s.useBranchingPriorities = false;
  s.warmstartType = MiqpPlannerWarmstartType::NO_WARMSTART;
  s.parallelMode = MiqpPlannerParallelMode::AUTO;
  s.max_velocity_fitting = 20.0;
  s.buffer_cplex_outputs = false;
  s.obstacle_roi_filter = false;
  s.obstacle_roi_behind_distance = 5.0;
  s.obstacle_roi_front_distance = 30.0;
  s.obstacle_roi_side_distance = 15;
  return s;
}

inline Settings ApolloDefaultSettings() {
  Settings s = DefaultSettings();
  strcpy(s.cplexModelpath, "../bazel-bin/modules/planning/libplanning_component.so.runfiles/miqp_planner/cplex_modfiles/");
  s.buffer_cplex_outputs = true;
  return s;
}

}  // namespace planner
}  // namespace miqp

#endif  // MIQP_PLANNER_DATA_HEADER