// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/miqp_planner.hpp"
#include "gtest/gtest.h"

using namespace miqp::planner;
using namespace bark::models::dynamic;
using namespace miqp::planner::cplex;
using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::Polygon;

Settings DefaultTestSettings() {
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
  s.bufferReference = 2.0;
  s.buffer_for_merging_tolerance = 0.1;
  s.refLineInterpInc = 0.2;
  s.additionalStepsForReferenceLongerHorizon = 4;
  s.cplexModelpath = "../cplex_models/";
  s.useSos = false;
  s.useBranchingPriorities = false;
  s.warmstartType = CplexWrapper::WarmstartType::NO_WARMSTART;
  s.max_velocity_fitting = 20.0;
  s.buffer_cplex_outputs = false;
  s.parallelMode = MiqpPlannerParallelMode::AUTO;
  s.obstacle_roi_filter = true;
  s.obstacle_roi_behind_distance = 10.0;
  s.obstacle_roi_front_distance = 100.0;
  s.obstacle_roi_side_distance = 15.0;
  return s;
}

TEST(miqp_planner, construction) {
  Polygon envPoly;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  ASSERT_NE(nullptr, &planner);
}

TEST(miqp_planner, construction_no_envpoly) {
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings);
  ASSERT_NE(nullptr, &planner);
}

TEST(miqp_planner, update_envpoly) {
  Polygon ep;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, ep);

  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
  EXPECT_TRUE(planner.Plan());
  auto polymap1 = planner.GetConvexShrinkedEnvPolygonsAllCars();
  EXPECT_EQ(polymap1.size(), 0);
  int nrenvs1 = parameters->nr_environments;
  EXPECT_EQ(nrenvs1, 0);

  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  planner.UpdateConvexifiedMap(envPoly);

  EXPECT_TRUE(planner.Plan());
  auto polymap2 = planner.GetConvexShrinkedEnvPolygonsAllCars();
  EXPECT_EQ(polymap2.size(), 1);
  int nrenvs2 = parameters->nr_environments;
  EXPECT_EQ(nrenvs2, 1);
}

TEST(miqp_planner, constructor_params) {
  Polygon envPoly;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  auto parameters = planner.GetParameters();
  EXPECT_EQ(settings.nr_regions, parameters->nr_regions);
  if (settings.nr_regions == 32) {
    EXPECT_EQ(0.18825, parameters->poly_orientation_params.POLY_SINT_UB(0, 0));
    EXPECT_EQ(0.97651, parameters->poly_orientation_params.POLY_COSS_LB(0, 0));
  }
  if (settings.nr_regions == 16) {
    EXPECT_EQ(0.348508875688441,
              parameters->poly_orientation_params.POLY_SINT_UB(0, 0));
  }
  EXPECT_EQ(settings.max_solution_time, parameters->max_solution_time);
  EXPECT_EQ(settings.constant_agent_safety_distance_slack,
            parameters->agent_safety_distance_slack(0));
  EXPECT_EQ(3, parameters->poly_orientation_params.POLY_COSS_LB.cols());
  EXPECT_EQ(settings.nr_regions,
            parameters->poly_orientation_params.POLY_COSS_LB.rows());
  EXPECT_EQ(settings.nr_regions, parameters->fraction_parameters.rows());
  EXPECT_EQ(4, parameters->fraction_parameters.cols());
  EXPECT_EQ(0, parameters->fraction_parameters(0, 1))
      << parameters->fraction_parameters;
  EXPECT_EQ(20, parameters->fraction_parameters(0, 0))
      << parameters->fraction_parameters;  // = v_max
}

TEST(miqp_planner, add_car_0) {
  Polygon envPoly;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 10;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({5, 0});
  ref.AddPoint({10, 0});
  ref.AddPoint({15, 0});
  ref.AddPoint({20, 20});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 0, 0, 1, 0.01, 0;
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
  auto parameters = planner.GetParameters();
  EXPECT_EQ(settings.wheelBase, parameters->WheelBase(idx));
  EXPECT_TRUE(initialState.isApprox(parameters->IntitialState.row(idx)));
  EXPECT_GT(parameters->total_max_acc,
            parameters->acc_limit_params.min_x.maxCoeff());
  EXPECT_LT(parameters->total_min_acc,
            parameters->acc_limit_params.min_x.minCoeff());
  EXPECT_EQ(1, parameters->acc_limit_params.max_x.rows());
  EXPECT_EQ(settings.nr_regions, parameters->acc_limit_params.max_x.cols());
  EXPECT_EQ(1, parameters->jerk_limit_params.min_y.rows());
  EXPECT_EQ(settings.nr_regions, parameters->jerk_limit_params.min_y.cols());
  EXPECT_EQ(settings.nr_steps, parameters->x_ref.cols());
  EXPECT_EQ(1, parameters->x_ref.rows());
  EXPECT_EQ(initialState(0), parameters->x_ref(0, 0));
  EXPECT_EQ(initialState(3), parameters->y_ref(0, 0));
  EXPECT_EQ(settings.lambda * settings.positionWeight,
            parameters->WEIGHTS_POS_X(0));
  EXPECT_EQ(1, parameters->possible_region(idx, 0))
      << parameters->possible_region;
}

TEST(miqp_planner, add_car_0_1) {
  Polygon envPoly;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 10;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref1, ref2;
  ref1.AddPoint({0, 0});
  ref1.AddPoint({50, 0});
  ref2.AddPoint({50, 5});
  ref2.AddPoint({0, 5});
  Eigen::MatrixXd initialState1(1, 6);
  initialState1 << 0, 0, 0, 1, 0.01, 0;
  Eigen::MatrixXd initialState2(1, 6);
  initialState2 << 50, 0, 0, 4, -0.01, 0;
  int idx1 = planner.AddCar(initialState1, ref1, vDes, deltaSForDesiredVel);
  int idx2 = planner.AddCar(initialState2, ref2, vDes, deltaSForDesiredVel);
  auto parameters = planner.GetParameters();

  EXPECT_TRUE(initialState1.isApprox(parameters->IntitialState.row(idx1)));
  EXPECT_TRUE(initialState2.isApprox(parameters->IntitialState.row(idx2)));
  EXPECT_EQ(2, parameters->x_ref.rows());
  EXPECT_EQ(2, parameters->jerk_limit_params.min_y.rows());
  EXPECT_EQ(1, parameters->possible_region(idx1, 0))
      << parameters->possible_region;
  EXPECT_EQ(1, parameters->possible_region(idx2, settings.nr_regions / 2 - 1))
      << parameters->possible_region;
  EXPECT_EQ(2, parameters->WEIGHTS_POS_X.size());
  EXPECT_EQ(2, parameters->x_ref.rows()) << parameters->x_ref;
  EXPECT_EQ(2, parameters->y_ref.rows()) << parameters->y_ref;
}

TEST(miqp_planner, delete_car_2) {
  Polygon envPoly;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 10;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref1, ref2;
  ref1.AddPoint({0, 0});
  ref1.AddPoint({50, 0});
  ref2.AddPoint({50, 5});
  ref2.AddPoint({0, 5});
  Eigen::MatrixXd initialState1(1, 6);
  initialState1 << 0, 0, 0, 1, 0.01, 0;
  Eigen::MatrixXd initialState2(1, 6);
  initialState2 << 50, 0, 0, 4, -0.01, 0;
  planner.AddCar(initialState1, ref1, vDes, deltaSForDesiredVel);
  int idx2 = planner.AddCar(initialState2, ref2, vDes, deltaSForDesiredVel);
  auto parameters = planner.GetParameters();

  EXPECT_EQ(2, parameters->x_ref.rows());
  EXPECT_EQ(2, parameters->jerk_limit_params.min_y.rows());
  EXPECT_EQ(2, parameters->WEIGHTS_POS_X.size());
  EXPECT_EQ(2, parameters->x_ref.rows()) << parameters->x_ref;
  EXPECT_EQ(2, parameters->y_ref.rows()) << parameters->y_ref;

  planner.RemoveCar(idx2);

  EXPECT_EQ(1, parameters->acc_limit_params.max_x.rows());
  EXPECT_EQ(1, parameters->jerk_limit_params.min_y.rows());
  EXPECT_EQ(1, parameters->x_ref.rows());
  EXPECT_EQ(1, parameters->WEIGHTS_POS_X.size());
  EXPECT_EQ(1, parameters->x_ref.rows()) << parameters->x_ref;
  EXPECT_EQ(1, parameters->y_ref.rows()) << parameters->y_ref;
}

TEST(miqp_planner, add_dynamic_obstacle) {
  using namespace bark::models::dynamic;
  Polygon envPoly;
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  Polygon obstacleShape = Polygon(
      bark::geometry::Pose(1.25, 1, 0),
      std::vector<Point2d>{Point2d(-1, -1), Point2d(-1, 1), Point2d(3, 1),
                           Point2d(3, -1), Point2d(-1, -1)});
  Trajectory predictedTraj(settings.nr_steps,
                           (int)StateDefinition::MIN_STATE_SIZE);
  State initialState(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  initialState << 0, 10, 5, 0, 0;
  for (int i = 0; i < predictedTraj.rows(); ++i) {
    predictedTraj.row(i) = initialState;
  }
  bool is_soft = false;
  bool is_static = false;
  planner.AddObstacle(predictedTraj, obstacleShape, is_soft, is_static);
  auto parameters = planner.GetParameters();
  EXPECT_EQ(1, parameters->nr_obstacles);

  planner.RemoveAllObstacles();
  parameters = planner.GetParameters();
  EXPECT_EQ(0, parameters->nr_obstacles);
}

TEST(miqp_planner, plan1) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 0, 0.1, 0;  // TODO region calculation is VERY
                                       // crucial !!!!!!!
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  bool succ = planner.Plan();

  EXPECT_EQ(1, parameters->initial_region(idx)) << parameters->initial_region;
  ASSERT_TRUE(parameters->MultiEnvironmentConvexPolygon.size() > 0);
  EXPECT_EQ(parameters->MultiEnvironmentConvexPolygon.size(),
            parameters->nr_environments);

  EXPECT_TRUE(succ);
}

TEST(miqp_planner, receding_horizon) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 10;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({1000, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 1, -0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  float tend = 2;
  float dt = settings.ts;

  std::vector<float> x, y, v;
  x.push_back(initialState(0, MIQP_STATE_X));
  y.push_back(initialState(0, MIQP_STATE_Y));
  v.push_back(sqrt(pow(initialState(0, MIQP_STATE_VY), 2) +
                   pow(initialState(0, MIQP_STATE_VX), 2)));

  for (float t = 0; t < tend; t += dt) {
    bool succ = planner.Plan();
    EXPECT_TRUE(succ);
    Trajectory traj = planner.GetBarkTrajectory(idx, t);
    std::cout << "planned traj" << traj;

    x.push_back(traj(1, X_POSITION));
    y.push_back(traj(1, Y_POSITION));
    v.push_back(traj(1, VEL_POSITION));

    float a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / dt;
    initialState.row(0) = MiqpPlanner::CarStateToMiqpState(
        traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION),
        traj(1, VEL_POSITION), a);
    planner.UpdateCar(idx, initialState, ref);
  }

  EXPECT_LT(x.front(), x.back()) << x;
  EXPECT_GT(y.front(), y.back()) << y;
  EXPECT_LT(v.front(), v.back()) << x << v;
}

TEST(miqp_planner, receding_horizon_twocars) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 10;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref1, ref2;
  ref1.AddPoint({0, 0});
  ref1.AddPoint({50, 0});
  ref2.AddPoint({20, 0});
  ref2.AddPoint({-30, 0});
  Eigen::MatrixXd initialState1(1, 6);
  initialState1 << 0, 4, 0, 1, -0.1, 0;
  Eigen::MatrixXd initialState2(1, 6);
  initialState2 << 20, -4, 0, -1, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx1 = planner.AddCar(initialState1, ref1, vDes, deltaSForDesiredVel);
  int idx2 = planner.AddCar(initialState2, ref2, vDes, deltaSForDesiredVel);

  float tend = 1;
  float dt = settings.ts;

  std::vector<float> x1, y1, x2, y2;
  x1.push_back(initialState1(0, MIQP_STATE_X));
  y1.push_back(initialState1(0, MIQP_STATE_Y));
  x2.push_back(initialState2(0, MIQP_STATE_X));
  y2.push_back(initialState2(0, MIQP_STATE_Y));

  for (float t = 0; t < tend; t += dt) {
    std::cout << std::endl
              << "==== Planner loop t=" << t << " ==== " << std::endl
              << std::endl;

    bool succ = planner.Plan();
    ASSERT_TRUE(succ);
    {
      Trajectory traj = planner.GetBarkTrajectory(idx1, t);

      x1.push_back(traj(1, X_POSITION));
      y1.push_back(traj(1, Y_POSITION));

      float a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / dt;
      initialState1.row(0) = MiqpPlanner::CarStateToMiqpState(
          traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION),
          traj(1, VEL_POSITION), a);
      planner.UpdateCar(idx1, initialState1, ref1);
    }
    {
      Trajectory traj = planner.GetBarkTrajectory(idx2, t);

      x2.push_back(traj(1, X_POSITION));
      y2.push_back(traj(1, Y_POSITION));

      float a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / dt;
      initialState2.row(0) = MiqpPlanner::CarStateToMiqpState(
          traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION),
          traj(1, VEL_POSITION), a);
      planner.UpdateCar(idx2, initialState2, ref2);
    }
  }

  EXPECT_LT(x1.front(), x1.back()) << x1;
  EXPECT_NEAR(y1.back(), 1, 0.2) << y1;
  EXPECT_GT(x2.front(), x2.back()) << x2;
  EXPECT_NEAR(y2.back(), -1, 0.2) << y2;
}

TEST(miqp_planner, nonconvex_env) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-100, -50));
  envPoly.AddPoint(Point2d(-100, 50));
  envPoly.AddPoint(Point2d(0, 40));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-100, -50));
  EXPECT_TRUE(envPoly.Valid()) << "poly not valid";
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({-50, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << -50, 4, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  bool succ = planner.Plan();

  ASSERT_TRUE(parameters->MultiEnvironmentConvexPolygon.size() > 0);
  EXPECT_EQ(parameters->MultiEnvironmentConvexPolygon.size(),
            parameters->nr_environments);
  EXPECT_TRUE(succ);

  Eigen::MatrixXd initialState2(1, 6);
  initialState2 << 0, 4, 0, 0, 0.1, 0;
  planner.UpdateCar(idx, initialState2, ref);
  bool succ2 = planner.Plan();
  EXPECT_TRUE(succ2);
}

TEST(miqp_planner, nonconvex_env_receding_horizon) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-100, -50));
  envPoly.AddPoint(Point2d(-100, 50));
  envPoly.AddPoint(Point2d(0, 40));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-100, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({-100, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << -35, 5, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  bool succ = planner.Plan();

  ASSERT_TRUE(parameters->MultiEnvironmentConvexPolygon.size() > 0);
  EXPECT_EQ(parameters->MultiEnvironmentConvexPolygon.size(),
            parameters->nr_environments);
  EXPECT_TRUE(succ);

  // simulate for a while to get further...
  for (float t = settings.ts; t < 10 * settings.ts; t += settings.ts) {
    Trajectory traj = planner.GetBarkTrajectory(idx, t);

    float a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / settings.ts;
    initialState.row(0) = MiqpPlanner::CarStateToMiqpState(
        traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION),
        traj(1, VEL_POSITION), a);
    planner.UpdateCar(0, initialState, ref);

    bool succ = planner.Plan();
    EXPECT_TRUE(succ);

    if (parameters->nr_environments > 1) {
      break;
    }
  }
}

TEST(miqp_planner, testStaticObstacle_matlab) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-10, 20));
  envPoly.AddPoint(Point2d(80, 20));
  envPoly.AddPoint(Point2d(80, -10));
  envPoly.AddPoint(Point2d(-10, -10));
  envPoly.AddPoint(Point2d(-10, 20));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({100, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 5, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  double d_o = 0.5;
  double x_o = 20;
  Polygon obstacleShape =
      Polygon(bark::geometry::Pose(x_o, 0, 0),
              std::vector<Point2d>{Point2d(-d_o, -d_o), Point2d(-d_o, d_o),
                                   Point2d(d_o, d_o), Point2d(d_o, -d_o),
                                   Point2d(-d_o, -d_o)});
  planner.AddStaticObstacle(obstacleShape);
  EXPECT_EQ(1, parameters->nr_obstacles);
  std::cout << parameters->ObstacleConvexPolygon.at(0).at(0) << std::endl;

  bool succ = planner.Plan();
  EXPECT_TRUE(succ);

  auto traj = planner.GetBarkTrajectory(idx, 0);
  for (int i = 0; i < settings.nr_steps; ++i) {
    double x_i = traj(i, StateDefinition::X_POSITION);
    if (x_i > x_o - d_o - settings.collisionRadius &&
        x_i < x_o + d_o + settings.collisionRadius) {
      // miqp performs evasive maneuver
      EXPECT_LT(1, fabs(traj(i, StateDefinition::Y_POSITION))) << traj;
    }
  }
}

TEST(miqp_planner, testTwoStaticObstacles_matlab) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-10, 4));
  envPoly.AddPoint(Point2d(80, 4));
  envPoly.AddPoint(Point2d(80, -4));
  envPoly.AddPoint(Point2d(-10, -4));
  envPoly.AddPoint(Point2d(-10, 4));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 10;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({100, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 10, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  Polygon obstacleShape1 =
      Polygon(bark::geometry::Pose(20, -1.5, 0),
              std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                   Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                   Point2d(-0.5, -0.5)});
  planner.AddStaticObstacle(obstacleShape1);
  EXPECT_EQ(1, parameters->nr_obstacles);

  Polygon obstacleShape2 =
      Polygon(bark::geometry::Pose(40, +1.5, 0),
              std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                   Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                   Point2d(-0.5, -0.5)});
  planner.AddStaticObstacle(obstacleShape2);
  EXPECT_EQ(2, parameters->nr_obstacles);
  std::cout << parameters->ObstacleConvexPolygon.size() << " " << std::endl;

  bool succ = planner.Plan();
  auto traj = planner.GetBarkTrajectory(idx, 0);

  EXPECT_GT(traj(10, StateDefinition::Y_POSITION), 0)
      << traj;  // passing 1st obstacle on left side
  EXPECT_NEAR(0, traj(10, StateDefinition::Y_POSITION), 0.3) << traj;
  EXPECT_LT(traj(19, StateDefinition::Y_POSITION), 0)
      << traj;  // passing 2nd obstacle on right side
  EXPECT_NEAR(0, traj(19, StateDefinition::Y_POSITION), 0.3) << traj;
  EXPECT_GT(traj(19, StateDefinition::X_POSITION),
            traj(0, StateDefinition::X_POSITION))
      << traj;
  EXPECT_TRUE(succ);
}

TEST(miqp_planner, testDynamicObstacle_matlab) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-10, 20));
  envPoly.AddPoint(Point2d(80, 20));
  envPoly.AddPoint(Point2d(80, -10));
  envPoly.AddPoint(Point2d(-10, -10));
  envPoly.AddPoint(Point2d(-10, 20));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 4;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({100, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 5, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  Polygon obstacleShape =
      Polygon(bark::geometry::Pose(10, 0, 0),
              std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                   Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                   Point2d(-0.5, -0.5)});

  Trajectory predictedTrajObstacle(
      settings.nr_steps, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  State initialStateObstacle(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  initialStateObstacle << 0, obstacleShape.center_(0), obstacleShape.center_(1),
      0, 0;
  for (int i = 0; i < predictedTrajObstacle.rows(); ++i) {
    initialStateObstacle(StateDefinition::X_POSITION) =
        obstacleShape.center_(0) + 5 * i;
    predictedTrajObstacle.row(i) = initialStateObstacle;
  }
  bool is_soft = false;
  bool is_static = false;
  planner.AddObstacle(predictedTrajObstacle, obstacleShape, is_soft, is_static);

  EXPECT_EQ(1, parameters->nr_obstacles);

  bool succ = planner.Plan();
  auto traj = planner.GetBarkTrajectory(idx, 0);

  EXPECT_LT(traj(0, 1), traj(19, 1)) << traj;
  EXPECT_TRUE(succ);
}

TEST(miqp_planner, testSoftDynamicObstacle) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-10, 20));
  envPoly.AddPoint(Point2d(80, 20));
  envPoly.AddPoint(Point2d(80, -10));
  envPoly.AddPoint(Point2d(-10, -10));
  envPoly.AddPoint(Point2d(-10, 20));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 4;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({20, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 5, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  Polygon obstacleShape = Polygon(
      bark::geometry::Pose(10, 0, 0),
      std::vector<Point2d>{Point2d(-2, -2), Point2d(-2, 2), Point2d(2, 2),
                           Point2d(2, -2), Point2d(-2, -2)});

  Trajectory predictedTrajObstacle(
      settings.nr_steps, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  State initialStateObstacle(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  initialStateObstacle << 0, obstacleShape.center_(0), obstacleShape.center_(1),
      0, 0;
  for (int i = 0; i < predictedTrajObstacle.rows(); ++i) {
    initialStateObstacle(StateDefinition::X_POSITION) =
        obstacleShape.center_(0) + 0 * i;
    predictedTrajObstacle.row(i) = initialStateObstacle;
  }
  bool is_soft = true;
  bool is_static = false;
  planner.AddObstacle(predictedTrajObstacle, obstacleShape, is_soft, is_static);

  EXPECT_EQ(1, parameters->nr_obstacles);

  bool succ = planner.Plan();
  auto traj = planner.GetBarkTrajectory(idx, 0);

  EXPECT_TRUE(succ);
}

TEST(miqp_planner, max_time) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 0, 0.1, 0;
  double timeeps = 0.1;

  {
    settings.max_solution_time = 1.0;
    MiqpPlanner planner = MiqpPlanner(settings, envPoly);
    planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
    planner.Plan();
    EXPECT_LT(planner.GetSolutionProperties().time,
              settings.max_solution_time + timeeps);
  }

  {
    settings.max_solution_time = 0.4;
    MiqpPlanner planner = MiqpPlanner(settings, envPoly);
    planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
    planner.Plan();
    EXPECT_LT(planner.GetSolutionProperties().time,
              settings.max_solution_time + timeeps);
  }
}

TEST(miqp_planner, testWarmstart) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-10, 20));
  envPoly.AddPoint(Point2d(80, 20));
  envPoly.AddPoint(Point2d(80, -10));
  envPoly.AddPoint(Point2d(-10, -10));
  envPoly.AddPoint(Point2d(-10, 20));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  settings.max_solution_time = 2;
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  planner.SetDoWarmstart(
      CplexWrapper::WarmstartType::RECEDING_HORIZON_WARMSTART);
  double vDes = 4;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({100, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 5, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  Polygon obstacleShape =
      Polygon(bark::geometry::Pose(40, 0, 0),
              std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                   Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                   Point2d(-0.5, -0.5)});
  planner.AddStaticObstacle(obstacleShape);

  bool succ = planner.Plan();
  EXPECT_TRUE(succ);
  miqp::planner::cplex::SolutionProperties sp1 =
      planner.GetSolutionProperties();

  Eigen::MatrixXd initialState2(1, 6);
  initialState2 << 1.25, 5, 0, 0, 0.1, 0;
  planner.UpdateCar(idx, initialState2, ref);

  succ = planner.Plan();
  EXPECT_TRUE(succ);
  miqp::planner::cplex::SolutionProperties sp2 =
      planner.GetSolutionProperties();

  // EXPECT_GT(sp1.gap, sp2.gap); // not deterministic
  // EXPECT_GT(sp1.objective, sp2.objective); // objective get's worse as long
  // as solution time is small enough to have a sub-optimal solutio (<1s)
}

void recedingHorizonWarmstartTestHelper(
    const Settings& settings, const Polygon& envPoly,
    Eigen::MatrixXd& initialState, const bark::geometry::Line& ref,
    const double& vDes, const double& deltaSForDesiredVel, const double& tend,
    const CplexWrapper::WarmstartType warmstartType) {
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  MiqpPlanner plannerNoWs = MiqpPlanner(settings, envPoly);
  planner.SetDoWarmstart(warmstartType);
  planner.ActivateDebugFileWrite("/tmp", "warmstart_receding_horizon_");

  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
  auto parametersNoWs = plannerNoWs.GetParameters();
  int idxNoWs =
      plannerNoWs.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  std::vector<double> soltime, obj, soltimeNoWs, objNoWs;
  std::vector<bool> did_warmstart_vec;

  double dt = settings.ts;
  for (double t = 0; t < tend; t += dt) {
    // Capture stdout:
    //
  https
      :  // stackoverflow.com/questions/3803465/how-to-capture-stdout-stderr-with-googletest
    std::stringstream buffer;
    std::streambuf* sbuf = std::cout.rdbuf();
    std::cout.rdbuf(buffer.rdbuf());

    // Plan WITH warmstart
    bool succ = planner.Plan(t);
    EXPECT_TRUE(succ);

    if (t != 0) {
      bool did_warmstart =
          buffer.str().find("defined initial solution with objective") !=
          std::string::npos;
      did_warmstart_vec.push_back(did_warmstart);
    }
    std::cout.rdbuf(sbuf);
    std::cout << buffer.str();

    soltime.push_back(planner.GetSolutionProperties().time);
    obj.push_back(planner.GetSolutionProperties().objective);

    // Plan WITHOUT warmstart
    bool succNoWs = plannerNoWs.Plan(t);
    EXPECT_TRUE(succNoWs);  // could fail due to less available time
    soltimeNoWs.push_back(plannerNoWs.GetSolutionProperties().time);
    objNoWs.push_back(plannerNoWs.GetSolutionProperties().objective);

    // End Test if optimization fails
    if (!succ) {
      // Display
      std::cout << "Time with warmstart: ";
      std::copy(soltime.begin(), soltime.end(),
                std::ostream_iterator<double>(std::cout, " "));
      std::cout << std::endl << "Time withOUT warmstart: ";
      std::copy(soltimeNoWs.begin(), soltimeNoWs.end(),
                std::ostream_iterator<double>(std::cout, " "));
      std::cout << std::endl;
      return;
    }

    // Shift car to the next position, use warmstart solution
    Trajectory traj = planner.GetBarkTrajectory(idx, t);
    double a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / dt;
    initialState.row(0) = MiqpPlanner::CarStateToMiqpState(
        traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION),
        traj(1, VEL_POSITION), a);
    planner.UpdateCar(idx, initialState, ref);
    plannerNoWs.UpdateCar(idxNoWs, initialState, ref);
  }

  // Assertions
  const double timeeps = 1.0;                     // runtime is a bit fuzzy
  const double objeps = 2 * abs(log(obj.at(0)));  // some normalization
  std::cout << objeps << std::endl;
  for (size_t i = 1; i < soltime.size(); ++i) {  // 1st is equal anyway
    EXPECT_LT(soltime.at(i) - timeeps, soltimeNoWs.at(i)) << "at idx i=" << i;
    EXPECT_LT(obj.at(i) - objeps, objNoWs.at(i)) << "at idx i=" << i;
  }
  int nr_warmstarts = 0;
  int max_failed_warmstarts = 2;
  for (auto dw : did_warmstart_vec) {
    if (dw) {
      nr_warmstarts++;
    }
  }
  EXPECT_LE(did_warmstart_vec.size(), nr_warmstarts + max_failed_warmstarts)
      << did_warmstart_vec.size() - nr_warmstarts << " warmstarts failed!";

  // Display
  std::cout << "Time with warmstart: ";
  std::copy(soltime.begin(), soltime.end(),
            std::ostream_iterator<double>(std::cout, " "));
  std::cout << std::endl << "Time withOUT warmstart: ";
  std::copy(soltimeNoWs.begin(), soltimeNoWs.end(),
            std::ostream_iterator<double>(std::cout, " "));
  std::cout << std::endl;
  std::cout << std::endl << "Warmstart was performed: ";
  std::copy(did_warmstart_vec.begin(), did_warmstart_vec.end(),
            std::ostream_iterator<bool>(std::cout, " "));
  std::cout << std::endl;
}

TEST(miqp_planner, warmstart_receding_horizon_type1) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();

  double vDes = 10;
  double deltaSForDesiredVel = 1;
  Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({100, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 1, -0.1, 0;

  double tend = 2;

  CplexWrapper::WarmstartType wt =
      CplexWrapper::WarmstartType::RECEDING_HORIZON_WARMSTART;

  recedingHorizonWarmstartTestHelper(settings, envPoly, initialState, ref, vDes,
                                     deltaSForDesiredVel, tend, wt);
}

TEST(miqp_planner, warmstart_receding_horizon_type2) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();

  double vDes = 10;
  double deltaSForDesiredVel = 1;
  Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 1, -0.1, 0;

  double tend = 2;

  CplexWrapper::WarmstartType wt =
      CplexWrapper::WarmstartType::LAST_SOLUTION_WARMSTART;

  recedingHorizonWarmstartTestHelper(settings, envPoly, initialState, ref, vDes,
                                     deltaSForDesiredVel, tend, wt);
}

TEST(miqp_planner, warmstart_receding_horizon_type3) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();

  double vDes = 10;
  double deltaSForDesiredVel = 1;
  Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 1, -0.1, 0;

  double tend = 2;

  CplexWrapper::WarmstartType wt =
      CplexWrapper::WarmstartType::BOTH_WARMSTART_STRATEGIES;

  recedingHorizonWarmstartTestHelper(settings, envPoly, initialState, ref, vDes,
                                     deltaSForDesiredVel, tend, wt);
}

// envpoly and reference line from the py_debugging_toolchain_test unit test
TEST(miqp_planner, warmstart_osterwald_curve) {
  miqp::common::geometry::BoostPolygon bp_tmp;
  const char* envpoly_str =
      "POLYGON((-265.75953657 -545.16659241, -266.5641071  -546.56161547, "
      "-267.17109773 -547.60727811, -267.87923966 -548.81996045, -267.87924745 "
      "-548.81995589, -319.01006226 -636.09913197, -325.04993624 "
      "-632.56078175, -273.919443   -545.28215515, -273.9193367  -545.2819737 "
      ", -273.91923066 -545.28179211, -273.2161219  -544.07773383, "
      "-273.21592481 -544.07739631, -273.2157286  -544.07705829, -272.61787832 "
      "-543.04709932, -272.61766721 -543.04673561, -272.61745711 "
      "-543.04637132, -271.82391022 -541.67040135, -271.82367051 -541.6699857 "
      ", -271.82343213 -541.66956929, -271.03456682 -540.29155035, "
      "-271.03432826 -540.29113362, -271.03409103 -540.29071613, -270.24917011 "
      "-538.90935005, -270.24893288 -538.90893255, -270.24869699 -538.9085143 "
      ", -269.46910748 -537.52624772, -269.4688896  -537.52586141, "
      "-269.46867286 -537.52547445, -252.414427   -507.07705068, -249.61988507 "
      "-508.64228827, -249.36261677 -508.78638636, -249.21410748 "
      "-508.53163898, -249.20136679 -508.50978406, -249.09470212 "
      "-508.34044796, -248.98220693 -508.17479702, -248.86432786 "
      "-508.01344273, -248.74092384 -507.85617643, -248.6120645  "
      "-507.70310422, -248.4782232  -507.55478176, -248.33917944 "
      "-507.41094963, -248.19546486 -507.27216852, -248.04663361 "
      "-507.13803815, -247.89348127 -507.00928588, -247.73619199 "
      "-506.88602418, -247.57449553 -506.7680596 , -247.40895371 "
      "-506.65580572, -247.24010322 -506.54958784, -247.0671183  "
      "-506.44888155, -246.89076995 -506.35420225, -246.7114421  "
      "-506.26572644, -246.52916696 -506.18346501, -246.34416176 "
      "-506.10751631, -246.1564322  -506.03789925, -245.96689996 "
      "-505.97493295, -245.77512123 -505.9184613 , -245.54646316 "
      "-505.86011423, -245.35103791 -505.81755967, -245.1545472  "
      "-505.78138782, -244.95627164 -505.75150681, -244.7576448  "
      "-505.72811218, -244.55877168 -505.71120658, -244.35865206 "
      "-505.70070538, -244.15913093 -505.6966987 , -243.95916298 "
      "-505.69915314, -243.75937314 -505.70808906, -243.55958482 "
      "-505.72352552, -243.36073887 -505.74539177, -243.16304173 "
      "-505.77362613, -242.96603041 -505.80831115, -242.77008895 "
      "-505.84938577, -242.57637571 -505.89658982, -242.38341229 "
      "-505.95028714, -242.19282979 -506.01002613, -242.0040076  "
      "-506.07600954, -241.81731838 -506.14810485, -241.63276943 "
      "-506.22634867, -241.45168382 -506.31012968, -241.36998396 "
      "-506.35119327, -241.10898089 -506.48237737, -240.9708942  "
      "-506.22495955, -240.97038646 -506.22401304, -240.97037139 "
      "-506.22398495, -240.97035633 -506.22395686, -239.45722987 "
      "-503.40179471, -206.04616744 -521.31534866, -209.35383584 "
      "-527.48457311, -242.49522961 -509.71560966, -242.76005765 "
      "-509.57362046, -242.76019532 -509.57387823, -242.92682175 "
      "-509.48420313, -242.93199803 -509.48141742, -242.93727945 "
      "-509.47883661, -242.99478678 -509.45073529, -242.9991519  "
      "-509.44860225, -243.0035838  -509.44661167, -243.07376676 "
      "-509.41508915, -243.07819852 -509.41309863, -243.08269242 "
      "-509.41125266, -243.15389426 -509.38200481, -243.15839148 "
      "-509.38015747, -243.1629462  -509.37845683, -243.23503961 "
      "-509.35153856, -243.23958991 -509.34983957, -243.24419282 "
      "-509.34828877, -243.31713371 -509.32371379, -243.32173692 "
      "-509.32216289, -243.32638793 -509.32076183, -243.40006073 "
      "-509.29856882, -243.40471321 -509.29716732, -243.40940864 "
      "-509.29591723, -243.4837649  -509.27612083, -243.48845859 -509.2748712 "
      ", -243.49319028 -509.27377416, -243.56816251 -509.256392  , "
      "-243.57289582 -509.25529459, -243.57766218 -509.25435104, -243.65317113 "
      "-509.23940329, -243.65793913 -509.23845942, -243.66273521 -509.2376705 "
      ", -243.73863662 -509.22518528, -243.74342989 -509.22439682, "
      "-243.74824617 -509.22376396, -243.82455177 -509.21373745, -243.82936767 "
      "-509.21310465, -243.83420152 -509.21262808, -243.91080806 "
      "-509.20507549, -243.91564427 -509.20459869, -243.92049339 "
      "-509.20427879, -243.99728075 -509.19921294, -244.00212771 "
      "-509.19889318, -244.00698248 -509.1987305 , -244.08387569 "
      "-509.19615389, -244.0887322  -509.19599115, -244.09359143 "
      "-509.19598579, -244.1705603  -509.19590079, -244.17541909 "
      "-509.19589542, -244.18027552 -509.1960474 , -244.2571977  "
      "-509.19845468, -244.26205484 -509.19860669, -244.26690451 "
      "-509.19891593, -244.34370943 -509.20381339, -244.34855752 "
      "-509.20412253, -244.35339306 -509.20458848, -244.43000946 "
      "-509.21197114, -244.43484698 -509.21243728, -244.43966687 "
      "-509.21305986, -244.5159819  -509.22291749, -244.52079785 "
      "-509.22353957, -244.52559115 -509.22431718, -244.60265424 "
      "-509.23681915, -244.6063178  -509.23741349, -244.60996555 "
      "-509.23809829, -244.6748583  -509.25028066, -244.68114206 "
      "-509.25146032, -244.68736986 -509.25290662, -244.78169023 "
      "-509.27481091, -244.78639531 -509.27590358, -244.79106278 "
      "-509.27714716, -244.85840349 -509.29508917, -244.86338361 "
      "-509.29641605, -244.86831522 -509.29791322, -244.93465173 "
      "-509.31805201, -244.93959269 -509.31955202, -244.9444791  "
      "-509.32122117, -245.0100747  -509.34362801, -245.0149585  "
      "-509.34529627, -245.01978203 -509.34713153, -245.08460669 "
      "-509.37179599, -245.08942578 -509.37362956, -245.09417903 "
      "-509.37562765, -245.15807807 -509.40248847, -245.1628375  "
      "-509.40448916, -245.16752528 -509.40665244, -245.23052327 "
      "-509.43572421, -245.23520466 -509.43788453, -245.23980905 "
      "-509.44020445, -245.30170028 -509.47138821, -245.30631085 "
      "-509.47371123, -245.31083876 -509.47619152, -245.37166566 "
      "-509.50951119, -245.37619163 -509.51199042, -245.38062966 "
      "-509.51462384, -245.44025339 -509.55000325, -245.44468747 "
      "-509.55263433, -245.44902852 -509.55541623, -245.50740169 "
      "-509.59282382, -245.51174882 -509.59560961, -245.51599751 "
      "-509.59854334, -245.57306587 -509.63794917, -245.57730979 "
      "-509.64087961, -245.58145048 -509.6439542 , -245.6371265  "
      "-509.68529523, -245.6412713  -509.68837286, -245.64530775 "
      "-509.69159128, -245.69952443 -509.73482026, -245.70355845 "
      "-509.73803674, -245.70747948 -509.74139002, -245.76016335 "
      "-509.78644549, -245.76408333 -509.78979787, -245.76788575 "
      "-509.79328303, -245.81900832 -509.84014007, -245.82281449 "
      "-509.84362868, -245.82649837 -509.84724618, -245.87598798 "
      "-509.89584413, -245.87966738 -509.89945723, -245.88322043 "
      "-509.90319466, -245.93097784 -509.95343035, -245.93453325 "
      "-509.95717026, -245.93795793 -509.96103023, -245.98396972 "
      "-510.01289025, -245.98739382 -510.01674956, -245.99068319 "
      "-510.02072433, -246.03489717 -510.07415107, -246.03818677 "
      "-510.07812612, -246.04133773 -510.08221194, -246.08369297 "
      "-510.13713353, -246.08684523 -510.14122102, -246.089855   "
      "-510.14541454, -246.13025398 -510.20170251, -246.13327224 "
      "-510.20590785, -246.13614362 -510.21021481, -246.17286163 "
      "-510.26529047, -246.17756251 -510.27234162, -246.18185785 "
      "-510.27964691, -246.24620003 -510.38907661, -246.24779401 "
      "-510.39178755, -246.2493308  -510.39453131, -263.3715071  "
      "-540.96416166, -264.16287211 -542.3672438 , -264.16287629 "
      "-542.36724142, -264.95937186 -543.76890655, -265.75953657 "
      "-545.16659241))";
  boost::geometry::read_wkt(envpoly_str, bp_tmp);
  boost::geometry::correct(bp_tmp);
  EXPECT_TRUE(boost::geometry::is_valid(bp_tmp));

  miqp::common::geometry::BoostLinestring bl_tmp;
  const char* line_string =
      "LINESTRING(-310.0 -617.5 , -309.5933532714844 -616.5792236328125 , "
      "-309.08758544921875 -615.716552734375 , -308.5818176269531 "
      "-614.8538818359375 , -308.0760498046875 -613.9912109375 , "
      "-307.5702819824219 -613.1285400390625 , -307.06451416015625 "
      "-612.265869140625 , -306.55877685546875 -611.4031982421875 , "
      "-306.05303955078125 -610.5404663085938 , -305.54730224609375 "
      "-609.6777954101562 ,-305.04156494140625 -608.8151245117188 "
      ",-304.5358581542969 -607.952392578125 , -304.0301208496094 "
      "-607.0897216796875 , -303.5244140625 -606.2269897460938 , "
      "-303.0187072753906 -605.3643188476562 , -302.51300048828125 "
      "-604.5015869140625 , -302.00732421875 -603.6388549804688 , "
      "-301.50164794921875 -602.7761840820312 , -300.9959716796875 "
      "-601.9134521484375 , -300.49029541015625 -601.0507202148438)";
  boost::geometry::read_wkt(line_string, bl_tmp);
  boost::geometry::correct(bl_tmp);
  EXPECT_TRUE(boost::geometry::is_valid(bl_tmp));

  Settings settings = DefaultTestSettings();
  Polygon envPoly = miqp::common::geometry::BoostPolygonToBarkPolygon(bp_tmp);
  EXPECT_TRUE(envPoly.Valid());
  Line ref = miqp::common::geometry::BoostLinestringToBarkLine(bl_tmp);
  EXPECT_TRUE(ref.Valid());

  double vDes = 4;
  double deltaSForDesiredVel = 1;
  Eigen::MatrixXd initialState(1, 6);
  initialState << -310.0, cos(1.05) * vDes, 0, -617.5, sin(1.05) * vDes, 0;

  double tend = 0.5;

  CplexWrapper::WarmstartType wt =
      CplexWrapper::WarmstartType::LAST_SOLUTION_WARMSTART;

  recedingHorizonWarmstartTestHelper(settings, envPoly, initialState, ref, vDes,
                                     deltaSForDesiredVel, tend, wt);
}

TEST(miqp_planner, warmstart_multiple_environments) {
  miqp::common::geometry::BoostPolygon bp_tmp;
  const char* envpoly_str =
      "POLYGON((0 15, 5 10, 10 15, 15 10, 20 15, 25 10, 30 15, 35 10, 40 15, "
      "45 10, "
      "50 15, 50 0, 25 5, 0 0, 0 15))";
  boost::geometry::read_wkt(envpoly_str, bp_tmp);
  boost::geometry::correct(bp_tmp);
  EXPECT_TRUE(boost::geometry::is_valid(bp_tmp));

  miqp::common::geometry::BoostLinestring bl_tmp;
  const char* line_string = "LINESTRING(5 5, 45 5)";
  boost::geometry::read_wkt(line_string, bl_tmp);
  boost::geometry::correct(bl_tmp);
  EXPECT_TRUE(boost::geometry::is_valid(bl_tmp));

  Settings settings = DefaultTestSettings();
  Polygon envPoly = miqp::common::geometry::BoostPolygonToBarkPolygon(bp_tmp);
  EXPECT_TRUE(envPoly.Valid());
  Line ref = miqp::common::geometry::BoostLinestringToBarkLine(bl_tmp);
  EXPECT_TRUE(ref.Valid());

  double vDes = 5;
  double deltaSForDesiredVel = 1;
  Eigen::MatrixXd initialState(1, 6);
  initialState << 3, 2, 0, 5, 0.1, 0;

  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  planner.SetDoWarmstart(CplexWrapper::WarmstartType::LAST_SOLUTION_WARMSTART);
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
  bool succ = planner.Plan(0.0);
  auto envpolys0 = parameters->environmentPolygonIds;
  auto nwer0 = planner.GetWarmstart()->notWithinEnvironmentRear;

  Eigen::MatrixXd nextState(1, 6);
  nextState << planner.GetSolution()->pos_x(0, 1), 2, 0,
      planner.GetSolution()->pos_y(0, 1), 0.1, 0;
  planner.UpdateCar(idx, nextState, ref);
  succ = planner.Plan(1.0);
  auto envpolys1 = parameters->environmentPolygonIds;
  auto nwer1 = planner.GetWarmstart()->notWithinEnvironmentRear;

  EXPECT_EQ(envpolys0, envpolys1);
  // 4 is magic, some deviation of optimal solution and warmstart is ok. 4
  // implies a difference at at most one timesteps if the environemnt itself did
  // not change
  auto tmp = (nwer0 - nwer1).abs().sum();
  Eigen::Tensor<double, 0> four;
  four.setConstant(4);
  Eigen::Tensor<bool, 0> compare = tmp < four;
  EXPECT_TRUE(compare(0)) << tmp << " " << four;

  Eigen::MatrixXd nextnextState(1, 6);
  nextnextState << 15, 2, 0, 5, 0.1, 0;
  planner.UpdateCar(idx, nextnextState, ref);
  succ = planner.Plan(2.0);
  auto envpolys2 = parameters->environmentPolygonIds;
  auto nwer2 = planner.GetWarmstart()->notWithinEnvironmentRear;

  EXPECT_NE(envpolys0, envpolys2);
}

// ACHTUNG diverse hacks in diesem testcase!!
TEST(miqp_planner, stop_reference_end) {
  Settings settings = DefaultTestSettings();
  settings.jerkWeight = 0;
  MiqpPlanner planner = MiqpPlanner(settings);
  planner.ActivateDebugFileWrite("/tmp", "stop_reference_end_");
  double vDes = 1;
  double deltaSForDesiredVel = 10;
  double xend = 10;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({xend, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 1, -0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel, 0.0);

  float tend = 8;
  float dt = settings.ts;
  float x_near_end_threshold = 0.5;
  float v_near_end_threshold = 0.25;

  std::vector<float> x, y, v, vx;
  x.push_back(initialState(0, MIQP_STATE_X));
  y.push_back(initialState(0, MIQP_STATE_Y));
  vx.push_back(initialState(0, MIQP_STATE_VX));
  v.push_back(sqrt(pow(initialState(0, MIQP_STATE_VY), 2) +
                   pow(initialState(0, MIQP_STATE_VX), 2)));

  for (float t = 0; t < tend; t += dt) {
    bool succ = planner.Plan(t);
    EXPECT_TRUE(succ);
    if (!succ || (fabs(v.back()) < 0.1 &&
                  fabs(x.back() - xend) < x_near_end_threshold)) {
      break;
    }
    Trajectory traj = planner.GetBarkTrajectory(idx, t);

    x.push_back(traj(1, X_POSITION));
    y.push_back(traj(1, Y_POSITION));
    v.push_back(traj(1, VEL_POSITION));
    vx.push_back(planner.GetSolution()->vel_x(idx, 1));

    float a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / dt;
    float v = traj(1, VEL_POSITION);
    if (fabs(v) < v_near_end_threshold) {
      // v = 0.0001;
      // a = 0;
      // bark::geometry::Line lastpt;
      // lastpt.AddPoint({traj(1, X_POSITION), traj(1, Y_POSITION)});
      // lastpt.AddPoint({1,2});
      // lastpt.AddPoint({3,4});
      // ref = lastpt;
    }
    initialState.row(0) = MiqpPlanner::CarStateToMiqpState(
        traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION), v,
        a);
    if (x_near_end_threshold > fabs(traj(1, X_POSITION) - xend)) {
      planner.UpdateDesiredVelocity(idx, 0.0f, deltaSForDesiredVel);
    }
    planner.UpdateCar(idx, initialState, ref, t);
  }

  EXPECT_NEAR(v.back(), 0.0, 0.1) << "v = " << v << " vx = " << vx;
  EXPECT_NEAR(x.back(), xend, x_near_end_threshold) << "x = " << x;
}

TEST(miqp_planner, stop_dont_drive) {
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings);
  planner.ActivateDebugFileWrite("/tmp", "stop_dont_drive_");
  double vDes = 5;
  double deltaSForDesiredVel = 10;  // does not matter as we are at end of line
  double xend = 20;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({xend, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << xend, 0.01, 0, 0, 0, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel, 0.0);

  float tend = 0.5;
  float dt = settings.ts;

  std::vector<float> x, y, v;
  x.push_back(initialState(0, MIQP_STATE_X));
  y.push_back(initialState(0, MIQP_STATE_Y));
  v.push_back(sqrt(pow(initialState(0, MIQP_STATE_VY), 2) +
                   pow(initialState(0, MIQP_STATE_VX), 2)));

  for (float t = 0; t < tend; t += dt) {
    bool succ = planner.Plan(t);
    EXPECT_TRUE(succ);
    if (!succ) {
      break;
    }
    Trajectory traj = planner.GetBarkTrajectory(idx, t);

    x.push_back(traj(1, X_POSITION));
    y.push_back(traj(1, Y_POSITION));
    v.push_back(traj(1, VEL_POSITION));

    float a = (traj(1, VEL_POSITION) - traj(0, VEL_POSITION)) / dt;
    initialState.row(0) = MiqpPlanner::CarStateToMiqpState(
        traj(1, X_POSITION), traj(1, Y_POSITION), traj(1, THETA_POSITION),
        traj(1, VEL_POSITION), a);
    planner.UpdateCar(idx, initialState, ref, t);
  }

  EXPECT_NEAR(v.back(), 0.0, 0.1) << v;
}

TEST(miqp_planner, plan_max_fitting_speed_10) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 1, 0.1, 0;

  // Fitting with default max speed 20
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
  EXPECT_TRUE(planner.Plan());
  auto traj = planner.GetBarkTrajectory(idx, 0.0);

  // Fitting with slower max speed 10
  settings.max_velocity_fitting = 10.0;
  MiqpPlanner planner2 = MiqpPlanner(settings, envPoly);
  int idx2 = planner2.AddCar(initialState, ref, vDes, deltaSForDesiredVel);
  EXPECT_TRUE(planner2.Plan());
  auto traj2 = planner2.GetBarkTrajectory(idx2, 0.0);

  // in y-direction the overall error is larger than 0.01, but the fit is
  // different, so deviations are ok
  EXPECT_LT((traj.col(1) - traj2.col(1)).norm(), 0.01);  // x
  EXPECT_LT((traj.col(2) - traj2.col(2)).norm(), 0.35);  // y
  EXPECT_LT((traj.col(3) - traj2.col(3)).norm(), 0.6);   // theta
  EXPECT_LT((traj.col(4) - traj2.col(4)).norm(), 0.02);  // v
}

TEST(miqp_planner, obstacle_inside_outside_env) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-10, 20));
  envPoly.AddPoint(Point2d(80, 20));
  envPoly.AddPoint(Point2d(80, -10));
  envPoly.AddPoint(Point2d(-10, -10));
  envPoly.AddPoint(Point2d(-10, 20));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  auto parameters = planner.GetParameters();

  // dynamic obstacle INSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(0, 0, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});

    Trajectory predictedTrajObstacle(
        settings.nr_steps, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    State initialStateObstacle(
        static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    initialStateObstacle << 0, obstacleShape.center_(0),
        obstacleShape.center_(1), 0, 0;
    for (int i = 0; i < predictedTrajObstacle.rows(); ++i) {
      initialStateObstacle(StateDefinition::X_POSITION) =
          obstacleShape.center_(0) + 5 * i;
      predictedTrajObstacle.row(i) = initialStateObstacle;
    }
    bool is_soft = false;
    bool is_static = false;
    int obs_id = planner.AddObstacle(predictedTrajObstacle, obstacleShape,
                                     is_soft, is_static);
    EXPECT_EQ(0, obs_id);
    EXPECT_EQ(1, parameters->nr_obstacles);
  }

  // dynamic obstacle OUTSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(1000, 1000, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});

    Trajectory predictedTrajObstacle(
        settings.nr_steps, static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    State initialStateObstacle(
        static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    initialStateObstacle << 0, obstacleShape.center_(0),
        obstacleShape.center_(1), 0, 0;
    for (int i = 0; i < predictedTrajObstacle.rows(); ++i) {
      initialStateObstacle(StateDefinition::X_POSITION) =
          obstacleShape.center_(0) + 5 * i;
      predictedTrajObstacle.row(i) = initialStateObstacle;
    }
    bool is_soft = false;
    bool is_static = false;
    int obs_id = planner.AddObstacle(predictedTrajObstacle, obstacleShape,
                                     is_soft, is_static);
    EXPECT_EQ(-1, obs_id) << parameters->ObstacleConvexPolygon.at(1).at(
        10);                                 // invalid
    EXPECT_EQ(1, parameters->nr_obstacles);  // still only one obstacle
  }

  // Static Obstacle INSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(0, 0, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});
    int obs_id = planner.AddStaticObstacle(obstacleShape);
    EXPECT_EQ(1, obs_id);                    // added a second obstacle
    EXPECT_EQ(2, parameters->nr_obstacles);  // two obstacles present
  }

  // Static Obstacle OUTSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(1000, 1000, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});
    int obs_id = planner.AddStaticObstacle(obstacleShape);
    EXPECT_EQ(-1, obs_id);                   // obstacle not added
    EXPECT_EQ(2, parameters->nr_obstacles);  // still two obstacles
  }
}

TEST(miqp_planner, plan_with_far_away_obstacle) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  bool succ = planner.Plan();
  EXPECT_TRUE(succ);

  // Static Obstacle OUTSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(1000, 1000, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});
    int obs_id = planner.AddStaticObstacle(obstacleShape);
    EXPECT_EQ(-1, obs_id);                   // obstacle not added
    EXPECT_EQ(0, parameters->nr_obstacles);  // still no obstacles
  }
}

TEST(miqp_planner, plan_with_inside_poly_outside_ROI) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -50));
  envPoly.AddPoint(Point2d(-50, 50));
  envPoly.AddPoint(Point2d(50, 50));
  envPoly.AddPoint(Point2d(50, -50));
  envPoly.AddPoint(Point2d(-50, -50));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  bool succ = planner.Plan();
  EXPECT_TRUE(succ);

  // Static Obstacle OUTSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(0, -30, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});
    int obs_id = planner.AddStaticObstacle(obstacleShape);
    EXPECT_EQ(-1, obs_id);                   // obstacle not added
    EXPECT_EQ(0, parameters->nr_obstacles);  // still no obstacles
  }
}

TEST(miqp_planner, plan_with_outside_poly_inside_ROI) {
  Polygon envPoly;
  envPoly.AddPoint(Point2d(-50, -5));
  envPoly.AddPoint(Point2d(-50, 5));
  envPoly.AddPoint(Point2d(50, 5));
  envPoly.AddPoint(Point2d(50, -5));
  envPoly.AddPoint(Point2d(-50, -5));
  ASSERT_TRUE(envPoly.Valid());
  Settings settings = DefaultTestSettings();
  MiqpPlanner planner = MiqpPlanner(settings, envPoly);
  double vDes = 5;
  double deltaSForDesiredVel = 1;
  bark::geometry::Line ref;
  ref.AddPoint({0, 0});
  ref.AddPoint({50, 0});
  Eigen::MatrixXd initialState(1, 6);
  initialState << 0, 4, 0, 0, 0.1, 0;
  auto parameters = planner.GetParameters();
  int idx = planner.AddCar(initialState, ref, vDes, deltaSForDesiredVel);

  bool succ = planner.Plan();
  EXPECT_TRUE(succ);

  // Static Obstacle OUTSIDE
  {
    Polygon obstacleShape =
        Polygon(bark::geometry::Pose(-10, 10, 0),
                std::vector<Point2d>{Point2d(-0.5, -0.5), Point2d(-0.5, 0.5),
                                     Point2d(0.5, 0.5), Point2d(0.5, -0.5),
                                     Point2d(-0.5, -0.5)});
    int obs_id = planner.AddStaticObstacle(obstacleShape);
    EXPECT_EQ(-1, obs_id);                   // obstacle not added
    EXPECT_EQ(0, parameters->nr_obstacles);  // still no obstacles
  }
}

// TODO tobias Unit Test for Warmstarting Obstacles

// TODO tobias Unit Test for Warmstarting Multiple Cars
