// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "common/reference/reference_trajectory_generator.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "gtest/gtest.h"
// #include "bark/geometry/commons.hpp"
// #include "bark/geometry/line.hpp"

using miqp::common::reference::ReferenceTrajectoryGenerator;

using bark::commons::SetterParams;
using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::models::dynamic::State;
using SD = bark::models::dynamic::StateDefinition;

TEST(reference_trajectory_generator_test, straight_ref_line) {
  Point2d point_1(0.0, 0.0);
  Point2d point_2(50.0, 0.0);
  Point2d point_3(100.0, 0.0);

  Line center_line;

  center_line.AddPoint(point_1);
  center_line.AddPoint(point_2);
  center_line.AddPoint(point_3);

  State current_state(static_cast<int>(SD::MIN_STATE_SIZE));
  current_state << 0.0, 0.0, 0.0, 0.0, 5.0;

  auto params = std::make_shared<SetterParams>();
  float vel_desired = 10.0;
  float delta_s_desired = 0.1;
  float acc_lat_max = 1.0;
  double line_interp_inc = 0.2;
  double dt = 0.2;
  ReferenceTrajectoryGenerator generator(params, dt, 20, line_interp_inc,
                                         vel_desired, delta_s_desired,
                                         acc_lat_max, true);

  auto traj = generator.GenerateTrajectory(current_state, center_line);

  // velocity at t=0 must be current one
  EXPECT_NEAR(traj(0, SD::VEL_POSITION), current_state(SD::VEL_POSITION), 1e-3);

  // velocity at t>0 must be vel_desired
  EXPECT_NEAR(traj(1, SD::VEL_POSITION), vel_desired, 1e-3) << traj;
  EXPECT_NEAR(traj(traj.rows() - 1, SD::VEL_POSITION), vel_desired, 1e-3);
  // x-position has been integrated
  EXPECT_NEAR(traj(1, SD::X_POSITION), current_state(SD::VEL_POSITION) * dt,
              1e-3) << traj;
  // x-position has been integrated
  EXPECT_NEAR(traj(2, SD::X_POSITION),
              current_state(SD::VEL_POSITION) * dt + vel_desired * dt, 1e-3);
}

TEST(reference_trajectory_generator_test, starting) {
  Point2d point_1(0.0, 0.0);
  Point2d point_2(50.0, 0.0);
  Point2d point_3(100.0, 0.0);

  Line center_line;

  center_line.AddPoint(point_1);
  center_line.AddPoint(point_2);
  center_line.AddPoint(point_3);

  State current_state(static_cast<int>(SD::MIN_STATE_SIZE));
  current_state << 0.0, 0.0, 0.0, 0.0, 0.1;

  auto params = std::make_shared<SetterParams>();
  float vel_desired = 10.0;
  float delta_s_desired = 0.1;
  float acc_lat_max = 1.0;
  double line_interp_inc = 0.2;
  double dt = 0.2;
  ReferenceTrajectoryGenerator generator(params, dt, 20, line_interp_inc,
                                         vel_desired, delta_s_desired,
                                         acc_lat_max, true);

  auto traj = generator.GenerateTrajectory(current_state, center_line);

  // velocity at t=0 must be current one
  EXPECT_NEAR(traj(0, SD::VEL_POSITION), current_state(SD::VEL_POSITION), 1e-3);

  // velocity at t>0 must be vel_desired
  EXPECT_GT(traj(1, SD::VEL_POSITION), current_state(SD::VEL_POSITION));
  EXPECT_NEAR(traj(traj.rows() - 1, SD::VEL_POSITION), vel_desired, 1e-3) << traj;
}

TEST(reference_trajectory_generator_test, curved_ref_line) {
  Point2d point_1(0.0, 0.0);
  Point2d point_2(5.0, 0.0);
  Point2d point_3(10.0, 1.5);
  Point2d point_4(20.0, 1.5);
  Point2d point_5(30.0, 1.5);
  Point2d point_6(40.0, 1.5);
  Point2d point_7(50.0, 1.5);
  Point2d point_8(60.0, 1.5);

  Line center_line;

  center_line.AddPoint(point_1);
  center_line.AddPoint(point_2);
  center_line.AddPoint(point_3);
  center_line.AddPoint(point_4);
  center_line.AddPoint(point_5);
  center_line.AddPoint(point_6);
  center_line.AddPoint(point_7);
  center_line.AddPoint(point_8);

  State current_state(static_cast<int>(SD::MIN_STATE_SIZE));
  float vel_desired = 10.0;
  float delta_s_desired = 0.1;
  current_state << 0.0, 0.0, 0.0, 0.0, vel_desired;

  auto params = std::make_shared<SetterParams>();
  float acc_lat_max = 1.8;
  double line_interp_inc = 0.2;
  ReferenceTrajectoryGenerator generator(params, 0.2, 20, line_interp_inc,
                                         vel_desired, delta_s_desired,
                                         acc_lat_max, true);

  auto traj = generator.GenerateTrajectory(current_state, center_line);

  // velocity at t=0 must be current one
  EXPECT_NEAR(traj(0, SD::VEL_POSITION), current_state(SD::VEL_POSITION), 1e-3);

  // velocity at t>0 must be vel_desired
  EXPECT_TRUE(traj(1, SD::VEL_POSITION) < vel_desired) << traj;
}

TEST(reference_trajectory_generator_test, onept) {
  Line center_line;
  center_line.AddPoint({1.0, 0.0});
  center_line.AddPoint({2.0, 0.0});
  center_line.AddPoint({3.0, 0.0});

  State current_state(static_cast<int>(SD::MIN_STATE_SIZE));
  current_state << 0.0, 1.0, 1.0, 0.0, 5.0;

  auto params = std::make_shared<SetterParams>();
  float vel_desired = 0.0;
  float delta_s_desired = 0.1;
  float acc_lat_max = 1.0;
  double line_interp_inc = 0.2;
  double dt = 0.2;
  ReferenceTrajectoryGenerator generator(params, dt, 20, line_interp_inc,
                                         vel_desired, delta_s_desired,
                                         acc_lat_max, true);

  auto traj = generator.GenerateTrajectory(current_state, center_line);

  // velocity at t=0 must be current one
  EXPECT_NEAR(traj(0, SD::VEL_POSITION), current_state(SD::VEL_POSITION), 1e-3);

  double x_traveled = current_state(SD::VEL_POSITION) * dt;
  EXPECT_NEAR(traj(1, SD::X_POSITION),
              current_state(SD::X_POSITION) + x_traveled, 1e-3);
  EXPECT_NEAR(traj(19, SD::X_POSITION),
              current_state(SD::X_POSITION) + x_traveled, 1e-3);
}