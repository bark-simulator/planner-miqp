// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "reference_trajectory_generator.hpp"
#include "bark/commons/timer/timer.hpp"
#include "bark/geometry/commons.hpp"
#include "common/math/math.hpp"

namespace miqp {
namespace common {
namespace reference {
using bark::commons::ParamsPtr;

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;
using bark::models::dynamic::Trajectory;
using common::math::InterpolateWithBounds;

ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(
    const ParamsPtr& params, const float dt,
    const int num_trajectory_time_points, const double line_interp_inc,
    const double vel_desired, const double delta_s_desired,
    const double acc_lat_max, const bool vel_curve_dep)
    : bark::commons::BaseType(params),
      num_trajectory_time_points_(num_trajectory_time_points),
      dt_(dt),
      line_interp_inc_(line_interp_inc),
      vel_desired_(vel_desired),
      delta_s_desired_(delta_s_desired),
      acc_lat_max_(acc_lat_max),
      vel_curve_dep_(vel_curve_dep) {}

Trajectory ReferenceTrajectoryGenerator::GenerateTrajectory(
    const State& state, const Line& center_line) {
  return GenerateTrajectoryInternal(state, vel_desired_, delta_s_desired_,
                                    center_line);
}

Trajectory ReferenceTrajectoryGenerator::GenerateTrajectoryWithSpeedOffset(
    const State& state, const double& speed_offset, const Line& center_line) {
  return GenerateTrajectoryInternal(state, speed_offset * vel_desired_,
                                    delta_s_desired_, center_line);
}

Trajectory ReferenceTrajectoryGenerator::GenerateTrajectoryInternal(
    const State& state, const double& vel_desired,
    const double& delta_s_desired, const Line& center_line) {
  if (std::abs(state(StateDefinition::VEL_POSITION)) < 1e-3) {
    LOG(WARNING)
        << "ReferenceTrajectoryGenerator does not work if velocity equals zero";
  }
  auto timer = bark::commons::timer::Timer();
  timer.Start();
  Line center_line_sub =
      bark::geometry::SmoothLine(center_line, line_interp_inc_);
  Trajectory traj(num_trajectory_time_points_,
                  static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  Point2d pose = Point2d(state(StateDefinition::X_POSITION),
                         state(StateDefinition::Y_POSITION));

  Eigen::VectorXd kappa_line = bark::geometry::GetCurvature(center_line_sub);
  Eigen::VectorXd a_lat_max =
      Eigen::VectorXd::Constant(kappa_line.size(), acc_lat_max_);
  Eigen::VectorXd vel_curve =
      (a_lat_max.cwiseQuotient(kappa_line.cwiseAbs())).cwiseSqrt();

  // adding state at t=0
  traj.block<1, StateDefinition::MIN_STATE_SIZE>(0, 0) =
      state.transpose().block<1, StateDefinition::MIN_STATE_SIZE>(0, 0);

  // match state to center line
  double s_start = GetNearestS(center_line_sub, pose);
  BARK_EXPECT_TRUE(delta_s_desired >= 0.0);
  double s_end = center_line_sub.s_.back();
  double s_desired_vel = std::min(s_end, s_start + delta_s_desired);

  double start_time = state(StateDefinition::TIME_POSITION);

  double t_i = 0;
  double s_i = s_start;
  double vel_0;
  if (delta_s_desired <= 0.0) {
    vel_0 = vel_desired;
  } else {
    vel_0 = state(StateDefinition::VEL_POSITION);
  }

  double vel_i = vel_0;
  double vel_end = vel_desired;

  if (vel_i * num_trajectory_time_points_ * dt_ + s_i > s_end) {
    if (vel_end > 0 || s_desired_vel > s_end) {
      LOG(INFO) << "reference line ends, reference traj using vel_end = 0";
      vel_end = 0;
      s_desired_vel = s_end;
    } else {
      LOG(INFO) << "reference line ends, but intending to stop early anyway";
    }
  }

  // calculate new states on center line
  for (int i = 1; i < num_trajectory_time_points_; ++i) {
    s_i += vel_i * dt_;
    t_i = static_cast<double>(i) * dt_ + start_time;

    Point2d traj_point = GetPointAtS(center_line_sub, s_i);
    double traj_angle = GetTangentAngleAtS(center_line_sub, s_i);

    BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<0>(traj_point)));
    BARK_EXPECT_TRUE(!std::isnan(boost::geometry::get<1>(traj_point)));
    BARK_EXPECT_TRUE(!std::isnan(traj_angle));

    if ((s_desired_vel - s_start) < 1e-2) {
      // e.g. if we are close to lane ending or have driven past it
      // required, otherwise division by zero
      vel_i = 0;
    } else {
      vel_i =
          InterpolateWithBounds(s_start, vel_0, s_desired_vel, vel_end, s_i);
    }
    if (vel_curve_dep_) {
      uint line_idx = FindNearestIdx(center_line_sub, traj_point);

      // get the max possible vel still fullfilling acc_lat_max at that path
      // segment
      double vel_curve_i = vel_curve(line_idx);

      vel_i = std::min(vel_i, vel_curve_i);
    }
    traj(i, StateDefinition::TIME_POSITION) = t_i;
    traj(i, StateDefinition::X_POSITION) = boost::geometry::get<0>(traj_point);
    traj(i, StateDefinition::Y_POSITION) = boost::geometry::get<1>(traj_point);
    traj(i, StateDefinition::THETA_POSITION) = traj_angle;
    traj(i, StateDefinition::VEL_POSITION) = vel_i;
  }

  last_trajectory_ = traj;
  LOG(INFO) << "GenerateTrajectoryInternal took Time [s] ="
            << timer.DurationInSeconds();
  return traj;
}

}  // namespace reference
}  // namespace common
}  // namespace miqp