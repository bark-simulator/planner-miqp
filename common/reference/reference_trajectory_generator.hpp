// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef COMMON_REFERENCE_TRAJECTORY_GENERATOR_HPP_
#define COMMON_REFERENCE_TRAJECTORY_GENERATOR_HPP_

#include "bark/commons/base_type.hpp"
#include "bark/geometry/line.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace miqp {
namespace common {
namespace reference {

class ReferenceTrajectoryGenerator : public bark::commons::BaseType {
 public:
  explicit ReferenceTrajectoryGenerator(
      const bark::commons::ParamsPtr& params, const float dt,
      const int num_trajectory_time_points, const double line_interp_inc,
      const double vel_desired, const double delta_s_desired,
      const double acc_lat_max, const bool vel_curve_dep);

  virtual ~ReferenceTrajectoryGenerator() {}

  /**
   * @brief generates reference trajectory from start state and center line
   *
   * @param state starting state of reference trajectory
   * @param center_line
   * @return bark::models::dynamic::Trajectory
   */
  bark::models::dynamic::Trajectory GenerateTrajectory(
      const bark::models::dynamic::State& state,
      const bark::geometry::Line& center_line);

  /**
   * @brief generates reference trajectory with a desired velocity scaled by
   * speed_offset
   *
   * @param state starting state of reference trajectory
   * @param speed_offset scaling factor [0, 1]
   * @param center_line
   * @return bark::models::dynamic::Trajectory
   */
  bark::models::dynamic::Trajectory GenerateTrajectoryWithSpeedOffset(
      const bark::models::dynamic::State& state, const double& speed_offset,
      const bark::geometry::Line& center_line);

  /**
   * @brief Get the number of trajectory points
   *
   * @return const int
   */
  const int GetNumTrajectoryTimePoints() const {
    return num_trajectory_time_points_;
  }

  /**
   * @brief Get the Delta Time between two trajectory points
   *
   * @return const float
   */
  const float GetDeltaTime() const { return dt_; }

  /**
   * @brief resets centerline to passed centerline object
   *
   * @param center_line
   */
  void ResetCenterLine(const bark::geometry::Line& center_line) {
    center_line_ = center_line;
  }

  /**
   * @brief resets desired velocity and spatial distance when this desired
   * velocity shall be reached
   *
   * @param vel_desired
   * @param delta_s_desired
   */
  void ResetDesiredVelocity(const double& vel_desired,
                            const double& delta_s_desired) {
    vel_desired_ = vel_desired;
    delta_s_desired_ = delta_s_desired;
  }

  /**
   * @brief Get the Center Line object
   *
   * @return bark::geometry::Line
   */
  bark::geometry::Line GetCenterLine() { return center_line_; }

  /**
   * @brief Get the Desired Velocity
   *
   * @return double
   */
  double GetDesiredVelocity() { return vel_desired_; }

  /**
   * @brief Get the Last Trajectory
   *
   * @return bark::models::dynamic::Trajectory
   */
  bark::models::dynamic::Trajectory GetLastTrajectory() const {
    return last_trajectory_;
  }

 private:
  int num_trajectory_time_points_;
  float dt_;
  bark::geometry::Line center_line_;
  double line_interp_inc_;
  double vel_desired_;
  double delta_s_desired_;
  double acc_lat_max_;
  bool vel_curve_dep_;
  bark::models::dynamic::Trajectory last_trajectory_;

  /**
   * @brief generates reference trajectory from start state, desired velocity,
   * and center line
   *
   * @param state
   * @param vel_desired
   * @param delta_s_desired
   * @param center_line
   * @return bark::models::dynamic::Trajectory
   */
  bark::models::dynamic::Trajectory GenerateTrajectoryInternal(
      const bark::models::dynamic::State& state, const double& vel_desired,
      const double& delta_s_desired, const bark::geometry::Line& center_line);
};

}  // namespace reference
}  // namespace common
}  // namespace miqp

#endif  // COMMON_REFERENCE_TRAJECTORY_GENERATOR_HPP_
