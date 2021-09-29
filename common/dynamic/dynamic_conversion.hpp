// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_DYNAMIC_CONVERSION_HPP_
#define MIQP_COMMON_DYNAMIC_CONVERSION_HPP_

#include <Eigen/Core>
#include <vector>

#include "bark/models/dynamic/dynamic_model.hpp"

namespace miqp {
namespace common {
namespace dynamic {

using bark::models::dynamic::StateDefinition;

inline Eigen::MatrixXd ConvertBarkStateTo2ndOrder(
    Eigen::Matrix<double, Eigen::Dynamic, 1> const& state, const double acc_x,
    const double acc_y) {
  Eigen::MatrixXd state_out(1, 6);
  const double theta = state(StateDefinition::THETA_POSITION);
  const double vel = state(StateDefinition::VEL_POSITION);
  state_out(0) = state(StateDefinition::X_POSITION);
  state_out(1) = vel * cos(theta);
  state_out(2) = acc_x;
  state_out(3) = state(StateDefinition::Y_POSITION);
  state_out(4) = vel * sin(theta);
  state_out(5) = acc_y;
  return state_out;
}

}  // namespace dynamic
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_DYNAMIC_CONVERSION_HPP_
