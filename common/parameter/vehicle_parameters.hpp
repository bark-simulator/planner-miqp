// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef COMMON_PARAMETER_VEHICLE_PARAMETERS_HPP_
#define COMMON_PARAMETER_VEHICLE_PARAMETERS_HPP_

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

namespace miqp {
namespace common {
namespace parameter {

/**
 * @brief structure for pair of (x,y) values
 *
 */
struct XYPair {
  explicit XYPair(float x, float y) : x(x), y(y) {}
  XYPair() : XYPair(0, 0) {}
  XYPair operator+(const XYPair& b) const {
    XYPair sum;
    sum.x = x + b.x;
    sum.y = y + b.y;
    return sum;
  }
  float x;
  float y;
};

/**
 * @brief rotates xypair by some orientation
 *
 * @param input_pair
 * @param theta
 * @return XYPair
 */
inline XYPair RotateVector(XYPair input_pair, float theta) {
  XYPair output_pair;
  output_pair.x = input_pair.x * cos(theta) - input_pair.y * sin(theta);
  output_pair.y = input_pair.x * sin(theta) + input_pair.y * cos(theta);
  return output_pair;
}

/**
 * @brief container for straight vehicle limits
 *
 */
struct StraightLimits {
  explicit StraightLimits(float long_min, float long_max, float lat_min,
                          float lat_max)
      : straight_long_min(long_min, 0),
        straight_long_max(long_max, 0),
        straight_lat_min(0, lat_min),
        straight_lat_max(0, lat_max) {}
  XYPair straight_long_min;
  XYPair straight_long_max;
  XYPair straight_lat_min;
  XYPair straight_lat_max;
};

/**
 * @brief container for vehicle parameters
 *
 */
struct VehicleParameters {
  explicit VehicleParameters(float acc_max, float acc_min, float jerk_max,
                             float acc_lat_min_max, float jerk_lat_min_max)
      : acc_max(acc_max),
        acc_min(acc_min),
        jerk_max(jerk_max),
        acc_lat_min_max(acc_lat_min_max),
        jerk_lat_min_max(jerk_lat_min_max) {}
  float acc_max;
  float acc_min;
  float jerk_max;
  float acc_lat_min_max;
  float jerk_lat_min_max;

  StraightLimits straight_acc_limits =
      StraightLimits(acc_min, acc_max, -acc_lat_min_max, acc_lat_min_max);
  StraightLimits straight_jerk_limits =
      StraightLimits(-jerk_max, jerk_max, -jerk_lat_min_max, jerk_lat_min_max);
};
}  // namespace parameter
}  // namespace common
}  // namespace miqp

#endif  // COMMON_PARAMETER_VEHICLE_PARAMETERS_HPP_