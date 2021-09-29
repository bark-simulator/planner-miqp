// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_PARAMETER_LIMITS_PARAMETERS_HPP_
#define MIQP_COMMON_PARAMETER_LIMITS_PARAMETERS_HPP_

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

namespace miqp {
namespace common {
namespace parameter {

/**
 * @brief Structure containing state limits for all regions
 *
 */
struct LimitPerRegionParameters {
  LimitPerRegionParameters() {}
  explicit LimitPerRegionParameters(int NumCars, int nr_regions) {
    min_x.resize(NumCars, nr_regions);
    max_x.resize(NumCars, nr_regions);
    min_y.resize(NumCars, nr_regions);
    max_y.resize(NumCars, nr_regions);
  }
  Eigen::MatrixXd min_x;
  Eigen::MatrixXd max_x;
  Eigen::MatrixXd min_y;
  Eigen::MatrixXd max_y;
};

}  // namespace parameter
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_PARAMETER_LIMITS_PARAMETERS_HPP_