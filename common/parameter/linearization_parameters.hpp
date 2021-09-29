// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_PARAMETER_LINEARIZATION_PARAMETERS_HPP_
#define MIQP_COMMON_PARAMETER_LINEARIZATION_PARAMETERS_HPP_

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

namespace miqp {
namespace common {
namespace parameter {

struct PolynomialCurvatureParameters {
  Eigen::MatrixXd POLY_KAPPA_AX_MAX;
  Eigen::MatrixXd POLY_KAPPA_AX_MIN;
};

struct PolynomialOrientationParameters {
  Eigen::MatrixXd POLY_SINT_UB;
  Eigen::MatrixXd POLY_SINT_LB;
  Eigen::MatrixXd POLY_COSS_UB;
  Eigen::MatrixXd POLY_COSS_LB;
};

typedef Eigen::MatrixXd FractionParameters;

}  // namespace parameter
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_PARAMETER_LINEARIZATION_PARAMETERS_HPP_