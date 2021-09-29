// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "parameter_preparer.hpp"
#include <math.h>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include "common/math/math.hpp"

namespace miqp {
namespace common {
namespace parameter {
using bark::commons::ParamsPtr;
using miqp::common::parameter::RotateVector;

ParameterPreparer::ParameterPreparer(
    const ParamsPtr& params, const int nrRegions,
    const float maxVelocityFitting, const float minVelocityFitting,
    const float accLonMaxLimit, const float accLonMinLimit,
    const float jerkLonMaxLimit, const float accLatMinMaxLimit,
    const float jerkLatMinMaxLimit)
    : bark::commons::BaseType(params),
      vehicle_params_(accLonMaxLimit, accLonMinLimit, jerkLonMaxLimit,
                      accLatMinMaxLimit, jerkLatMinMaxLimit),
      fittingPolynomial_params_(FittingPolynomialParameters(
          nrRegions, maxVelocityFitting, minVelocityFitting)),
      nrRegions_(nrRegions),
      maxVelocityFitting_(maxVelocityFitting) {
  CalculateFractionParameters();
  CalculateMeanAngleVector();
}

void ParameterPreparer::CalculateFractionParameters() {
  const int nrRegions = nrRegions_;
  Eigen::VectorXd alphaVec(nrRegions_ + 1);
  alphaVec.setLinSpaced(0, 2 * M_PI);
  Eigen::MatrixXd fraction_parameters_col(nrRegions_, 2);
  fraction_parameters_col.col(0) =
      maxVelocityFitting_ * alphaVec.head(nrRegions_).array().cos();
  fraction_parameters_col.col(1) =
      maxVelocityFitting_ * alphaVec.head(nrRegions_).array().sin();
  fraction_params_.resize(nrRegions, 4);
  fraction_params_.block(0, 0, nrRegions, 2) = fraction_parameters_col;
  fraction_params_.block(0, 2, nrRegions - 1, 2) =
      fraction_parameters_col.block(1, 0, nrRegions - 1, 2);
  fraction_params_.block(nrRegions - 1, 2, 1, 2) =
      fraction_parameters_col.block(0, 0, 1, 2);
}

LimitPerRegionParameters ParameterPreparer::CalculateAccLimitsPerCar() const {
  int nr_regions = fraction_params_.rows();
  const int NumCars = 1;  // only per-car calculation, see addCar() for multiple

  LimitPerRegionParameters acc_limits_per_regions =
      LimitPerRegionParameters(NumCars, nr_regions);

  for (std::size_t i = 0; i < mean_angle_vector_.size(); ++i) {
    XYPair max_acc_pair, min_acc_pair;
    RotateLimitVectors(vehicle_params_.straight_acc_limits,
                       mean_angle_vector_[i], max_acc_pair, min_acc_pair);

    acc_limits_per_regions.min_x(0, i) = min_acc_pair.x;
    acc_limits_per_regions.min_y(0, i) = min_acc_pair.y;
    acc_limits_per_regions.max_x(0, i) = max_acc_pair.x;
    acc_limits_per_regions.max_y(0, i) = max_acc_pair.y;
  }
  return acc_limits_per_regions;
};

LimitPerRegionParameters ParameterPreparer::CalculateJerkLimitsPerCar() const {
  int nr_regions = fraction_params_.rows();
  int NumCars = 1;  // only per-car calculation, see addCar() for multiple

  LimitPerRegionParameters jerk_limits_per_regions =
      LimitPerRegionParameters(NumCars, nr_regions);

  for (std::size_t i = 0; i < mean_angle_vector_.size(); ++i) {
    XYPair max_jerk_pair, min_jerk_pair;
    RotateLimitVectors(vehicle_params_.straight_jerk_limits,
                       mean_angle_vector_[i], max_jerk_pair, min_jerk_pair);

    jerk_limits_per_regions.min_x(0, i) = min_jerk_pair.x;
    jerk_limits_per_regions.min_y(0, i) = min_jerk_pair.y;
    jerk_limits_per_regions.max_x(0, i) = max_jerk_pair.x;
    jerk_limits_per_regions.max_y(0, i) = max_jerk_pair.y;
  }
  return jerk_limits_per_regions;
};

void ParameterPreparer::CalculateMeanAngleVector() {
  int nr_regions = fraction_params_.rows();
  for (int idxreg = 0; idxreg < nr_regions; ++idxreg) {
    double angle_line1 =
        atan2(fraction_params_(idxreg, 1), fraction_params_(idxreg, 0));
    double angle_line2 =
        atan2(fraction_params_(idxreg, 3), fraction_params_(idxreg, 2));

    miqp::common::math::WrapRadiantTo2Pi(angle_line1);
    miqp::common::math::WrapRadiantTo2Pi(angle_line2);

    if (idxreg + 1 == nr_regions) {
      angle_line2 = angle_line2 + 2 * boost::math::constants::pi<double>();
    }
    mean_angle_vector_.push_back((angle_line1 + angle_line2) / 2);
  }
}

void ParameterPreparer::RotateLimitVectors(const StraightLimits& straight_lim,
                                           const float angle, XYPair& max_pair,
                                           XYPair& min_pair) const {
  XYPair max_max = RotateVector(
      straight_lim.straight_long_max + straight_lim.straight_lat_max, angle);
  XYPair max_min = RotateVector(
      straight_lim.straight_long_max + straight_lim.straight_lat_min, angle);
  XYPair min_max = RotateVector(
      straight_lim.straight_long_min + straight_lim.straight_lat_max, angle);
  XYPair min_min = RotateVector(
      straight_lim.straight_long_min + straight_lim.straight_lat_min, angle);

  float max_x =
      std::max(std::max(max_max.x, max_min.x), std::max(min_max.x, min_min.x));
  float min_x =
      std::min(std::min(max_max.x, max_min.x), std::min(min_max.x, min_min.x));
  float max_y =
      std::max(std::max(max_max.y, min_max.y), std::max(min_min.y, max_min.y));
  float min_y =
      std::min(std::min(max_max.y, min_max.y), std::min(min_min.y, max_min.y));

  miqp::common::math::SwapIfNeeded(max_x, min_x);
  miqp::common::math::SwapIfNeeded(max_y, min_y);
  assert(max_x >= min_x);
  assert(max_y >= min_y);

  max_pair.x = max_x;
  max_pair.y = max_y;

  min_pair.x = min_x;
  min_pair.y = min_y;
};

}  // namespace parameter
}  // namespace common
}  // namespace miqp