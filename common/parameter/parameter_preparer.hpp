// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_PARAMETER_PARAMETER_PREPARER_HPP_
#define MIQP_COMMON_PARAMETER_PARAMETER_PREPARER_HPP_

#include "bark/commons/base_type.hpp"
#include "fitting_polynomial_parameters.hpp"
#include "limits_parameters.hpp"
#include "linearization_parameters.hpp"
#include "vehicle_parameters.hpp"

namespace miqp {
namespace common {
namespace parameter {

/**
 * @brief class that prepares and stores all relevant parameters for the miqp
 * model
 *
 */
class ParameterPreparer : public bark::commons::BaseType {
 public:
  explicit ParameterPreparer(
      const bark::commons::ParamsPtr& params, const int nrRegions,
      const float maxVelocityFitting, const float minVelocityFitting,
      const float accLonMaxLimit, const float accLonMinLimit,
      const float jerkLonMaxLimit, const float accLatMinMaxLimit,
      const float jerkLatMinMaxLimit);

  virtual ~ParameterPreparer() {}

  /**
   * @brief Get the Mean Angle Vector object
   *
   * @return std::vector<double>
   */
  std::vector<double> GetMeanAngleVector() const { return mean_angle_vector_; }

  /**
   * @brief calculates acceleration limits per car
   *
   * @return LimitPerRegionParameters
   */
  LimitPerRegionParameters CalculateAccLimitsPerCar() const;

  /**
   * @brief calculates jerk limits per car
   *
   * @return LimitPerRegionParameters
   */
  LimitPerRegionParameters CalculateJerkLimitsPerCar() const;

  /**
   * @brief calculates the mean angle per region and stores them in a vector
   *
   */
  void CalculateMeanAngleVector();

  /**
   * @brief rotates straight limit vectors to max and min rotated limits
   *
   * @param straight_lim straight limits
   * @param angle angle that is used for rotation
   * @param max_pair
   * @param min_pair
   */
  void RotateLimitVectors(const StraightLimits& straight_lim, const float angle,
                          XYPair& max_pair, XYPair& min_pair) const;

  /**
   * @brief Get the Fraction Parameters object
   *
   * @return FractionParameters
   */
  FractionParameters GetFractionParameters() { return fraction_params_; }

  /**
   * @brief Get the Vehicle Parameters object
   *
   * @return VehicleParameters
   */
  VehicleParameters GetVehicleParameters() const { return vehicle_params_; }

  /**
   * @brief Get the Fitting Polynomial Parameters object
   *
   * @return FittingPolynomialParameters
   */
  FittingPolynomialParameters GetFittingPolynomialParameters() const {
    return fittingPolynomial_params_;
  }

 private:
  VehicleParameters vehicle_params_;
  FractionParameters fraction_params_;
  FittingPolynomialParameters fittingPolynomial_params_;
  std::vector<double> mean_angle_vector_;
  const int nrRegions_;
  const float maxVelocityFitting_;

  void CalculateFractionParameters();
};

}  // namespace parameter
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_PARAMETER_PARAMETER_PREPARER_HPP_
