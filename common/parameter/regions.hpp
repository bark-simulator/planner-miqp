// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_PARAMETER_REGIONS_HPP_
#define MIQP_COMMON_PARAMETER_REGIONS_HPP_

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include <map>
#include <set>
#include "linearization_parameters.hpp"

namespace miqp {
namespace common {
namespace parameter {

typedef int RegionIdx;
typedef std::multimap<bool, RegionIdx, std::greater<bool>> Regions;

/**
 * @brief calculates region based on vx and vy. this might yield more than one
 * region at the borders
 *
 * @param frac_params
 * @param vx
 * @param vy
 * @return Regions
 */
Regions CalculateRegionIdx(const FractionParameters& frac_params,
                           const float vx, const float vy);

/**
 * @brief Set the Initial Region
 *
 * @param regions_map
 * @param idx
 */
void SetInitialRegion(Regions& regions_map, RegionIdx idx);

/**
 * @brief Calculates all possible combinations of region vectors of N cars
 *
 * @param region_vec_cars_in
 * @param region_per_car
 * @param region_comb_out
 * @param idx
 */
void CalculateRegionCombinations(
    const std::vector<Regions>& region_vec_cars_in,
    std::vector<RegionIdx>& region_per_car,
    std::vector<std::vector<RegionIdx>>& region_comb_out, RegionIdx idx = 0);

/**
 * @brief converts region number to binary array
 *
 * @param regions
 * @param nrRegions
 * @return Eigen::MatrixXi
 */
Eigen::MatrixXi RegionNumberToBinaryArray(const std::set<RegionIdx>& regions,
                                          const int& nrRegions);

/**
 * @brief reserves neighbour regions as possible to reach for the car
 *
 * @param regions
 * @param row
 * @param expansions
 * @return true if everything worked okay
 */
bool ReserveNeighborRegions(Eigen::MatrixXi& regions, const int row,
                            const int expansions);

/**
 * @brief calculates possible regions from orientation of reference line
 *
 * @param frac_params
 * @param thetaRef
 * @return std::set<RegionIdx>
 */
std::set<RegionIdx> CalculatePossibleRegions(
    const FractionParameters& frac_params, Eigen::VectorXd& thetaRef);

}  // namespace parameter
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_PARAMETER_REGIONS_HPP_