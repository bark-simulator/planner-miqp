// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "regions.hpp"
#include <math.h>
#include <iostream>

namespace miqp {
namespace common {
namespace parameter {

Regions CalculateRegionIdx(const FractionParameters& frac_params,
                           const float vx, const float vy) {
  const float eps =
      1e-3;  // parameter to account for ambiguity with floating point
             // conversion, which is why function might return multiple indices
  Regions possible_regions;
  int nr_regions = frac_params.rows();
  for (int idxreg = 0; idxreg < nr_regions; ++idxreg) {
    bool below_ub, above_lb;
    below_ub = frac_params(idxreg, 2) * vy <= frac_params(idxreg, 3) * vx + eps;
    above_lb = frac_params(idxreg, 0) * vy >= frac_params(idxreg, 1) * vx - eps;
    if (below_ub && above_lb) {
      possible_regions.insert(std::pair<bool, RegionIdx>(false, idxreg));
    }
  }
  assert(possible_regions.size() > 0);
  return possible_regions;
}

void SetInitialRegion(Regions& regions_map, RegionIdx idx) {
  for (auto it = regions_map.begin(); it != regions_map.end(); it++) {
    if (it->second == idx) {
      regions_map.erase(it);
      regions_map.emplace(true, idx);
    }
  }
}

void CalculateRegionCombinations(
    const std::vector<Regions>& region_vec_cars_in,
    std::vector<RegionIdx>& region_per_car,
    std::vector<std::vector<RegionIdx>>& region_comb_out, RegionIdx idx) {
  if (static_cast<size_t>(idx) ==
      region_vec_cars_in.size())  // done, no more rows
  {
    region_comb_out.push_back(region_per_car);
  } else {
    Regions regions_car_of_idx = region_vec_cars_in[idx];
    for (auto const& r : regions_car_of_idx) {
      std::vector<int> tmp(region_per_car);
      tmp.push_back(r.second);
      CalculateRegionCombinations(region_vec_cars_in, tmp, region_comb_out,
                                  idx + 1);
    }
  }
}

Eigen::MatrixXi RegionNumberToBinaryArray(const std::set<RegionIdx>& regions,
                                          const int& nrRegions) {
  Eigen::MatrixXi regionVector(1, nrRegions);
  regionVector.setZero();

  for (auto const& r : regions) {
    assert(r < nrRegions);
    regionVector(r) = 1;
  }
  return regionVector;
}

bool ReserveNeighborRegions(Eigen::MatrixXi& regions, const int row,
                            const int expansions) {
  int first = std::numeric_limits<int>::quiet_NaN();
  int last = std::numeric_limits<int>::quiet_NaN();

  const int s = regions.cols();
  for (int i = 0; i < s; ++i) {
    if (regions(row, i) == 1) {
      if (i >= 1 && regions(row, i - 1) == 0) {
        // expand to previous region
        first = i - 1;
      }
      if (i + 1 < s && regions(row, i + 1) == 0) {
        // expand to next region
        last = i + 1;
      }
      if (i == 0 && regions(row, s - 1) == 0) {
        // wrap around front
        first = s - 1;
      }
      if (i == s - 1 && regions(row, 0) == 0) {
        // wrap around end
        last = 0;
      }
    }
  }

  if (std::isnan(first) || std::isnan(last)) {
    return false;  // unexpected input pattern
  } else {
    regions(row, first) = 1;
    regions(row, last) = 1;
  }

  if (expansions > 1) {
    return ReserveNeighborRegions(regions, row, expansions - 1);
  } else {
    return true;
  }
}

std::set<RegionIdx> CalculatePossibleRegions(
    const FractionParameters& frac_params, Eigen::VectorXd& thetaRef) {
  std::set<RegionIdx> possible_regions;
  for (int i = 0; i < thetaRef.size(); ++i) {
    Regions regions = CalculateRegionIdx(frac_params, std::cos(thetaRef(i)),
                                         std::sin(thetaRef(i)));
    for (auto const& r : regions) {
      possible_regions.insert(r.second);
    }
  }
  return possible_regions;
}

}  // namespace parameter
}  // namespace common
}  // namespace miqp
