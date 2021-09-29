// Copyright (c) 2021 fortiss GmbH
//
// Authors: Klemens Esterle and Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_COMMON_MATH_HPP_
#define MIQP_COMMON_MATH_HPP_

#include <boost/math/constants/constants.hpp>

namespace miqp {
namespace common {
namespace math {

inline void WrapRadiantTo2Pi(double& x) {
  x = fmod(x, 2 * boost::math::constants::pi<double>());
  if (x < 0) {
    x += 2 * boost::math::constants::pi<double>();
  }
  assert(x >= 0);
  assert(x <= 2 * boost::math::constants::pi<double>());
}

inline void SwapIfNeeded(float& maxval, float& minval) {
  if (maxval < minval) {
    float tmp = maxval;
    maxval = minval;
    minval = tmp;
  }
}

inline double InterpolateWithBounds(const double x0, const double y0,
                                    const double x1, const double y1,
                                    const double xInterp) {
  double yInterp;
  if (xInterp > x1) {
    yInterp = y1;
  } else if (xInterp < x0) {
    yInterp = y0;
  } else {
    yInterp = (y1 - y0) / (x1 - x0) * (xInterp - x0) + y0;
  }
  return yInterp;
};

}  // namespace math
}  // namespace common
}  // namespace miqp

#endif  // MIQP_COMMON_MATH_HPP_
