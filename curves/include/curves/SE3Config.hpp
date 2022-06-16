/*
 * ScalarCurveConfig.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Paul Furgale, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <Eigen/Core>
#include <kindr/Core>

namespace curves {

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct SE3Config {
  using ValueType = kindr::HomTransformQuatD;
  using DerivativeType = kindr::TwistGlobalD;
};

}  // namespace curves
