/*
 * ScalarCurveConfig.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Paul Furgale, Renaud Dube, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <Eigen/Core>

namespace curves {

template <int N>
struct VectorSpaceConfig {
  using ValueType = Eigen::Matrix<double, N, 1>;
  using DerivativeType = Eigen::Matrix<double, N, 1>;
};

}  // namespace curves
