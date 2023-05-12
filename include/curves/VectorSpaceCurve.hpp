/*
 * ScalarCurveConfig.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Paul Furgale, Renaud Dube, Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "curves/Curve.hpp"
#include "curves/VectorSpaceConfig.hpp"

namespace curves {

template <int N>
class VectorSpaceCurve : public Curve<VectorSpaceConfig<N> > {
 public:
  using Parent = Curve<VectorSpaceConfig<N> >;
  using ValueType = typename Parent::ValueType;
  using DerivativeType = typename Parent::DerivativeType;

  VectorSpaceCurve() : dimension_(N) {}

  /// \brief Get the dimension of this curve
  size_t dim() const { return N; }

 private:
  /// The dimension of the vector space.
  size_t dimension_;
};

}  // namespace curves
