#pragma once

#include <Eigen/Core>
#include <kindr/Core>

#include "curves/Curve.hpp"

namespace curves {

struct SE3Config {
  using ValueType = kindr::HomTransformQuatD;
  using DerivativeType = kindr::TwistGlobalD;
};

// Curves over SE3 inherit the interface from Curve and CurveBase and define specific
// methods to support physical interpretations of temporal derivatives.
//
// For the purposes of these uses, the curve is defined between Frame a and Frame b
// such that evaluate() returns \f$ \mathbf{T}_{a,b} \f$, the transformation that takes
// points from Frame b to Frame a.
//
class SE3Curve : public Curve<SE3Config> {
 public:
  using Parent = Curve<SE3Config>;
  using ValueType = Parent::ValueType;
  using DerivativeType = Parent::DerivativeType;

  // Following functions added from SlerpSE3 curve .. todo clean

  /// \brief set the minimum sampling period
  virtual void setMinSamplingPeriod(Time time) = 0;

  /// \brief Set the sampling ratio.
  ///   eg. 4 will add a coefficient every 4 extend
  virtual void setSamplingRatio(int ratio) = 0;

  virtual bool isEmpty() const = 0;

  virtual int size() const = 0;
};

}  // namespace curves
