#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <std_utils/containers/VectorMaxSize.hpp>

#include "curves/PolynomialSpline.hpp"

namespace curves {

template <int splineOrder_>
class PolynomialSplineContainer {
 public:
  static constexpr int maxKnots_ = 10;

  using SplineType = PolynomialSpline<splineOrder_>;
  using SplineList = std_utils::VectorMaxSize<SplineType, maxKnots_ - 1>;
  using SplineDurations = std_utils::VectorMaxSize<double, maxKnots_ - 1>;

  PolynomialSplineContainer();

  //! Get a pointer to the spline with a given index.
  SplineType* getSpline(int splineIndex);

  //! Update the spline internal time by dt [seconds].
  bool advance(double dt);

  //! Jump to a specific point in time domain.
  void setContainerTime(double t);

  //! Add a spline to the container.
  template <typename SplineType_>
  bool addSpline(SplineType_&& spline) {
    containerDuration_ += spline.getSplineDuration();
    splines_.push_back(std::forward<SplineType_>(spline));
    return true;
  }

  //! Clear spline container.
  bool reset();

  //! Set container.
  bool resetTime();

  //! Get total trajectory duration.
  double getContainerDuration() const;

  //! True if splines are empty.
  bool isEmpty() const;

  //! Get the position evaluated at the internal time.
  double getPosition() const;

  //! Get the velocity evaluated at the internal time.
  double getVelocity() const;

  //! Get the acceleration evaluated at the internal time.
  double getAcceleration() const;

  /*! Get the index of the spline active at time t [seconds].
   *  Update timeOffset with the duration of the container at the beginning of the active spline.
   */
  int getActiveSplineIndexAtTime(double t, double& timeOffset) const;

  //! Get position at time t[seconds];
  double getPositionAtTime(double t) const;

  //! Get velocity at time t[seconds];
  double getVelocityAtTime(double t) const;

  //! Get acceleration at time t[seconds];
  double getAccelerationAtTime(double t) const;

  /*!
   * Minimize spline coefficients s.t. position, velocity and acceleration constraints are satisfied
   * (i.e., s.t. the spline conjunction is smooth up the second derivative).
   */
  bool setData(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions, double initialVelocity,
               double initialAcceleration, double finalVelocity, double finalAcceleration);

  /*!
   * Minimize spline coefficients s.t. position and velocity constraints are satisfied
   * (i.e., s.t. the spline conjunction is smooth up the first derivative).
   */
  bool setData(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions, double initialVelocity,
               double finalVelocity);

  /*!
   * Find linear part of the spline coefficients (a0, a1) s.t. position constraints are satisfied.
   * If the spline order is larger than 1, the remaining spline coefficients are set to zero.
   */
  virtual bool setData(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions);

  //! Get the maximum number of splines allowed in a container.
  static constexpr int getMaxNumSplines() { return maxKnots_ - 1; }

 protected:
  /*!
   * aijh:
   *  i --> spline id (1,...,n)
   *  j --> spline coefficient aj (a5,...,a1,a0)
   *  h --> dimX, dimY
   *
   * Coefficient vector is:
   *    q = [a15x a14x ... a10x a15y ... a10y a25x ... a20y ... an5x ... an0y]
   */
  inline int getCoeffIndex(const int splineIdx, const int aIdx) const { return splineIdx * (splineOrder_ + 1) + aIdx; }

  inline int getSplineColumnIndex(const int splineIdx) const { return getCoeffIndex(splineIdx, 0); }

  void addInitialConditions(const Eigen::Ref<const Eigen::VectorXd>& initialConditions, unsigned int& constraintIdx);

  void addFinalConditions(const Eigen::Ref<const Eigen::VectorXd>& finalConditions, unsigned int& constraintIdx, double lastSplineDuration,
                          unsigned int lastSplineId);

  void addJunctionsConditions(const SplineDurations& splineDurations, const std::vector<double>& knotPositions, unsigned int& constraintIdx,
                              unsigned int num_junctions);

  bool extractSplineCoefficients(const Eigen::Ref<const Eigen::VectorXd>& coeffs, const SplineDurations& splineDurations,
                                 unsigned int numSplines);

 private:
  //! Returns the maximum dimension of the solution of the quadratic program in compile time.
  static inline constexpr int getMaxSolutionDimension();

  //! Returns the maximum number of constraints for the quadratic program in compile time.
  static inline constexpr int getMaxNumEqualityConstraints();

  //! @throws invalid_argument if the number of knots provided are valid.
  void checkNumKnotsValid(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions) const;

  //! Conjunction of smoothly interconnected splines.
  SplineList splines_;

  //! Helper variable.
  double timeOffset_;

  //! Current time of spline conjunction.
  double containerTime_;

  //! Total duration of spline conjunction.
  double containerDuration_;

  //! Spline index currently active.
  int activeSplineIdx_;

  //! Equality matrix of quadratic program (A in Ax=b).
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, getMaxNumEqualityConstraints(), getMaxSolutionDimension()>
      equalityConstraintJacobian_;

  //! Equality target values of quadratic program (b in Ax=b).
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, getMaxNumEqualityConstraints(), 1> equalityConstraintTargetValues_;

  //! Solution to the quadratic program (x in Ax=b).
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, getMaxSolutionDimension(), 1> splineCoefficients_;

  //! Container for all spline durations.
  SplineDurations splineDurations_;
};

}  // namespace curves

#include <curves/PolynomialSplineContainer.tpp>
