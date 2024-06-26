#include "curves/PolynomialSplineContainer.hpp"

namespace curves {

template <int splineOrder_>
PolynomialSplineContainer<splineOrder_>::PolynomialSplineContainer()
    : splines_(),
      timeOffset_(0.0),
      containerTime_(0.0),
      containerDuration_(0.0),
      activeSplineIdx_(0),
      equalityConstraintJacobian_(),
      equalityConstraintTargetValues_(),
      splineCoefficients_(),
      splineDurations_() {
  // Make sure that the container is correctly emptied.
  reset();
}

template <int splineOrder_>
typename PolynomialSplineContainer<splineOrder_>::SplineType* PolynomialSplineContainer<splineOrder_>::getSpline(int splineIndex) {
  return &splines_.at(splineIndex);
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::advance(double dt) {
  if (splines_.empty() || containerTime_ >= containerDuration_) {
    return false;
  }

  // Advance in time.
  containerTime_ += dt;

  // Check if spline index needs to be increased.
  if ((containerTime_ - timeOffset_ >= splines_[activeSplineIdx_].getSplineDuration())) {
    if (activeSplineIdx_ < (splines_.size() - 1)) {
      timeOffset_ += splines_[activeSplineIdx_].getSplineDuration();
      ++activeSplineIdx_;
    }
  }

  return true;
}

template <int splineOrder_>
void PolynomialSplineContainer<splineOrder_>::setContainerTime(double t) {
  containerTime_ = t;
  activeSplineIdx_ = getActiveSplineIndexAtTime(t, timeOffset_);
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::reset() {
  splines_.clear();
  activeSplineIdx_ = 0;
  containerDuration_ = 0.0;
  resetTime();
  return true;
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::resetTime() {
  timeOffset_ = 0.0;
  containerTime_ = 0.0;
  activeSplineIdx_ = 0;
  return true;
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getContainerDuration() const {
  return containerDuration_;
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::isEmpty() const {
  return splines_.empty();
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getPosition() const {
  if (splines_.empty()) {
    return 0.0;
  }
  return splines_[activeSplineIdx_].getPositionAtTime(containerTime_ - timeOffset_);
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getVelocity() const {
  if (splines_.empty()) {
    return 0.0;
  }
  return splines_[activeSplineIdx_].getVelocityAtTime(containerTime_ - timeOffset_);
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getAcceleration() const {
  if (splines_.empty()) {
    return 0.0;
  }
  return splines_[activeSplineIdx_].getAccelerationAtTime(containerTime_ - timeOffset_);
}

template <int splineOrder_>
int PolynomialSplineContainer<splineOrder_>::getActiveSplineIndexAtTime(double t, double& timeOffset) const {
  timeOffset = 0.0;
  if (splines_.empty()) {
    return -1;
  }

  for (size_t i = 0; i < splines_.size(); ++i) {
    if ((t - timeOffset < splines_[i].getSplineDuration())) {
      return i;
    }
    if (i < (splines_.size() - 1)) {
      timeOffset += splines_[i].getSplineDuration();
    }
  }

  return (splines_.size() - 1);
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getPositionAtTime(double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  double timeOffset = 0.0;
  const int activeSplineIdx = getActiveSplineIndexAtTime(t, timeOffset);

  // Spline container is empty.
  if (activeSplineIdx < 0) {
    return 0.0;
  }

  return splines_[activeSplineIdx].getPositionAtTime(t - timeOffset);
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getVelocityAtTime(double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  double timeOffset = 0.0;
  const int activeSplineIdx = getActiveSplineIndexAtTime(t, timeOffset);

  // Spline container is empty.
  if (activeSplineIdx < 0) {
    return 0.0;
  }

  return splines_[activeSplineIdx].getVelocityAtTime(t - timeOffset);
}

template <int splineOrder_>
double PolynomialSplineContainer<splineOrder_>::getAccelerationAtTime(double t) const {
  if (splines_.empty()) {
    return 0.0;
  }
  double timeOffset = 0.0;
  const int activeSplineIdx = getActiveSplineIndexAtTime(t, timeOffset);

  // Spline container is empty.
  if (activeSplineIdx < 0) {
    return 0.0;
  }

  return splines_[activeSplineIdx].getAccelerationAtTime(t - timeOffset);
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::setData(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions,
                                                      double initialVelocity, double initialAcceleration, double finalVelocity,
                                                      double finalAcceleration) {
  checkNumKnotsValid(knotDurations, knotPositions);
  bool success = reset();

  // Set up optimization parameters.
  const unsigned int numSplines = knotDurations.size() - 1;
  constexpr auto num_coeffs_spline = SplineType::coefficientCount;
  const unsigned int solutionSpaceDimension = numSplines * num_coeffs_spline;
  const unsigned int num_junctions = numSplines - 1;

  if (numSplines < 1) {
    std::cout << "[PolynomialSplineContainer::setData] Not enough knot points available!" << std::endl;
    return false;
  }

  // Total number of constraints.
  constexpr unsigned int num_initial_constraints = 3;  // pos, vel, accel
  constexpr unsigned int num_final_constraints = 3;    // pos, vel, accel
  constexpr unsigned int num_constraint_junction = 4;  // pos (2x), vel, accel
  const unsigned int num_junction_constraints = num_junctions * num_constraint_junction;
  const unsigned int num_constraints = num_junction_constraints + num_initial_constraints + num_final_constraints;

  // Drop constraints if necessary.
  if (num_constraints > solutionSpaceDimension) {
    std::cout << "[PolynomialSplineContainer::setData] Number of equality constraints is larger than number of coefficients. Drop "
                 "acceleration constraints!"
              << std::endl;
    return setData(knotDurations, knotPositions, initialVelocity, finalVelocity);
  }

  // Vector containing durations of splines.
  splineDurations_.resize(numSplines);
  for (unsigned int splineId = 0; splineId < numSplines; splineId++) {
    splineDurations_[splineId] = knotDurations[splineId + 1] - knotDurations[splineId];

    if (splineDurations_[splineId] <= 0.0) {
      std::cout << "[PolynomialSplineContainer::setData] Invalid spline duration at index" << splineId << ": " << splineDurations_[splineId]
                << std::endl;
      return false;
    }
  }

  // Initialize Equality matrices.
  equalityConstraintJacobian_.setZero(num_constraints, solutionSpaceDimension);
  equalityConstraintTargetValues_.setZero(num_constraints);
  unsigned int constraintIdx = 0;

  // Initial conditions.
  Eigen::Vector3d initialConditions(knotPositions.front(), initialVelocity, initialAcceleration);
  addInitialConditions(initialConditions, constraintIdx);

  // Final conditions.
  Eigen::Vector3d finalConditions(knotPositions.back(), finalVelocity, finalAcceleration);
  addFinalConditions(finalConditions, constraintIdx, splineDurations_.back(), num_junctions);

  // Junction conditions.
  addJunctionsConditions(splineDurations_, knotPositions, constraintIdx, num_junctions);

  if (num_constraints != constraintIdx) {
    std::cout << "[PolynomialSplineContainer::setData] Wrong number of equality constraints!" << std::endl;
    return false;
  }

  // Find spline coefficients.
  splineCoefficients_ = equalityConstraintJacobian_.colPivHouseholderQr().solve(equalityConstraintTargetValues_);

  // Extract spline coefficients and add splines.
  success &= extractSplineCoefficients(splineCoefficients_, splineDurations_, numSplines);

  return success;
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::setData(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions,
                                                      double initialVelocity, double finalVelocity) {
  checkNumKnotsValid(knotDurations, knotPositions);
  bool success = reset();

  // Set up optimization parameters.
  const unsigned int numSplines = knotDurations.size() - 1u;
  constexpr auto num_coeffs_spline = SplineType::coefficientCount;
  const unsigned int solutionSpaceDimension = numSplines * num_coeffs_spline;
  const unsigned int num_junctions = numSplines - 1u;

  if (numSplines < 1u) {
    std::cout << "[PolynomialSplineContainer::setData] Not enough knot points available!" << std::endl;
    return false;
  }

  // Total number of constraints.
  constexpr unsigned int num_initial_constraints = 2u;  // pos, vel
  constexpr unsigned int num_final_constraints = 2u;    // pos, vel
  constexpr unsigned int num_constraint_junction = 4u;  // pos (2x), vel, acc
  const unsigned int num_junction_constraints = num_junctions * num_constraint_junction;
  const unsigned int num_constraints = num_junction_constraints + num_initial_constraints + num_final_constraints;

  // Drop constraints if necessary.
  if (num_constraints > solutionSpaceDimension) {
    std::cout << "[PolynomialSplineContainer::setData] Number of equality constraints is larger than number of coefficients. Drop "
                 "acceleration constraints!"
              << std::endl;
    return setData(knotDurations, knotPositions);
  }

  // Vector containing durations of splines.
  splineDurations_.resize(numSplines);
  for (unsigned int splineId = 0; splineId < numSplines; splineId++) {
    splineDurations_[splineId] = knotDurations[splineId + 1] - knotDurations[splineId];

    if (splineDurations_[splineId] <= 0.0) {
      std::cout << "[PolynomialSplineContainer::setData] Invalid spline duration at index" << splineId << ": " << splineDurations_[splineId]
                << std::endl;
      return false;
    }
  }

  // Initialize Equality matrices.
  equalityConstraintJacobian_.setZero(num_constraints, solutionSpaceDimension);
  equalityConstraintTargetValues_.setZero(num_constraints);
  unsigned int constraintIdx = 0;

  // Initial conditions.
  Eigen::Vector2d initialConditions(knotPositions.front(), initialVelocity);
  addInitialConditions(initialConditions, constraintIdx);

  // Final conditions.
  Eigen::Vector2d finalConditions(knotPositions.back(), finalVelocity);
  addFinalConditions(finalConditions, constraintIdx, splineDurations_.back(), num_junctions);

  // Junction conditions.
  addJunctionsConditions(splineDurations_, knotPositions, constraintIdx, num_junctions);

  if (num_constraints != constraintIdx) {
    std::cout << "[PolynomialSplineContainer::setData] Wrong number of equality constraints!" << std::endl;
    return false;
  }

  // Find spline coefficients.
  splineCoefficients_ = equalityConstraintJacobian_.colPivHouseholderQr().solve(equalityConstraintTargetValues_);

  // Extract spline coefficients and add splines.
  success &= extractSplineCoefficients(splineCoefficients_, splineDurations_, numSplines);

  return success;
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::setData(const std::vector<double>& knotDurations, const std::vector<double>& knotPositions) {
  checkNumKnotsValid(knotDurations, knotPositions);
  bool success = true;

  const auto numSplines = knotDurations.size() - 1;
  constexpr auto num_coeffs_spline = SplineType::coefficientCount;
  typename SplineType::SplineCoefficients coefficients;

  if (splineOrder_ == 0 || numSplines < 1) {
    return false;
  }

  std::fill(coefficients.begin(), coefficients.end(), 0.0);

  success &= reset();

  for (std::size_t splineId = 0; splineId < numSplines; ++splineId) {
    const double duration = knotDurations[splineId + 1] - knotDurations[splineId];

    if (duration <= 0.0) {
      return false;
    }

    coefficients[num_coeffs_spline - 1] = knotPositions[splineId];                                             // a0
    coefficients[num_coeffs_spline - 2] = (knotPositions[splineId + 1] - knotPositions[splineId]) / duration;  // a1
    success &= this->addSpline(SplineType(coefficients, duration));
  }

  return success;
}

template <int splineOrder_>
void PolynomialSplineContainer<splineOrder_>::addInitialConditions(const Eigen::Ref<const Eigen::VectorXd>& initialConditions,
                                                                   unsigned int& constraintIdx) {
  // Initial position.
  if (initialConditions.size() > 0) {
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(0)) =
        SplineType::getTimeVectorAtZero();
    equalityConstraintTargetValues_(constraintIdx) = initialConditions(0);
    ++constraintIdx;
  }

  // Initial velocity.
  if (initialConditions.size() > 1) {
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(0)) =
        SplineType::getDTimeVectorAtZero();
    equalityConstraintTargetValues_(constraintIdx) = initialConditions(1);
    ++constraintIdx;
  }

  // Initial acceleration.
  if (initialConditions.size() > 2) {
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(0)) =
        SplineType::getDDTimeVectorAtZero();
    equalityConstraintTargetValues_(constraintIdx) = initialConditions(2);
    ++constraintIdx;
  }
}

template <int splineOrder_>
void PolynomialSplineContainer<splineOrder_>::addFinalConditions(const Eigen::Ref<const Eigen::VectorXd>& finalConditions,
                                                                 unsigned int& constraintIdx, const double lastSplineDuration,
                                                                 unsigned int lastSplineId) {
  // Initial position.
  if (finalConditions.size() > 0) {
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(lastSplineId)) =
        SplineType::getTimeVector(lastSplineDuration);
    equalityConstraintTargetValues_(constraintIdx) = finalConditions(0);
    constraintIdx++;
  }

  // Initial velocity.
  if (finalConditions.size() > 1) {
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(lastSplineId)) =
        SplineType::getDTimeVector(lastSplineDuration);
    ;
    equalityConstraintTargetValues_(constraintIdx) = finalConditions(1);
    constraintIdx++;
  }

  // Initial acceleration.
  if (finalConditions.size() > 2) {
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(lastSplineId)) =
        SplineType::getDDTimeVector(lastSplineDuration);
    ;
    equalityConstraintTargetValues_(constraintIdx) = finalConditions(2);
    constraintIdx++;
  }
}

template <int splineOrder_>
void PolynomialSplineContainer<splineOrder_>::addJunctionsConditions(const SplineDurations& splineDurations,
                                                                     const std::vector<double>& knotPositions, unsigned int& constraintIdx,
                                                                     unsigned int num_junctions) {
  for (unsigned int splineId = 0; splineId < num_junctions; splineId++) {
    const unsigned int nextSplineId = splineId + 1;

    // Smooth position transition with fixed positions.
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(splineId)) =
        SplineType::getTimeVector(splineDurations[splineId]);
    equalityConstraintTargetValues_(constraintIdx) = knotPositions[nextSplineId];
    constraintIdx++;

    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(nextSplineId)) =
        SplineType::getTimeVectorAtZero();
    equalityConstraintTargetValues_(constraintIdx) = knotPositions[nextSplineId];
    constraintIdx++;

    // Smooth velocity transition.
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(splineId)) =
        SplineType::getDTimeVector(splineDurations[splineId]);
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(nextSplineId)) =
        -SplineType::getDTimeVectorAtZero();
    equalityConstraintTargetValues_(constraintIdx) = 0.0;
    constraintIdx++;

    // Smooth acceleration transition.
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(splineId)) =
        SplineType::getDDTimeVector(splineDurations[splineId]);
    equalityConstraintJacobian_.template block<1, SplineType::coefficientCount>(constraintIdx, getSplineColumnIndex(nextSplineId)) =
        -SplineType::getDDTimeVectorAtZero();
    equalityConstraintTargetValues_(constraintIdx) = 0.0;
    constraintIdx++;
  }
}

template <int splineOrder_>
bool PolynomialSplineContainer<splineOrder_>::extractSplineCoefficients(const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                                                                        const SplineDurations& splineDurations,
                                                                        const unsigned int numSplines) {
  typename SplineType::SplineCoefficients coefficients;

  for (unsigned int splineId = 0; splineId < numSplines; ++splineId) {
    Eigen::Map<Eigen::VectorXd>(coefficients.data(), SplineType::coefficientCount, 1) =
        coeffs.segment<SplineType::coefficientCount>(getSplineColumnIndex(splineId));
    this->addSpline(SplineType(coefficients, splineDurations[splineId]));
  }

  return true;
}

template <int splineOrder_>
constexpr int PolynomialSplineContainer<splineOrder_>::getMaxSolutionDimension() {
  return (maxKnots_ - 1) * SplineType::coefficientCount;
}

template <int splineOrder_>
constexpr int PolynomialSplineContainer<splineOrder_>::getMaxNumEqualityConstraints() {
  /*
   * max_num_junctions = maxKnots_ - 2
   * max_constraints_per_junction = 4 (2x pos + vel + acc)
   * max_num_junction_constraints = max_num_junctions * max_constraints_per_junction
   *
   * max_initial_constraints = 3 (pos + vel + acc)
   * max_final_constraints = 3 (pos + vel + acc)
   *
   * max_num_constraints = max_initial_constraints + max_num_junction_constraints + max_final_constraints
   *                     = 4 * maxKnots_ - 2 (on simplification)
   */
  return 4 * maxKnots_ - 2;
}

template <int splineOrder_>
void PolynomialSplineContainer<splineOrder_>::checkNumKnotsValid(const std::vector<double>& knotDurations,
                                                                 const std::vector<double>& knotPositions) const {
  if (knotDurations.size() != knotPositions.size()) {
    throw std::invalid_argument("Number of knots in duration and position vectors do not match.");
  }
  if (knotDurations.size() > maxKnots_) {
    throw std::invalid_argument("Number of knots has to be less than or equal to " + std::to_string(maxKnots_) + ".");
  }
}

}  // namespace curves
