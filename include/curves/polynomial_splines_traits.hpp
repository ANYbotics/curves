#pragma once

#include <array>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/QR>

namespace curves {

struct SplineOptions {
  SplineOptions() : tf_(0.0), pos0_(0.0), posT_(0.0), vel0_(0.0), velT_(0.0), acc0_(0.0), accT_(0.0) {}

  constexpr SplineOptions(double tf, double pos0, double posT, double vel0, double velT, double acc0, double accT)
      : tf_(tf), pos0_(pos0), posT_(posT), vel0_(vel0), velT_(velT), acc0_(acc0), accT_(accT) {}

  //! The total duration of the spline in seconds.
  double tf_;

  //! The scalar position at time 0.
  double pos0_;

  //! The scalar position at time tf.
  double posT_;

  //! The scalar velocity at time 0.
  double vel0_;

  //! The scalar velocity at time tf.
  double velT_;

  //! The scalar acceleration at time 0.
  double acc0_;

  //! The scalar acceleration at time tf.
  double accT_;
};

namespace spline_traits {

// Main struct template.
template <typename Core_, int SplineOrder_>
struct spline_rep {
  static constexpr unsigned int splineOrder = SplineOrder_;
  static constexpr unsigned int numCoefficients = SplineOrder_ + 1;

  using TimeVectorType = std::array<Core_, numCoefficients>;
  using SplineCoefficients = std::array<Core_, numCoefficients>;

  static inline TimeVectorType tau(Core_ tk) noexcept;
  static inline TimeVectorType dtau(Core_ tk) noexcept;
  static inline TimeVectorType ddtau(Core_ tk) noexcept;

  static bool compute(const SplineOptions& opts, SplineCoefficients& coefficients);
};

// Specialization for linear splines.
template <>
struct spline_rep<double, 1> {
  static constexpr unsigned int splineOrder = 1;
  static constexpr unsigned int numCoefficients = splineOrder + 1;

  using TimeVectorType = std::array<double, numCoefficients>;
  using SplineCoefficients = std::array<double, numCoefficients>;

  static inline TimeVectorType tau(double tk) noexcept { return {tk, 1.0}; }

  static inline TimeVectorType dtau(double /*tk*/) noexcept { return {1.0, 0.0}; }

  static inline TimeVectorType ddtau(double /*tk*/) noexcept { return {0.0, 0.0}; }

  static constexpr TimeVectorType tauZero{{0.0, 1.0}};
  static constexpr TimeVectorType dtauZero{{1.0, 0.0}};
  static constexpr TimeVectorType ddtauZero{{0.0, 0.0}};

  //! Map initial pos and final pos to spline coefficients.
  static bool compute(const SplineOptions& opts, SplineCoefficients& coefficients) {
    coefficients[1] = opts.pos0_;                            // a0
    coefficients[0] = (opts.posT_ - opts.pos0_) / opts.tf_;  // a1

    return true;
  }
};

// Specialization for quadratic splines.
template <>
struct spline_rep<double, 2> {
  static constexpr unsigned int splineOrder = 2;
  static constexpr unsigned int numCoefficients = splineOrder + 1;

  using TimeVectorType = std::array<double, numCoefficients>;
  using SplineCoefficients = std::array<double, numCoefficients>;

  static inline TimeVectorType tau(double tk) noexcept { return {std::pow(tk, 2), tk, 1.0}; }

  static inline TimeVectorType dtau(double tk) noexcept { return {2.0 * tk, 1.0, 0.0}; }

  static inline TimeVectorType ddtau(double /*tk*/) noexcept { return {2.0, 0.0, 0.0}; }

  static constexpr TimeVectorType tauZero{{0.0, 0.0, 1.0}};
  static constexpr TimeVectorType dtauZero{{0.0, 1.0, 0.0}};
  static constexpr TimeVectorType ddtauZero{{2.0, 0.0, 0.0}};

  //! Map initial pos, initial vel and final pos to spline coefficients.
  static bool compute(const SplineOptions& opts, SplineCoefficients& coefficients) {
    coefficients[2] = opts.pos0_;                                                                 // a0
    coefficients[1] = opts.vel0_;                                                                 // a1
    coefficients[0] = (opts.posT_ - opts.pos0_ - opts.vel0_ * opts.tf_) / std::pow(opts.tf_, 2);  // a2

    return true;
  }
};

// Specialization for third order splines (cubic).
template <>
struct spline_rep<double, 3> {
  static constexpr unsigned int splineOrder = 3;
  static constexpr unsigned int numCoefficients = splineOrder + 1;

  using TimeVectorType = std::array<double, numCoefficients>;
  using SplineCoefficients = std::array<double, numCoefficients>;

  static inline TimeVectorType tau(double tk) noexcept { return {std::pow(tk, 3), std::pow(tk, 2), tk, 1.0}; }

  static inline TimeVectorType dtau(double tk) noexcept { return {3.0 * std::pow(tk, 2), 2.0 * tk, 1.0, 0.0}; }

  static inline TimeVectorType ddtau(double tk) noexcept { return {6.0 * tk, 2.0, 0.0, 0.0}; }

  static constexpr TimeVectorType tauZero{{0.0, 0.0, 0.0, 1.0}};
  static constexpr TimeVectorType dtauZero{{0.0, 0.0, 1.0, 0.0}};
  static constexpr TimeVectorType ddtauZero{{0.0, 2.0, 0.0, 0.0}};

  //! Map initial pos, initial vel, final pos and final vel to spline coefficients.
  static bool compute(const SplineOptions& opts, SplineCoefficients& coefficients) {
    Eigen::Matrix<double, numCoefficients, 1> b;
    b << opts.pos0_, opts.vel0_, opts.posT_, opts.velT_;

    Eigen::Matrix<double, numCoefficients, numCoefficients> A;
    A << Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(tauZero.data()),
        Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(dtauZero.data()),
        Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((tau(opts.tf_)).data()),
        Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((dtau(opts.tf_)).data());

    Eigen::Map<Eigen::VectorXd>(coefficients.data(), numCoefficients, 1) = A.colPivHouseholderQr().solve(b);

    return true;
  }
};

// Specialization for fourth order (quartic) splines.
template <>
struct spline_rep<double, 4> {
  static constexpr unsigned int splineOrder = 4;
  static constexpr unsigned int numCoefficients = splineOrder + 1;

  using TimeVectorType = std::array<double, numCoefficients>;
  using SplineCoefficients = std::array<double, numCoefficients>;

  static inline TimeVectorType tau(double tk) noexcept { return {std::pow(tk, 4), std::pow(tk, 3), std::pow(tk, 2), tk, 1.0}; }

  static inline TimeVectorType dtau(double tk) noexcept { return {4.0 * std::pow(tk, 3), 3.0 * std::pow(tk, 2), 2.0 * tk, 1.0, 0.0}; }

  static inline TimeVectorType ddtau(double tk) noexcept { return {12.0 * std::pow(tk, 2), 6.0 * tk, 2.0, 0.0, 0.0}; }

  static constexpr TimeVectorType tauZero{{0.0, 0.0, 0.0, 0.0, 1.0}};
  static constexpr TimeVectorType dtauZero{{0.0, 0.0, 0.0, 1.0, 0.0}};
  static constexpr TimeVectorType ddtauZero{{0.0, 0.0, 2.0, 0.0, 0.0}};

  //! Map initial pos/vel/accel and final pos/vel to spline coefficients.
  static bool compute(const SplineOptions& opts, SplineCoefficients& coefficients) {
    Eigen::Matrix<double, numCoefficients, 1> b;
    b << opts.pos0_, opts.vel0_, opts.acc0_, opts.posT_, opts.velT_;

    Eigen::Matrix<double, numCoefficients, numCoefficients> A;
    A << Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(tauZero.data()),
        Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(dtauZero.data()),
        Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(ddtauZero.data()),
        Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((tau(opts.tf_)).data()),
        Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((dtau(opts.tf_)).data());

    Eigen::Map<Eigen::VectorXd>(coefficients.data(), numCoefficients, 1) = A.colPivHouseholderQr().solve(b);

    return true;
  }
};

// Specialization for fifth order (quintic) splines.
template <>
struct spline_rep<double, 5> {
  static constexpr unsigned int splineOrder = 5;
  static constexpr unsigned int numCoefficients = splineOrder + 1;

  using TimeVectorType = std::array<double, numCoefficients>;
  using SplineCoefficients = std::array<double, numCoefficients>;

  static inline TimeVectorType tau(double tk) noexcept {
    return {std::pow(tk, 5), std::pow(tk, 4), std::pow(tk, 3), std::pow(tk, 2), tk, 1.0};
  }

  static inline TimeVectorType dtau(double tk) noexcept {
    return {5.0 * std::pow(tk, 4), 4.0 * std::pow(tk, 3), 3.0 * std::pow(tk, 2), 2.0 * tk, 1.0, 0.0};
  }

  static inline TimeVectorType ddtau(double tk) noexcept {
    return {20.0 * std::pow(tk, 3), 12.0 * std::pow(tk, 2), 6.0 * tk, 2.0, 0.0, 0.0};
  }

  static constexpr TimeVectorType tauZero{{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  static constexpr TimeVectorType dtauZero{{0.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
  static constexpr TimeVectorType ddtauZero{{0.0, 0.0, 0.0, 2.0, 0.0, 0.0}};

  //! Map initial pos/vel/accel and final pos/vel/accel to spline coefficients.
  static inline bool compute(const SplineOptions& opts, SplineCoefficients& coefficients) {
    Eigen::Map<Eigen::VectorXd>(coefficients.data(), numCoefficients, 1) =
        (Eigen::Matrix<double, numCoefficients, numCoefficients>()
             << Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(tauZero.data()),
         Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(dtauZero.data()),
         Eigen::Map<const Eigen::Matrix<double, 1, numCoefficients>>(ddtauZero.data()),
         Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((tau(opts.tf_)).data()),
         Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((dtau(opts.tf_)).data()),
         Eigen::Map<Eigen::Matrix<double, 1, numCoefficients>>((ddtau(opts.tf_)).data()))
            .finished()
            .colPivHouseholderQr()
            .solve((Eigen::Matrix<double, numCoefficients, 1>() << opts.pos0_, opts.vel0_, opts.acc0_, opts.posT_, opts.velT_, opts.accT_)
                       .finished());

    return true;
  }
};

}  // namespace spline_traits

}  // namespace curves
