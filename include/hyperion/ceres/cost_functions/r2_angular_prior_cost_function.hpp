// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>
#include <ceres/sized_cost_function.h>

namespace hyperion::ceres {

class R2AngularPriorCostFunction final : public ::ceres::SizedCostFunction<1, 2> {
 public:
  /// Constructor.
  explicit R2AngularPriorCostFunction(const Eigen::Matrix<double, 2, 1>& prior,
                                      const Eigen::Matrix<double, 1, 1>& sqrtInfo)
      : y_{prior}, sqrt_info_{sqrtInfo} {}

  /// prior accessor.
  [[nodiscard]] auto prior() const -> const Eigen::Matrix<double, 2, 1>& {
    return y_;
  }

  /// prior modifier.
  auto prior() -> Eigen::Matrix<double, 2, 1>& {
    return y_;
  }

  /// sqrtInfo accessor.
  [[nodiscard]] auto sqrtInfo() const -> const Eigen::Matrix<double, 1, 1>& {
    return sqrt_info_;
  }

  /// sqrtInfo modifier.
  auto sqrtInfo() -> Eigen::Matrix<double, 1, 1>& {
    return sqrt_info_;
  }

  /// See documentation of base class.
  /// @note Returns (tangent_dim x ambient_dim) Jacobians stored in row-major order.
  auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
      -> bool override;

 private:
  Eigen::Matrix<double, 2, 1> y_;
  Eigen::Matrix<double, 1, 1> sqrt_info_;
};

}  // namespace hyperion::ceres
