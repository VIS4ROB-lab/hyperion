// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <ceres/sized_cost_function.h>

#include <sym/pose2.h>

namespace hyperion::ceres {

class Pose2PriorCostFunction final : public ::ceres::SizedCostFunction<3, 4> {
 public:
  /// Constructor.
  explicit Pose2PriorCostFunction(const sym::Pose2<double>& prior,
                                  const Eigen::Matrix<double, 3, 3>& sqrtInfo)
      : y_{prior}, sqrt_info_{sqrtInfo} {}

  /// prior accessor.
  [[nodiscard]] auto prior() const -> const sym::Pose2<double>& {
    return y_;
  }

  /// prior modifier.
  auto prior() -> sym::Pose2<double>& {
    return y_;
  }

  /// sqrtInfo accessor.
  [[nodiscard]] auto sqrtInfo() const -> const Eigen::Matrix<double, 3, 3>& {
    return sqrt_info_;
  }

  /// sqrtInfo modifier.
  auto sqrtInfo() -> Eigen::Matrix<double, 3, 3>& {
    return sqrt_info_;
  }

  /// See documentation of base class.
  /// @note Returns (tangent_dim x ambient_dim) Jacobians stored in row-major order.
  auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
      -> bool override;

 private:
  sym::Pose2<double> y_;
  Eigen::Matrix<double, 3, 3> sqrt_info_;
};

}  // namespace hyperion::ceres
