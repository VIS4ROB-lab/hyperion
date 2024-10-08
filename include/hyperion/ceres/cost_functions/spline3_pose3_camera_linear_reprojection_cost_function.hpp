// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <ceres/sized_cost_function.h>

#include <sym/linear_camera_cal.h>
#include <sym/pose3.h>

namespace hyperion::ceres {

class Spline3Pose3CameraLinearReprojectionCostFunction final
    : public ::ceres::SizedCostFunction<2, 7, 7, 7, 7, 7, 4, 3> {
 public:
  /// Constructor.
  explicit Spline3Pose3CameraLinearReprojectionCostFunction(
      const Eigen::Matrix<double, 3, 1>& lambdas, const Eigen::Matrix<double, 2, 1>& pixel,
      const Eigen::Matrix<double, 2, 2>& sqrtInfo)
      : lambdas_{lambdas}, pixel_{pixel}, sqrt_info_{sqrtInfo} {}

  /// lambdas accessor.
  [[nodiscard]] auto lambdas() const -> const Eigen::Matrix<double, 3, 1>& {
    return lambdas_;
  }

  /// lambdas modifier.
  auto lambdas() -> Eigen::Matrix<double, 3, 1>& {
    return lambdas_;
  }

  /// pixel accessor.
  [[nodiscard]] auto pixel() const -> const Eigen::Matrix<double, 2, 1>& {
    return pixel_;
  }

  /// pixel modifier.
  auto pixel() -> Eigen::Matrix<double, 2, 1>& {
    return pixel_;
  }

  /// sqrtInfo accessor.
  [[nodiscard]] auto sqrtInfo() const -> const Eigen::Matrix<double, 2, 2>& {
    return sqrt_info_;
  }

  /// sqrtInfo modifier.
  auto sqrtInfo() -> Eigen::Matrix<double, 2, 2>& {
    return sqrt_info_;
  }

  /// See documentation of base class.
  /// @note Returns (tangent_dim x ambient_dim) Jacobians stored in row-major order.
  auto Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
      -> bool override;

 private:
  Eigen::Matrix<double, 3, 1> lambdas_;
  Eigen::Matrix<double, 2, 1> pixel_;
  Eigen::Matrix<double, 2, 2> sqrt_info_;
};

}  // namespace hyperion::ceres
