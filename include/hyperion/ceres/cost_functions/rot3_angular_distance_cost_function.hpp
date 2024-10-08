// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <ceres/sized_cost_function.h>

#include <sym/rot3.h>

namespace hyperion::ceres {

class Rot3AngularDistanceCostFunction final : public ::ceres::SizedCostFunction<1, 4, 4> {
 public:
  /// Constructor.
  explicit Rot3AngularDistanceCostFunction(const Eigen::Matrix<double, 1, 1>& xdy,
                                           const Eigen::Matrix<double, 1, 1>& sqrtInfo)
      : x_d_y_{xdy}, sqrt_info_{sqrtInfo} {}

  /// xdy accessor.
  [[nodiscard]] auto xdy() const -> const Eigen::Matrix<double, 1, 1>& {
    return x_d_y_;
  }

  /// xdy modifier.
  auto xdy() -> Eigen::Matrix<double, 1, 1>& {
    return x_d_y_;
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
  Eigen::Matrix<double, 1, 1> x_d_y_;
  Eigen::Matrix<double, 1, 1> sqrt_info_;
};

}  // namespace hyperion::ceres
