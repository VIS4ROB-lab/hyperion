// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <ceres/sized_cost_function.h>

#include <sym/pose2.h>
#include <sym/rot2.h>

namespace hyperion::ceres {

class Pose2Rot2SensorRelativeBetweenCostFunction final
    : public ::ceres::SizedCostFunction<1, 4, 4, 4, 4> {
 public:
  /// Constructor.
  explicit Pose2Rot2SensorRelativeBetweenCostFunction(const sym::Rot2<double>& aTb,
                                                      const Eigen::Matrix<double, 1, 1>& sqrtInfo)
      : a_T_b_{aTb}, sqrt_info_{sqrtInfo} {}

  /// aTb accessor.
  [[nodiscard]] auto aTb() const -> const sym::Rot2<double>& {
    return a_T_b_;
  }

  /// aTb modifier.
  auto aTb() -> sym::Rot2<double>& {
    return a_T_b_;
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
  sym::Rot2<double> a_T_b_;
  Eigen::Matrix<double, 1, 1> sqrt_info_;
};

}  // namespace hyperion::ceres