// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <ceres/sized_cost_function.h>

#include <sym/pose3.h>
#include <sym/rot3.h>

namespace hyperion::ceres {

class Pose3Rot3SensorRelativeBetweenCostFunction final
    : public ::ceres::SizedCostFunction<3, 7, 7, 7, 7> {
 public:
  /// Constructor.
  explicit Pose3Rot3SensorRelativeBetweenCostFunction(const sym::Rot3<double>& aTb,
                                                      const Eigen::Matrix<double, 3, 3>& sqrtInfo)
      : a_T_b_{aTb}, sqrt_info_{sqrtInfo} {}

  /// aTb accessor.
  [[nodiscard]] auto aTb() const -> const sym::Rot3<double>& {
    return a_T_b_;
  }

  /// aTb modifier.
  auto aTb() -> sym::Rot3<double>& {
    return a_T_b_;
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
  sym::Rot3<double> a_T_b_;
  Eigen::Matrix<double, 3, 3> sqrt_info_;
};

}  // namespace hyperion::ceres