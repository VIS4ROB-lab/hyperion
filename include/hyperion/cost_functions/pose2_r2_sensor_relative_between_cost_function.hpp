// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <sym/pose2.h>

#include "hyperion/cost_functions/sized_cost_function.hpp"

namespace hyperion {

class Pose2R2SensorRelativeBetweenCostFunction final
    : public SizedCostFunction<2, ConstexprGroupDims<4, 3>, ConstexprGroupDims<4, 3>,
                               ConstexprGroupDims<4, 3>, ConstexprGroupDims<4, 3>> {
 public:
  /// Constructor.
  explicit Pose2R2SensorRelativeBetweenCostFunction(const Eigen::Matrix<Scalar, 2, 1>& aTb,
                                                    const Eigen::Matrix<Scalar, 2, 2>& sqrtInfo)
      : a_T_b_{aTb}, sqrt_info_{sqrtInfo} {}

  /// aTb accessor.
  [[nodiscard]] auto aTb() const -> const Eigen::Matrix<Scalar, 2, 1>& {
    return a_T_b_;
  }

  /// aTb modifier.
  auto aTb() -> Eigen::Matrix<Scalar, 2, 1>& {
    return a_T_b_;
  }

  /// sqrtInfo accessor.
  [[nodiscard]] auto sqrtInfo() const -> const Eigen::Matrix<Scalar, 2, 2>& {
    return sqrt_info_;
  }

  /// sqrtInfo modifier.
  auto sqrtInfo() -> Eigen::Matrix<Scalar, 2, 2>& {
    return sqrt_info_;
  }

  /// See documentation of base class.
  /// @note Returns (tangent_dim x tangent_dim) Jacobians stored in column-major order.
  auto evaluate(Scalar const* const* parameters, Scalar* residuals, Scalar** jacobians) const
      -> bool override;

 private:
  Eigen::Matrix<Scalar, 2, 1> a_T_b_;
  Eigen::Matrix<Scalar, 2, 2> sqrt_info_;
};

}  // namespace hyperion
