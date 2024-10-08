// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "hyperion/cost_functions/sized_cost_function.hpp"

namespace hyperion {

class R3DeltaCostFunction final
    : public SizedCostFunction<3, ConstexprGroupDims<3, 3>, ConstexprGroupDims<3, 3>> {
 public:
  /// Constructor.
  explicit R3DeltaCostFunction(const Eigen::Matrix<Scalar, 3, 3>& sqrtInfo)
      : sqrt_info_{sqrtInfo} {}

  /// sqrtInfo accessor.
  [[nodiscard]] auto sqrtInfo() const -> const Eigen::Matrix<Scalar, 3, 3>& {
    return sqrt_info_;
  }

  /// sqrtInfo modifier.
  auto sqrtInfo() -> Eigen::Matrix<Scalar, 3, 3>& {
    return sqrt_info_;
  }

  /// See documentation of base class.
  /// @note Returns (tangent_dim x tangent_dim) Jacobians stored in column-major order.
  auto evaluate(Scalar const* const* parameters, Scalar* residuals, Scalar** jacobians) const
      -> bool override;

 private:
  Eigen::Matrix<Scalar, 3, 3> sqrt_info_;
};

}  // namespace hyperion
