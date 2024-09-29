// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include "hyperion/cost_functions/sized_cost_function.hpp"

namespace hyperion {

class R3BetweenCostFunction final
    : public SizedCostFunction<3, ConstexprGroupDims<3, 3>, ConstexprGroupDims<3, 3>> {
 public:
  /// Constructor.
  explicit R3BetweenCostFunction(const Eigen::Matrix<Scalar, 3, 1>& xTy,
                                 const Eigen::Matrix<Scalar, 3, 3>& sqrtInfo)
      : x_T_y_{xTy}, sqrt_info_{sqrtInfo} {}

  /// xTy accessor.
  [[nodiscard]] auto xTy() const -> const Eigen::Matrix<Scalar, 3, 1>& {
    return x_T_y_;
  }

  /// xTy modifier.
  auto xTy() -> Eigen::Matrix<Scalar, 3, 1>& {
    return x_T_y_;
  }

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
  Eigen::Matrix<Scalar, 3, 1> x_T_y_;
  Eigen::Matrix<Scalar, 3, 3> sqrt_info_;
};

}  // namespace hyperion