// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <sym/pose2.h>
#include <sym/rot2.h>

#include "hyperion/cost_functions/sized_cost_function.hpp"

namespace hyperion {

class Pose2Rot2SensorBetweenCostFunction final
    : public SizedCostFunction<1, ConstexprGroupDims<4, 3>, ConstexprGroupDims<4, 3>,
                               ConstexprGroupDims<4, 3>> {
 public:
  /// Constructor.
  explicit Pose2Rot2SensorBetweenCostFunction(const sym::Rot2<Scalar>& sTy,
                                              const Eigen::Matrix<Scalar, 1, 1>& sqrtInfo)
      : s_T_y_{sTy}, sqrt_info_{sqrtInfo} {}

  /// sTy accessor.
  [[nodiscard]] auto sTy() const -> const sym::Rot2<Scalar>& {
    return s_T_y_;
  }

  /// sTy modifier.
  auto sTy() -> sym::Rot2<Scalar>& {
    return s_T_y_;
  }

  /// sqrtInfo accessor.
  [[nodiscard]] auto sqrtInfo() const -> const Eigen::Matrix<Scalar, 1, 1>& {
    return sqrt_info_;
  }

  /// sqrtInfo modifier.
  auto sqrtInfo() -> Eigen::Matrix<Scalar, 1, 1>& {
    return sqrt_info_;
  }

  /// See documentation of base class.
  /// @note Returns (tangent_dim x tangent_dim) Jacobians stored in column-major order.
  auto evaluate(Scalar const* const* parameters, Scalar* residuals, Scalar** jacobians) const
      -> bool override;

 private:
  sym::Rot2<Scalar> s_T_y_;
  Eigen::Matrix<Scalar, 1, 1> sqrt_info_;
};

}  // namespace hyperion