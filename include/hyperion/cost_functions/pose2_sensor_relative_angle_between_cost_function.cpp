// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.cpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#include "pose2_sensor_relative_angle_between_cost_function.hpp"

#include <sym/ops/storage_ops.h>
#include <sym/util/epsilon.h>

#include "sym/pose2_sensor_relative_angle_between_factor.hpp"
#include "sym/pose2_sensor_relative_angle_between_factor_with_jacobians0134.hpp"

namespace hyperion {

using namespace sym_hyperion;

auto Pose2SensorRelativeAngleBetweenCostFunction::evaluate(Scalar const* const* parameters,
                                                           Scalar* residuals,
                                                           Scalar** jacobians) const -> bool {
  // Create aliases.
  const auto _x = sym::StorageOps<sym::Pose2<Scalar>>::FromStorage(parameters[0]);
  const auto _x_T_a = sym::StorageOps<sym::Pose2<Scalar>>::FromStorage(parameters[1]);
  const auto _y = sym::StorageOps<sym::Pose2<Scalar>>::FromStorage(parameters[2]);
  const auto _y_T_b = sym::StorageOps<sym::Pose2<Scalar>>::FromStorage(parameters[3]);

  // Evaluation without Jacobians.
  if (!jacobians) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 1>>{residuals} =
        Pose2SensorRelativeAngleBetweenFactor<Scalar>(_x, _x_T_a, a_d_b_, _y, _y_T_b, sqrt_info_,
                                                      sym::kDefaultEpsilon<Scalar>);
    return true;
  }

  // Evaluation with Jacobians.
  Eigen::Map<Eigen::Matrix<Scalar, 1, 1>>{residuals} =
      Pose2SensorRelativeAngleBetweenFactorWithJacobians0134<Scalar>(
          _x, _x_T_a, a_d_b_, _y, _y_T_b, sqrt_info_, sym::kDefaultEpsilon<Scalar>, jacobians[0],
          jacobians[1], jacobians[2], jacobians[3]);
  return true;
}

}  // namespace hyperion