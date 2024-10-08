// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.cpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#include "pose2_r2_sensor_prior_cost_function.hpp"

#include <sym/ops/storage_ops.h>
#include <sym/util/epsilon.h>

#include "sym/pose2_r2_sensor_delta_factor.hpp"
#include "sym/pose2_r2_sensor_delta_factor_with_jacobians01.hpp"

namespace hyperion {

using namespace sym_hyperion;

auto Pose2R2SensorPriorCostFunction::evaluate(Scalar const* const* parameters, Scalar* residuals,
                                              Scalar** jacobians) const -> bool {
  // Create aliases.
  const auto _x = sym::StorageOps<sym::Pose2<Scalar>>::FromStorage(parameters[0]);
  const auto _x_T_y = sym::StorageOps<Eigen::Matrix<Scalar, 2, 1>>::FromStorage(parameters[1]);

  // Evaluation without Jacobians.
  if (!jacobians) {
    Eigen::Map<Eigen::Matrix<Scalar, 2, 1>>{residuals} =
        Pose2R2SensorDeltaFactor<Scalar>(_x, _x_T_y, y_, sqrt_info_, sym::kDefaultEpsilon<Scalar>);
    return true;
  }

  // Evaluation with Jacobians.
  Eigen::Map<Eigen::Matrix<Scalar, 2, 1>>{residuals} =
      Pose2R2SensorDeltaFactorWithJacobians01<Scalar>(
          _x, _x_T_y, y_, sqrt_info_, sym::kDefaultEpsilon<Scalar>, jacobians[0], jacobians[1]);
  return true;
}

}  // namespace hyperion
