// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.cpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#include "pose2_r2_sensor_relative_between_cost_function.hpp"

#include <sym/ops/storage_ops.h>
#include <sym/util/epsilon.h>

#include "sym/pose2_r2_sensor_relative_between_factor.hpp"
#include "sym/pose2_r2_sensor_relative_between_factor_with_jacobians0134.hpp"

namespace hyperion::ceres {

using namespace sym_ceres;

auto Pose2R2SensorRelativeBetweenCostFunction::Evaluate(double const* const* parameters,
                                                        double* residuals, double** jacobians) const
    -> bool {
  // Create aliases.
  const auto _x = sym::StorageOps<sym::Pose2<double>>::FromStorage(parameters[0]);
  const auto _x_T_a = sym::StorageOps<sym::Pose2<double>>::FromStorage(parameters[1]);
  const auto _y = sym::StorageOps<sym::Pose2<double>>::FromStorage(parameters[2]);
  const auto _y_T_b = sym::StorageOps<sym::Pose2<double>>::FromStorage(parameters[3]);

  // Evaluation without Jacobians.
  if (!jacobians) {
    Eigen::Map<Eigen::Matrix<double, 2, 1>>{residuals} = Pose2R2SensorRelativeBetweenFactor<double>(
        _x, _x_T_a, a_T_b_, _y, _y_T_b, sqrt_info_, sym::kDefaultEpsilon<double>);
    return true;
  }

  // Evaluation with Jacobians.
  Eigen::Map<Eigen::Matrix<double, 2, 1>>{residuals} =
      Pose2R2SensorRelativeBetweenFactorWithJacobians0134<double>(
          _x, _x_T_a, a_T_b_, _y, _y_T_b, sqrt_info_, sym::kDefaultEpsilon<double>, jacobians[0],
          jacobians[1], jacobians[2], jacobians[3]);
  return true;
}

}  // namespace hyperion::ceres