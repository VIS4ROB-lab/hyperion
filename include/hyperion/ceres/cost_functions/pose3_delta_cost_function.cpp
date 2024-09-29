// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.cpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#include "pose3_delta_cost_function.hpp"

#include <sym/ops/storage_ops.h>
#include <sym/util/epsilon.h>

#include "sym/pose3_delta_factor.hpp"
#include "sym/pose3_delta_factor_with_jacobians01.hpp"

namespace hyperion::ceres {

using namespace sym_ceres;

auto Pose3DeltaCostFunction::Evaluate(double const* const* parameters, double* residuals,
                                      double** jacobians) const -> bool {
  // Create aliases.
  const auto _x = sym::StorageOps<sym::Pose3<double>>::FromStorage(parameters[0]);
  const auto _y = sym::StorageOps<sym::Pose3<double>>::FromStorage(parameters[1]);

  // Evaluation without Jacobians.
  if (!jacobians) {
    Eigen::Map<Eigen::Matrix<double, 6, 1>>{residuals} =
        Pose3DeltaFactor<double>(_x, _y, sqrt_info_, sym::kDefaultEpsilon<double>);
    return true;
  }

  // Evaluation with Jacobians.
  Eigen::Map<Eigen::Matrix<double, 6, 1>>{residuals} = Pose3DeltaFactorWithJacobians01<double>(
      _x, _y, sqrt_info_, sym::kDefaultEpsilon<double>, jacobians[0], jacobians[1]);
  return true;
}

}  // namespace hyperion::ceres