// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.cpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#include "spline3_rot2_velocity_global_cost_function.hpp"

#include <sym/ops/storage_ops.h>
#include <sym/util/epsilon.h>

#include "sym/spline3_rot2_velocity_global_factor.hpp"
#include "sym/spline3_rot2_velocity_global_factor_with_jacobians2345.hpp"

namespace hyperion::ceres {

using namespace sym_ceres;

auto Spline3Rot2VelocityGlobalCostFunction::Evaluate(double const* const* parameters,
                                                     double* residuals, double** jacobians) const
    -> bool {
  // Create aliases.
  const auto _x0 = sym::StorageOps<sym::Rot2<double>>::FromStorage(parameters[0]);
  const auto _x1 = sym::StorageOps<sym::Rot2<double>>::FromStorage(parameters[1]);
  const auto _x2 = sym::StorageOps<sym::Rot2<double>>::FromStorage(parameters[2]);
  const auto _x3 = sym::StorageOps<sym::Rot2<double>>::FromStorage(parameters[3]);

  // Evaluation without Jacobians.
  if (!jacobians) {
    Eigen::Map<Eigen::Matrix<double, 1, 1>>{residuals} = Spline3Rot2VelocityGlobalFactor<double>(
        dt_, lambdas_, _x0, _x1, _x2, _x3, velocity_, sqrt_info_, sym::kDefaultEpsilon<double>);
    return true;
  }

  // Evaluation with Jacobians.
  Eigen::Map<Eigen::Matrix<double, 1, 1>>{residuals} =
      Spline3Rot2VelocityGlobalFactorWithJacobians2345<double>(
          dt_, lambdas_, _x0, _x1, _x2, _x3, velocity_, sqrt_info_, sym::kDefaultEpsilon<double>,
          jacobians[0], jacobians[1], jacobians[2], jacobians[3]);
  return true;
}

}  // namespace hyperion::ceres