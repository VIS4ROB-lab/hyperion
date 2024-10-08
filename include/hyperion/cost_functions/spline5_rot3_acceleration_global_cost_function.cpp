// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     COST_FUNCTION.cpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#include "spline5_rot3_acceleration_global_cost_function.hpp"

#include <sym/ops/storage_ops.h>
#include <sym/util/epsilon.h>

#include "sym/spline5_rot3_acceleration_global_factor.hpp"
#include "sym/spline5_rot3_acceleration_global_factor_with_jacobians234567.hpp"

namespace hyperion {

using namespace sym_hyperion;

auto Spline5Rot3AccelerationGlobalCostFunction::evaluate(Scalar const* const* parameters,
                                                         Scalar* residuals,
                                                         Scalar** jacobians) const -> bool {
  // Create aliases.
  const auto _x0 = sym::StorageOps<sym::Rot3<Scalar>>::FromStorage(parameters[0]);
  const auto _x1 = sym::StorageOps<sym::Rot3<Scalar>>::FromStorage(parameters[1]);
  const auto _x2 = sym::StorageOps<sym::Rot3<Scalar>>::FromStorage(parameters[2]);
  const auto _x3 = sym::StorageOps<sym::Rot3<Scalar>>::FromStorage(parameters[3]);
  const auto _x4 = sym::StorageOps<sym::Rot3<Scalar>>::FromStorage(parameters[4]);
  const auto _x5 = sym::StorageOps<sym::Rot3<Scalar>>::FromStorage(parameters[5]);

  // Evaluation without Jacobians.
  if (!jacobians) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>{residuals} =
        Spline5Rot3AccelerationGlobalFactor<Scalar>(dt_, lambdas_, _x0, _x1, _x2, _x3, _x4, _x5,
                                                    acceleration_, sqrt_info_,
                                                    sym::kDefaultEpsilon<Scalar>);
    return true;
  }

  // Evaluation with Jacobians.
  Eigen::Map<Eigen::Matrix<Scalar, 3, 1>>{residuals} =
      Spline5Rot3AccelerationGlobalFactorWithJacobians234567<Scalar>(
          dt_, lambdas_, _x0, _x1, _x2, _x3, _x4, _x5, acceleration_, sqrt_info_,
          sym::kDefaultEpsilon<Scalar>, jacobians[0], jacobians[1], jacobians[2], jacobians[3],
          jacobians[4], jacobians[5]);
  return true;
}

}  // namespace hyperion
