/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/ceres/manifolds/manifold_pose3.hpp"

#include "hyperion/ceres/groups/group_ops.hpp"
#include "hyperion/ceres/groups/sym/storage_d_tangent_pose3.h"
#include "hyperion/ceres/groups/sym/tangent_d_storage_pose3.h"

namespace hyperion::ceres {

using namespace sym;

auto ManifoldPose3::Plus(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) const -> bool {
  return GroupPlus<Group>(x, delta, x_plus_delta);
}

auto ManifoldPose3::PlusJacobian(const Scalar* x, Scalar* jacobian) const -> bool {
  const auto _x = StorageOps<Group>::FromStorage(x);
  MatrixMap<kAmbientSize, kTangentSize>{jacobian} = StorageDTangentPose3(_x);
  return true;
}

auto ManifoldPose3::Minus(const Scalar* y, const Scalar* x, Scalar* y_minus_x) const -> bool {
  return GroupMinus<Group>(y, x, y_minus_x);
}

auto ManifoldPose3::MinusJacobian(const Scalar* x, Scalar* jacobian) const -> bool {
  const auto _x = StorageOps<Group>::FromStorage(x);
  MatrixMap<kTangentSize, kAmbientSize>{jacobian} = TangentDStoragePose3(_x);
  return true;
}

}  // namespace hyperion::ceres
