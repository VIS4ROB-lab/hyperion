/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/ceres/manifolds/manifold_rot3.hpp"

#include "hyperion/ceres/groups/group_ops.hpp"
#include "hyperion/ceres/groups/sym/storage_d_tangent_rot3.h"
#include "hyperion/ceres/groups/sym/tangent_d_storage_rot3.h"

namespace hyperion::ceres {

using namespace sym;

auto ManifoldRot3::Plus(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) const -> bool {
  return GroupPlus<Group>(x, delta, x_plus_delta);
}

auto ManifoldRot3::PlusJacobian(const Scalar* x, Scalar* jacobian) const -> bool {
  const auto _x = StorageOps<Group>::FromStorage(x);
  MatrixMap<kAmbientSize, kTangentSize>{jacobian} = StorageDTangentRot3(_x);
  return true;
}

auto ManifoldRot3::Minus(const Scalar* y, const Scalar* x, Scalar* y_minus_x) const -> bool {
  return GroupMinus<Group>(y, x, y_minus_x);
}

auto ManifoldRot3::MinusJacobian(const Scalar* x, Scalar* jacobian) const -> bool {
  const auto _x = StorageOps<Group>::FromStorage(x);
  MatrixMap<kTangentSize, kAmbientSize>{jacobian} = TangentDStorageRot3(_x);
  return true;
}

}  // namespace hyperion::ceres
