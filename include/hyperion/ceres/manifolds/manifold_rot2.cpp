/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/ceres/manifolds/manifold_rot2.hpp"

#include "hyperion/ceres/groups/group_ops.hpp"
#include "hyperion/ceres/groups/sym/storage_d_tangent_rot2.h"
#include "hyperion/ceres/groups/sym/tangent_d_storage_rot2.h"

namespace hyperion::ceres {

using namespace sym;

auto ManifoldRot2::Plus(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) const -> bool {
  return GroupPlus<Group>(x, delta, x_plus_delta);
}

auto ManifoldRot2::PlusJacobian(const Scalar* x, Scalar* jacobian) const -> bool {
  const auto _x = StorageOps<Group>::FromStorage(x);
  MatrixMap<kAmbientSize, kTangentSize>{jacobian} = StorageDTangentRot2(_x);
  return true;
}

auto ManifoldRot2::Minus(const Scalar* y, const Scalar* x, Scalar* y_minus_x) const -> bool {
  return GroupMinus<Group>(y, x, y_minus_x);
}

auto ManifoldRot2::MinusJacobian(const Scalar* x, Scalar* jacobian) const -> bool {
  const auto _x = StorageOps<Group>::FromStorage(x);
  MatrixMap<kTangentSize, kAmbientSize>{jacobian} = TangentDStorageRot2(_x);
  return true;
}

}  // namespace hyperion::ceres
