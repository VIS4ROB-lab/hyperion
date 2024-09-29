/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/ops/group_ops.h>
#include <sym/ops/lie_group_ops.h>
#include <sym/ops/storage_ops.h>

#include "hyperion/ceres/forward.hpp"

namespace hyperion::ceres {

template <typename TGroup>
auto GroupPlus(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) -> bool {
  const auto _x = sym::StorageOps<TGroup>::FromStorage(x);
  const auto _delta = sym::StorageOps<typename TGroup::TangentVec>::FromStorage(delta);
  const auto _x_plus_delta = sym::LieGroupOps<TGroup>::Retract(_x, _delta, kDefaultEpsilon);
  sym::StorageOps<TGroup>::ToStorage(_x_plus_delta, x_plus_delta);
  return true;
}

template <typename TGroup>
auto GroupMinus(const Scalar* y, const Scalar* x, Scalar* y_minus_x) -> bool {
  const auto _x = sym::StorageOps<TGroup>::FromStorage(x);
  const auto _y = sym::StorageOps<TGroup>::FromStorage(y);
  const auto _y_minus_x = sym::GroupOps<TGroup>::Between(_x, _y).ToTangent(kDefaultEpsilon);
  sym::StorageOps<typename TGroup::TangentVec>::ToStorage(_y_minus_x, y_minus_x);
  return true;
};

}  // namespace hyperion::ceres
