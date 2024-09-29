/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/manifold.h>
#include <sym/ops/lie_group_ops.h>
#include <sym/ops/storage_ops.h>
#include <sym/rot3.h>

#include "hyperion/ceres/forward.hpp"

namespace hyperion::ceres {

class ManifoldRot3 final : public ::ceres::Manifold {
 public:
  // Definitions.
  using Group = sym::Rot3<Scalar>;
  using Tangent = Group::TangentVec;

  // Constants.
  static constexpr auto kAmbientSize = sym::StorageOps<Group>::StorageDim();
  static constexpr auto kTangentSize = sym::LieGroupOps<Group>::TangentDim();

  /// \brief See Ceres.
  [[nodiscard]] constexpr auto AmbientSize() const -> int override { return kAmbientSize; }

  /// \brief See Ceres.
  [[nodiscard]] constexpr auto TangentSize() const -> int override { return kTangentSize; }

  /// \brief See Ceres.
  auto Plus(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) const -> bool override;

  /// \brief See Ceres.
  auto PlusJacobian(const Scalar* x, Scalar* jacobian) const -> bool override;

  /// \brief See Ceres.
  auto Minus(const Scalar* y, const Scalar* x, Scalar* y_minus_x) const -> bool override;

  /// \brief See Ceres.
  auto MinusJacobian(const Scalar* x, Scalar* jacobian) const -> bool override;
};

}  // namespace hyperion::ceres
