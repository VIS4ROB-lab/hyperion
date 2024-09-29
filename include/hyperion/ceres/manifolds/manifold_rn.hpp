/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/ops/lie_group_ops.h>
#include <sym/ops/storage_ops.h>

#include "hyperion/ceres/forward.hpp"

namespace hyperion::ceres {

template <int Dim>
class ManifoldRn final : public ::ceres::Manifold {
 public:
  // Definitions.
  using Group = Vector<Dim>;
  using Tangent = Vector<Dim>;

  // Constants.
  static constexpr auto kAmbientSize = sym::StorageOps<Group>::StorageDim();
  static constexpr auto kTangentSize = sym::LieGroupOps<Tangent>::TangentDim();

  /// \brief Constructor.
  ManifoldRn() : euclidean_manifold_{} {}

  /// \brief Constructor.
  explicit ManifoldRn(const int size) : euclidean_manifold_{size} {}

  /// \brief See Ceres.
  [[nodiscard]] constexpr auto AmbientSize() const -> int override { return kAmbientSize; }

  /// \brief See Ceres.
  [[nodiscard]] constexpr auto TangentSize() const -> int override { return kTangentSize; }

  /// \brief See Ceres.
  auto Plus(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) const -> bool override {
    return euclidean_manifold_.Plus(x, delta, x_plus_delta);
  }

  /// \brief See Ceres.
  auto PlusJacobian(const Scalar* x, Scalar* jacobian) const -> bool override {
    return euclidean_manifold_.PlusJacobian(x, jacobian);
  }

  /// \brief See Ceres.
  auto RightMultiplyByPlusJacobian(
      const Scalar* x,
      const int num_rows,
      const Scalar* ambient_matrix,
      Scalar* tangent_matrix) const -> bool override {
    return euclidean_manifold_.RightMultiplyByPlusJacobian(x, num_rows, ambient_matrix, tangent_matrix);
  }

  /// \brief See Ceres.
  auto Minus(const Scalar* y, const Scalar* x, Scalar* y_minus_x) const -> bool override {
    return euclidean_manifold_.Minus(y, x, y_minus_x);
  }

  /// \brief See Ceres.
  auto MinusJacobian(const Scalar* x, Scalar* jacobian) const -> bool override {
    return euclidean_manifold_.MinusJacobian(x, jacobian);
  }

 private:
  ::ceres::EuclideanManifold<Dim> euclidean_manifold_;  ///< Internal (Euclidean) manifold.
};

}  // namespace hyperion::ceres
