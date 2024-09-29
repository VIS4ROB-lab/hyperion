/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/ops/lie_group_ops.h>
#include <sym/ops/storage_ops.h>

#include "hyperion/factors/factor.hpp"
#include "hyperion/nodes/node.hpp"

namespace hyperion {

template <typename TGroup>
class SizedNode final : public Node {
 public:
  // Constants.
  static constexpr Dim kAmbientDim = sym::StorageOps<TGroup>::StorageDim();
  static constexpr Dim kTangentDim = sym::LieGroupOps<TGroup>::TangentDim();
  static constexpr GroupDims kDims = {.ambient = kAmbientDim, .tangent = kTangentDim};

  // Definitions.
  using Value = TGroup;
  using Tangent = Vector<kTangentDim>;
  using Lambda = Matrix<kTangentDim>;
  using Covariance = Matrix<kTangentDim>;
  using Precision = Matrix<kTangentDim>;
  using State = SizedMessage<TGroup>;

  /// Constructor.
  /// @return Value type.
  explicit SizedNode(const ValueType& value_type);

  /// Dimensions accessor.
  /// @return Dimensions.
  [[nodiscard]] auto dims() const -> const GroupDims& override;

  /// Number of parameters accessor.
  /// @return Number of parameters.
  [[nodiscard]] auto numParameters() const -> Dim override;

  /// Number of effective parameters accessor.
  /// @return Effective number of parameters.
  [[nodiscard]] auto numEffectiveParameters() const -> Dim override;

  /// Initial state accessor.
  /// @return Initial state.
  auto initialState() const -> const State&;

  /// Estimated state accessor.
  /// @return Estimated state.
  auto estimatedState() const -> const State&;

  /// Initial covariance accessor.
  /// @return Initial covariance.
  [[nodiscard]] auto initialCovariance() const -> Covariance;

  /// Estimated covariance accessor.
  /// @return Estimated covariance.
  [[nodiscard]] auto estimatedCovariance() const -> Covariance;

  /// Adds a factor and allocates
  /// a linearization message.
  /// @param factor Factor.
  /// @return Linearization message.
  auto addFactor(const Factor* factor) const -> std::unique_ptr<LinearizationMessage> override;

  /// Adds a factor.
  /// @param factor Factor.
  auto removeFactor(const Factor* factor) const -> void override;

  /// Loads the node from memory.
  /// @param value Value.
  /// @param covariance Covariance.
  /// @param parameter_tolerance Parameter tolerance.
  auto load(const Scalar* value, const Scalar* covariance, const Scalar& parameter_tolerance) -> void override;

  /// Stores the node to memory.
  /// @param value Value.
  /// @param covariance Covariance.
  auto store(Scalar* value, Scalar* covariance) -> void override;

 private:
  /// Converts a (local) precision at x to a (global) covariance at identity.
  /// @param x Lie group element definint the (local) frame.
  /// @param P_x Local precision at x.
  /// @return Global covariance at identity.
  static auto ToGlobalCovariance(const Value& x, const Precision& P_x) -> Covariance;

  /// Receives a factor-to-node message
  /// from a factor to this node.
  /// @param factor Factor.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  auto receiveMessage(
      const Factor* factor,
      const ConstVectorRef<>& x0,
      const ConstVectorRef<>& e0,
      const ConstMatrixRef<>& L0,
      const Scalar& parameter_tolerance,
      const Scalar& step_size) const -> void override;

  /// Evaluates this.
  /// @param step_size Step size.
  /// @param parameter_tolerance Parameter tolerance.
  auto evaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> void override;

  State initial_state_;    ///< Initial state.
  State estimated_state_;  ///< Estimated state.

  Scalar v0_l2_sq;  ///< Vector-based L2-norm of x0.
};

}  // namespace hyperion
