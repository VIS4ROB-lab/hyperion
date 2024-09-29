/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <absl/container/flat_hash_map.h>

#include <memory>
#include <ranges>

#include "hyperion/factors/forward.hpp"
#include "hyperion/graph/vertex.hpp"
#include "hyperion/messages/messages.hpp"

namespace hyperion {

class Node : public Vertex {
 public:
  // Definitions.
  template <typename TRange>
  using KeysView = std::ranges::keys_view<std::ranges::ref_view<TRange>>;
  template <typename TRange>
  using ValuesView = std::ranges::values_view<std::ranges::ref_view<TRange>>;

  // Note: Values must be pointer stable!
  using FactorToNodeMessages = absl::flat_hash_map<const Factor*, std::unique_ptr<Message>>;

  /// Value type accessor.
  /// @return Node value type.
  [[nodiscard]] auto valueType() const -> ValueType;

  /// Sets the value type.
  /// @param value_type Value type.
  auto setValueType(const ValueType& value_type) -> void;

  /// Dimensions accessor.
  /// @return Dimensions.
  [[nodiscard]] virtual auto dims() const -> const GroupDims& = 0;

  /// Number of parameters accessor.
  /// @return Number of parameters.
  [[nodiscard]] virtual auto numParameters() const -> Dim = 0;

  /// Number of effective parameters accessor.
  /// @return Effective number of parameters.
  [[nodiscard]] virtual auto numEffectiveParameters() const -> Dim = 0;

  /// Number of factors accessor.
  /// @return Number of factors.
  auto numFactors() const -> std::size_t;

  /// Factor view accessor.
  /// @note Do not use this to evaluate the number of factors.
  /// @return View of factors of this node.
  [[nodiscard]] auto factors() const -> KeysView<FactorToNodeMessages>;

  /// Adds a factor and allocates
  /// a linearization message.
  /// @param factor Factor.
  /// @return Linearization message.
  virtual auto addFactor(const Factor* factor) const -> std::unique_ptr<LinearizationMessage> = 0;

  /// Removes a factor.
  /// @param factor Factor.
  virtual auto removeFactor(const Factor* factor) const -> void = 0;

  /// Loads the node from memory.
  /// @param value Value.
  /// @param covariance Covariance.
  /// @param parameter_tolerance Parameter tolerance.
  virtual auto load(const Scalar* value, const Scalar* covariance, const Scalar& parameter_tolerance) -> void = 0;

  /// Stores the node to memory.
  /// @param value Value.
  /// @param covariance Covariance.
  virtual auto store(Scalar* value, Scalar* covariance) -> void = 0;

  /// Sends a factor-to-node message
  /// from a factor to this node.
  /// Note: Blocking, guaranteed.
  /// @param factor Factor.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  auto sendMessage(
      const Factor* factor,
      const ConstVectorRef<>& x0,
      const ConstVectorRef<>& e0,
      const ConstMatrixRef<>& L0,
      const Scalar& parameter_tolerance,
      const Scalar& step_size) const -> void;

  /// Sends a factor-to-node message
  /// from a factor to this node.
  /// Note: Best-effort, not guaranteed.
  /// @param factor Factor.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  /// @return True on sucess.
  auto trySendMessage(
      const Factor* factor,
      const ConstVectorRef<>& x0,
      const ConstVectorRef<>& e0,
      const ConstMatrixRef<>& L0,
      const Scalar& parameter_tolerance,
      const Scalar& step_size) const -> bool;

  /// Checks whether this node is constant.
  /// @return True if this node is constant.
  auto isConstant() const -> bool;

  /// Checks whether an evaluation is required.
  /// @return True if evaluation is required.
  [[nodiscard]] auto needsEvaluation() const -> bool;

  /// Maybe evaluates this.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  /// @return Evaluation flag.
  auto maybeEvaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> bool;

 protected:
  /// Constructor.
  /// @param value_type Value type.
  explicit Node(const ValueType& value_type);

  /// Receives a factor-to-node message
  /// from a factor to this node.
  /// @param factor Factor.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  virtual auto receiveMessage(
      const Factor* factor,
      const ConstVectorRef<>& x0,
      const ConstVectorRef<>& e0,
      const ConstMatrixRef<>& L0,
      const Scalar& parameter_tolerance,
      const Scalar& step_size) const -> void = 0;

  /// Evaluates this.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  virtual auto evaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> void = 0;

  mutable FactorToNodeMessages factor_to_node_messages_;  ///< Factor-to-node messages.
  mutable bool evaluated_;                                ///< Evaluation flag.

 private:
  ValueType value_type_;  ///< Value type.
};

}  // namespace hyperion
