/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <absl/container/flat_hash_map.h>
#include <glog/logging.h>

#include <memory>

#include "hyperion/cost_functions/cost_functions.hpp"
#include "hyperion/loss_functions/loss_functions.hpp"
#include "hyperion/graph/vertex.hpp"
#include "hyperion/groups/group_ops.hpp"
#include "hyperion/messages/linearization_message.hpp"
#include "hyperion/nodes/forward.hpp"

namespace hyperion {

class Factor final : public Vertex {
 public:
  // Definitions.
  using Nodes = std::vector<const Node*>;

  template <typename TGroup>
  using TangentVector = Vector<sym::LieGroupOps<TGroup>::TangentDim()>;
  template <typename TGroup>
  using TangentMatrix = Matrix<sym::LieGroupOps<TGroup>::TangentDim()>;

  /// Constructor.
  /// @param cost_function Cost function.
  /// @param loss_function Loss function.
  /// @param nodes Input nodes.
  explicit Factor(const CostFunction* cost_function, const LossFunction* loss_function, const Nodes& nodes);

  /// Nodes accessor.
  /// @return Nodes.
  [[nodiscard]] auto nodes() const -> const Nodes&;

  /// Loads this.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  /// @return Initial energy.
  auto load(const Scalar& parameter_tolerance, const Scalar& step_size) -> Scalar;

  /// Sends a node-to-factor message
  /// from a node to this factor.
  /// Note: Blocking, guaranteed.
  /// @tparam TGroup Group type.
  /// @param node Node.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  template <typename TGroup>
  auto sendMessage(
      const Node* node,
      const TGroup& x0,
      const TangentVector<TGroup>& e0,
      const TangentMatrix<TGroup>& L0,
      const Scalar& parameter_tolerance) const -> void {
    const auto lock = this->lock();
    receiveMessages(node, x0, e0, L0, parameter_tolerance);
  }

  /// Sends a node-to-factor message
  /// from a node to this factor.
  /// Note: Best-effort, not guaranteed.
  /// @tparam TGroup Group type.
  /// @param node Node.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  /// @return True on sucess.
  template <typename TGroup>
  auto trySendMessage(
      const Node* node,
      const TGroup& x0,
      const TangentVector<TGroup>& e0,
      const TangentMatrix<TGroup>& L0,
      const Scalar& parameter_tolerance) const -> bool {
    if (const auto& [locked, lock] = tryLock(); locked) {
      receiveMessages(node, x0, e0, L0, parameter_tolerance);
      return true;
    }
    return false;
  }

  /// Checks whether this factor is constant.
  /// @return True if this factor is constant.
  auto isConstant() const -> bool;

  /// Number of residuals accessor.
  /// @return Number of residuals.
  auto numResiduals() const -> Dim;

  /// Number of parameters accessor.
  /// @return Number of parameters.
  auto numParameters() const -> Dim;

  /// Number of effective parameters accessor.
  /// @return Effective number of parameters.
  auto numEffectiveParameters() const -> Dim;

  /// Checks whether an evaluation is required.
  /// @return True if evaluation is required.
  [[nodiscard]] auto needsEvaluation() const -> bool;

  /// Maybe evaluates this.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  /// @return Evaluation flag and energy.
  auto maybeEvaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> std::tuple<bool, Scalar>;

  /// Energy accessor.
  /// @return Energy.
  [[nodiscard]] auto energy() const -> const Scalar&;

 private:
  // Definitions.
  struct GaussianBlock {  // NOLINT(*-pro-type-member-init)
    const Node* node;
    ConstLinearizationMessageRef ref;
    Index subtangent_index;
    Index subtangent_offset;
    Index subtangent_dim;
  };

  // Note: Values must be pointer stable!
  using NodeToFactorMessages = absl::flat_hash_map<const Node*, std::unique_ptr<LinearizationMessage>>;

  using GaussianBlocks = std::vector<GaussianBlock>;
  using ParameterBlocks = std::vector<const Scalar*>;
  using JacobianBlocks = std::vector<Scalar*>;

  /// Sized linearization message modifier.
  /// @tparam TGroup Group type.
  /// @param node Node.
  /// @return Sized linearization message.
  template <typename TGroup>
  auto sizedLinearizationMessageAt(const Node* node) const -> SizedLinearizationMessage<TGroup>& {
    DCHECK(node_to_factor_messages_.contains(node));
    return static_cast<SizedLinearizationMessage<TGroup>&>(*node_to_factor_messages_.at(node));
  }

  /// Receives a node-to-factor message
  /// from a node to this factor.
  /// @tparam TGroup Group type.
  /// @param node Node.
  /// @param x0 Linearization point.
  /// @param e0 Eta at linearization point.
  /// @param L0 Lambda at linearization point.
  /// @param parameter_tolerance Parameter tolerance.
  template <typename TGroup>
  auto receiveMessages(
      const Node* node,
      const TGroup& x0,
      const TangentVector<TGroup>& e0,
      const TangentMatrix<TGroup>& L0,
      const Scalar& parameter_tolerance) const -> void {
    DCHECK(node_to_factor_messages_.contains(node));
    auto& message = *node_to_factor_messages_.at(node);
    auto& sized_message = static_cast<SizedLinearizationMessage<TGroup>&>(message);
    const auto& d0 = sym::LieGroupOps<TGroup>::LocalCoordinates(x0, sized_message.x0, kDefaultEpsilon);
    const auto& v0 = GroupOps<TGroup>::GetVector(x0);
    const auto d0_l2_sq = d0.squaredNorm();
    const auto v0_l2_sq = v0.squaredNorm();
    evaluated_ &= d0_l2_sq <= parameter_tolerance * (v0_l2_sq + parameter_tolerance);
    sized_message = SizedLinearizationMessage<TGroup>{x0, e0, L0};
  }

  /// Maybe applies a loss.
  /// @param residual Residual.
  /// @param jacobian Jacobian.
  /// @return Flag whether loss has been
  /// applied and corresponding energy.
  auto maybeApplyLoss(Vector<>& residual, Matrix<>& jacobian) const -> std::tuple<bool, Scalar>;

  /// Evaluates this.
  /// @param parameter_tolerance Parameter tolerance.
  /// @param step_size Step size.
  auto evaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> void;

  Nodes nodes_;  ///< Nodes.

  bool is_constant_;                   ///< Are all connected nodes constant?
  const CostFunction* cost_function_;  ///< Cost function.
  const LossFunction* loss_function_;  ///< Loss function.

  mutable NodeToFactorMessages node_to_factor_messages_;  ///< Node-to-factor messages.
  mutable bool evaluated_;                                ///< Evaluation flag.

  Dim residual_dim_, ambient_dim_, tangent_dim_;  ///< Residual, ambient and tangent dimensions.
  ParameterBlocks parameter_blocks_;              ///< Parameter blocks.
  JacobianBlocks jacobian_blocks_;                ///< Jacobian blocks.
  GaussianBlocks gaussian_blocks_;                ///< Gaussian blocks.
  Scalar energy_;                                 ///< Energy.
};

}  // namespace hyperion
