/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <glog/logging.h>

#include <ranges>

#include "hyperion/nodes/sized_node.hpp"
#include "hyperion/utils/inversion.hpp"

namespace hyperion {

struct SizedNodeTraits {
  static constexpr auto kAllowPartialUpdate = false;  ///< Should be disabled for coherent node updates.
};

template <typename TGroup>
SizedNode<TGroup>::SizedNode(const ValueType& value_type) : Node{value_type},
                                                            v0_l2_sq{0} {}

template <typename TGroup>
auto SizedNode<TGroup>::dims() const -> const GroupDims& {
  return kDims;
}

template <typename TGroup>
auto SizedNode<TGroup>::numParameters() const -> Dim {
  return kAmbientDim;
}

template <typename TGroup>
auto SizedNode<TGroup>::numEffectiveParameters() const -> Dim {
  return kTangentDim;
}

template <typename TGroup>
auto SizedNode<TGroup>::initialState() const -> const State& {
  return initial_state_;
}

template <typename TGroup>
auto SizedNode<TGroup>::estimatedState() const -> const State& {
  return estimated_state_;
}

template <typename TGroup>
auto SizedNode<TGroup>::initialCovariance() const -> Covariance {
  return ToGlobalCovariance(initial_state_.x, initial_state_.P_x);
}

template <typename TGroup>
auto SizedNode<TGroup>::estimatedCovariance() const -> Covariance {
  return ToGlobalCovariance(estimated_state_.x, estimated_state_.P_x);
}

template <typename TGroup>
auto SizedNode<TGroup>::addFactor(const Factor* factor) const -> std::unique_ptr<LinearizationMessage> {
  const auto [_, emplaced] = factor_to_node_messages_.try_emplace(factor, std::make_unique<SizedMessage<TGroup>>());
  CHECK(emplaced) << "Failed to add factor. Factor already exists.";
  return std::make_unique<SizedLinearizationMessage<TGroup>>();
}

template <typename TGroup>
auto SizedNode<TGroup>::removeFactor(const Factor* factor) const -> void {
  const auto erased = factor_to_node_messages_.erase(factor);
  CHECK(erased) << "Failed to remove factor. Factor does not exist.";
}

template <typename TGroup>
auto SizedNode<TGroup>::load(const Scalar* value, const Scalar* covariance, const Scalar& parameter_tolerance) -> void {
  // Frame conversion from global to local frame.
  CHECK(value != nullptr && covariance != nullptr);
  const auto x0 = sym::StorageOps<TGroup>::FromStorage(value);
  const auto C_I = Eigen::Map<const Covariance>{covariance};
  CHECK(C_I.isApprox(C_I.transpose())) << "Covariance must be symmetric.";
  const auto x_I = sym::GroupOps<TGroup>::Identity();
  const auto P_I = invertPSDSMatrix<kTangentDim>(C_I);
  const auto [d_x0_I, P_x] = GroupOps<TGroup>::MeanAndPrecisionToDeltaAndPrecision(x0, x_I, P_I);

  // Initialize state.
  initial_state_.x = x0;
  initial_state_.P_x = P_x;
  estimated_state_ = initial_state_;
  v0_l2_sq = GroupOps<TGroup>::GetVector(x0).squaredNorm();

  // Send factor-to-node messages.
  for (auto& factor : factor_to_node_messages_ | std::views::keys) {
    factor->sendMessage(this, initial_state_.x, Tangent::Zero(), P_x, parameter_tolerance);
  }
}

template <typename TGroup>
auto SizedNode<TGroup>::store(Scalar* value, Scalar* covariance) -> void {
  CHECK(value != nullptr && covariance != nullptr);
  sym::StorageOps<TGroup>::ToStorage(estimated_state_.x, value);
  Eigen::Map<Covariance>{covariance} = estimatedCovariance();
}

template <typename TGroup>
auto SizedNode<TGroup>::ToGlobalCovariance(const Value& x, const Precision& P_x) -> Covariance {
  const auto x_I = sym::GroupOps<TGroup>::Identity();
  const auto [_, P_I] = GroupOps<TGroup>::MeanAndPrecisionToDeltaAndPrecision(x_I, x, P_x);
  return invertPSDSMatrix<kTangentDim>(P_I);
}

template <typename TGroup>
auto SizedNode<TGroup>::receiveMessage(
    const Factor* factor,
    const ConstVectorRef<>& x0,
    const ConstVectorRef<>& e0,
    const ConstMatrixRef<>& L0,
    const Scalar& parameter_tolerance,
    const Scalar& step_size) const -> void {
  DCHECK(factor_to_node_messages_.contains(factor));
  auto& message = *factor_to_node_messages_.at(factor);
  auto& sized_message = static_cast<SizedMessage<TGroup>&>(message);
  const auto v0 = GroupOps<TGroup>::FromVector(x0);
  std::tie(sized_message.x, sized_message.P_x) =
      GroupOps<TGroup>::EtaAndLambdaToMeanAndPrecision(v0, e0, L0, step_size);
  const auto delta = sym::LieGroupOps<TGroup>::LocalCoordinates(estimated_state_.x, sized_message.x, kDefaultEpsilon);
  evaluated_ &= delta.squaredNorm() <= parameter_tolerance * (v0_l2_sq + parameter_tolerance);
}

template <typename TGroup>
auto SizedNode<TGroup>::evaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> void {
  // No evaluation needed.
  if (factor_to_node_messages_.empty()) {
    return;
  }

  // Allocate memory.
  const auto num_factors = factor_to_node_messages_.size();
  Matrix<kTangentDim, Eigen::Dynamic> etas{kTangentDim, num_factors};
  Matrix<kTangentDim, Eigen::Dynamic> lambdas{kTangentDim, kTangentDim * num_factors};
  Vector<kTangentDim> eta_sum = Vector<kTangentDim>::Zero();
  Matrix<kTangentDim> lambda_sum = Matrix<kTangentDim>::Zero();

  // Convert factor-to-node messages.
  auto& [x0, P0] = estimated_state_;
  for (Index i = 0; const auto& message : factor_to_node_messages_ | std::views::values) {
    auto eta_i = etas.template block<kTangentDim, 1>(0, i);
    auto lambda_i = lambdas.template block<kTangentDim, kTangentDim>(0, i * kTangentDim);
    const auto& [x, P_x] = static_cast<const SizedMessage<TGroup>&>(*message);
    std::tie(eta_i, lambda_i) = GroupOps<TGroup>::MeanAndPrecisionToEtaAndLambda(x0, x, P_x);
    lambda_sum += lambda_i;
    eta_sum += eta_i;
    ++i;
  }

  // Update estimated state.
  std::tie(x0, P0) = GroupOps<TGroup>::EtaAndLambdaToMeanAndPrecision(x0, eta_sum, lambda_sum, step_size);
  v0_l2_sq = GroupOps<TGroup>::GetVector(x0).squaredNorm();

  // Re-converting the factor-to-node message
  // guarantees that the node-to-factor messages
  // leverage the most recent estimated state.
  if constexpr (!SizedNodeTraits::kAllowPartialUpdate) {
    eta_sum = Vector<kTangentDim>::Zero();
    lambda_sum = Matrix<kTangentDim>::Zero();

    // Convert factor-to-node messages.
    for (Index i = 0; const auto& message : factor_to_node_messages_ | std::views::values) {
      auto eta_i = etas.template block<kTangentDim, 1>(0, i);
      auto lambda_i = lambdas.template block<kTangentDim, kTangentDim>(0, i * kTangentDim);
      const auto& [x, P_x] = static_cast<const SizedMessage<TGroup>&>(*message);
      std::tie(eta_i, lambda_i) = GroupOps<TGroup>::MeanAndPrecisionToEtaAndLambda(x0, x, P_x);
      lambda_sum += lambda_i;
      eta_sum += eta_i;
      ++i;
    }
  }

  // Send node-to-factor messages.
  for (Index i = 0; auto& factor : factor_to_node_messages_ | std::views::keys) {
    if (!factor->isConstant()) {
      const Tangent eta_i = eta_sum - etas.col(i);
      const Lambda lambda_i = lambda_sum - lambdas.template block<kTangentDim, kTangentDim>(0, i * kTangentDim);
      factor->trySendMessage(this, x0, eta_i, lambda_i, parameter_tolerance);
    }
    ++i;
  }

  evaluated_ = true;
}

}  // namespace hyperion
