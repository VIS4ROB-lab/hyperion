/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/factors/factor.hpp"

#include <glog/logging.h>

#include "hyperion/messages/messages.hpp"
#include "hyperion/nodes/node.hpp"
#include "hyperion/utils/inversion.hpp"

namespace hyperion {

Factor::Factor(const CostFunction* cost_function, const LossFunction* loss_function, const Nodes& nodes)
    : Vertex{VertexType::FACTOR},
      nodes_{nodes},
      is_constant_{false},
      cost_function_{cost_function},
      loss_function_{loss_function},
      evaluated_{false},
      residual_dim_{-1},
      ambient_dim_{-1},
      tangent_dim_{-1},
      energy_{-1} {
  CHECK_NOTNULL(cost_function);
  const auto num_nodes = nodes.size();
  CHECK_EQ(num_nodes, cost_function->parameterBlockDims().size());
  node_to_factor_messages_.reserve(num_nodes);
  residual_dim_ = cost_function->residualDim();
  for (std::size_t i = 0; const auto& node : nodes_) {
    CHECK_NOTNULL(node);
    CHECK_EQ(node->dims(), cost_function->parameterBlockDims()[i]);
    const auto [_, inserted] = node_to_factor_messages_.try_emplace(node, node->addFactor(this));
    DCHECK(inserted);
    ++i;
  }
}

auto Factor::nodes() const -> const Nodes& {
  return nodes_;
}

auto Factor::load(const Scalar& parameter_tolerance, const Scalar& step_size) -> Scalar {
  // Update the parameter and Gaussian blocks.
  is_constant_ = true;
  parameter_blocks_.clear();
  gaussian_blocks_.clear();
  ambient_dim_ = 0;
  tangent_dim_ = 0;

  const auto num_nodes = nodes().size();
  parameter_blocks_.resize(num_nodes, nullptr);
  jacobian_blocks_.resize(num_nodes, nullptr);
  gaussian_blocks_.reserve(num_nodes);

  for (Index index = 0, offset = 0; const auto& node : nodes()) {
    // Add parameter block.
    const auto& ref = node_to_factor_messages_.at(node)->constRef();
    const auto& [ambient_dim, tangent_dim] = node->dims();

    // Add non-constant block.
    parameter_blocks_[index] = ref.x0.data();
    if (const auto node_value_type = node->valueType(); node_value_type != ValueType::CONSTANT) {
      // Factors become non-constant if any connected node is non-constant.
      if (node_value_type == ValueType::GAUSSIAN) {
        is_constant_ = false;
      }

      // Add gaussian block.
      gaussian_blocks_.emplace_back(node, ref, index, offset, tangent_dim);
      tangent_dim_ += tangent_dim;
      offset += tangent_dim;
    }

    // Update index.
    ambient_dim_ += ambient_dim;
    ++index;
  }

  // Evaluate this factors.
  evaluate(parameter_tolerance, step_size);
  return energy_;
}

auto Factor::isConstant() const -> bool {
  return is_constant_;
}

auto Factor::numResiduals() const -> Dim {
  return residual_dim_;
}

auto Factor::numParameters() const -> Dim {
  return ambient_dim_;
}

auto Factor::numEffectiveParameters() const -> Dim {
  return tangent_dim_;
}

auto Factor::needsEvaluation() const -> bool {
  return !isConstant() && !evaluated_;
}

auto Factor::maybeEvaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> std::tuple<bool, Scalar> {
  if (const auto& [locked, lock] = tryLock(); locked && needsEvaluation()) {
    evaluate(parameter_tolerance, step_size);
    return {true, energy_};
  }
  return {false, energy_};
}

auto Factor::energy() const -> const Scalar& {
  return energy_;
}

auto Factor::maybeApplyLoss(Vector<>& residual, Matrix<>& jacobian) const -> std::tuple<bool, Scalar> {
  // See ceres-solver/internal/ceres/corrector.cc.
  const auto sq_norm = residual.squaredNorm();

  if (!loss_function_) {
    const auto energy = Scalar{0.5} * sq_norm;
    return {false, energy};
  }

  // Evaluate and apply loss.
  Scalar rho[3];
  loss_function_->evaluate(sq_norm, rho);
  const auto energy = Scalar{0.5} * rho[0];

  CHECK_GE(sq_norm, 0);
  const auto sqrt_rho1 = std::sqrt(rho[1]);

  Scalar residual_scaling, alpha_sq_norm;
  if ((sq_norm == Scalar{0}) || (rho[2] <= Scalar{0})) {
    residual_scaling = sqrt_rho1;
    alpha_sq_norm = Scalar{0};
  } else {
    CHECK_GT(rho[1], 0);
    const auto D = Scalar{1} + Scalar{2} * sq_norm * rho[2] / rho[1];
    const auto alpha = Scalar{1} - std::sqrt(D);
    residual_scaling = sqrt_rho1 / (Scalar{1} - alpha);
    alpha_sq_norm = alpha / sq_norm;
  }

  // Jacobian correction.
  if (alpha_sq_norm == Scalar{0}) {
    jacobian *= sqrt_rho1;
  } else {
    const auto num_cols = jacobian.cols();
    const auto num_rows = jacobian.rows();
    for (Index c = 0; c < num_cols; ++c) {
      Scalar r_transpose_j = 0;
      for (Index r = 0; r < num_rows; ++r) {
        r_transpose_j += jacobian(r, c) * residual[r];
      }

      for (Index r = 0; r < num_rows; ++r) {
        jacobian(r, c) = sqrt_rho1 * (jacobian(r, c) - alpha_sq_norm * residual[r] * r_transpose_j);
      }
    }
  }

  // Residual correction.
  residual *= residual_scaling;
  return {true, energy};
}

auto Factor::evaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> void {
  // Allocate memory.
  Vector residual{residual_dim_};
  Matrix jacobian{residual_dim_, tangent_dim_};
  std::ranges::fill(jacobian_blocks_, nullptr);
  for (const auto& [node, ref, index, offset, dim] : gaussian_blocks_) {
    jacobian_blocks_[index] = jacobian.col(offset).data();
  }

  // Evaluate residual, loss and reset max delta norm.
  cost_function_->evaluate(parameter_blocks_.data(), residual.data(), jacobian_blocks_.data());
  const auto& [applied, energy] = maybeApplyLoss(residual, jacobian);
  energy_ = energy;

  Vector eta = jacobian.transpose() * (-residual);
  Matrix lambda = jacobian.transpose() * jacobian;

  // Evaluate the messages.
  Vector<> eta_prime = eta;
  Matrix<> lambda_prime = lambda;
  for (const auto& [node, ref, index, offset, dim] : gaussian_blocks_) {
    eta_prime.segment(offset, dim) += ref.e0;
    lambda_prime.block(offset, offset, dim, dim) += ref.L0;
  }

  // Sanity check.
  DCHECK_EQ(eta.size(), eta_prime.size());
  DCHECK_EQ(lambda.rows(), lambda_prime.rows());
  DCHECK_EQ(lambda.cols(), lambda_prime.cols());

  // Allocate permutation.
  const auto S_t = eta.size();
  auto P_i = Eigen::PermutationMatrix<Eigen::Dynamic>{S_t};

  // Marginalization.
  for (const auto& [node, ref, index, offset, dim] : gaussian_blocks_) {
    if (!node->isConstant()) {
      // Apply permutation (i.e. permute s.t. current node
      // eta and lambda are moved to top left corner).
      P_i.setIdentity();
      for (Index k = 0; k < dim; ++k) {
        P_i.applyTranspositionOnTheRight(k, offset + k);
      }
      eta_prime.applyOnTheLeft(P_i);
      lambda_prime.applyOnTheLeft(P_i);
      lambda_prime.applyOnTheRight(P_i);

      // Schur complement and update.
      const auto s_o = S_t - dim;
      const auto n_o = eta_prime.tail(s_o);
      const auto L_oi = lambda_prime.bottomLeftCorner(s_o, dim);
      const auto L_oo = lambda_prime.bottomRightCorner(s_o, s_o);
      const auto [n_i, L_i] = solvePSDSMatrix<Eigen::Dynamic>(L_oo, n_o, L_oi);
      eta.segment(offset, dim).noalias() -= L_oi.transpose() * n_i;
      lambda.block(offset, offset, dim, dim).noalias() -= L_oi.transpose() * L_i;
    }
  }

  // Send factor-to-node messages.
  for (const auto& [node, ref, index, offset, dim] : gaussian_blocks_) {
    if (!node->isConstant()) {
      const auto eta_i = eta.segment(offset, dim);
      const auto lambda_i = lambda.block(offset, offset, dim, dim);
      node->trySendMessage(this, ref.x0, eta_i, lambda_i, parameter_tolerance, step_size);
    }
  }

  evaluated_ = true;
}

}  // namespace hyperion
