/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/graph/graph.hpp"

#include <ranges>

#include "hyperion/factors/factors.hpp"

namespace hyperion {

Graph::Graph(const Options& options) : options_{options} {}

auto Graph::options() const -> const Options& {
  return options_;
}

auto Graph::nodes() const -> const Nodes& {
  return nodes_;
}

auto Graph::factors() const -> const Factors& {
  return factors_;
}

auto Graph::numEdges() const -> std::size_t {
  std::size_t num_edges = 0;
  for (const auto& [_, node] : nodes_ | std::views::values) {
    num_edges += node->numFactors();
  }
  return num_edges;
}

auto Graph::hasValue(const Scalar* value) const -> bool {
  return nodes_.contains(value);
}

auto Graph::setValueType(const Scalar* value, const ValueType& value_type) -> void {
  getNode(value)->setValueType(value_type);
}

auto Graph::removeValue(const Scalar* value) -> void {
  const auto itr = nodes_.find(value);
  for (const auto node = AccessNode(itr).get(); const auto& factor : node->factors()) {
    removeFactor(factor, node);
  }
  nodes_.erase(itr);
}

auto Graph::addFactor(CostFunction* cost, LossFunction* loss, const std::vector<const Scalar*>& values)
    -> const Factor* {
  // Take ownership?
  CHECK(cost != nullptr);
  if (options_.cost_ownership == Ownership::TAKE_OWNERSHIP) {
    auto unique_cost = std::unique_ptr<CostFunction>(cost);
    costs_.emplace(std::move(unique_cost));
  }

  // Take ownership?
  if (options_.loss_ownership == Ownership::TAKE_OWNERSHIP) {
    if (loss != nullptr) {
      auto unique_loss = std::unique_ptr<LossFunction>(loss);
      losses_.emplace(std::move(unique_loss));
    }
  }

  // Create factor.
  std::vector<const Node*> nodes;
  nodes.reserve(values.size());
  for (const auto& value : values) {
    nodes.emplace_back(getNode(value));
  }

  auto factor = std::make_unique<Factor>(cost, loss, std::move(nodes));
  const auto& [factor_itr, factor_emplaced] = factors_.emplace(std::move(factor));
  return factor_itr->get();
}

auto Graph::removeFactor(const Factor* factor) -> void {
  removeFactor(factor, nullptr);
}

auto Graph::getNode(const Scalar* value) const -> const Node* {
  try {
    CHECK_NOTNULL(value);
    const auto& [_, node] = nodes_.at(value);
    return node.get();
  } catch (const std::out_of_range&) {
    LOG(FATAL) << "Invalid node access. Value does not exist.";
    return nullptr;
  }
}

auto Graph::getNode(const Scalar* value) -> Node* {
  return const_cast<Node*>(std::as_const(*this).getNode(value));
}

auto Graph::removeFactor(const Factor* factor, const Node* skipped_node) -> void {
  CHECK_NOTNULL(factor);
  const auto itr = factors_.find(factor);
  CHECK(itr != factors_.cend());
  for (const auto& node : factor->nodes()) {
    if (node != skipped_node) {
      node->removeFactor(factor);
    }
  }
  factors_.erase(itr);
}

}  // namespace hyperion
