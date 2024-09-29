/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <absl/container/btree_map.h>
#include <absl/container/btree_set.h>
#include <glog/logging.h>

#include "hyperion/graph/forward.hpp"
#include "hyperion/nodes/nodes.hpp"
#include "hyperion/utils/memory.hpp"

namespace hyperion {

struct GraphOptions {
  Ownership cost_ownership = Ownership::TAKE_OWNERSHIP;
  Ownership loss_ownership = Ownership::TAKE_OWNERSHIP;
};

class Graph {
 public:
  // Definitions.
  using Options = GraphOptions;

  struct ValueWithCovariance {
    Scalar* value;
    Scalar* covariance;
  };

  using UniqueNode = Unique<Node>;
  using UniqueFactor = Unique<Factor>;
  using UniqueCost = Unique<CostFunction>;
  using UniqueLoss = Unique<LossFunction>;

  using Nodes = absl::btree_map<const Scalar*, std::tuple<ValueWithCovariance, UniqueNode>, std::less<>>;
  using Factors = absl::btree_set<UniqueFactor, UniquePtrCompare<Factor>>;

  using Costs = absl::btree_set<UniqueCost, UniquePtrCompare<CostFunction>>;
  using Losses = absl::btree_set<UniqueLoss, UniquePtrCompare<LossFunction>>;

  /// Value accessor from node iterator.
  /// @param node_itr Node iterator.
  /// @return Value.
  template <typename TIterator>
  static auto AccessValue(const TIterator& node_itr) -> const Scalar* {
    return node_itr->first;
  }

  /// Value with covariance
  /// accessor from node iterator.
  /// @param node_itr Node iterator.
  /// @return Value with covariance.
  template <typename TIterator>
  static auto AccessValueWithCovariance(const TIterator& node_itr) -> const ValueWithCovariance& {
    return std::get<ValueWithCovariance>(node_itr->second);
  }

  /// Node accessor from node iterator.
  /// @param node_itr Node iterator.
  /// @return Node.
  template <typename TIterator>
  static auto AccessNode(const TIterator& node_itr) -> const UniqueNode& {
    return std::get<UniqueNode>(node_itr->second);
  }

  /// Constructor.
  /// \param options Input options.
  explicit Graph(const Options& options = {});

  /// Options accessor.
  /// \return Options.
  [[nodiscard]] auto options() const -> const Options&;

  /// Nodes accessor.
  /// \return Nodes.
  [[nodiscard]] auto nodes() const -> const Nodes&;

  /// Factors accessor.
  /// \return Factors.
  [[nodiscard]] auto factors() const -> const Factors&;

  /// Retrieves the number of edges.
  /// \return Number of edges.
  [[nodiscard]] auto numEdges() const -> std::size_t;

  /// Checks whether this has a value.
  /// \param value Value.
  /// \return True if this graph has the value.
  auto hasValue(const Scalar* value) const -> bool;

  /// \brief Adds a value.
  /// \tparam TGroup Group.
  /// \param value_type Value type.
  /// \param value Value.
  /// \param covariance Covariance
  /// (in the tangent space).
  template <typename TGroup>
  auto addValue(const ValueType& value_type, Scalar* value, Scalar* covariance) -> void {
    const auto value_with_covariance = ValueWithCovariance{value, covariance};
    auto node = std::make_unique<SizedNode<TGroup>>(value_type);
    const auto& [node_itr, node_emplaced] = nodes_.try_emplace(value, value_with_covariance, std::move(node));
    CHECK(node_emplaced);
  }

  /// Sets the value type.
  /// \param value Value.
  /// \param value_type Value type.
  auto setValueType(const Scalar* value, const ValueType& value_type) -> void;

  /// Removes a value.
  /// \param value Value.
  auto removeValue(const Scalar* value) -> void;

  /// Adds a factor.
  /// \param cost Cost function.
  /// \param loss Loss function.
  /// \param values Values.
  /// \return Factor.
  auto addFactor(CostFunction* cost, LossFunction* loss, const std::vector<const Scalar*>& values) -> const Factor*;

  /// Removes a factor.
  /// \param factor Factor.
  auto removeFactor(const Factor* factor) -> void;

 private:
  /// Retrieves a node.
  /// \param value Value.
  /// \return Node.
  auto getNode(const Scalar* value) const -> const Node*;

  /// Retrieves a node.
  /// \param value Value.
  /// \return Node iterator.
  auto getNode(const Scalar* value) -> Node*;

  /// Removes a factor.
  /// \param factor Factor.
  /// \param skipped_node Skipped node (to avoid cyclic calls).
  auto removeFactor(const Factor* factor, const Node* skipped_node) -> void;

  Nodes nodes_;      ///< Nodes.
  Factors factors_;  ///< Factors.

  Costs costs_;      ///< Costs.
  Losses losses_;    ///< Losses.
  Options options_;  ///< Options.
};

}  // namespace hyperion
