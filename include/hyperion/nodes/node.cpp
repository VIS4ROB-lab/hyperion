/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/nodes/node.hpp"

namespace hyperion {

auto Node::valueType() const -> ValueType {
  return value_type_;
}

auto Node::setValueType(const ValueType& value_type) -> void {
  value_type_ = value_type;
}

auto Node::numFactors() const -> std::size_t {
  return factor_to_node_messages_.size();
}

auto Node::factors() const -> KeysView<FactorToNodeMessages> {
  return factor_to_node_messages_ | std::views::keys;
}

auto Node::sendMessage(
    const Factor* factor,
    const ConstVectorRef<>& x0,
    const ConstVectorRef<>& e0,
    const ConstMatrixRef<>& L0,
    const Scalar& parameter_tolerance,
    const Scalar& step_size) const -> void {
  const auto lock = this->lock();
  receiveMessage(factor, x0, e0, L0, parameter_tolerance, step_size);
}

auto Node::trySendMessage(
    const Factor* factor,
    const ConstVectorRef<>& x0,
    const ConstVectorRef<>& e0,
    const ConstMatrixRef<>& L0,
    const Scalar& parameter_tolerance,
    const Scalar& step_size) const -> bool {
  if (const auto& [locked, lock] = tryLock(); locked) {
    receiveMessage(factor, x0, e0, L0, parameter_tolerance, step_size);
    return true;
  }
  return false;
}

auto Node::isConstant() const -> bool {
  return value_type_ != ValueType::GAUSSIAN;
}

auto Node::needsEvaluation() const -> bool {
  return !isConstant() && !evaluated_;
}

auto Node::maybeEvaluate(const Scalar& parameter_tolerance, const Scalar& step_size) -> bool {
  if (const auto& [locked, lock] = tryLock(); locked && needsEvaluation()) {
    evaluate(parameter_tolerance, step_size);
    return true;
  }
  return false;
}

Node::Node(const ValueType& value_type) : Vertex{VertexType::NODE}, evaluated_{false}, value_type_{value_type} {}

}  // namespace hyperion
