/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#pragma once

namespace YAML {

/// Safely reads a value from a YAML node with checks.
/// \param node YAML node to read from.
/// \param key Key of the value to read.
/// \return Read value node.
auto Read(const Node& node, const std::string& key) -> Node;

/// Safely reads a value as type from a YAML node with checks.
/// \tparam TValue Type of value to be read.
/// \param node YAML node to read from.
/// \param key Key of the value to read.
/// \return Read value node.
template <typename TValue>
auto ReadAs(const Node& node, const std::string& key) -> TValue {
  const auto value_node = Read(node, key);
  return value_node.template as<TValue>();
}

}  // namespace YAML
