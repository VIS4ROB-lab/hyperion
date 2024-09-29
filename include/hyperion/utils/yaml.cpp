/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/utils/yaml.hpp"

namespace YAML {

auto Read(const Node& node, const std::string& key) -> Node {
  const auto value = node[key];
  CHECK(value) << "'" << key << "' does not exist.";
  return value;
}

}  // namespace YAML
