/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/solver/solver_options.hpp"

namespace hyperion {

auto SolverOptions::FromFile(const Path& path) -> SolverOptions {
  const auto node = YAML::LoadFile(path);
  return SolverOptions(node);
}

auto SolverOptions::FromNode(const YAML::Node& node) -> SolverOptions {
  const auto solver_options_node = YAML::Read(node, "solver_options");
  return SolverOptions{solver_options_node};
}

SolverOptions::SolverOptions(const YAML::Node& yaml_node)
    : num_threads{YAML::ReadAs<int>(yaml_node, "num_threads")},
      optimizer_type{OptimizerTypeFromString(YAML::ReadAs<std::string>(yaml_node, "optimizer_type"))},
      strategy_type{StrategyTypeFromString(YAML::ReadAs<std::string>(yaml_node, "strategy_type"))},
      update_type{UpdateTypeFromString(YAML::ReadAs<std::string>(yaml_node, "update_type"))},
      max_num_iterations{YAML::ReadAs<int>(yaml_node, "max_num_iterations")},
      parameter_tolerance{YAML::ReadAs<Scalar>(yaml_node, "parameter_tolerance")},
      function_tolerance{YAML::ReadAs<Scalar>(yaml_node, "function_tolerance")},
      node_step_size{YAML::ReadAs<Scalar>(yaml_node, "node_step_size")},
      factor_step_size{YAML::ReadAs<Scalar>(yaml_node, "factor_step_size")},
      minimizer_progress_to_stdout{YAML::ReadAs<bool>(yaml_node, "minimizer_progress_to_stdout")} {}

}  // namespace hyperion
