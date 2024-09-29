/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <filesystem>

#include "hyperion/solver/types.hpp"
#include "hyperion/utils/yaml.hpp"

#pragma once

namespace hyperion {

struct SolverOptions {
  // Definitions.
  using Path = std::filesystem::path;

  /// Loads the solver options from a file.
  /// \param path Path to file.
  /// \return Options.
  static auto FromFile(const Path& path) -> SolverOptions;

  /// Loads the solver options from a node.
  /// \param node Solver options node.
  /// \return Options.
  static auto FromNode(const YAML::Node& node) -> SolverOptions;

  // Default constructor.
  SolverOptions() = default;

  /// Constructor from YAML node.
  /// \param yaml_node YAML node.
  explicit SolverOptions(const YAML::Node& yaml_node);

  [[nodiscard]] auto areValid(std::string& /* message */) const -> bool { return true; }

  int num_threads = 4;  ///< Number of threads.

  OptimizerType optimizer_type = OptimizerType::GBP;     ///< Optimizer type.
  StrategyType strategy_type = StrategyType::BIPARTITE;  ///< Strategy type.
  UpdateType update_type = UpdateType::FULL;             ///< Update type.

  int max_num_iterations = 50;                 ///< Maximum number of iterations.
  Scalar max_optimizer_time_in_seconds = 1e9;  ///< Maximum optimizer time in seconds.
  Scalar parameter_tolerance = 0;              ///< Parameter tolerance.
  Scalar function_tolerance = 0;               ///< Function tolerance.

  Scalar node_step_size = 0.7;    ///< Node step size.
  Scalar factor_step_size = 0.7;  ///< Factor step size.

  LoggingType logging_type = LoggingType::PER_MINIMIZER_ITERATION;  ///< Logging type.
  bool minimizer_progress_to_stdout = true;                         ///< Print progress to STDOUT?
  bool update_state_every_iteration = false;                        ///< Update the state at every iteration?

  std::vector<IterationCallback*> callbacks;  ///< Callbacks.
};

}  // namespace hyperion
