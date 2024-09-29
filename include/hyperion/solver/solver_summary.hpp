/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/forward.hpp"
#include "hyperion/solver/types.hpp"

namespace hyperion {

struct SolverSummary {
  /// Generates a brief solver report.
  /// @return Brief solver report.
  [[nodiscard]] auto briefReport() const -> std::string;

  /// Generates a full solver report.
  /// @return Full solver report.
  [[nodiscard]] auto fullReport() const -> std::string;

  // See ceres/solver.h for documentation.
  OptimizerType optimizer_type = OptimizerType::GBP;
  TerminationType termination_type = TerminationType::FAILURE;
  std::string message = "Solver was not called.";

  Scalar initial_cost = -1;
  Scalar final_cost = -1;
  Scalar fixed_cost = -1;
  std::vector<IterationSummary> iterations;
  int num_successful_iterations = 0;
  int num_unsuccessful_iterations = 0;
  Scalar preprocessor_time_in_seconds = -1;
  Scalar optimizer_time_in_seconds = -1;
  Scalar max_iteration_time_in_seconds = -1;
  Scalar mean_iteration_time_in_seconds = -1;
  Scalar min_iteration_time_in_seconds = -1;
  Scalar postprocessor_time_in_seconds = -1;
  Scalar total_time_in_seconds = -1;

  int num_parameter_blocks = -1;
  int num_parameters = -1;
  int num_effective_parameters = -1;
  int num_residual_blocks = -1;
  int num_residuals = -1;

  int num_parameter_blocks_reduced = -1;
  int num_parameters_reduced = -1;
  int num_effective_parameters_reduced = -1;
  int num_residual_blocks_reduced = -1;
  int num_residuals_reduced = -1;

  int num_threads_given = -1;
  int num_threads_used = -1;

  StrategyType strategy_type = StrategyType::BIPARTITE;
  UpdateType update_type = UpdateType::FULL;
};

}  // namespace hyperion
