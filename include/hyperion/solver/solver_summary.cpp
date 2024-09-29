/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/solver/solver_summary.hpp"

#include <fmt/format.h>

namespace hyperion {

auto SolverSummary::briefReport() const -> std::string {
  std::string str;
  str += "Hyperion Solver Report: ";
  str += fmt::format("Iterations: {}, ", num_successful_iterations + num_unsuccessful_iterations);
  str += fmt::format("Initial cost: {:.3e}, ", initial_cost);
  str += fmt::format("Final cost: {:.3e}, ", final_cost);
  str += fmt::format("Termination: {}", TerminationTypeToString(termination_type));
  return str;
}

// clang-format off
auto SolverSummary::fullReport() const -> std::string {
  std::string str;
  str += "\nHyperion Solver Report:\n\n";
  // Parameter information.
  str += fmt::format("{:>45}", "Original") + fmt::format("{:>25}\n", "Reduced");
  str += "Parameter blocks    " + fmt::format("{:>25}", num_parameter_blocks) + fmt::format("{:>25}\n", num_parameter_blocks_reduced);
  str += "Parameters          " + fmt::format("{:>25}", num_parameters) + fmt::format("{:>25}\n", num_parameters_reduced);
  str += "Effective parameters" + fmt::format("{:>25}", num_effective_parameters) + fmt::format("{:>25}\n", num_effective_parameters_reduced);
  str += "Residual blocks     " + fmt::format("{:>25}", num_residual_blocks) + fmt::format("{:>25}\n", num_residual_blocks_reduced);
  str += "Residuals           " + fmt::format("{:>25}", num_residuals) + fmt::format("{:>25}\n\n", num_residuals_reduced);
  // Optimizer information.
  str += "Optimizer           " + fmt::format("{:>25}\n", OptimizerTypeToString(optimizer_type));
  str += "Strategy            " + fmt::format("{:>25}\n", StrategyTypeToString(strategy_type));
  str += "Update              " + fmt::format("{:>25}\n\n", UpdateTypeToString(update_type));
  // Solver information.
  str += fmt::format("{:>45}", "Given") + fmt::format("{:>25}\n", "Used");
  str += "Threads             " + fmt::format("{:>25}", num_threads_given) + fmt::format("{:>25}\n\n", num_threads_used);
  // Cost information.
  str += "Cost:\n";
  str += "Initial             " + fmt::format("{:>25.6e}\n", initial_cost);
  str += "Final               " + fmt::format("{:>25.6e}\n", final_cost);
  str += "Change              " + fmt::format("{:>25.6e}\n\n", initial_cost - final_cost);
  // Iteration information.
  str += "Iterations          " + fmt::format("{:>25}\n", num_successful_iterations + num_unsuccessful_iterations);
  str += "Successful Iter.    " + fmt::format("{:>25}\n", num_successful_iterations);
  str += "Unsuccessful Iter.  " + fmt::format("{:>25}\n\n", num_unsuccessful_iterations);
  // Timing information.
  str += "Time (in seconds):\n";
  str += "Preprocessor        " + fmt::format("{:>25.6e}\n", preprocessor_time_in_seconds);
  str += "Optimizer           " + fmt::format("{:>25.6e}\n", optimizer_time_in_seconds);
  str += "  Max. Iter.        " + fmt::format("{:>25.6e}\n", max_iteration_time_in_seconds);
  str += "  Mean Iter.        " + fmt::format("{:>25.6e}\n", mean_iteration_time_in_seconds);
  str += "  Min. Iter.        " + fmt::format("{:>25.6e}\n", min_iteration_time_in_seconds);
  str += "Postprocessor       " + fmt::format("{:>25.6e}\n", postprocessor_time_in_seconds);
  str += "Total               " + fmt::format("{:>25.6e}\n\n", total_time_in_seconds);
  // Convergence information.
  str += "Termination:        " + fmt::format("{:>25} ({})\n", TerminationTypeToString(termination_type), message);
  return str;
}

// clang-format on

}  // namespace hyperion
