/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/graph/graph.hpp"
#include "hyperion/solver/iteration_callback.hpp"
#include "hyperion/solver/solver_options.hpp"
#include "hyperion/solver/solver_summary.hpp"

namespace hyperion {

class Solver {
 public:
  /// Solves a graph.
  /// \param graph Graph.
  /// \param options Options.
  /// \param summary Summary.
  static auto Solve(const Graph& graph, const SolverOptions& options, SolverSummary& summary) -> void;

  /// Solves a graph.
  /// \param graph Graph.
  /// \param options Options.
  /// \return Summary.
  static auto Solve(const Graph& graph, const SolverOptions& options) -> SolverSummary;
};

}  // namespace hyperion
