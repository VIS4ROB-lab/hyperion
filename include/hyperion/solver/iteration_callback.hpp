/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/solver/types.hpp"

namespace hyperion {

struct IterationSummary {
  int iteration = 0;

  bool step_is_successful = false;

  Scalar cost = 0;
  Scalar cost_change = 0;
  Scalar relative_decrease = 0;
  Scalar iteration_time_in_seconds = 0;
  Scalar step_solver_time_in_seconds = 0;
  Scalar cumulative_time_in_seconds = 0;
};

class IterationCallback {
  /// Virtual destrcutor.
  virtual ~IterationCallback();

  /// Call operator.
  /// @param summary Iteration summary.
  /// @return Callback return type.
  virtual auto operator()(const IterationSummary& summary) -> CallbackReturnType = 0;
};

}  // namespace hyperion
