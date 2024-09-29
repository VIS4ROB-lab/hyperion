/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/solver/forward.hpp"
#include "hyperion/utils/enum.hpp"

namespace hyperion {

// See ceres/types.h for documentation on most types.
DEFINE_SERIALIZABLE_ENUM(LoggingType, (SILENT)(PER_MINIMIZER_ITERATION));
DEFINE_SERIALIZABLE_ENUM(OptimizerType, (GBP))
DEFINE_SERIALIZABLE_ENUM(StrategyType, (BIPARTITE))
DEFINE_SERIALIZABLE_ENUM(UpdateType, (FULL))
DEFINE_SERIALIZABLE_ENUM(TerminationType, (CONVERGENCE)(NO_CONVERGENCE)(FAILURE)(USER_SUCCESS)(USER_FAILURE))
DEFINE_SERIALIZABLE_ENUM(CallbackReturnType, (SOLVER_CONTINUE)(SOLVER_EXIT_FAILURE)(SOLVER_EXIT_SUCCESS));

}  // namespace hyperion
