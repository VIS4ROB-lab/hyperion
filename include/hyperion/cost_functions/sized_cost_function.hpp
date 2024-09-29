/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

// #include <ceres/internal/parameter_dims.h>

#include "hyperion/cost_functions/cost_function.hpp"

namespace hyperion {

template <int kResidualDim, typename... TConstexprGroupDims>
class SizedCostFunction : public CostFunction {
 public:
  static_assert(
      kResidualDim > 0 || kResidualDim == Eigen::Dynamic,
      "Cost functions must have at least one residual block.");

  /* static_assert(ceres::internal::StaticParameterDims<GroupTraits::kAmbientDim...>::kIsValid,
                "Invalid parameter block dimension detected. Each parameter "
                "block dimension must be bigger than zero."); */

  // Definitions.
  /* using AmbientDims = ceres::internal::StaticParameterDims<GroupTraits::kAmbientDim...>; */

  /// \brief Constructor.
  explicit SizedCostFunction() {
    setResidualDim(kResidualDim);
    mutableParameterBlockDims() =
        ParameterBlockDims{{TConstexprGroupDims::kAmbientDim, TConstexprGroupDims::kTangentDim}...};
  }
};

}  // namespace hyperion
