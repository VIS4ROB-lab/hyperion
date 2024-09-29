/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "cost_function.hpp"

namespace hyperion {

CostFunction::CostFunction() : residual_dim_{0} {}

CostFunction::~CostFunction() = default;

auto CostFunction::parameterBlockDims() const -> const ParameterBlockDims& {
  return parameter_block_dims_;
}

auto CostFunction::residualDim() const -> ResidualDim {
  return residual_dim_;
}

auto CostFunction::mutableParameterBlockDims() -> ParameterBlockDims& {
  return parameter_block_dims_;
}

auto CostFunction::setResidualDim(const ResidualDim residual_dim) -> void {
  residual_dim_ = residual_dim;
}

}  // namespace hyperion
