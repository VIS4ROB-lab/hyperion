/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <vector>

#include "hyperion/groups/forward.hpp"

namespace hyperion {

class CostFunction {
 public:
  // Definitions.
  using ResidualDim = Dim;
  using ParameterBlockDim = GroupDims;
  using ParameterBlockDims = std::vector<ParameterBlockDim>;

  /// \brief Constructor.
  CostFunction();

  /// \brief Deleted copy constructor.
  CostFunction(const CostFunction&) = delete;

  /// \brief Deleted assignment operator.
  void operator=(const CostFunction&) = delete;

  /// \brief Virtual destructor.
  virtual ~CostFunction();

  /// \brief Evaluates this.
  /// \param parameters Parameters.
  /// \param residuals Residuals.
  /// \param jacobians Jacobian.
  /// \return True on success.
  virtual bool evaluate(Scalar const* const* parameters, Scalar* residuals, Scalar** jacobians) const = 0;

  /// \brief Parameter block dims accessor.
  /// \return Parameter block dims.
  [[nodiscard]] auto parameterBlockDims() const -> const ParameterBlockDims&;

  /// \brief Residual dim accessor.
  /// \return Residual dim.
  [[nodiscard]] auto residualDim() const -> ResidualDim;

 protected:
  /// \brief Parameter block dims modifier.
  /// \return Parameter block dims.
  auto mutableParameterBlockDims() -> ParameterBlockDims&;

  /// \brief Residual dim setter.
  /// \param residual_dim Residual dim.
  auto setResidualDim(ResidualDim residual_dim) -> void;

 private:
  ResidualDim residual_dim_;
  ParameterBlockDims parameter_block_dims_;
};

}  // namespace hyperion
