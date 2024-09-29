/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "forward.hpp"

namespace hyperion {

class LossFunction {
 public:
  /// \brief Destructor.
  virtual ~LossFunction();

  /// \brief Evaluates this.
  /// \param sq_norm Squared norm.
  /// \param out Output.
  virtual auto evaluate(double sq_norm, double out[3]) const -> void = 0;
};

}  // namespace hyperion
