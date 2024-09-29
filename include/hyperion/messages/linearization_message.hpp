/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/messages/forward.hpp"

namespace hyperion {

struct ConstLinearizationMessageRef {
  ConstVectorRef<> x0;  ///< Linearization point.
  ConstVectorRef<> e0;  ///< Eta at linearization point.
  ConstMatrixRef<> L0;  ///< Lambda at linearization point.
};

struct LinearizationMessageRef {
  VectorRef<> x0;  ///< Linearization point.
  VectorRef<> e0;  ///< Eta at linearization point.
  MatrixRef<> L0;  ///< Lambda at linearization point.
};

struct LinearizationMessage {
  /// Virtual default destructor.
  virtual ~LinearizationMessage() = default;

  /// Const reference.
  /// \return Const reference.
  [[nodiscard]] virtual auto constRef() const -> ConstLinearizationMessageRef = 0;

  /// Reference.
  /// \return Reference.
  [[nodiscard]] virtual auto ref() -> LinearizationMessageRef = 0;
};

}  // namespace hyperion
