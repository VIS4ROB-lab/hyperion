/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/messages/forward.hpp"

namespace hyperion {

struct ConstMessageRef {
  ConstVectorRef<> x;    ///< Point (i.e. mean).
  ConstMatrixRef<> P_x;  ///< Precision at x.
};

struct MessageRef {
  VectorRef<> x;    ///< Point (i.e. mean).
  MatrixRef<> P_x;  ///< Precision at x.
};

struct Message {
  /// Virtual default destructor.
  virtual ~Message() = default;

  /// Const reference.
  /// \return Const reference.
  [[nodiscard]] virtual auto constRef() const -> ConstMessageRef = 0;

  /// Reference.
  /// \return Reference.
  [[nodiscard]] virtual auto ref() -> MessageRef = 0;
};

}  // namespace hyperion
