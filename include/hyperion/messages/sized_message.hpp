/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/ops/lie_group_ops.h>

#include "hyperion/groups/groups.hpp"
#include "hyperion/messages/message.hpp"

namespace hyperion {

template <typename TGroup>
struct SizedMessage final : Message {
  // Definitions.
  using Value = TGroup;
  using Precision = Matrix<sym::LieGroupOps<TGroup>::TangentDim()>;

  /// Constructor.
  explicit SizedMessage(const Value& x = sym::GroupOps<Value>::Identity(), const Precision& P_x = Precision::Zero())
      : x{x}, P_x{P_x} {}

  Value x;        ///< Point (i.e. mean).
  Precision P_x;  ///< Precision at x.

 private:
  /// Const reference.
  /// \return Const reference.
  [[nodiscard]] auto constRef() const -> ConstMessageRef override {
    return {.x = GroupOps<TGroup>::GetVector(x), .P_x = P_x};
  }

  /// Reference.
  /// \return Reference.
  [[nodiscard]] auto ref() -> MessageRef override { return {.x = GroupOps<TGroup>::GetMutableVector(x), .P_x = P_x}; }
};

}  // namespace hyperion
