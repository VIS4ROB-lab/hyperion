/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <sym/ops/lie_group_ops.h>

#include "hyperion/groups/group_ops.hpp"
#include "hyperion/messages/linearization_message.hpp"

namespace hyperion {

template <typename TGroup>
struct SizedLinearizationMessage final : LinearizationMessage {
  // Definitions.
  using Value = TGroup;
  using Eta = Vector<sym::LieGroupOps<TGroup>::TangentDim()>;
  using Lambda = Matrix<sym::LieGroupOps<TGroup>::TangentDim()>;

  /// Constructor.
  explicit SizedLinearizationMessage(const Value& x0 = sym::GroupOps<Value>::Identity(), const Eta& e0 = Eta::Zero(),
                                     const Lambda& L0 = Lambda::Zero())
      : x0{x0}, e0{e0}, L0{L0} {}

  Value x0;   ///< Linearization point.
  Eta e0;     ///< Eta at linearization point.
  Lambda L0;  ///< Lambda at linearization point.

 private:
  /// Const reference.
  /// \return Const reference.
  [[nodiscard]] auto constRef() const -> ConstLinearizationMessageRef override {
    return {.x0 = GroupOps<TGroup>::GetVector(x0), .e0 = e0, .L0 = L0};
  }

  /// Reference.
  /// \return Reference.
  [[nodiscard]] auto ref() -> LinearizationMessageRef override {
    return {.x0 = GroupOps<TGroup>::GetMutableVector(x0), .e0 = e0, .L0 = L0};
  }
};

}  // namespace hyperion
