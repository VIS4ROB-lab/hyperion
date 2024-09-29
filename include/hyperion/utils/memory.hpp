/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <memory>

namespace hyperion {

template <typename TClass>
struct UniquePtrCompare {
  using is_transparent = std::true_type;

  auto operator()(const std::unique_ptr<TClass>& lhs, const std::unique_ptr<TClass>& rhs) const -> bool {
    return lhs.get() < rhs.get();
  }

  auto operator()(const TClass* lhs, const std::unique_ptr<TClass>& rhs) const -> bool { return lhs < rhs.get(); }

  auto operator()(const std::unique_ptr<TClass>& lhs, const TClass* rhs) const -> bool { return lhs.get() < rhs; }
};

}  // namespace hyperion
