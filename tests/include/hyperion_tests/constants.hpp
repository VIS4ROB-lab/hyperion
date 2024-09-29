/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

namespace hyperion::tests {

template <typename TScalar>
struct NumTraits;

template <>
struct NumTraits<double> {
  static constexpr auto kDelta = 1e-8;
  static constexpr auto kStrictTol = 1e-8;
  static constexpr auto kNormalTol = 1e-5;
  static constexpr auto kWeakTol = 1e-3;
};

}  // namespace hyperion::tests
