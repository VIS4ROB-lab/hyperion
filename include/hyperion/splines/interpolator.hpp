/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/splines/forward.hpp"

namespace hyperion::splines {

template <SplineType TSplineType, int TOrder>
struct Interpolator<IntervalType::UNIFORM, TSplineType, TOrder> {
  // Constants.
  static constexpr auto kNumInputs = TOrder;
  static constexpr auto kNumLhsInputs = (TOrder + 1) / 2;
  static constexpr auto kNumRhsInputs = TOrder / 2;

  static constexpr auto kNumLhsPaddingInputs = 0;
  static constexpr auto kNumRhsPaddingInputs = 0;
  static constexpr auto kNumInputsWithPadding = kNumInputs + kNumLhsPaddingInputs + kNumRhsPaddingInputs;

  template <int TDerivative>
  using Lambdas = Matrix<TOrder - 1, TDerivative + 1>;

  static auto Lambdas0(const Scalar& ut) -> Lambdas<0>;  ///< Value lambdas (in compact, cumulative form).
  static auto Lambdas1(const Scalar& ut) -> Lambdas<1>;  ///< Velocity lambdas (in compact, cumulative form).
  static auto Lambdas2(const Scalar& ut) -> Lambdas<2>;  ///< Acceleration lambdas (in compact, cumulative form).
};

}  // namespace hyperion::splines
