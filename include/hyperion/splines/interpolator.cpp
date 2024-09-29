/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/splines/interpolator.hpp"

#include "hyperion/splines/sym/sym.hpp"

namespace hyperion::splines {

namespace {

template <bool flag = false>
auto assertUnreachable() -> void {
  static_assert(flag, "Unmatched template parameters.");
}

}  // namespace

template <SplineType TSplineType, int TOrder>
auto Interpolator<IntervalType::UNIFORM, TSplineType, TOrder>::Lambdas0(const Scalar& ut) -> Lambdas<0> {
  if constexpr (TSplineType == SplineType::BSPLINE) {
    if constexpr (TOrder == 4) {
      return sym::CumulativeUniformBSplineLambdas40<Scalar>(ut);
    } else if constexpr (TOrder == 5) {
      return sym::CumulativeUniformBSplineLambdas50<Scalar>(ut);
    } else if constexpr (TOrder == 6) {
      return sym::CumulativeUniformBSplineLambdas60<Scalar>(ut);
    } else {
      assertUnreachable();
    }
  } else if constexpr (TSplineType == SplineType::ZSPLINE) {
    if constexpr (TOrder == 4) {
      return sym::CumulativeUniformZSplineLambdas40<Scalar>(ut);
    } else if constexpr (TOrder == 6) {
      return sym::CumulativeUniformZSplineLambdas60<Scalar>(ut);
    } else {
      assertUnreachable();
    }
  } else {
    assertUnreachable();
  }
  return {};
}

template <SplineType TSplineType, int TOrder>
auto Interpolator<IntervalType::UNIFORM, TSplineType, TOrder>::Lambdas1(const Scalar& ut) -> Lambdas<1> {
  if constexpr (TSplineType == SplineType::BSPLINE) {
    if constexpr (TOrder == 4) {
      return sym::CumulativeUniformBSplineLambdas41<Scalar>(ut);
    } else if constexpr (TOrder == 5) {
      return sym::CumulativeUniformBSplineLambdas51<Scalar>(ut);
    } else if constexpr (TOrder == 6) {
      return sym::CumulativeUniformBSplineLambdas61<Scalar>(ut);
    } else {
      assertUnreachable();
    }
  } else if constexpr (TSplineType == SplineType::ZSPLINE) {
    if constexpr (TOrder == 4) {
      return sym::CumulativeUniformZSplineLambdas41<Scalar>(ut);
    } else if constexpr (TOrder == 6) {
      return sym::CumulativeUniformZSplineLambdas61<Scalar>(ut);
    } else {
      assertUnreachable();
    }
  } else {
    assertUnreachable();
  }
  return {};
}

template <SplineType TSplineType, int TOrder>
auto Interpolator<IntervalType::UNIFORM, TSplineType, TOrder>::Lambdas2(const Scalar& ut) -> Lambdas<2> {
  if constexpr (TSplineType == SplineType::BSPLINE) {
    if constexpr (TOrder == 4) {
      return sym::CumulativeUniformBSplineLambdas42<Scalar>(ut);
    } else if constexpr (TOrder == 5) {
      return sym::CumulativeUniformBSplineLambdas52<Scalar>(ut);
    } else if constexpr (TOrder == 6) {
      return sym::CumulativeUniformBSplineLambdas62<Scalar>(ut);
    } else {
      assertUnreachable();
    }
  } else if constexpr (TSplineType == SplineType::ZSPLINE) {
    if constexpr (TOrder == 4) {
      return sym::CumulativeUniformZSplineLambdas42<Scalar>(ut);
    } else if constexpr (TOrder == 6) {
      return sym::CumulativeUniformZSplineLambdas62<Scalar>(ut);
    } else {
      assertUnreachable();
    }
  } else {
    assertUnreachable();
  }
  return {};
}

template struct Interpolator<IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template struct Interpolator<IntervalType::UNIFORM, SplineType::BSPLINE, 5>;
template struct Interpolator<IntervalType::UNIFORM, SplineType::BSPLINE, 6>;
template struct Interpolator<IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template struct Interpolator<IntervalType::UNIFORM, SplineType::ZSPLINE, 6>;

}  // namespace hyperion::splines
