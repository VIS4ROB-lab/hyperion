/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/forward.hpp"
#include "hyperion/splines/types.hpp"
#include "hyperion/utils/clock.hpp"

namespace hyperion::splines {

template <IntervalType TIntervalType, SplineType TSplineType, int TOrder>
struct Interpolator;

template <int TOrder>
using UniformBSplineInterpolator = Interpolator<IntervalType::UNIFORM, SplineType::BSPLINE, TOrder>;

template <int TOrder>
using UniformZSplineInterpolator = Interpolator<IntervalType::UNIFORM, SplineType::ZSPLINE, TOrder>;

template <typename TGroup, int TOrder>
struct Evaluator;

template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock = Clock>
class Spline;

template <typename TGroup, int TOrder, typename TClock = Clock>
using UniformBSpline = Spline<TGroup, IntervalType::UNIFORM, SplineType::BSPLINE, TOrder, TClock>;

template <typename TGroup, int TOrder, typename TClock = Clock>
using UniformZSpline = Spline<TGroup, IntervalType::UNIFORM, SplineType::ZSPLINE, TOrder, TClock>;

template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock = Clock>
class SplineWithCovariance;

template <typename TGroup, int TOrder, typename TClock = Clock>
using UniformBSplineWithCovariance =
    SplineWithCovariance<TGroup, IntervalType::UNIFORM, SplineType::BSPLINE, TOrder, TClock>;

template <typename TGroup, int TOrder, typename TClock = Clock>
using UniformZSplineWithCovariance =
    SplineWithCovariance<TGroup, IntervalType::UNIFORM, SplineType::ZSPLINE, TOrder, TClock>;

}  // namespace hyperion::splines
