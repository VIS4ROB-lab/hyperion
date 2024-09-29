/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/splines/spline_with_covariance.hpp"

#include <glog/logging.h>

#include "hyperion/groups/group_ops.hpp"
#include "hyperion/utils/random.hpp"

namespace hyperion::splines {

#define CLASS_CONSTRUCTOR                                                                                     \
  template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock> \
  SplineWithCovariance<TGroup, TIntervalType, TSplineType, TOrder, TClock>

#define CLASS_MEMBER_FUNCTION                                                                                 \
  template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock> \
  auto SplineWithCovariance<TGroup, TIntervalType, TSplineType, TOrder, TClock>

CLASS_CONSTRUCTOR::SplineWithCovariance(const Time& t0, const Duration& dt, const int num_segments)
    : SplineWithCovariance{t0, dt, num_segments, sym::GroupOps<Value>::Identity, []() -> Covariance {
                             return Covariance::Identity();
                           }} {}

CLASS_MEMBER_FUNCTION::Identity(const Time& t0, const Duration& dt, const int num_segments)->SplineWithCovariance {
  return {t0, dt, num_segments, sym::GroupOps<Value>::Identity, []() -> Covariance {
            return Covariance::Identity();
          }};
}

CLASS_MEMBER_FUNCTION::Random(const Time& t0, const Duration& dt, const int num_segments)->SplineWithCovariance {
  return {
      t0,
      dt,
      num_segments,
      []() -> Value { return sym::StorageOps<Value>::Random(Generator()); },
      []() -> Covariance {
        const auto matrix = Covariance::Random();
        return matrix.template selfadjointView<Eigen::Lower>();  // Covariances must be symmetric.
      }};
}

CLASS_MEMBER_FUNCTION::controlCovariances() const->const ControlCovariances& {
  return control_covariances_;
}

CLASS_MEMBER_FUNCTION::controlCovariances()->ControlCovariances& {
  return const_cast<ControlCovariances&>(std::as_const(*this).controlCovariances());
}

CLASS_MEMBER_FUNCTION::emplace_front(const Value& value, const Covariance& covariance)->ValueWithCovarianceRef {
  auto& _value = Super::emplace_front(value);
  auto& _covariance = control_covariances_.emplace_front(covariance);
  DCHECK_EQ(Super::controlPoints().size(), control_covariances_.size());
  return {_value, _covariance};
}

CLASS_MEMBER_FUNCTION::emplace_back(const Value& value, const Covariance& covariance)->ValueWithCovarianceRef {
  auto& _value = Super::emplace_back(value);
  auto& _covariance = control_covariances_.emplace_back(covariance);
  DCHECK_EQ(Super::controlPoints().size(), control_covariances_.size());
  return {_value, _covariance};
}

CLASS_MEMBER_FUNCTION::pop_front()->void {
  Super::pop_front();
  control_covariances_.pop_front();
  DCHECK_EQ(Super::controlPoints().size(), control_covariances_.size());
}

CLASS_MEMBER_FUNCTION::pop_back()->void {
  Super::pop_back();
  control_covariances_.pop_back();
  DCHECK_EQ(Super::controlPoints().size(), control_covariances_.size());
}

CLASS_MEMBER_FUNCTION::getParameterWithCovarianceBlocks(const Time& time)->ParameterWithCovarianceBlocksArray {
  ParameterWithCovarianceBlocksArray parameter_with_covariance_blocks_array;
  const auto& [ut, index_array] = Super::getNormalizedTimeAndIndexArray(time);
  for (Index i = 0; const auto& index : index_array) {
    auto& value = this->control_points_[index];
    auto& covariance = control_covariances_[index];
    parameter_with_covariance_blocks_array[i] = {GroupOps<Value>::GetMutableData(value), covariance.data()};
    ++i;
  }
  return parameter_with_covariance_blocks_array;
}

template class SplineWithCovariance<R1, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class SplineWithCovariance<R2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class SplineWithCovariance<R3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class SplineWithCovariance<Rot2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class SplineWithCovariance<Rot3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class SplineWithCovariance<Pose2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class SplineWithCovariance<Pose3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;

template class SplineWithCovariance<R1, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class SplineWithCovariance<R2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class SplineWithCovariance<R3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class SplineWithCovariance<Rot2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class SplineWithCovariance<Rot3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class SplineWithCovariance<Pose2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class SplineWithCovariance<Pose3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;

}  // namespace hyperion::splines
