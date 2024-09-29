/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/splines/spline.hpp"

#include <glog/logging.h>

#include "hyperion/groups/group_ops.hpp"
#include "hyperion/utils/random.hpp"

namespace hyperion::splines {

#define CLASS_CONSTRUCTOR                                                                                      \
  template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock> \
  Spline<TGroup, TIntervalType, TSplineType, TOrder, TClock>

#define CLASS_MEMBER_FUNCTION                                                                                  \
  template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock> \
  auto Spline<TGroup, TIntervalType, TSplineType, TOrder, TClock>

CLASS_CONSTRUCTOR::Spline(const Time& t0, const Duration& dt, const int num_segments)
    : Spline{t0, dt, num_segments, sym::GroupOps<Value>::Identity} {}

CLASS_CONSTRUCTOR::~Spline() = default;

CLASS_MEMBER_FUNCTION::Identity(const Time& t0, const Duration& dt, const int num_segments)->Spline {
  return {t0, dt, num_segments, sym::GroupOps<Value>::Identity};
}

CLASS_MEMBER_FUNCTION::Random(const Time& t0, const Duration& dt, const int num_segments)->Spline {
  return {t0, dt, num_segments, []() {
            return sym::StorageOps<Value>::Random(Generator());
          }};
}

CLASS_MEMBER_FUNCTION::numSegments() const->std::size_t {
  const auto num_segments = controlPoints().size() - Interpolator::kNumInputs + 1;
  DCHECK_LT(0, num_segments);
  return num_segments;
}

CLASS_MEMBER_FUNCTION::controlPoints() const->const ControlPoints& {
  return control_points_;
}

CLASS_MEMBER_FUNCTION::controlPoints()->ControlPoints& {
  return const_cast<ControlPoints&>(std::as_const(*this).controlPoints());
}

CLASS_MEMBER_FUNCTION::dt() const->const Duration& {
  return dt_;
}

CLASS_MEMBER_FUNCTION::t0() const->const Time& {
  return t0_;
}

CLASS_MEMBER_FUNCTION::tn() const->const Time& {
  return tn_;
}

CLASS_MEMBER_FUNCTION::getNormalizedTime(const Time& time) const->Scalar {
  CHECK(contains(time)) << "Invalid query time.";
  const auto idx = (time - t0_) / dt_;
  const auto Dt = std::chrono::time_point<Clock>(t0_ + idx * dt_);
  const auto dt = std::chrono::duration<Scalar>(time - Dt);
  const auto ut = dt / dt_;
  CHECK_LE(0.0, ut);
  CHECK_LT(ut, 1.0);
  CHECK_LE(0, idx);
  CHECK_LE(idx + Interpolator::kNumInputs, control_points_.size());
  return ut;
}

CLASS_MEMBER_FUNCTION::getTimeForIndex(const Index index) const->Time {
  return t0_ + (index - Interpolator::kNumLhsInputs + 1) * dt_;
}

CLASS_MEMBER_FUNCTION::duration() const->Duration {
  return tn_ - t0_;
}

CLASS_MEMBER_FUNCTION::contains(const Time& time) const->bool {
  return t0_ <= time && time < tn_;
}

CLASS_MEMBER_FUNCTION::sample() const->Time {
  return std::chrono::time_point_cast<Duration>(t0_ + standardUniformReal<Scalar>() * duration());
}

CLASS_MEMBER_FUNCTION::emplace_front(const Value& value)->Value& {
  auto& _value = control_points_.emplace_front(value);
  t0_ -= dt_;
  return _value;
}

CLASS_MEMBER_FUNCTION::emplace_back(const Value& value)->Value& {
  auto& _value = control_points_.emplace_back(value);
  tn_ += dt_;
  return _value;
}

CLASS_MEMBER_FUNCTION::pop_front()->void {
  control_points_.pop_front();
  t0_ += dt_;
}

CLASS_MEMBER_FUNCTION::pop_back()->void {
  control_points_.pop_back();
  tn_ -= dt_;
}

CLASS_MEMBER_FUNCTION::getParameterBlocks(const Time& time)->ParameterBlocksArray {
  ParameterBlocksArray parameter_blocks_array;
  const auto& [ut, index_array] = getNormalizedTimeAndIndexArray(time);
  for (Index i = 0; const auto& index : index_array) {
    auto& value = control_points_[index];
    parameter_blocks_array[i] = GroupOps<Value>::GetMutableData(value);
    ++i;
  }
  return parameter_blocks_array;
}

CLASS_MEMBER_FUNCTION::value(const Time& time) const->Value {
  Scalar ut;
  ValuesArray inputs;
  getNormalizedTimeAndControlPoints(time, ut, inputs);
  const auto lambdas0 = Interpolator::Lambdas0(ut);
  return Evaluator::GetValue(lambdas0, inputs);
}

CLASS_MEMBER_FUNCTION::velocity(const FrameType& frame_type, const Time& time) const->Velocity {
  Scalar ut;
  ValuesArray inputs;
  getNormalizedTimeAndControlPoints(time, ut, inputs);
  const auto dt = std::chrono::duration<Scalar>(dt_).count();
  const auto lambdas1 = Interpolator::Lambdas1(ut);
  return Evaluator::GetVelocity(frame_type, dt, lambdas1, inputs);
}

CLASS_MEMBER_FUNCTION::sensorVelocity(const FrameType& frame_type, const Time& time, const Value& b_T_s) const->Velocity
  requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>
{
  Scalar ut;
  ValuesArray inputs;
  getNormalizedTimeAndControlPoints(time, ut, inputs);
  const auto dt = std::chrono::duration<Scalar>(dt_).count();
  const auto lambdas1 = Interpolator::Lambdas1(ut);
  return Evaluator::GetSensorVelocity(frame_type, dt, lambdas1, inputs, b_T_s);
}

CLASS_MEMBER_FUNCTION::acceleration(const FrameType& frame_type, const Time& time) const->Acceleration {
  Scalar ut;
  ValuesArray inputs;
  getNormalizedTimeAndControlPoints(time, ut, inputs);
  const auto dt = std::chrono::duration<Scalar>(dt_).count();
  const auto lambdas2 = Interpolator::Lambdas2(ut);
  return Evaluator::GetAcceleration(frame_type, dt, lambdas2, inputs);
}

CLASS_MEMBER_FUNCTION::sensorAcceleration(const FrameType& frame_type, const Time& time, const Value& b_T_s) const
    ->Velocity
  requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>
{
  Scalar ut;
  ValuesArray inputs;
  getNormalizedTimeAndControlPoints(time, ut, inputs);
  const auto dt = std::chrono::duration<Scalar>(dt_).count();
  const auto lambdas2 = Interpolator::Lambdas2(ut);
  return Evaluator::GetSensorAcceleration(frame_type, dt, lambdas2, inputs, b_T_s);
}

CLASS_MEMBER_FUNCTION::getNormalizedTimeAndIndexArray(const Time& time) const->std::tuple<Scalar, IndexArray> {
  CHECK(contains(time)) << "Invalid query time.";
  const auto idx = (time - t0_) / dt_;
  const auto Dt = std::chrono::time_point<Clock>(t0_ + idx * dt_);
  const auto dt = std::chrono::duration<Scalar>(time - Dt);
  const auto ut = dt / dt_;
  CHECK_LE(0.0, ut);
  CHECK_LT(ut, 1.0);
  CHECK_LE(0, idx);
  CHECK_LE(idx + Interpolator::kNumInputs, control_points_.size());
  IndexArray index_array;
  std::iota(index_array.begin(), index_array.end(), idx);
  return {ut, index_array};
}

CLASS_MEMBER_FUNCTION::getNormalizedTimeAndControlPoints(const Time& time, Scalar& ut, ValuesArray& values_array) const
    ->void {
  IndexArray index_array;
  std::tie(ut, index_array) = getNormalizedTimeAndIndexArray(time);
  for (Index i = 0; const auto& index : index_array) {
    values_array[i] = control_points_[index];
    ++i;
  }
}

template class Spline<R1, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class Spline<R2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class Spline<R3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class Spline<Rot2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class Spline<Rot3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class Spline<Pose2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
template class Spline<Pose3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;

template class Spline<R1, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class Spline<R2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class Spline<R3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class Spline<Rot2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class Spline<Rot3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class Spline<Pose2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
template class Spline<Pose3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;

}  // namespace hyperion::splines
