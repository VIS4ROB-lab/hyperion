/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <deque>

#include "hyperion/splines/evaluator.hpp"
#include "hyperion/splines/interpolator.hpp"

namespace hyperion::splines {

template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock>
class Spline {
 public:
  // Definitions.
  using Evaluator = splines::Evaluator<TGroup, TOrder>;
  using Interpolator = splines::Interpolator<TIntervalType, TSplineType, TOrder>;

  using Value = typename Evaluator::Value;
  using Velocity = typename Evaluator::Velocity;
  using Acceleration = typename Evaluator::Acceleration;

  using ValuesArray = std::array<Value, Interpolator::kNumInputs>;
  using ParameterBlocksArray = std::array<Scalar*, Interpolator::kNumInputs>;
  using IndexArray = std::array<std::ptrdiff_t, Interpolator::kNumInputs>;

  using ControlPoints = std::deque<Value>;

  /// Constructor.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  Spline(const Time& t0, const Duration& dt, int num_segments = 1);

  /// Destructor.
  virtual ~Spline();

  /// Creates an identity spline.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  /// @return Spline.
  static auto Identity(const Time& t0, const Duration& dt, int num_segments = 1) -> Spline;

  /// Creates a random spline.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  /// @return Spline.
  static auto Random(const Time& t0, const Duration& dt, int num_segments = 1) -> Spline;

  /// Returns the number segments.
  /// @return Number of segments.
  [[nodiscard]] auto numSegments() const -> std::size_t;

  /// Control point accessor.
  /// @return Control points.
  [[nodiscard]] auto controlPoints() const -> const ControlPoints&;

  /// Control point modifier.
  /// @return Control points.
  [[nodiscard]] auto controlPoints() -> ControlPoints&;

  /// Duration between control points accessor.
  /// @return Duration between control points.
  [[nodiscard]] auto dt() const -> const Duration&;

  /// First valid time (inclusive) accessor.
  /// @return First valid time (inclusive).
  [[nodiscard]] auto t0() const -> const Time&;

  /// Last valid time (exclusive) accessor.
  /// @return Last valid time (exclusive).
  [[nodiscard]] auto tn() const -> const Time&;

  /// Retrieves the normalized
  /// time for a query time.
  /// @param time Query time.
  /// @return Normalized time.
  [[nodiscard]] auto getNormalizedTime(const Time& time) const -> Scalar;

  /// Retrieves the time associated with an index.
  /// @param index Query index.
  /// @return Time for index.
  [[nodiscard]] auto getTimeForIndex(Index index) const -> Time;

  /// Duration accessor.
  /// @return Duration.
  [[nodiscard]] auto duration() const -> Duration;

  /// Checks whether the spline contain a query time.
  /// @return True if query time is in the valid range of this spline.
  [[nodiscard]] auto contains(const Time& time) const -> bool;

  /// Samples a random time of this.
  /// @return Random time.
  [[nodiscard]] auto sample() const -> Time;

  /// Emplaces a control point at the front.
  /// @param value Value.
  /// @return Value.
  auto emplace_front(const Value& value) -> Value&;

  /// Emplaces a control point at the back.
  /// @param value Value.
  /// @return Value.
  auto emplace_back(const Value& value) -> Value&;

  /// Removes the control point at the front.
  virtual auto pop_front() -> void;

  /// Removes the control point at the back.
  virtual auto pop_back() -> void;

  /// Retrieves the parameter
  /// blocks for a query time.
  /// @param time Query time.
  /// @return Parameter blocks.
  auto getParameterBlocks(const Time& time) -> ParameterBlocksArray;

  /// Evaluates the spline value.
  /// @param time Query time.
  /// @return Value.
  [[nodiscard]] auto value(const Time& time) const -> Value;

  /// Evaluates the spline velocity.
  /// @param frame_type Query frame type.
  /// @param time Query time.
  /// @return Velocity.
  [[nodiscard]] auto velocity(const FrameType& frame_type, const Time& time) const -> Velocity;

  /// Evaluates the sensor velocity.
  /// @param frame_type Query frame type.
  /// @param time Query time.
  /// @param b_T_s Body to sensor transformation.
  /// @return Sensor velocity.
  [[nodiscard]] auto sensorVelocity(const FrameType& frame_type, const Time& time, const Value& b_T_s) const -> Velocity
    requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>;

  /// Evaluates the spline acceleration.
  /// @param frame_type Query frame type.
  /// @param time Query time.
  /// @return Velocity.
  [[nodiscard]] auto acceleration(const FrameType& frame_type, const Time& time) const -> Acceleration;

  /// Evaluates the sensor acceleration.
  /// @param frame_type Query frame type.
  /// @param time Query time.
  /// @param b_T_s Body to sensor transformation.
  /// @return Sensor velocity.
  [[nodiscard]] auto sensorAcceleration(const FrameType& frame_type, const Time& time, const Value& b_T_s) const
      -> Velocity
    requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>;

 protected:
  /// Constructor from functor.
  /// @tparam TFunctor Functor type.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  /// @param functor Functor.
  template <typename TFunctor>
  Spline(const Time& t0, const Duration& dt, const int num_segments, TFunctor&& functor)
      : t0_{t0},
        tn_{t0_ + num_segments * dt},
        dt_{dt} {
    for (Index i = -Interpolator::kNumLhsInputs; i < Interpolator::kNumRhsInputs + (num_segments - 1); ++i) {
      control_points_.emplace_back(functor());
    }
  }

  /// Retrieves the normalized
  /// time and index array for a query time.
  /// @param time Query time.
  /// @return Index array.
  [[nodiscard]] auto getNormalizedTimeAndIndexArray(const Time& time) const -> std::tuple<Scalar, IndexArray>;

  /// Retrieves the normalized time and the
  /// associated control points for a query time.
  /// @param time Query time.
  /// @param ut Normalized time.
  /// @param values_array Control points.
  auto getNormalizedTimeAndControlPoints(const Time& time, Scalar& ut, ValuesArray& values_array) const -> void;

  Time t0_;                       ///< First time in the valid range of the spline (inclusive).
  Time tn_;                       ///< Last time in the valid range of the spline (exclusive).
  Duration dt_;                   ///< Time increment between consecutive control points.
  ControlPoints control_points_;  ///< Control points.
};

extern template class Spline<R1, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class Spline<R2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class Spline<R3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class Spline<Rot2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class Spline<Rot3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class Spline<Pose2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class Spline<Pose3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;

extern template class Spline<R1, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class Spline<R2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class Spline<R3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class Spline<Rot2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class Spline<Rot3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class Spline<Pose2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class Spline<Pose3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;

}  // namespace hyperion::splines
