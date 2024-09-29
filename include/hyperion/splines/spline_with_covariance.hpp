/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/splines/spline.hpp"

namespace hyperion::splines {

template <typename TGroup, IntervalType TIntervalType, SplineType TSplineType, int TOrder, typename TClock>
class SplineWithCovariance final : public Spline<TGroup, TIntervalType, TSplineType, TOrder, TClock> {
 public:
  // Definitions.
  using Super = Spline<TGroup, TIntervalType, TSplineType, TOrder, TClock>;

  using Evaluator = typename Super::Evaluator;
  using Interpolator = typename Super::Interpolator;

  using Value = typename Super::Value;
  using Velocity = typename Super::Velocity;
  using Acceleration = typename Super::Acceleration;

  using Covariance = Matrix<sym::LieGroupOps<TGroup>::TangentDim()>;

  struct ValueWithCovariance {
    Value value;
    Covariance covariance;
  };

  struct ValueWithCovarianceRef {
    Value& value;
    Covariance& covariance;
  };

  using ValuesWithCovariancesArray = std::array<ValueWithCovariance, Interpolator::kNumInputs>;
  using ParameterWithCovarianceBlocksArray = std::array<std::pair<Scalar*, Scalar*>, Interpolator::kNumInputs>;

  using ControlCovariances = std::deque<Covariance>;

  /// Constructor.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  SplineWithCovariance(const Time& t0, const Duration& dt, int num_segments = 1);

  /// Creates an identity spline.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  /// @return Spline.
  static auto Identity(const Time& t0, const Duration& dt, int num_segments = 1) -> SplineWithCovariance;

  /// Creates a random spline.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  /// @return Spline.
  static auto Random(const Time& t0, const Duration& dt, int num_segments = 1) -> SplineWithCovariance;

  /// Control covariances accessor.
  /// @return Control covariances.
  [[nodiscard]] auto controlCovariances() const -> const ControlCovariances&;

  /// Control covariances modifier.
  /// @return Control covariances.
  [[nodiscard]] auto controlCovariances() -> ControlCovariances&;

  /// Emplaces a pose at the front.
  /// @param value Value.
  /// @param covariance Covariance.
  /// @return Value with covariance.
  auto emplace_front(const Value& value, const Covariance& covariance) -> ValueWithCovarianceRef;

  /// Emplaces a pose at the back.
  /// @param value Value.
  /// @param covariance Covariance.
  /// @return Value with covariance.
  auto emplace_back(const Value& value, const Covariance& covariance) -> ValueWithCovarianceRef;

  /// Removes the pose at the front.
  auto pop_front() -> void override;

  /// Removes the pose at the back.
  auto pop_back() -> void override;

  /// Retrieves the parameter and
  /// covariance blocks for a query time.
  /// @param time Query time.
  /// @return Parameter and covariance blocks.
  auto getParameterWithCovarianceBlocks(const Time& time) -> ParameterWithCovarianceBlocksArray;

 private:
  /// Constructor from functor.
  /// @tparam TValueFunctor Value functor type.
  /// @tparam TCovarianceFunctor Covariance functor type.
  /// @param t0 Zero time.
  /// @param dt Duration between control points.
  /// @param num_segments Number of segments.
  /// @param value_functor Value functor.
  /// @param covariance_functor Covariance functor.
  template <typename TValueFunctor, typename TCovarianceFunctor>
  SplineWithCovariance(
      const Time& t0,
      const Duration& dt,
      const int num_segments,
      TValueFunctor&& value_functor,
      TCovarianceFunctor&& covariance_functor)
      : Super{t0, dt, num_segments, value_functor} {
    for (Index i = -Interpolator::kNumLhsInputs; i < Interpolator::kNumRhsInputs + (num_segments - 1); ++i) {
      control_covariances_.emplace_back(covariance_functor());
    }
  }

  ControlCovariances control_covariances_;  ///< Control covariances.
};

extern template class SplineWithCovariance<R1, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class SplineWithCovariance<R2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class SplineWithCovariance<R3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class SplineWithCovariance<Rot2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class SplineWithCovariance<Rot3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class SplineWithCovariance<Pose2, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;
extern template class SplineWithCovariance<Pose3, IntervalType::UNIFORM, SplineType::BSPLINE, 4>;

extern template class SplineWithCovariance<R1, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class SplineWithCovariance<R2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class SplineWithCovariance<R3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class SplineWithCovariance<Rot2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class SplineWithCovariance<Rot3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class SplineWithCovariance<Pose2, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;
extern template class SplineWithCovariance<Pose3, IntervalType::UNIFORM, SplineType::ZSPLINE, 4>;

}  // namespace hyperion::splines
