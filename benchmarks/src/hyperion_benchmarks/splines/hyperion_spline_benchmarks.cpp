/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include <catch2/catch.hpp>

#include "hyperion/ceres/cost_functions/cost_functions.hpp"
#include "hyperion/splines/interpolator.hpp"
#include "hyperion/utils/random.hpp"

namespace hyperion::benchmarks {

template <typename TGroup, int TOrder>
struct GroupHelper;

template <>
struct GroupHelper<sym::Rot3d, 4> {
  using ValueCost = ceres::Spline3Rot3PriorCostFunction;
  using VelocityCost = ceres::Spline3Rot3VelocityLocalCostFunction;
  using AccelerationCost = ceres::Spline3Rot3AccelerationLocalCostFunction;
  using Interpolator = splines::Interpolator<splines::IntervalType::UNIFORM, splines::SplineType::BSPLINE, 4>;
};

template <>
struct GroupHelper<sym::Pose3d, 4> {
  using ValueCost = ceres::Spline3Pose3PriorCostFunction;
  using VelocityCost = ceres::Spline3Pose3VelocityLocalCostFunction;
  using AccelerationCost = ceres::Spline3Pose3AccelerationLocalCostFunction;
  using Interpolator = splines::Interpolator<splines::IntervalType::UNIFORM, splines::SplineType::BSPLINE, 4>;
};

template <>
struct GroupHelper<sym::Rot3d, 5> {
  using ValueCost = ceres::Spline4Rot3PriorCostFunction;
  using VelocityCost = ceres::Spline4Rot3VelocityLocalCostFunction;
  using AccelerationCost = ceres::Spline4Rot3AccelerationLocalCostFunction;
  using Interpolator = splines::Interpolator<splines::IntervalType::UNIFORM, splines::SplineType::BSPLINE, 5>;
};

template <>
struct GroupHelper<sym::Pose3d, 5> {
  using ValueCost = ceres::Spline4Pose3PriorCostFunction;
  using VelocityCost = ceres::Spline4Pose3VelocityLocalCostFunction;
  using AccelerationCost = ceres::Spline4Pose3AccelerationLocalCostFunction;
  using Interpolator = splines::Interpolator<splines::IntervalType::UNIFORM, splines::SplineType::BSPLINE, 5>;
};

template <>
struct GroupHelper<sym::Rot3d, 6> {
  using ValueCost = ceres::Spline5Rot3PriorCostFunction;
  using VelocityCost = ceres::Spline5Rot3VelocityLocalCostFunction;
  using AccelerationCost = ceres::Spline5Rot3AccelerationLocalCostFunction;
  using Interpolator = splines::Interpolator<splines::IntervalType::UNIFORM, splines::SplineType::BSPLINE, 6>;
};

template <>
struct GroupHelper<sym::Pose3d, 6> {
  using ValueCost = ceres::Spline5Pose3PriorCostFunction;
  using VelocityCost = ceres::Spline5Pose3VelocityLocalCostFunction;
  using AccelerationCost = ceres::Spline5Pose3AccelerationLocalCostFunction;
  using Interpolator = splines::Interpolator<splines::IntervalType::UNIFORM, splines::SplineType::BSPLINE, 6>;
};

template <typename TGroup, int TOrder>
struct SplineCostFunctionArgs {
  static constexpr auto kAmbientDim = sym::StorageOps<TGroup>::StorageDim();
  static constexpr auto kTangentDim = sym::LieGroupOps<TGroup>::TangentDim();

  // Definitions.
  using Scalar = typename TGroup::Scalar;
  using AmbientVector = Eigen::Vector<Scalar, kAmbientDim>;
  using TangentVector = Eigen::Vector<Scalar, kTangentDim>;

  using Residual = Eigen::Vector<Scalar, kTangentDim>;
  using Jacobian = Eigen::Matrix<Scalar, kTangentDim, kAmbientDim, Eigen::RowMajor>;
  using SqrtInfo = Eigen::Matrix<Scalar, kTangentDim, kTangentDim>;

  using Parameters = std::array<TGroup, TOrder>;
  using Jacobians = std::array<Jacobian, TOrder>;

  using ParameterPtrs = std::array<const Scalar*, TOrder>;
  using ResidualPtr = Scalar*;
  using JacobianPtrs = std::array<Scalar*, TOrder>;

  using ValueCost = typename GroupHelper<TGroup, TOrder>::ValueCost;
  using VelocityCost = typename GroupHelper<TGroup, TOrder>::VelocityCost;
  using AccelerationCost = typename GroupHelper<TGroup, TOrder>::AccelerationCost;
  using Interpolator = typename GroupHelper<TGroup, TOrder>::Interpolator;

  // Generates a random set of parameters.
  SplineCostFunctionArgs()
      : value_cost{Interpolator::Lambdas0(standardUniformReal<Scalar>()), TGroup::Random(Generator()), SqrtInfo::Identity()},
        velocity_cost{
            0.5 + standardUniformReal<Scalar>(),
            Interpolator::Lambdas1(standardUniformReal<Scalar>()),
            TangentVector::Random(),
            SqrtInfo::Identity()},
        acceleration_cost{
            0.5 + standardUniformReal<Scalar>(),
            Interpolator::Lambdas2(standardUniformReal<Scalar>()),
            TangentVector::Random(),
            SqrtInfo::Identity()} {
    residual_ptr = residual.data();
    for (auto i = 0; i < TOrder; ++i) {
      parameters[i] = TGroup::Random(Generator());
      parameter_ptrs[i] = parameters[i].Data().data();
      jacobian_ptrs[i] = jacobians[i].data();
    }
  }

  /// Evaluates the value residual.
  /// \return True on success.
  auto evaluateValue() -> bool { return value_cost.Evaluate(parameter_ptrs.data(), residual_ptr, nullptr); }

  /// Evaluates the value residual and its Jacobians.
  /// \return True on success.
  auto evaluateValueAndJacobians() -> bool {
    return value_cost.Evaluate(parameter_ptrs.data(), residual_ptr, jacobian_ptrs.data());
  }

  /// Evaluates the velocity residual.
  /// \return True on success.
  auto evaluateVelocity() -> bool { return velocity_cost.Evaluate(parameter_ptrs.data(), residual_ptr, nullptr); }

  /// Evaluates the velocity residual and its Jacobians.
  /// \return True on success.
  auto evaluateVelocityAndJacobians() -> bool {
    return velocity_cost.Evaluate(parameter_ptrs.data(), residual_ptr, jacobian_ptrs.data());
  }

  /// Evaluates the acceleration residual.
  /// \return True on success.
  auto evaluateAcceleration() -> bool {
    return acceleration_cost.Evaluate(parameter_ptrs.data(), residual_ptr, nullptr);
  }

  /// Evaluates the acceleration residual and its Jacobians.
  /// \return True on success.
  auto evaluateAccelerationAndJacobians() -> bool {
    return acceleration_cost.Evaluate(parameter_ptrs.data(), residual_ptr, jacobian_ptrs.data());
  }

 private:
  Parameters parameters;
  Residual residual;
  Jacobians jacobians;

  ValueCost value_cost;
  VelocityCost velocity_cost;
  AccelerationCost acceleration_cost;

  ParameterPtrs parameter_ptrs;
  ResidualPtr residual_ptr;
  JacobianPtrs jacobian_ptrs;
};

using Chronometer = Catch::Benchmark::Chronometer;
using SO3_4 = SplineCostFunctionArgs<sym::Rot3d, 4>;
using SO3_5 = SplineCostFunctionArgs<sym::Rot3d, 5>;
using SO3_6 = SplineCostFunctionArgs<sym::Rot3d, 6>;
using SE3_4 = SplineCostFunctionArgs<sym::Pose3d, 4>;
using SE3_5 = SplineCostFunctionArgs<sym::Pose3d, 5>;
using SE3_6 = SplineCostFunctionArgs<sym::Pose3d, 6>;

TEMPLATE_TEST_CASE("B-Spline Cost Functions (Hyperion)", "[benchmark]", SO3_4, SO3_5, SO3_6, SE3_4, SE3_5, SE3_6) {
  BENCHMARK_ADVANCED("Pose")(Chronometer chronometer) {
    std::vector<TestType> args(chronometer.runs());
    chronometer.measure([&args](int i) { return args[i].evaluateValue(); });
  };
  BENCHMARK_ADVANCED("PoseWithJacobians")(Chronometer chronometer) {
    std::vector<TestType> args(chronometer.runs());
    chronometer.measure([&args](int i) { return args[i].evaluateValueAndJacobians(); });
  };
  BENCHMARK_ADVANCED("Velocity")(Chronometer chronometer) {
    std::vector<TestType> args(chronometer.runs());
    chronometer.measure([&args](int i) { return args[i].evaluateVelocity(); });
  };
  BENCHMARK_ADVANCED("VelocityWithJacobians")(Chronometer chronometer) {
    std::vector<TestType> args(chronometer.runs());
    chronometer.measure([&args](int i) { return args[i].evaluateVelocityAndJacobians(); });
  };
  BENCHMARK_ADVANCED("Acceleration")(Chronometer chronometer) {
    std::vector<TestType> args(chronometer.runs());
    chronometer.measure([&args](int i) { return args[i].evaluateAcceleration(); });
  };
  BENCHMARK_ADVANCED("AccelerationWithJacobians")(Chronometer chronometer) {
    std::vector<TestType> args(chronometer.runs());
    chronometer.measure([&args](int i) { return args[i].evaluateAccelerationAndJacobians(); });
  };
}

}  // namespace hyperion::benchmarks
