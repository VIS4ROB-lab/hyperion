/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include <ceres/ceres.h>
#include <ceres_lie_spline.h>

#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include <hyperion/utils/random.hpp>
#include <iomanip>
#include <sophus/se3.hpp>

namespace hyperion::benchmarks {

using namespace basalt;

template <template <class, int = 0> class TGroup, int TOrder>
struct SplineCostFunctionArgs {
  using Group = TGroup<double>;
  static constexpr auto kAmbientDim = Group::num_parameters;
  static constexpr auto kTangentDim = Group::DoF;

  // Definitions.
  using Scalar = typename Group::Scalar;
  using AmbientVector = Eigen::Vector<Scalar, kAmbientDim>;
  using TangentVector = Eigen::Vector<Scalar, kTangentDim>;

  using Residual = Eigen::Vector<Scalar, kTangentDim>;
  using Jacobian = Eigen::Matrix<Scalar, kTangentDim, kAmbientDim, Eigen::RowMajor>;

  using Parameters = std::array<Group, TOrder>;
  using Jacobians = std::array<Jacobian, TOrder>;

  using ParameterPtrs = std::array<const Scalar*, TOrder>;
  using ResidualPtr = Scalar*;
  using JacobianPtrs = std::array<Scalar*, TOrder>;

  using ValueFunctor = LieGroupSplineValueCostFunctor<TOrder, TGroup>;
  using VelocityFunctor = LieGroupSplineVelocityCostFunctor<TOrder, TGroup, false>;
  using AccelerationFunctor = LieGroupSplineAccelerationCostFunctor<TOrder, TGroup, false>;

  using ValueCost = ceres::DynamicAutoDiffCostFunction<ValueFunctor>;
  using VelocityCost = ceres::DynamicAutoDiffCostFunction<VelocityFunctor>;
  using AccelerationCost = ceres::DynamicAutoDiffCostFunction<AccelerationFunctor>;

  // Generates a random set of parameters.
  SplineCostFunctionArgs()
      : value_cost{new ValueFunctor(Group::exp(TangentVector::Random()), standardUniformReal<Scalar>())},
        velocity_cost{new VelocityFunctor(
            TangentVector::Random(),
            standardUniformReal<Scalar>(),
            0.5 + standardUniformReal<Scalar>())},
        acceleration_cost{new AccelerationFunctor(
            TangentVector::Random(),
            standardUniformReal<Scalar>(),
            0.5 + standardUniformReal<Scalar>())} {
    residual_ptr = residual.data();
    for (auto i = 0; i < TOrder; ++i) {
      parameters[i] = Group::exp(TangentVector::Random());
      parameter_ptrs[i] = parameters[i].data();
      jacobian_ptrs[i] = jacobians[i].data();
    }
    {
      for (int i = 0; i < TOrder; i++) {
        value_cost.AddParameterBlock(Group::num_parameters);
      }
      value_cost.SetNumResiduals(Group::DoF);
    }
    {
      for (int i = 0; i < TOrder; i++) {
        velocity_cost.AddParameterBlock(Group::num_parameters);
      }
      velocity_cost.SetNumResiduals(Group::DoF);
    }
    {
      for (int i = 0; i < TOrder; i++) {
        acceleration_cost.AddParameterBlock(Group::num_parameters);
      }
      acceleration_cost.SetNumResiduals(Group::DoF);
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
using SO3_4 = SplineCostFunctionArgs<Sophus::SO3, 4>;
using SO3_5 = SplineCostFunctionArgs<Sophus::SO3, 5>;
using SO3_6 = SplineCostFunctionArgs<Sophus::SO3, 6>;
using SE3_4 = SplineCostFunctionArgs<Sophus::SE3, 4>;
using SE3_5 = SplineCostFunctionArgs<Sophus::SE3, 5>;
using SE3_6 = SplineCostFunctionArgs<Sophus::SE3, 6>;

TEMPLATE_TEST_CASE("B-Spline Cost Functions (Basalt)", "[benchmark]", SO3_4, SO3_5, SO3_6, SE3_4, SE3_5, SE3_6) {
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
