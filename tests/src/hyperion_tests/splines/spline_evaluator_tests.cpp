/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <catch2/catch.hpp>

#include "hyperion/splines/evaluator.hpp"
#include "hyperion/splines/interpolator.hpp"
#include "hyperion/utils/random.hpp"
#include "hyperion_tests/constants.hpp"

namespace hyperion::splines::tests {

template <typename TScalar>
using NumTraits = hyperion::tests::NumTraits<TScalar>;

template <typename TEvaluator, typename TInterpolator>
struct TestType {
  using Evaluator = TEvaluator;
  using Interpolator = TInterpolator;
};

using TestTypes = std::tuple<
    TestType<Evaluator<R3, 4>, UniformBSplineInterpolator<4>>,
    TestType<Evaluator<Rot2, 4>, UniformBSplineInterpolator<4>>,
    TestType<Evaluator<Rot3, 4>, UniformBSplineInterpolator<4>>,
    TestType<Evaluator<Pose2, 4>, UniformBSplineInterpolator<4>>,
    TestType<Evaluator<Pose3, 4>, UniformBSplineInterpolator<4>>,
    TestType<Evaluator<R3, 4>, UniformZSplineInterpolator<4>>,
    TestType<Evaluator<Rot2, 4>, UniformZSplineInterpolator<4>>,
    TestType<Evaluator<Rot3, 4>, UniformZSplineInterpolator<4>>,
    TestType<Evaluator<Pose2, 4>, UniformZSplineInterpolator<4>>,
    TestType<Evaluator<Pose3, 4>, UniformZSplineInterpolator<4>>,
    TestType<Evaluator<R3, 5>, UniformBSplineInterpolator<5>>,
    TestType<Evaluator<Rot2, 5>, UniformBSplineInterpolator<5>>,
    TestType<Evaluator<Rot3, 5>, UniformBSplineInterpolator<5>>,
    TestType<Evaluator<Pose2, 5>, UniformBSplineInterpolator<5>>,
    TestType<Evaluator<Pose3, 5>, UniformBSplineInterpolator<5>>,
    TestType<Evaluator<R3, 6>, UniformBSplineInterpolator<6>>,
    TestType<Evaluator<Rot2, 6>, UniformBSplineInterpolator<6>>,
    TestType<Evaluator<Rot3, 6>, UniformBSplineInterpolator<6>>,
    TestType<Evaluator<Pose2, 6>, UniformBSplineInterpolator<6>>,
    TestType<Evaluator<Pose3, 6>, UniformBSplineInterpolator<6>>,
    TestType<Evaluator<R3, 6>, UniformZSplineInterpolator<6>>,
    TestType<Evaluator<Rot2, 6>, UniformZSplineInterpolator<6>>,
    TestType<Evaluator<Rot3, 6>, UniformZSplineInterpolator<6>>,
    TestType<Evaluator<Pose2, 6>, UniformZSplineInterpolator<6>>,
    TestType<Evaluator<Pose3, 6>, UniformZSplineInterpolator<6>>>;

template <typename T>
constexpr auto isLieGroup() -> bool {
  return std::is_same_v<T, Rot2> || std::is_same_v<T, Rot3> || std::is_same_v<T, Pose2> || std::is_same_v<T, Pose3>;
}

template <typename T>
constexpr auto isSensorGroup() -> bool {
  return std::is_same_v<T, Pose2> || std::is_same_v<T, Pose3>;
}

TEMPLATE_LIST_TEST_CASE("Spline Evaluator: Spline Derivatives", "[spline]", TestTypes) {
  using Evaluator = typename TestType::Evaluator;
  using Interpolator = typename TestType::Interpolator;

  using Value = typename Evaluator::Value;
  using Velocity = typename Evaluator::Velocity;
  using Acceleration = typename Evaluator::Acceleration;
  using Inputs = typename Evaluator::Inputs;

  constexpr auto is_lie_group = isLieGroup<Value>();
  constexpr auto is_sensor_group = isSensorGroup<Value>();

  for (auto i = 0; i < 10; ++i) {
    // Setup.
    Inputs inputs;
    for (auto& arg : inputs) {
      if constexpr (is_lie_group) {
        arg = Value::Random(Generator());
      } else {
        arg = Value::Random();
      }
    }

    // Evaluate.
    const auto dt = uniformReal(0.5, 1.5);
    const auto ut_0 = standardUniformReal<Scalar>();
    const auto ut_p = ut_0 + 0.5 * NumTraits<Scalar>::kDelta;
    const auto ut_m = ut_0 - 0.5 * NumTraits<Scalar>::kDelta;

    //const auto L0_0 = Interpolator::Lambdas0(ut_0);
    const auto L0_p = Interpolator::Lambdas0(ut_p);
    const auto L0_m = Interpolator::Lambdas0(ut_m);

    //const auto value_0 = Evaluator::GetValue(L0_0, inputs);
    const auto value_p = Evaluator::GetValue(L0_p, inputs);
    const auto value_m = Evaluator::GetValue(L0_m, inputs);

    const auto L1_0 = Interpolator::Lambdas1(ut_0);
    const auto L1_p = Interpolator::Lambdas1(ut_p);
    const auto L1_m = Interpolator::Lambdas1(ut_m);

    const auto global_velocity_0 = Evaluator::GetVelocity(FrameType::GLOBAL, dt, L1_0, inputs);
    const auto global_velocity_p = Evaluator::GetVelocity(FrameType::GLOBAL, dt, L1_p, inputs);
    const auto global_velocity_m = Evaluator::GetVelocity(FrameType::GLOBAL, dt, L1_m, inputs);

    const auto local_velocity_0 = Evaluator::GetVelocity(FrameType::LOCAL, dt, L1_0, inputs);
    const auto local_velocity_p = Evaluator::GetVelocity(FrameType::LOCAL, dt, L1_p, inputs);
    const auto local_velocity_m = Evaluator::GetVelocity(FrameType::LOCAL, dt, L1_m, inputs);

    const auto L2_0 = Interpolator::Lambdas2(ut_0);
    //const auto L2_p = Interpolator::Lambdas2(ut_p);
    //const auto L2_m = Interpolator::Lambdas2(ut_m);

    const auto global_acceleration_0 = Evaluator::GetAcceleration(FrameType::GLOBAL, dt, L2_0, inputs);
    //const auto global_acceleration_p = Evaluator::GetAcceleration(FrameType::GLOBAL, dt, L2_p, inputs);
    //const auto global_acceleration_m = Evaluator::GetAcceleration(FrameType::GLOBAL, dt, L2_m, inputs);

    const auto local_acceleration_0 = Evaluator::GetAcceleration(FrameType::LOCAL, dt, L2_0, inputs);
    //const auto local_acceleration_p = Evaluator::GetAcceleration(FrameType::LOCAL, dt, L2_p, inputs);
    //const auto local_acceleration_m = Evaluator::GetAcceleration(FrameType::LOCAL, dt, L2_m, inputs);

    if constexpr (is_lie_group) {
      SECTION("Global Velocity (Iter: " + std::to_string(i) + ")") {  // Tends to be numerically unstable.
        const Velocity numeric_velocity =
            value_p.Compose(value_m.Inverse()).ToTangent() / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_velocity - global_velocity_0).isZero(NumTraits<Scalar>::kWeakTol));
      }
      SECTION("Local Velocity (Iter: " + std::to_string(i) + ")") {
        const Velocity numeric_velocity =
            value_m.Inverse().Compose(value_p).ToTangent() / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_velocity - local_velocity_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
      SECTION("Global Acceleration (Iter: " + std::to_string(i) + ")") {
        const Acceleration numeric_acceleration =
            (global_velocity_p - global_velocity_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_acceleration - global_acceleration_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
      SECTION("Local Acceleration (Iter: " + std::to_string(i) + ")") {
        const Acceleration numeric_acceleration =
            (local_velocity_p - local_velocity_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_acceleration - local_acceleration_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
    } else {
      SECTION("Global Velocity (Iter: " + std::to_string(i) + ")") {  // Tends to be numerically unstable.
        const Velocity numeric_velocity = (value_p - value_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_velocity - global_velocity_0).isZero(NumTraits<Scalar>::kWeakTol));
      }
      SECTION("Local Velocity (Iter: " + std::to_string(i) + ")") {
        const Velocity numeric_velocity = (value_p - value_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_velocity - local_velocity_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
      SECTION("Global Acceleration (Iter: " + std::to_string(i) + ")") {
        const Acceleration numeric_acceleration =
            (global_velocity_p - global_velocity_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_acceleration - global_acceleration_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
      SECTION("Local Acceleration (Iter: " + std::to_string(i) + ")") {
        const Acceleration numeric_acceleration =
            (local_velocity_p - local_velocity_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_acceleration - local_acceleration_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
    }

    if constexpr (is_sensor_group) {
      const auto b_T_s = Value::Random(Generator());
      //const auto sensor_value_0 = Evaluator::GetValue(L0_0, inputs);
      const auto sensor_value_p = Evaluator::GetValue(L0_p, inputs).Compose(b_T_s);
      const auto sensor_value_m = Evaluator::GetValue(L0_m, inputs).Compose(b_T_s);

      const auto global_sensor_velocity_0 = Evaluator::GetSensorVelocity(FrameType::GLOBAL, dt, L1_0, inputs, b_T_s);
      const auto global_sensor_velocity_p = Evaluator::GetSensorVelocity(FrameType::GLOBAL, dt, L1_p, inputs, b_T_s);
      const auto global_sensor_velocity_m = Evaluator::GetSensorVelocity(FrameType::GLOBAL, dt, L1_m, inputs, b_T_s);

      const auto local_sensor_velocity_0 = Evaluator::GetSensorVelocity(FrameType::LOCAL, dt, L1_0, inputs, b_T_s);
      const auto local_sensor_velocity_p = Evaluator::GetSensorVelocity(FrameType::LOCAL, dt, L1_p, inputs, b_T_s);
      const auto local_sensor_velocity_m = Evaluator::GetSensorVelocity(FrameType::LOCAL, dt, L1_m, inputs, b_T_s);

      const auto global_sensor_acceleration_0 =
          Evaluator::GetSensorAcceleration(FrameType::GLOBAL, dt, L2_0, inputs, b_T_s);
      //const auto global_sensor_acceleration_p = Evaluator::GetAcceleration(FrameType::GLOBAL, dt, L2_p, inputs, b_T_s);
      //const auto global_sensor_acceleration_m = Evaluator::GetAcceleration(FrameType::GLOBAL, dt, L2_m, inputs, b_T_s);

      const auto local_sensor_acceleration_0 =
          Evaluator::GetSensorAcceleration(FrameType::LOCAL, dt, L2_0, inputs, b_T_s);
      //const auto local_sensor_acceleration_p = Evaluator::GetAcceleration(FrameType::LOCAL, dt, L2_p, inputs, b_T_s);
      //const auto local_sensor_acceleration_m = Evaluator::GetAcceleration(FrameType::LOCAL, dt, L2_m, inputs, b_T_s);

      SECTION("Global Sensor Velocity (Iter: " + std::to_string(i) + ")") {  // Tends to be numerically unstable.
        const Velocity numeric_velocity =
            sensor_value_p.Compose(sensor_value_m.Inverse()).ToTangent() / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_velocity - global_sensor_velocity_0).isZero(NumTraits<Scalar>::kWeakTol));
      }
      SECTION("Local Sensor Velocity (Iter: " + std::to_string(i) + ")") {
        const Velocity numeric_velocity =
            sensor_value_m.Inverse().Compose(sensor_value_p).ToTangent() / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_velocity - local_sensor_velocity_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
      SECTION("Global Sensor Acceleration (Iter: " + std::to_string(i) + ")") {
        const Acceleration numeric_acceleration =
            (global_sensor_velocity_p - global_sensor_velocity_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_acceleration - global_sensor_acceleration_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
      SECTION("Local Sensor Acceleration (Iter: " + std::to_string(i) + ")") {
        const Acceleration numeric_acceleration =
            (local_sensor_velocity_p - local_sensor_velocity_m) / dt / NumTraits<Scalar>::kDelta;
        REQUIRE((numeric_acceleration - local_sensor_acceleration_0).isZero(NumTraits<Scalar>::kNormalTol));
      }
    }
  }
}

}  // namespace hyperion::splines::tests
