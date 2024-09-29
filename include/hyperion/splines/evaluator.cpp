/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/splines/evaluator.hpp"

#include "hyperion/splines/sym/sym.hpp"

namespace hyperion::splines {

using namespace sym;

template <typename TGroup, int TDegree>
auto GetValueCallable() {
  if constexpr (std::is_same_v<TGroup, R1>) {
    if constexpr (TDegree == 3) {
      return Spline3R1Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4R1Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5R1Value<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, R2>) {
    if constexpr (TDegree == 3) {
      return Spline3R2Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4R2Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5R2Value<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, R3>) {
    if constexpr (TDegree == 3) {
      return Spline3R3Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4R3Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5R3Value<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Rot2d>) {
    if constexpr (TDegree == 3) {
      return Spline3Rot2Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4Rot2Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5Rot2Value<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Rot3d>) {
    if constexpr (TDegree == 3) {
      return Spline3Rot3Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4Rot3Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5Rot3Value<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose2d>) {
    if constexpr (TDegree == 3) {
      return Spline3Pose2Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4Pose2Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5Pose2Value<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose3d>) {
    if constexpr (TDegree == 3) {
      return Spline3Pose3Value<Scalar>;
    } else if constexpr (TDegree == 4) {
      return Spline4Pose3Value<Scalar>;
    } else if constexpr (TDegree == 5) {
      return Spline5Pose3Value<Scalar>;
    }
  }
}

template <typename TGroup, int TDegree>
auto GetVelocityCallable(const FrameType& frame_type) {
  if constexpr (std::is_same_v<TGroup, R1>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3R1VelocityGlobal<Scalar> : Spline3R1VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4R1VelocityGlobal<Scalar> : Spline4R1VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5R1VelocityGlobal<Scalar> : Spline5R1VelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, R2>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3R2VelocityGlobal<Scalar> : Spline3R2VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4R2VelocityGlobal<Scalar> : Spline4R2VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5R2VelocityGlobal<Scalar> : Spline5R2VelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, R3>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3R3VelocityGlobal<Scalar> : Spline3R3VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4R3VelocityGlobal<Scalar> : Spline4R3VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5R3VelocityGlobal<Scalar> : Spline5R3VelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Rot2d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Rot2VelocityGlobal<Scalar> : Spline3Rot2VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Rot2VelocityGlobal<Scalar> : Spline4Rot2VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Rot2VelocityGlobal<Scalar> : Spline5Rot2VelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Rot3d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Rot3VelocityGlobal<Scalar> : Spline3Rot3VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Rot3VelocityGlobal<Scalar> : Spline4Rot3VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Rot3VelocityGlobal<Scalar> : Spline5Rot3VelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose2d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose2VelocityGlobal<Scalar> : Spline3Pose2VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose2VelocityGlobal<Scalar> : Spline4Pose2VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose2VelocityGlobal<Scalar> : Spline5Pose2VelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose3d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose3VelocityGlobal<Scalar> : Spline3Pose3VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose3VelocityGlobal<Scalar> : Spline4Pose3VelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose3VelocityGlobal<Scalar> : Spline5Pose3VelocityLocal<Scalar>;
    }
  }
}

template <typename TGroup, int TDegree>
auto GetSensorVelocityCallable(const FrameType& frame_type) {
  if constexpr (std::is_same_v<TGroup, sym::Pose2d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose2SensorVelocityGlobal<Scalar>
                                       : Spline3Pose2SensorVelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose2SensorVelocityGlobal<Scalar>
                                       : Spline4Pose2SensorVelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose2SensorVelocityGlobal<Scalar>
                                       : Spline5Pose2SensorVelocityLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose3d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose3SensorVelocityGlobal<Scalar>
                                       : Spline3Pose3SensorVelocityLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose3SensorVelocityGlobal<Scalar>
                                       : Spline4Pose3SensorVelocityLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose3SensorVelocityGlobal<Scalar>
                                       : Spline5Pose3SensorVelocityLocal<Scalar>;
    }
  }
}

template <typename TGroup, int TDegree>
auto GetAccelerationCallable(const FrameType& frame_type) {
  if constexpr (std::is_same_v<TGroup, R1>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3R1AccelerationGlobal<Scalar> : Spline3R1AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4R1AccelerationGlobal<Scalar> : Spline4R1AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5R1AccelerationGlobal<Scalar> : Spline5R1AccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, R2>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3R2AccelerationGlobal<Scalar> : Spline3R2AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4R2AccelerationGlobal<Scalar> : Spline4R2AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5R2AccelerationGlobal<Scalar> : Spline5R2AccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, R3>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3R3AccelerationGlobal<Scalar> : Spline3R3AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4R3AccelerationGlobal<Scalar> : Spline4R3AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5R3AccelerationGlobal<Scalar> : Spline5R3AccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Rot2d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Rot2AccelerationGlobal<Scalar> : Spline3Rot2AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Rot2AccelerationGlobal<Scalar> : Spline4Rot2AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Rot2AccelerationGlobal<Scalar> : Spline5Rot2AccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Rot3d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Rot3AccelerationGlobal<Scalar> : Spline3Rot3AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Rot3AccelerationGlobal<Scalar> : Spline4Rot3AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Rot3AccelerationGlobal<Scalar> : Spline5Rot3AccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose2d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose2AccelerationGlobal<Scalar> : Spline3Pose2AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose2AccelerationGlobal<Scalar> : Spline4Pose2AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose2AccelerationGlobal<Scalar> : Spline5Pose2AccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose3d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose3AccelerationGlobal<Scalar> : Spline3Pose3AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose3AccelerationGlobal<Scalar> : Spline4Pose3AccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose3AccelerationGlobal<Scalar> : Spline5Pose3AccelerationLocal<Scalar>;
    }
  }
}

template <typename TGroup, int TDegree>
auto GetSensorAccelerationCallable(const FrameType& frame_type) {
  if constexpr (std::is_same_v<TGroup, sym::Pose2d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose2SensorAccelerationGlobal<Scalar>
                                       : Spline3Pose2SensorAccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose2SensorAccelerationGlobal<Scalar>
                                       : Spline4Pose2SensorAccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose2SensorAccelerationGlobal<Scalar>
                                       : Spline5Pose2SensorAccelerationLocal<Scalar>;
    }
  } else if constexpr (std::is_same_v<TGroup, sym::Pose3d>) {
    if constexpr (TDegree == 3) {
      return isGlobalFrame(frame_type) ? Spline3Pose3SensorAccelerationGlobal<Scalar>
                                       : Spline3Pose3SensorAccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 4) {
      return isGlobalFrame(frame_type) ? Spline4Pose3SensorAccelerationGlobal<Scalar>
                                       : Spline4Pose3SensorAccelerationLocal<Scalar>;
    } else if constexpr (TDegree == 5) {
      return isGlobalFrame(frame_type) ? Spline5Pose3SensorAccelerationGlobal<Scalar>
                                       : Spline5Pose3SensorAccelerationLocal<Scalar>;
    }
  }
}

template <SplineType TSplineType, int TDegree>
auto UniformSplineValueLambdas(const Scalar& ut) {
  if constexpr (TSplineType == SplineType::BSPLINE) {
    if constexpr (TDegree == 3) {
      return CumulativeUniformBSplineLambdas40<Scalar>(ut);
    } else if constexpr (TDegree == 4) {
      return CumulativeUniformBSplineLambdas50<Scalar>(ut);
    } else if constexpr (TDegree == 5) {
      return CumulativeUniformBSplineLambdas60<Scalar>(ut);
    }
  } else if constexpr (TSplineType == SplineType::ZSPLINE) {
    if constexpr (TDegree == 3) {
      return CumulativeUniformZSplineLambdas40<Scalar>(ut);
    } else if constexpr (TDegree == 5) {
      return CumulativeUniformZSplineLambdas60<Scalar>(ut);
    }
  }
}

template <SplineType TSplineType, int TDegree>
auto UniformSplineVelocityLambdas(const Scalar& ut) {
  if constexpr (TSplineType == SplineType::BSPLINE) {
    if constexpr (TDegree == 3) {
      return CumulativeUniformBSplineLambdas41<Scalar>(ut);
    } else if constexpr (TDegree == 4) {
      return CumulativeUniformBSplineLambdas51<Scalar>(ut);
    } else if constexpr (TDegree == 5) {
      return CumulativeUniformBSplineLambdas61<Scalar>(ut);
    }
  } else if constexpr (TSplineType == SplineType::ZSPLINE) {
    if constexpr (TDegree == 3) {
      return CumulativeUniformZSplineLambdas41<Scalar>(ut);
    } else if constexpr (TDegree == 5) {
      return CumulativeUniformZSplineLambdas61<Scalar>(ut);
    }
  }
}

template <SplineType TSplineType, int TDegree>
auto UniformSplineAccelerationLambdas(const Scalar& ut) {
  if constexpr (TSplineType == SplineType::BSPLINE) {
    if constexpr (TDegree == 3) {
      return CumulativeUniformBSplineLambdas42<Scalar>(ut);
    } else if constexpr (TDegree == 4) {
      return CumulativeUniformBSplineLambdas52<Scalar>(ut);
    } else if constexpr (TDegree == 5) {
      return CumulativeUniformBSplineLambdas62<Scalar>(ut);
    }
  } else if constexpr (TSplineType == SplineType::ZSPLINE) {
    if constexpr (TDegree == 3) {
      return CumulativeUniformZSplineLambdas42<Scalar>(ut);
    } else if constexpr (TDegree == 5) {
      return CumulativeUniformZSplineLambdas62<Scalar>(ut);
    }
  }
}

template <typename TGroup, int TNumInputs>
auto Evaluator<TGroup, TNumInputs>::GetValue(const Lambdas0& lambdas0, const Inputs& inputs) -> Value {
  const auto callable = GetValueCallable<TGroup, kDegree>();
  return BindValueFunction(callable, lambdas0, inputs, kDefaultEpsilon);
}

template <typename TGroup, int TNumInputs>
auto Evaluator<TGroup, TNumInputs>::GetVelocity(
    const FrameType& frame_type,
    const Scalar& dt,
    const Lambdas1& lambdas1,
    const Inputs& inputs) -> Velocity {
  const auto callable = GetVelocityCallable<TGroup, kDegree>(frame_type);
  return BindDerivativeFunction(callable, dt, lambdas1, inputs, kDefaultEpsilon);
}

template <typename TGroup, int TNumInputs>
auto Evaluator<TGroup, TNumInputs>::GetSensorVelocity(
    const FrameType& frame_type,
    const Scalar& dt,
    const Lambdas1& lambdas1,
    const Inputs& inputs,
    const Value& b_T_s) -> Velocity
  requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>
{
  const auto callable = GetSensorVelocityCallable<TGroup, kDegree>(frame_type);
  return BindDerivativeFunction(callable, dt, lambdas1, inputs, b_T_s, kDefaultEpsilon);
}

template <typename TGroup, int TNumInputs>
auto Evaluator<TGroup, TNumInputs>::GetAcceleration(
    const FrameType& frame_type,
    const Scalar& dt,
    const Lambdas2& lambdas2,
    const Inputs& inputs) -> Acceleration {
  const auto callable = GetAccelerationCallable<TGroup, kDegree>(frame_type);
  return BindDerivativeFunction(callable, dt, lambdas2, inputs, kDefaultEpsilon);
}

template <typename TGroup, int TNumInputs>
auto Evaluator<TGroup, TNumInputs>::GetSensorAcceleration(
    const FrameType& frame_type,
    const Scalar& dt,
    const Lambdas2& lambdas2,
    const Inputs& inputs,
    const Value& b_T_s) -> Velocity
  requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>
{
  const auto callable = GetSensorAccelerationCallable<TGroup, kDegree>(frame_type);
  return BindDerivativeFunction(callable, dt, lambdas2, inputs, b_T_s, kDefaultEpsilon);
}

template struct Evaluator<R1, 4>;
template struct Evaluator<R2, 4>;
template struct Evaluator<R3, 4>;
template struct Evaluator<Rot2, 4>;
template struct Evaluator<Rot3, 4>;
template struct Evaluator<Pose2, 4>;
template struct Evaluator<Pose3, 4>;

template struct Evaluator<R1, 5>;
template struct Evaluator<R2, 5>;
template struct Evaluator<R3, 5>;
template struct Evaluator<Rot2, 5>;
template struct Evaluator<Rot3, 5>;
template struct Evaluator<Pose2, 5>;
template struct Evaluator<Pose3, 5>;

template struct Evaluator<R1, 6>;
template struct Evaluator<R2, 6>;
template struct Evaluator<R3, 6>;
template struct Evaluator<Rot2, 6>;
template struct Evaluator<Rot3, 6>;
template struct Evaluator<Pose2, 6>;
template struct Evaluator<Pose3, 6>;

}  // namespace hyperion::splines
