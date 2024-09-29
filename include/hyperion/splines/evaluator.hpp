/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/groups/forward.hpp"
#include "hyperion/splines/forward.hpp"

namespace hyperion::splines {

template <typename TGroup, int TOrder>
struct Evaluator {
  // Definitions.
  using Value = TGroup;
  using Velocity = typename sym::LieGroupOps<TGroup>::TangentVec;
  using Acceleration = typename sym::LieGroupOps<TGroup>::TangentVec;

  using Lambdas0 = Matrix<TOrder - 1, 1>;  ///< Value lambdas (in compact, cumulative form).
  using Lambdas1 = Matrix<TOrder - 1, 2>;  ///< Velocity lambdas (in compact, cumulative form).
  using Lambdas2 = Matrix<TOrder - 1, 3>;  ///< Acceleration lambdas (in compact, cumulative form).

  using Inputs = std::array<TGroup, TOrder>;

  // Constants.
  static constexpr auto kOrder = TOrder;
  static constexpr auto kDegree = TOrder - 1;

  static auto GetValue(const Lambdas0& lambdas0, const Inputs& inputs) -> Value;

  static auto GetVelocity(const FrameType& frame_type, const Scalar& dt, const Lambdas1& lambdas1, const Inputs& inputs)
      -> Velocity;

  static auto GetSensorVelocity(
      const FrameType& frame_type,
      const Scalar& dt,
      const Lambdas1& lambdas1,
      const Inputs& inputs,
      const Value& b_T_s) -> Velocity
    requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>;

  static auto GetAcceleration(
      const FrameType& frame_type,
      const Scalar& dt,
      const Lambdas2& lambdas2,
      const Inputs& inputs) -> Acceleration;

  static auto GetSensorAcceleration(
      const FrameType& frame_type,
      const Scalar& dt,
      const Lambdas2& lambdas2,
      const Inputs& inputs,
      const Value& b_T_s) -> Velocity
    requires std::is_same_v<Value, Pose2> || std::is_same_v<Value, Pose3>;

 protected:
  template <typename TFunction, typename TLambdas, std::size_t... Is, typename... TArgs>
  static auto BindValueFunction(
      TFunction function,
      const TLambdas& lambdas,
      const Inputs& inputs,
      std::index_sequence<Is...>,
      TArgs&&... args) {
    return function(lambdas, std::get<Is>(inputs)..., std::forward<TArgs>(args)...);
  }

  template <typename TFunction, typename TLambdas, typename... TArgs>
  static auto BindValueFunction(TFunction function, const TLambdas& lambdas, const Inputs& inputs, TArgs&&... args) {
    return BindValueFunction(
        function,
        lambdas,
        inputs,
        std::make_index_sequence<kOrder>(),
        std::forward<TArgs>(args)...);
  }

  template <typename TFunction, typename TLambdas, std::size_t... Is, typename... TArgs>
  static auto BindDerivativeFunction(
      TFunction function,
      const Scalar& dt,
      const TLambdas& lambdas,
      const Inputs& inputs,
      std::index_sequence<Is...>,
      TArgs&&... args) {
    return function(dt, lambdas, std::get<Is>(inputs)..., std::forward<TArgs>(args)...);
  }

  template <typename TFunction, typename TLambdas, typename... TArgs>
  static auto BindDerivativeFunction(
      TFunction function,
      const Scalar& dt,
      const TLambdas& lambdas,
      const Inputs& inputs,
      TArgs&&... args) {
    return BindDerivativeFunction(
        function,
        dt,
        lambdas,
        inputs,
        std::make_index_sequence<kOrder>(),
        std::forward<TArgs>(args)...);
  }
};

extern template struct Evaluator<R1, 4>;
extern template struct Evaluator<R2, 4>;
extern template struct Evaluator<R3, 4>;
extern template struct Evaluator<Rot2, 4>;
extern template struct Evaluator<Rot3, 4>;
extern template struct Evaluator<Pose2, 4>;
extern template struct Evaluator<Pose3, 4>;

extern template struct Evaluator<R1, 5>;
extern template struct Evaluator<R2, 5>;
extern template struct Evaluator<R3, 5>;
extern template struct Evaluator<Rot2, 5>;
extern template struct Evaluator<Rot3, 5>;
extern template struct Evaluator<Pose2, 5>;
extern template struct Evaluator<Pose3, 5>;

extern template struct Evaluator<R1, 6>;
extern template struct Evaluator<R2, 6>;
extern template struct Evaluator<R3, 6>;
extern template struct Evaluator<Rot2, 6>;
extern template struct Evaluator<Rot3, 6>;
extern template struct Evaluator<Pose2, 6>;
extern template struct Evaluator<Pose3, 6>;

}  // namespace hyperion::splines
