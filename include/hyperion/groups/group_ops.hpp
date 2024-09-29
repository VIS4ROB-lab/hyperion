/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyperion/groups/forward.hpp"
#include "hyperion/messages/forward.hpp"

namespace hyperion {

template <int Dim>
struct GroupOps<Vector<Dim>> {
  // Definitions.
  using Value = Vector<Dim>;
  using AmbientVector = Vector<sym::StorageOps<Value>::StorageDim()>;
  using AmbientMatrix = Matrix<sym::StorageOps<Value>::StorageDim()>;
  using TangentVector = Vector<sym::LieGroupOps<Value>::TangentDim()>;
  using TangentMatrix = Matrix<sym::LieGroupOps<Value>::TangentDim()>;

  static auto GetData(const Value& value) -> const Scalar*;
  static auto GetMutableData(Value& value) -> Scalar*;
  static auto FromVector(const Eigen::Ref<const AmbientVector>& vector) -> Value;
  static auto GetVector(const Value& value) -> const AmbientVector&;
  static auto GetMutableVector(Value& value) -> AmbientVector&;

  static auto DeltaAndPrecisionToEtaAndLambda(const TangentVector& d_x0_y, const TangentMatrix& P_x0)
      -> std::tuple<TangentVector, TangentMatrix>;
  static auto MeanAndPrecisionToDeltaAndPrecision(const Value& x0, const Value& y, const TangentMatrix& P_y)
      -> std::tuple<TangentVector, TangentMatrix>;
  static auto MeanAndPrecisionToEtaAndLambda(const Value& x0, const Value& y, const TangentMatrix& P_y)
      -> std::tuple<TangentVector, TangentMatrix>;
  static auto EtaAndLambdaToMeanAndPrecision(
      const Value& x0,
      const TangentVector& e0,
      const TangentMatrix& L0,
      const Scalar& step_size) -> std::tuple<Value, TangentMatrix>;
};

template <typename TGroup>
struct GroupOps {
  // Definitions.
  using Value = TGroup;
  using AmbientVector = Vector<sym::StorageOps<TGroup>::StorageDim()>;
  using AmbientMatrix = Matrix<sym::StorageOps<TGroup>::StorageDim()>;
  using TangentVector = Vector<sym::LieGroupOps<TGroup>::TangentDim()>;
  using TangentMatrix = Matrix<sym::LieGroupOps<TGroup>::TangentDim()>;

  static auto GetData(const Value& value) -> const Scalar*;
  static auto GetMutableData(Value& value) -> Scalar*;
  static auto FromVector(const Eigen::Ref<const AmbientVector>& vector) -> Value;
  static auto GetVector(const Value& value) -> const AmbientVector&;
  static auto GetMutableVector(Value& value) -> AmbientVector&;

  static auto DeltaAndPrecisionToEtaAndLambda(const TangentVector& d_x0_y, const TangentMatrix& P_x0)
      -> std::tuple<TangentVector, TangentMatrix>;
  static auto MeanAndPrecisionToDeltaAndPrecision(const Value& x0, const Value& y, const TangentMatrix& P_y)
      -> std::tuple<TangentVector, TangentMatrix>;
  static auto MeanAndPrecisionToEtaAndLambda(const Value& x0, const Value& y, const TangentMatrix& P_y)
      -> std::tuple<TangentVector, TangentMatrix>;
  static auto EtaAndLambdaToMeanAndPrecision(
      const Value& x0,
      const TangentVector& e0,
      const TangentMatrix& L0,
      const Scalar& step_size) -> std::tuple<Value, TangentMatrix>;
};

}  // namespace hyperion
