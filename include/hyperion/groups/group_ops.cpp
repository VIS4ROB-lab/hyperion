/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyperion/groups/group_ops.hpp"

#include "hyperion/groups/sym/sym.hpp"
#include "hyperion/messages/sized_message.hpp"
#include "hyperion/utils/inversion.hpp"

namespace hyperion {

using namespace sym;

namespace internal {

template <typename TGroup>
auto DeltaWithJacobiansDispatcher(
    const TGroup& x,
    const TGroup& y,
    Matrix<LieGroupOps<TGroup>::TangentDim()>* J_x,
    Matrix<LieGroupOps<TGroup>::TangentDim()>* J_y) -> Vector<LieGroupOps<TGroup>::TangentDim()> {
  if constexpr (std::is_same_v<TGroup, Rot2>) {
    return DeltaRot2WithJacobians<Scalar>(x, y, sym::kDefaultEpsilon<Scalar>, J_x, J_y);
  } else if constexpr (std::is_same_v<TGroup, Rot3>) {
    return DeltaRot3WithJacobians<Scalar>(x, y, sym::kDefaultEpsilon<Scalar>, J_x, J_y);
  } else if constexpr (std::is_same_v<TGroup, Pose2>) {
    return DeltaPose2WithJacobians<Scalar>(x, y, sym::kDefaultEpsilon<Scalar>, J_x, J_y);
  } else {  // Pose3
    return DeltaPose3WithJacobians<Scalar>(x, y, sym::kDefaultEpsilon<Scalar>, J_x, J_y);
  }
}

template <typename TGroup>
auto RetractWithJacobiansDispatcher(
    const TGroup& x,
    const Vector<LieGroupOps<TGroup>::TangentDim()>& dx,
    Matrix<LieGroupOps<TGroup>::TangentDim()>* J_x,
    Matrix<LieGroupOps<TGroup>::TangentDim()>* J_dx) -> TGroup {
  if constexpr (std::is_same_v<TGroup, Rot2>) {
    return RetractRot2WithJacobians<Scalar>(x, dx, sym::kDefaultEpsilon<Scalar>, J_x, J_dx);
  } else if constexpr (std::is_same_v<TGroup, Rot3>) {
    return RetractRot3WithJacobians<Scalar>(x, dx, sym::kDefaultEpsilon<Scalar>, J_x, J_dx);
  } else if constexpr (std::is_same_v<TGroup, Pose2>) {
    return RetractPose2WithJacobians<Scalar>(x, dx, sym::kDefaultEpsilon<Scalar>, J_x, J_dx);
  } else {  // Pose3
    return RetractPose3WithJacobians<Scalar>(x, dx, sym::kDefaultEpsilon<Scalar>, J_x, J_dx);
  }
}

}  // namespace internal

template <int Dim>
auto GroupOps<Vector<Dim>>::GetData(const Value& value) -> const Scalar* {
  return value.data();
}

template <int Dim>
auto GroupOps<Vector<Dim>>::GetMutableData(Value& value) -> Scalar* {
  return const_cast<Scalar*>(GetData(value));
}

template <int Dim>
auto GroupOps<Vector<Dim>>::FromVector(const Eigen::Ref<const AmbientVector>& vector) -> Value {
  return vector;
}

template <int Dim>
auto GroupOps<Vector<Dim>>::GetVector(const Value& value) -> const AmbientVector& {
  return value;
}

template <int Dim>
auto GroupOps<Vector<Dim>>::GetMutableVector(Value& value) -> AmbientVector& {
  return value;
}

template <int Dim>
auto GroupOps<Vector<Dim>>::DeltaAndPrecisionToEtaAndLambda(const TangentVector& d_x0_y, const TangentMatrix& P_x0)
    -> std::tuple<TangentVector, TangentMatrix> {
  return {P_x0 * d_x0_y, P_x0};
}

template <int Dim>
auto GroupOps<Vector<Dim>>::MeanAndPrecisionToDeltaAndPrecision(
    const Value& x0,
    const Value& y,
    const TangentMatrix& P_y) -> std::tuple<TangentVector, TangentMatrix> {
  return {y - x0, P_y};
}

template <int Dim>
auto GroupOps<Vector<Dim>>::MeanAndPrecisionToEtaAndLambda(const Value& x0, const Value& y, const TangentMatrix& P_y)
    -> std::tuple<TangentVector, TangentMatrix> {
  const auto [d_x0_y, P_x0] = MeanAndPrecisionToDeltaAndPrecision(x0, y, P_y);
  return DeltaAndPrecisionToEtaAndLambda(d_x0_y, P_x0);
}

template <int Dim>
auto GroupOps<Vector<Dim>>::EtaAndLambdaToMeanAndPrecision(
    const Value& x0,
    const TangentVector& e0,
    const TangentMatrix& L0,
    const Scalar& step_size) -> std::tuple<Value, TangentMatrix> {
  const TangentVector dx = step_size * pseudoSolvePSDSMatrix<LieGroupOps<Vector<Dim>>::TangentDim()>(L0, e0);
  const Value y = x0 + dx;
  return {y, L0};
}

template <typename TGroup>
auto GroupOps<TGroup>::GetData(const Value& value) -> const Scalar* {
  return value.Data().data();
}

template <typename TGroup>
auto GroupOps<TGroup>::GetMutableData(Value& value) -> Scalar* {
  return const_cast<Scalar*>(GetData(value));
}

template <typename TGroup>
auto GroupOps<TGroup>::FromVector(const Eigen::Ref<const AmbientVector>& vector) -> Value {
  return Value{vector};
}

template <typename TGroup>
auto GroupOps<TGroup>::GetVector(const Value& value) -> const AmbientVector& {
  return value.Data();
}

template <typename TGroup>
auto GroupOps<TGroup>::GetMutableVector(Value& value) -> AmbientVector& {
  return const_cast<AmbientVector&>(value.Data());
}

template <typename TGroup>
auto GroupOps<TGroup>::DeltaAndPrecisionToEtaAndLambda(const TangentVector& d_x0_y, const TangentMatrix& P_x0)
    -> std::tuple<TangentVector, TangentMatrix> {
  return {P_x0 * d_x0_y, P_x0};
}

template <typename TGroup>
auto GroupOps<TGroup>::MeanAndPrecisionToDeltaAndPrecision(const Value& x0, const Value& y, const TangentMatrix& P_y)
    -> std::tuple<TangentVector, TangentMatrix> {
  TangentMatrix J_y;
  const TangentVector d_x0_y = internal::DeltaWithJacobiansDispatcher<TGroup>(x0, y, nullptr, &J_y);
  const TangentMatrix P_x0 = J_y.transpose() * P_y.template selfadjointView<Eigen::Lower>() * J_y;
  return {d_x0_y, P_x0};
}

template <typename TGroup>
auto GroupOps<TGroup>::MeanAndPrecisionToEtaAndLambda(const Value& x0, const Value& y, const TangentMatrix& P_y)
    -> std::tuple<TangentVector, TangentMatrix> {
  const auto [d_x0_y, P_x0] = MeanAndPrecisionToDeltaAndPrecision(x0, y, P_y);
  return DeltaAndPrecisionToEtaAndLambda(d_x0_y, P_x0);
}

template <typename TGroup>
auto GroupOps<TGroup>::EtaAndLambdaToMeanAndPrecision(
    const Value& x0,
    const TangentVector& e0,
    const TangentMatrix& L0,
    const Scalar& step_size) -> std::tuple<Value, TangentMatrix> {
  TangentMatrix J_dx;
  const TangentVector dx = step_size * pseudoSolvePSDSMatrix<LieGroupOps<TGroup>::TangentDim()>(L0, e0);
  const auto y = internal::RetractWithJacobiansDispatcher<TGroup>(x0, dx, nullptr, &J_dx);
  return {y, J_dx.transpose() * L0.template selfadjointView<Eigen::Lower>() * J_dx};
}

template struct GroupOps<R1>;
template struct GroupOps<R2>;
template struct GroupOps<R3>;
template struct GroupOps<R4>;
template struct GroupOps<R5>;
template struct GroupOps<R6>;
template struct GroupOps<Rot2>;
template struct GroupOps<Rot3>;
template struct GroupOps<Pose2>;
template struct GroupOps<Pose3>;

}  // namespace hyperion
