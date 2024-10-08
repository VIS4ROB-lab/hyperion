// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot2.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline5_rot2_velocity
 *
 * Args:
 *     dt: Scalar
 *     lambdas: Matrix52
 *     x0: Rot2
 *     x1: Rot2
 *     x2: Rot2
 *     x3: Rot2
 *     x4: Rot2
 *     x5: Rot2
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Spline5Rot2VelocityGlobal(
    const Scalar dt, const Eigen::Matrix<Scalar, 5, 2>& lambdas, const sym::Rot2<Scalar>& x0,
    const sym::Rot2<Scalar>& x1, const sym::Rot2<Scalar>& x2, const sym::Rot2<Scalar>& x3,
    const sym::Rot2<Scalar>& x4, const sym::Rot2<Scalar>& x5, const Scalar epsilon) {
  // Total ops: 65

  // Input arrays
  const Eigen::Matrix<Scalar, 2, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x4 = x4.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x5 = x5.Data();

  // Intermediate terms (5)
  const Scalar _tmp0 = _x2[0] * _x3[0] + _x2[1] * _x3[1];
  const Scalar _tmp1 = _x1[0] * _x2[0] + _x1[1] * _x2[1];
  const Scalar _tmp2 = _x4[0] * _x5[0] + _x4[1] * _x5[1];
  const Scalar _tmp3 = _x3[0] * _x4[0] + _x3[1] * _x4[1];
  const Scalar _tmp4 = _x0[0] * _x1[0] + _x0[1] * _x1[1];

  // Output terms (1)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = (lambdas(0, 1) *
                    std::atan2(_x0[0] * _x1[1] - _x0[1] * _x1[0],
                               _tmp4 + epsilon * ((((_tmp4) > 0) - ((_tmp4) < 0)) + Scalar(0.5))) +
                lambdas(1, 1) *
                    std::atan2(_x1[0] * _x2[1] - _x1[1] * _x2[0],
                               _tmp1 + epsilon * ((((_tmp1) > 0) - ((_tmp1) < 0)) + Scalar(0.5))) +
                lambdas(2, 1) *
                    std::atan2(_x2[0] * _x3[1] - _x2[1] * _x3[0],
                               _tmp0 + epsilon * ((((_tmp0) > 0) - ((_tmp0) < 0)) + Scalar(0.5))) +
                lambdas(3, 1) *
                    std::atan2(_x3[0] * _x4[1] - _x3[1] * _x4[0],
                               _tmp3 + epsilon * ((((_tmp3) > 0) - ((_tmp3) < 0)) + Scalar(0.5))) +
                lambdas(4, 1) *
                    std::atan2(_x4[0] * _x5[1] - _x4[1] * _x5[0],
                               _tmp2 + epsilon * ((((_tmp2) > 0) - ((_tmp2) < 0)) + Scalar(0.5)))) /
               dt;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
