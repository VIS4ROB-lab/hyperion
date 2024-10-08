// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/rot2.h>

namespace sym_ceres {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: rot2_delta_factor
 *
 * Args:
 *     x: Rot2
 *     y: Rot2
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 *     res_D_x: (1x2) jacobian (result_dim x storage_dim) of res (1) wrt arg x (2) (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Rot2DeltaFactorWithJacobian0(
    const sym::Rot2<Scalar>& x, const sym::Rot2<Scalar>& y,
    const Eigen::Matrix<Scalar, 1, 1>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr) {
  // Total ops: 26

  // Input arrays
  const Eigen::Matrix<Scalar, 2, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _y = y.Data();

  // Intermediate terms (8)
  const Scalar _tmp0 = _x[0] * _y[1] - _x[1] * _y[0];
  const Scalar _tmp1 = _x[1] * _y[1];
  const Scalar _tmp2 = _x[0] * _y[0];
  const Scalar _tmp3 = _tmp1 + _tmp2;
  const Scalar _tmp4 = _tmp3 + epsilon * ((((_tmp3) > 0) - ((_tmp3) < 0)) + Scalar(0.5));
  const Scalar _tmp5 = std::pow(_tmp0, Scalar(2));
  const Scalar _tmp6 = std::pow(_tmp4, Scalar(2));
  const Scalar _tmp7 =
      _tmp6 * sqrt_info(0, 0) * (-_tmp5 / _tmp6 + (-_tmp1 - _tmp2) / _tmp4) / (_tmp5 + _tmp6);

  // Output terms (2)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) * std::atan2(_tmp0, _tmp4);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 2>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = -_tmp7 * _x[1];
    _res_D_x(0, 1) = _tmp7 * _x[0];
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
