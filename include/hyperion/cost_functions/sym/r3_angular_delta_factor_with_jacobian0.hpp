// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: r3_angular_delta_factor
 *
 * Args:
 *     x: Matrix31
 *     y: Matrix31
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 *     res_D_x: (1x3) jacobian of res (1) wrt arg x (3)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> R3AngularDeltaFactorWithJacobian0(
    const Eigen::Matrix<Scalar, 3, 1>& x, const Eigen::Matrix<Scalar, 3, 1>& y,
    const Eigen::Matrix<Scalar, 1, 1>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr) {
  // Total ops: 60

  // Input arrays

  // Intermediate terms (13)
  const Scalar _tmp0 = x(0, 0) * y(1, 0) - x(1, 0) * y(0, 0);
  const Scalar _tmp1 = -x(0, 0) * y(2, 0) + x(2, 0) * y(0, 0);
  const Scalar _tmp2 = x(1, 0) * y(2, 0) - x(2, 0) * y(1, 0);
  const Scalar _tmp3 = std::pow(_tmp0, Scalar(2)) + std::pow(_tmp1, Scalar(2)) +
                       std::pow(_tmp2, Scalar(2)) + epsilon;
  const Scalar _tmp4 = std::sqrt(_tmp3);
  const Scalar _tmp5 = x(0, 0) * y(0, 0) + x(1, 0) * y(1, 0) + x(2, 0) * y(2, 0);
  const Scalar _tmp6 = _tmp5 + epsilon * ((((_tmp5) > 0) - ((_tmp5) < 0)) + Scalar(0.5));
  const Scalar _tmp7 = (Scalar(1) / Scalar(2)) / (_tmp4 * _tmp6);
  const Scalar _tmp8 = std::pow(_tmp6, Scalar(2));
  const Scalar _tmp9 = _tmp4 / _tmp8;
  const Scalar _tmp10 = _tmp8 * sqrt_info(0, 0) / (_tmp3 + _tmp8);
  const Scalar _tmp11 = 2 * y(0, 0);
  const Scalar _tmp12 = 2 * _tmp2;

  // Output terms (2)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) * std::atan2(_tmp4, _tmp6);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 3>> _res_D_x{res_D_x};

    _res_D_x(0, 0) =
        _tmp10 * (_tmp7 * (2 * _tmp0 * y(1, 0) - 2 * _tmp1 * y(2, 0)) - _tmp9 * y(0, 0));
    _res_D_x(0, 1) = _tmp10 * (_tmp7 * (-_tmp0 * _tmp11 + _tmp12 * y(2, 0)) - _tmp9 * y(1, 0));
    _res_D_x(0, 2) = _tmp10 * (_tmp7 * (_tmp1 * _tmp11 - _tmp12 * y(1, 0)) - _tmp9 * y(2, 0));
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
