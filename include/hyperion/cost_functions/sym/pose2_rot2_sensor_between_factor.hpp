// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose2.h>
#include <sym/rot2.h>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: pose2_rot2_sensor_between_factor
 *
 * Args:
 *     x: Pose2
 *     x_T_s: Pose2
 *     s_T_y: Rot2
 *     y: Pose2
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Pose2Rot2SensorBetweenFactor(
    const sym::Pose2<Scalar>& x, const sym::Pose2<Scalar>& x_T_s, const sym::Rot2<Scalar>& s_T_y,
    const sym::Pose2<Scalar>& y, const Eigen::Matrix<Scalar, 1, 1>& sqrt_info,
    const Scalar epsilon) {
  // Total ops: 24

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x_T_s = x_T_s.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _s_T_y = s_T_y.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y = y.Data();

  // Intermediate terms (5)
  const Scalar _tmp0 = _x[0] * _x_T_s[0] - _x[1] * _x_T_s[1];
  const Scalar _tmp1 = _x[0] * _x_T_s[1] + _x[1] * _x_T_s[0];
  const Scalar _tmp2 = _tmp0 * _y[1] - _tmp1 * _y[0];
  const Scalar _tmp3 = _tmp0 * _y[0] + _tmp1 * _y[1];
  const Scalar _tmp4 = _s_T_y[0] * _tmp3 + _s_T_y[1] * _tmp2;

  // Output terms (1)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) *
               std::atan2(_s_T_y[0] * _tmp2 - _s_T_y[1] * _tmp3,
                          _tmp4 + epsilon * ((((_tmp4) > 0) - ((_tmp4) < 0)) + Scalar(0.5)));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion