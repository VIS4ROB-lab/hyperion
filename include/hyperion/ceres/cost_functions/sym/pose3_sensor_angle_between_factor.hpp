// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose3.h>

namespace sym_ceres {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: pose3_sensor_angle_between_factor
 *
 * Args:
 *     x: Pose3
 *     x_T_s: Pose3
 *     s_d_y: Matrix11
 *     y: Pose3
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Pose3SensorAngleBetweenFactor(
    const sym::Pose3<Scalar>& x, const sym::Pose3<Scalar>& x_T_s,
    const Eigen::Matrix<Scalar, 1, 1>& s_d_y, const sym::Pose3<Scalar>& y,
    const Eigen::Matrix<Scalar, 1, 1>& sqrt_info, const Scalar epsilon) {
  // Total ops: 85

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_s = x_T_s.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (8)
  const Scalar _tmp0 =
      _x[0] * _x_T_s[1] - _x[1] * _x_T_s[0] + _x[2] * _x_T_s[3] + _x[3] * _x_T_s[2];
  const Scalar _tmp1 =
      -_x[0] * _x_T_s[2] + _x[1] * _x_T_s[3] + _x[2] * _x_T_s[0] + _x[3] * _x_T_s[1];
  const Scalar _tmp2 =
      _x[0] * _x_T_s[3] + _x[1] * _x_T_s[2] - _x[2] * _x_T_s[1] + _x[3] * _x_T_s[0];
  const Scalar _tmp3 =
      -_x[0] * _x_T_s[0] - _x[1] * _x_T_s[1] - _x[2] * _x_T_s[2] + _x[3] * _x_T_s[3];
  const Scalar _tmp4 = -_tmp0 * _y[2] - _tmp1 * _y[1] - _tmp2 * _y[0];
  const Scalar _tmp5 = _tmp3 * _y[3];
  const Scalar _tmp6 = std::min<Scalar>(1 - epsilon, std::fabs(_tmp4 - _tmp5));
  const Scalar _tmp7 =
      4 *
      std::pow(
          Scalar(2 * std::min<Scalar>(0, (((-_tmp4 + _tmp5) > 0) - ((-_tmp4 + _tmp5) < 0))) + 1),
          Scalar(2)) *
      std::pow(Scalar(std::acos(_tmp6)), Scalar(2)) / (1 - std::pow(_tmp6, Scalar(2)));

  // Output terms (1)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) =
      sqrt_info(0, 0) *
      (-s_d_y(0, 0) +
       std::sqrt(Scalar(
           _tmp7 * std::pow(Scalar(-_tmp0 * _y[0] - _tmp1 * _y[3] + _tmp2 * _y[2] + _tmp3 * _y[1]),
                            Scalar(2)) +
           _tmp7 * std::pow(Scalar(_tmp0 * _y[1] - _tmp1 * _y[2] - _tmp2 * _y[3] + _tmp3 * _y[0]),
                            Scalar(2)) +
           _tmp7 * std::pow(Scalar(-_tmp0 * _y[3] + _tmp1 * _y[0] - _tmp2 * _y[1] + _tmp3 * _y[2]),
                            Scalar(2)) +
           epsilon)));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres