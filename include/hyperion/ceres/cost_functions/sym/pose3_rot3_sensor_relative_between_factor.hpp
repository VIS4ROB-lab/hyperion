// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose3.h>
#include <sym/rot3.h>

namespace sym_ceres {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: pose3_rot3_sensor_relative_between_factor
 *
 * Args:
 *     x: Pose3
 *     x_T_a: Pose3
 *     a_T_b: Rot3
 *     y: Pose3
 *     y_T_b: Pose3
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Pose3Rot3SensorRelativeBetweenFactor(
    const sym::Pose3<Scalar>& x, const sym::Pose3<Scalar>& x_T_a, const sym::Rot3<Scalar>& a_T_b,
    const sym::Pose3<Scalar>& y, const sym::Pose3<Scalar>& y_T_b,
    const Eigen::Matrix<Scalar, 3, 3>& sqrt_info, const Scalar epsilon) {
  // Total ops: 144

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_a = x_T_a.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _a_T_b = a_T_b.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y_T_b = y_T_b.Data();

  // Intermediate terms (18)
  const Scalar _tmp0 =
      -_y[0] * _y_T_b[2] + _y[1] * _y_T_b[3] + _y[2] * _y_T_b[0] + _y[3] * _y_T_b[1];
  const Scalar _tmp1 =
      -_x[0] * _x_T_a[2] + _x[1] * _x_T_a[3] + _x[2] * _x_T_a[0] + _x[3] * _x_T_a[1];
  const Scalar _tmp2 =
      _y[0] * _y_T_b[1] - _y[1] * _y_T_b[0] + _y[2] * _y_T_b[3] + _y[3] * _y_T_b[2];
  const Scalar _tmp3 =
      _x[0] * _x_T_a[1] - _x[1] * _x_T_a[0] + _x[2] * _x_T_a[3] + _x[3] * _x_T_a[2];
  const Scalar _tmp4 =
      _y[0] * _y_T_b[3] + _y[1] * _y_T_b[2] - _y[2] * _y_T_b[1] + _y[3] * _y_T_b[0];
  const Scalar _tmp5 =
      _x[0] * _x_T_a[3] + _x[1] * _x_T_a[2] - _x[2] * _x_T_a[1] + _x[3] * _x_T_a[0];
  const Scalar _tmp6 =
      -_x[0] * _x_T_a[0] - _x[1] * _x_T_a[1] - _x[2] * _x_T_a[2] + _x[3] * _x_T_a[3];
  const Scalar _tmp7 =
      -_y[0] * _y_T_b[0] - _y[1] * _y_T_b[1] - _y[2] * _y_T_b[2] + _y[3] * _y_T_b[3];
  const Scalar _tmp8 = _tmp0 * _tmp1 + _tmp2 * _tmp3 + _tmp4 * _tmp5 + _tmp6 * _tmp7;
  const Scalar _tmp9 = _tmp0 * _tmp6 - _tmp1 * _tmp7 + _tmp2 * _tmp5 - _tmp3 * _tmp4;
  const Scalar _tmp10 = -_tmp0 * _tmp5 + _tmp1 * _tmp4 + _tmp2 * _tmp6 - _tmp3 * _tmp7;
  const Scalar _tmp11 = _tmp0 * _tmp3 - _tmp1 * _tmp2 + _tmp4 * _tmp6 - _tmp5 * _tmp7;
  const Scalar _tmp12 =
      _a_T_b[0] * _tmp11 + _a_T_b[1] * _tmp9 + _a_T_b[2] * _tmp10 + _a_T_b[3] * _tmp8;
  const Scalar _tmp13 = std::min<Scalar>(std::fabs(_tmp12), 1 - epsilon);
  const Scalar _tmp14 = 2 * (2 * std::min<Scalar>(0, (((_tmp12) > 0) - ((_tmp12) < 0))) + 1) *
                        std::acos(_tmp13) / std::sqrt(Scalar(1 - std::pow(_tmp13, Scalar(2))));
  const Scalar _tmp15 =
      _tmp14 * (-_a_T_b[0] * _tmp8 - _a_T_b[1] * _tmp10 + _a_T_b[2] * _tmp9 + _a_T_b[3] * _tmp11);
  const Scalar _tmp16 =
      _tmp14 * (-_a_T_b[0] * _tmp9 + _a_T_b[1] * _tmp11 - _a_T_b[2] * _tmp8 + _a_T_b[3] * _tmp10);
  const Scalar _tmp17 =
      _tmp14 * (_a_T_b[0] * _tmp10 - _a_T_b[1] * _tmp8 - _a_T_b[2] * _tmp11 + _a_T_b[3] * _tmp9);

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp15 * sqrt_info(0, 0) + _tmp16 * sqrt_info(0, 2) + _tmp17 * sqrt_info(0, 1);
  _res(1, 0) = _tmp15 * sqrt_info(1, 0) + _tmp16 * sqrt_info(1, 2) + _tmp17 * sqrt_info(1, 1);
  _res(2, 0) = _tmp15 * sqrt_info(2, 0) + _tmp16 * sqrt_info(2, 2) + _tmp17 * sqrt_info(2, 1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres