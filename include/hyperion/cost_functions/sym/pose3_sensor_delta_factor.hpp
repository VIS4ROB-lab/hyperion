// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose3.h>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: pose3_sensor_delta_factor
 *
 * Args:
 *     x: Pose3
 *     x_T_y: Pose3
 *     y: Pose3
 *     sqrt_info: Matrix66
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix61
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> Pose3SensorDeltaFactor(const sym::Pose3<Scalar>& x,
                                                   const sym::Pose3<Scalar>& x_T_y,
                                                   const sym::Pose3<Scalar>& y,
                                                   const Eigen::Matrix<Scalar, 6, 6>& sqrt_info,
                                                   const Scalar epsilon) {
  // Total ops: 205

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_y = x_T_y.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (34)
  const Scalar _tmp0 = _x[0] * _y[0] + _x[1] * _y[1] + _x[2] * _y[2] + _x[3] * _y[3];
  const Scalar _tmp1 = -_x[0] * _y[1] + _x[1] * _y[0] - _x[2] * _y[3] + _x[3] * _y[2];
  const Scalar _tmp2 = -_x[0] * _y[3] - _x[1] * _y[2] + _x[2] * _y[1] + _x[3] * _y[0];
  const Scalar _tmp3 = _x[0] * _y[2] - _x[1] * _y[3] - _x[2] * _y[0] + _x[3] * _y[1];
  const Scalar _tmp4 = -_tmp1 * _x_T_y[2] - _tmp2 * _x_T_y[0] - _tmp3 * _x_T_y[1];
  const Scalar _tmp5 = _tmp0 * _x_T_y[3];
  const Scalar _tmp6 = std::min<Scalar>(1 - epsilon, std::fabs(_tmp4 - _tmp5));
  const Scalar _tmp7 =
      2 * (2 * std::min<Scalar>(0, (((-_tmp4 + _tmp5) > 0) - ((-_tmp4 + _tmp5) < 0))) + 1) *
      std::acos(_tmp6) / std::sqrt(Scalar(1 - std::pow(_tmp6, Scalar(2))));
  const Scalar _tmp8 =
      _tmp7 * (-_tmp0 * _x_T_y[0] - _tmp1 * _x_T_y[1] + _tmp2 * _x_T_y[3] + _tmp3 * _x_T_y[2]);
  const Scalar _tmp9 = 2 * _x[0];
  const Scalar _tmp10 = _tmp9 * _x[1];
  const Scalar _tmp11 = 2 * _x[2];
  const Scalar _tmp12 = _tmp11 * _x[3];
  const Scalar _tmp13 = _tmp10 - _tmp12;
  const Scalar _tmp14 = -2 * std::pow(_x[2], Scalar(2));
  const Scalar _tmp15 = 1 - 2 * std::pow(_x[0], Scalar(2));
  const Scalar _tmp16 = _tmp14 + _tmp15;
  const Scalar _tmp17 = _tmp9 * _x[3];
  const Scalar _tmp18 = _tmp11 * _x[1];
  const Scalar _tmp19 = _tmp17 + _tmp18;
  const Scalar _tmp20 = -_tmp13 * _x[4] + _tmp13 * _y[4] - _tmp16 * _x[5] + _tmp16 * _y[5] -
                        _tmp19 * _x[6] + _tmp19 * _y[6] - _x_T_y[5];
  const Scalar _tmp21 =
      _tmp7 * (-_tmp0 * _x_T_y[1] + _tmp1 * _x_T_y[0] - _tmp2 * _x_T_y[2] + _tmp3 * _x_T_y[3]);
  const Scalar _tmp22 = _tmp11 * _x[0];
  const Scalar _tmp23 = 2 * _x[1] * _x[3];
  const Scalar _tmp24 = _tmp22 + _tmp23;
  const Scalar _tmp25 = -2 * std::pow(_x[1], Scalar(2));
  const Scalar _tmp26 = _tmp15 + _tmp25;
  const Scalar _tmp27 = -_tmp17 + _tmp18;
  const Scalar _tmp28 = -_tmp24 * _x[4] + _tmp24 * _y[4] - _tmp26 * _x[6] + _tmp26 * _y[6] -
                        _tmp27 * _x[5] + _tmp27 * _y[5] - _x_T_y[6];
  const Scalar _tmp29 = _tmp10 + _tmp12;
  const Scalar _tmp30 = _tmp22 - _tmp23;
  const Scalar _tmp31 = _tmp14 + _tmp25 + 1;
  const Scalar _tmp32 = -_tmp29 * _x[5] + _tmp29 * _y[5] - _tmp30 * _x[6] + _tmp30 * _y[6] -
                        _tmp31 * _x[4] + _tmp31 * _y[4] - _x_T_y[4];
  const Scalar _tmp33 =
      _tmp7 * (-_tmp0 * _x_T_y[2] + _tmp1 * _x_T_y[3] + _tmp2 * _x_T_y[1] - _tmp3 * _x_T_y[0]);

  // Output terms (1)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = _tmp20 * sqrt_info(0, 4) + _tmp21 * sqrt_info(0, 1) + _tmp28 * sqrt_info(0, 5) +
               _tmp32 * sqrt_info(0, 3) + _tmp33 * sqrt_info(0, 2) + _tmp8 * sqrt_info(0, 0);
  _res(1, 0) = _tmp20 * sqrt_info(1, 4) + _tmp21 * sqrt_info(1, 1) + _tmp28 * sqrt_info(1, 5) +
               _tmp32 * sqrt_info(1, 3) + _tmp33 * sqrt_info(1, 2) + _tmp8 * sqrt_info(1, 0);
  _res(2, 0) = _tmp20 * sqrt_info(2, 4) + _tmp21 * sqrt_info(2, 1) + _tmp28 * sqrt_info(2, 5) +
               _tmp32 * sqrt_info(2, 3) + _tmp33 * sqrt_info(2, 2) + _tmp8 * sqrt_info(2, 0);
  _res(3, 0) = _tmp20 * sqrt_info(3, 4) + _tmp21 * sqrt_info(3, 1) + _tmp28 * sqrt_info(3, 5) +
               _tmp32 * sqrt_info(3, 3) + _tmp33 * sqrt_info(3, 2) + _tmp8 * sqrt_info(3, 0);
  _res(4, 0) = _tmp20 * sqrt_info(4, 4) + _tmp21 * sqrt_info(4, 1) + _tmp28 * sqrt_info(4, 5) +
               _tmp32 * sqrt_info(4, 3) + _tmp33 * sqrt_info(4, 2) + _tmp8 * sqrt_info(4, 0);
  _res(5, 0) = _tmp20 * sqrt_info(5, 4) + _tmp21 * sqrt_info(5, 1) + _tmp28 * sqrt_info(5, 5) +
               _tmp32 * sqrt_info(5, 3) + _tmp33 * sqrt_info(5, 2) + _tmp8 * sqrt_info(5, 0);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
