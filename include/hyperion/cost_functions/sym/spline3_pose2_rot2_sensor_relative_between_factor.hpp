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
 * Symbolic function: spline3_pose2_rot2_sensor_relative_between_factor
 *
 * Args:
 *     lambdas_x: Matrix31
 *     x0: Pose2
 *     x1: Pose2
 *     x2: Pose2
 *     x3: Pose2
 *     x_T_a: Pose2
 *     a_T_b: Rot2
 *     lambdas_y: Matrix31
 *     y0: Pose2
 *     y1: Pose2
 *     y2: Pose2
 *     y3: Pose2
 *     y_T_b: Pose2
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Spline3Pose2Rot2SensorRelativeBetweenFactor(
    const Eigen::Matrix<Scalar, 3, 1>& lambdas_x, const sym::Pose2<Scalar>& x0,
    const sym::Pose2<Scalar>& x1, const sym::Pose2<Scalar>& x2, const sym::Pose2<Scalar>& x3,
    const sym::Pose2<Scalar>& x_T_a, const sym::Rot2<Scalar>& a_T_b,
    const Eigen::Matrix<Scalar, 3, 1>& lambdas_y, const sym::Pose2<Scalar>& y0,
    const sym::Pose2<Scalar>& y1, const sym::Pose2<Scalar>& y2, const sym::Pose2<Scalar>& y3,
    const sym::Pose2<Scalar>& y_T_b, const Eigen::Matrix<Scalar, 1, 1>& sqrt_info,
    const Scalar epsilon) {
  // Total ops: 150

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x_T_a = x_T_a.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _a_T_b = a_T_b.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y0 = y0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y1 = y1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y2 = y2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y3 = y3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y_T_b = y_T_b.Data();

  // Intermediate terms (43)
  const Scalar _tmp0 = _x2[0] * _x3[0] + _x2[1] * _x3[1];
  const Scalar _tmp1 =
      lambdas_x(2, 0) *
      std::atan2(_x2[0] * _x3[1] - _x2[1] * _x3[0],
                 _tmp0 + epsilon * ((((_tmp0) > 0) - ((_tmp0) < 0)) + Scalar(0.5)));
  const Scalar _tmp2 = std::cos(_tmp1);
  const Scalar _tmp3 = _x1[0] * _x2[0] + _x1[1] * _x2[1];
  const Scalar _tmp4 =
      lambdas_x(1, 0) *
      std::atan2(_x1[0] * _x2[1] - _x1[1] * _x2[0],
                 _tmp3 + epsilon * ((((_tmp3) > 0) - ((_tmp3) < 0)) + Scalar(0.5)));
  const Scalar _tmp5 = std::cos(_tmp4);
  const Scalar _tmp6 = std::sin(_tmp1);
  const Scalar _tmp7 = std::sin(_tmp4);
  const Scalar _tmp8 = _tmp2 * _tmp5 - _tmp6 * _tmp7;
  const Scalar _tmp9 = _x0[0] * _x1[0] + _x0[1] * _x1[1];
  const Scalar _tmp10 =
      lambdas_x(0, 0) *
      std::atan2(_x0[0] * _x1[1] - _x0[1] * _x1[0],
                 _tmp9 + epsilon * ((((_tmp9) > 0) - ((_tmp9) < 0)) + Scalar(0.5)));
  const Scalar _tmp11 = std::cos(_tmp10);
  const Scalar _tmp12 = _tmp2 * _tmp7 + _tmp5 * _tmp6;
  const Scalar _tmp13 = std::sin(_tmp10);
  const Scalar _tmp14 = _tmp11 * _tmp8 - _tmp12 * _tmp13;
  const Scalar _tmp15 = _tmp11 * _tmp12 + _tmp13 * _tmp8;
  const Scalar _tmp16 = _tmp14 * _x0[1] + _tmp15 * _x0[0];
  const Scalar _tmp17 = _tmp14 * _x0[0] - _tmp15 * _x0[1];
  const Scalar _tmp18 = _tmp16 * _x_T_a[0] + _tmp17 * _x_T_a[1];
  const Scalar _tmp19 = _y1[0] * _y2[0] + _y1[1] * _y2[1];
  const Scalar _tmp20 =
      lambdas_y(1, 0) *
      std::atan2(_y1[0] * _y2[1] - _y1[1] * _y2[0],
                 _tmp19 + epsilon * ((((_tmp19) > 0) - ((_tmp19) < 0)) + Scalar(0.5)));
  const Scalar _tmp21 = std::sin(_tmp20);
  const Scalar _tmp22 = _y2[0] * _y3[0] + _y2[1] * _y3[1];
  const Scalar _tmp23 =
      lambdas_y(2, 0) *
      std::atan2(_y2[0] * _y3[1] - _y2[1] * _y3[0],
                 _tmp22 + epsilon * ((((_tmp22) > 0) - ((_tmp22) < 0)) + Scalar(0.5)));
  const Scalar _tmp24 = std::cos(_tmp23);
  const Scalar _tmp25 = std::cos(_tmp20);
  const Scalar _tmp26 = std::sin(_tmp23);
  const Scalar _tmp27 = _tmp21 * _tmp24 + _tmp25 * _tmp26;
  const Scalar _tmp28 = _y0[0] * _y1[0] + _y0[1] * _y1[1];
  const Scalar _tmp29 =
      lambdas_y(0, 0) *
      std::atan2(_y0[0] * _y1[1] - _y0[1] * _y1[0],
                 _tmp28 + epsilon * ((((_tmp28) > 0) - ((_tmp28) < 0)) + Scalar(0.5)));
  const Scalar _tmp30 = std::sin(_tmp29);
  const Scalar _tmp31 = -_tmp21 * _tmp26 + _tmp24 * _tmp25;
  const Scalar _tmp32 = std::cos(_tmp29);
  const Scalar _tmp33 = -_tmp27 * _tmp30 + _tmp31 * _tmp32;
  const Scalar _tmp34 = _tmp27 * _tmp32 + _tmp30 * _tmp31;
  const Scalar _tmp35 = _tmp33 * _y0[1] + _tmp34 * _y0[0];
  const Scalar _tmp36 = _tmp33 * _y0[0] - _tmp34 * _y0[1];
  const Scalar _tmp37 = -_tmp35 * _y_T_b[1] + _tmp36 * _y_T_b[0];
  const Scalar _tmp38 = -_tmp16 * _x_T_a[1] + _tmp17 * _x_T_a[0];
  const Scalar _tmp39 = _tmp35 * _y_T_b[0] + _tmp36 * _y_T_b[1];
  const Scalar _tmp40 = -_tmp18 * _tmp37 + _tmp38 * _tmp39;
  const Scalar _tmp41 = _tmp18 * _tmp39 + _tmp37 * _tmp38;
  const Scalar _tmp42 = _a_T_b[0] * _tmp41 + _a_T_b[1] * _tmp40;

  // Output terms (1)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) *
               std::atan2(_a_T_b[0] * _tmp40 - _a_T_b[1] * _tmp41,
                          _tmp42 + epsilon * ((((_tmp42) > 0) - ((_tmp42) < 0)) + Scalar(0.5)));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
