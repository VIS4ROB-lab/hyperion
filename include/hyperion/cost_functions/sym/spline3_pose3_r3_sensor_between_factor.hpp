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
 * Symbolic function: spline3_pose3_r3_sensor_between_factor
 *
 * Args:
 *     lambdas: Matrix31
 *     x0: Pose3
 *     x1: Pose3
 *     x2: Pose3
 *     x3: Pose3
 *     x_T_s: Pose3
 *     s_T_y: Matrix31
 *     y: Pose3
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline3Pose3R3SensorBetweenFactor(
    const Eigen::Matrix<Scalar, 3, 1>& lambdas, const sym::Pose3<Scalar>& x0,
    const sym::Pose3<Scalar>& x1, const sym::Pose3<Scalar>& x2, const sym::Pose3<Scalar>& x3,
    const sym::Pose3<Scalar>& x_T_s, const Eigen::Matrix<Scalar, 3, 1>& s_T_y,
    const sym::Pose3<Scalar>& y, const Eigen::Matrix<Scalar, 3, 3>& sqrt_info,
    const Scalar epsilon) {
  // Total ops: 479

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_s = x_T_s.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (114)
  const Scalar _tmp0 = -_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2];
  const Scalar _tmp1 = 1 - epsilon;
  const Scalar _tmp2 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp3 = _x2[3] * _x3[3];
  const Scalar _tmp4 = std::min<Scalar>(_tmp1, std::fabs(_tmp2 - _tmp3));
  const Scalar _tmp5 = 1 - std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 =
      2 * std::min<Scalar>(0, (((-_tmp2 + _tmp3) > 0) - ((-_tmp2 + _tmp3) < 0))) + 1;
  const Scalar _tmp7 = std::acos(_tmp4);
  const Scalar _tmp8 = 4 * std::pow(_tmp6, Scalar(2)) * std::pow(_tmp7, Scalar(2)) *
                       std::pow(lambdas(2, 0), Scalar(2)) / _tmp5;
  const Scalar _tmp9 = _x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1];
  const Scalar _tmp10 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp11 = -_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0];
  const Scalar _tmp12 =
      std::sqrt(Scalar(std::pow(_tmp0, Scalar(2)) * _tmp8 + _tmp10 +
                       std::pow(_tmp11, Scalar(2)) * _tmp8 + _tmp8 * std::pow(_tmp9, Scalar(2))));
  const Scalar _tmp13 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp14 =
      _tmp6 * _tmp7 * lambdas(2, 0) * std::sin(_tmp13) / (_tmp12 * std::sqrt(_tmp5));
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp17 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp18 = _x1[3] * _x2[3];
  const Scalar _tmp19 = std::min<Scalar>(_tmp1, std::fabs(_tmp17 - _tmp18));
  const Scalar _tmp20 = 1 - std::pow(_tmp19, Scalar(2));
  const Scalar _tmp21 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp22 =
      2 * std::min<Scalar>(0, (((-_tmp17 + _tmp18) > 0) - ((-_tmp17 + _tmp18) < 0))) + 1;
  const Scalar _tmp23 = std::acos(_tmp19);
  const Scalar _tmp24 = 4 * std::pow(_tmp22, Scalar(2)) * std::pow(_tmp23, Scalar(2)) *
                        std::pow(lambdas(1, 0), Scalar(2)) / _tmp20;
  const Scalar _tmp25 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp26 = std::sqrt(Scalar(_tmp10 + std::pow(_tmp16, Scalar(2)) * _tmp24 +
                                         std::pow(_tmp21, Scalar(2)) * _tmp24 +
                                         _tmp24 * std::pow(_tmp25, Scalar(2))));
  const Scalar _tmp27 = (Scalar(1) / Scalar(2)) * _tmp26;
  const Scalar _tmp28 =
      _tmp22 * _tmp23 * lambdas(1, 0) * std::sin(_tmp27) / (std::sqrt(_tmp20) * _tmp26);
  const Scalar _tmp29 = 4 * _tmp28;
  const Scalar _tmp30 = _tmp16 * _tmp29;
  const Scalar _tmp31 = _tmp21 * _tmp29;
  const Scalar _tmp32 = _tmp14 * _tmp9;
  const Scalar _tmp33 = _tmp11 * _tmp14;
  const Scalar _tmp34 = _tmp25 * _tmp28;
  const Scalar _tmp35 = 4 * _tmp34;
  const Scalar _tmp36 = std::cos(_tmp27);
  const Scalar _tmp37 = std::cos(_tmp13);
  const Scalar _tmp38 = -_tmp15 * _tmp30 - _tmp31 * _tmp32 - _tmp33 * _tmp35 + _tmp36 * _tmp37;
  const Scalar _tmp39 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp40 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp41 = _x0[3] * _x1[3];
  const Scalar _tmp42 =
      2 * std::min<Scalar>(0, (((-_tmp40 + _tmp41) > 0) - ((-_tmp40 + _tmp41) < 0))) + 1;
  const Scalar _tmp43 = std::min<Scalar>(_tmp1, std::fabs(_tmp40 - _tmp41));
  const Scalar _tmp44 = std::acos(_tmp43);
  const Scalar _tmp45 = 1 - std::pow(_tmp43, Scalar(2));
  const Scalar _tmp46 = 4 * std::pow(_tmp42, Scalar(2)) * std::pow(_tmp44, Scalar(2)) *
                        std::pow(lambdas(0, 0), Scalar(2)) / _tmp45;
  const Scalar _tmp47 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp48 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp49 = std::sqrt(Scalar(_tmp10 + std::pow(_tmp39, Scalar(2)) * _tmp46 +
                                         _tmp46 * std::pow(_tmp47, Scalar(2)) +
                                         _tmp46 * std::pow(_tmp48, Scalar(2))));
  const Scalar _tmp50 = (Scalar(1) / Scalar(2)) * _tmp49;
  const Scalar _tmp51 = std::cos(_tmp50);
  const Scalar _tmp52 = 2 * _tmp37;
  const Scalar _tmp53 = 2 * _tmp36;
  const Scalar _tmp54 = _tmp15 * _tmp31 - _tmp30 * _tmp32 + _tmp33 * _tmp53 + _tmp34 * _tmp52;
  const Scalar _tmp55 =
      2 * _tmp42 * _tmp44 * lambdas(0, 0) * std::sin(_tmp50) / (std::sqrt(_tmp45) * _tmp49);
  const Scalar _tmp56 = _tmp48 * _tmp55;
  const Scalar _tmp57 = _tmp14 * _tmp53;
  const Scalar _tmp58 = _tmp28 * _tmp52;
  const Scalar _tmp59 = -_tmp15 * _tmp35 + _tmp21 * _tmp58 + _tmp30 * _tmp33 + _tmp57 * _tmp9;
  const Scalar _tmp60 = _tmp55 * _tmp59;
  const Scalar _tmp61 = _tmp0 * _tmp57 + _tmp16 * _tmp58 - _tmp31 * _tmp33 + _tmp32 * _tmp35;
  const Scalar _tmp62 = _tmp55 * _tmp61;
  const Scalar _tmp63 = _tmp38 * _tmp51 - _tmp39 * _tmp62 - _tmp47 * _tmp60 - _tmp54 * _tmp56;
  const Scalar _tmp64 = _tmp38 * _tmp55;
  const Scalar _tmp65 = _tmp54 * _tmp55;
  const Scalar _tmp66 = _tmp39 * _tmp65 + _tmp47 * _tmp64 + _tmp51 * _tmp59 - _tmp56 * _tmp61;
  const Scalar _tmp67 = _tmp39 * _tmp64 - _tmp47 * _tmp65 + _tmp51 * _tmp61 + _tmp56 * _tmp59;
  const Scalar _tmp68 = _tmp38 * _tmp56 - _tmp39 * _tmp60 + _tmp47 * _tmp62 + _tmp51 * _tmp54;
  const Scalar _tmp69 = _tmp63 * _x0[3] - _tmp66 * _x0[1] - _tmp67 * _x0[2] - _tmp68 * _x0[0];
  const Scalar _tmp70 = _tmp63 * _x0[2] + _tmp66 * _x0[0] + _tmp67 * _x0[3] - _tmp68 * _x0[1];
  const Scalar _tmp71 = _tmp63 * _x0[1] + _tmp66 * _x0[3] - _tmp67 * _x0[0] + _tmp68 * _x0[2];
  const Scalar _tmp72 = _tmp63 * _x0[0] - _tmp66 * _x0[2] + _tmp67 * _x0[1] + _tmp68 * _x0[3];
  const Scalar _tmp73 =
      _tmp69 * _x_T_s[2] + _tmp70 * _x_T_s[3] - _tmp71 * _x_T_s[0] + _tmp72 * _x_T_s[1];
  const Scalar _tmp74 =
      _tmp69 * _x_T_s[0] - _tmp70 * _x_T_s[1] + _tmp71 * _x_T_s[2] + _tmp72 * _x_T_s[3];
  const Scalar _tmp75 = 2 * _tmp74;
  const Scalar _tmp76 = _tmp73 * _tmp75;
  const Scalar _tmp77 =
      _tmp69 * _x_T_s[1] + _tmp70 * _x_T_s[0] + _tmp71 * _x_T_s[3] - _tmp72 * _x_T_s[2];
  const Scalar _tmp78 =
      _tmp69 * _x_T_s[3] - _tmp70 * _x_T_s[2] - _tmp71 * _x_T_s[1] - _tmp72 * _x_T_s[0];
  const Scalar _tmp79 = 2 * _tmp77 * _tmp78;
  const Scalar _tmp80 = _tmp76 - _tmp79;
  const Scalar _tmp81 = -2 * std::pow(_tmp77, Scalar(2));
  const Scalar _tmp82 = -2 * std::pow(_tmp73, Scalar(2));
  const Scalar _tmp83 = _tmp81 + _tmp82 + 1;
  const Scalar _tmp84 = _tmp75 * _tmp77;
  const Scalar _tmp85 = 2 * _tmp73;
  const Scalar _tmp86 = _tmp78 * _tmp85;
  const Scalar _tmp87 = _tmp84 + _tmp86;
  const Scalar _tmp88 = 2 * _tmp70;
  const Scalar _tmp89 = _tmp72 * _tmp88;
  const Scalar _tmp90 = 2 * _tmp69;
  const Scalar _tmp91 = _tmp71 * _tmp90;
  const Scalar _tmp92 = 2 * _tmp71 * _tmp72;
  const Scalar _tmp93 = _tmp69 * _tmp88;
  const Scalar _tmp94 = -2 * std::pow(_tmp70, Scalar(2));
  const Scalar _tmp95 = 1 - 2 * std::pow(_tmp71, Scalar(2));
  const Scalar _tmp96 = _x0[4] + _x_T_s[4] * (_tmp94 + _tmp95) + _x_T_s[5] * (_tmp92 - _tmp93) +
                        _x_T_s[6] * (_tmp89 + _tmp91) + lambdas(0, 0) * (-_x0[4] + _x1[4]) +
                        lambdas(1, 0) * (-_x1[4] + _x2[4]) + lambdas(2, 0) * (-_x2[4] + _x3[4]);
  const Scalar _tmp97 = _tmp71 * _tmp88;
  const Scalar _tmp98 = _tmp72 * _tmp90;
  const Scalar _tmp99 = -2 * std::pow(_tmp72, Scalar(2));
  const Scalar _tmp100 = _x0[6] + _x_T_s[4] * (_tmp89 - _tmp91) + _x_T_s[5] * (_tmp97 + _tmp98) +
                         _x_T_s[6] * (_tmp95 + _tmp99) + lambdas(0, 0) * (-_x0[6] + _x1[6]) +
                         lambdas(1, 0) * (-_x1[6] + _x2[6]) + lambdas(2, 0) * (-_x2[6] + _x3[6]);
  const Scalar _tmp101 = _x0[5] + _x_T_s[4] * (_tmp92 + _tmp93) +
                         _x_T_s[5] * (_tmp94 + _tmp99 + 1) + _x_T_s[6] * (_tmp97 - _tmp98) +
                         lambdas(0, 0) * (-_x0[5] + _x1[5]) + lambdas(1, 0) * (-_x1[5] + _x2[5]) +
                         lambdas(2, 0) * (-_x2[5] + _x3[5]);
  const Scalar _tmp102 = -_tmp100 * _tmp80 - _tmp101 * _tmp87 + _tmp80 * _y[6] - _tmp83 * _tmp96 +
                         _tmp83 * _y[4] + _tmp87 * _y[5] - s_T_y(0, 0);
  const Scalar _tmp103 = 1 - 2 * std::pow(_tmp74, Scalar(2));
  const Scalar _tmp104 = _tmp103 + _tmp81;
  const Scalar _tmp105 = _tmp76 + _tmp79;
  const Scalar _tmp106 = _tmp77 * _tmp85;
  const Scalar _tmp107 = _tmp75 * _tmp78;
  const Scalar _tmp108 = _tmp106 - _tmp107;
  const Scalar _tmp109 = -_tmp100 * _tmp104 - _tmp101 * _tmp108 + _tmp104 * _y[6] -
                         _tmp105 * _tmp96 + _tmp105 * _y[4] + _tmp108 * _y[5] - s_T_y(2, 0);
  const Scalar _tmp110 = _tmp106 + _tmp107;
  const Scalar _tmp111 = _tmp103 + _tmp82;
  const Scalar _tmp112 = _tmp84 - _tmp86;
  const Scalar _tmp113 = -_tmp100 * _tmp110 - _tmp101 * _tmp111 + _tmp110 * _y[6] +
                         _tmp111 * _y[5] - _tmp112 * _tmp96 + _tmp112 * _y[4] - s_T_y(1, 0);

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp102 * sqrt_info(0, 0) + _tmp109 * sqrt_info(0, 2) + _tmp113 * sqrt_info(0, 1);
  _res(1, 0) = _tmp102 * sqrt_info(1, 0) + _tmp109 * sqrt_info(1, 2) + _tmp113 * sqrt_info(1, 1);
  _res(2, 0) = _tmp102 * sqrt_info(2, 0) + _tmp109 * sqrt_info(2, 2) + _tmp113 * sqrt_info(2, 1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
