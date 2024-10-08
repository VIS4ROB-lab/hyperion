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
 * Symbolic function: spline3_pose3_sensor_relative_between_factor
 *
 * Args:
 *     lambdas_x: Matrix31
 *     x0: Pose3
 *     x1: Pose3
 *     x2: Pose3
 *     x3: Pose3
 *     x_T_a: Pose3
 *     a_T_b: Pose3
 *     lambdas_y: Matrix31
 *     y0: Pose3
 *     y1: Pose3
 *     y2: Pose3
 *     y3: Pose3
 *     y_T_b: Pose3
 *     sqrt_info: Matrix66
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix61
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> Spline3Pose3SensorRelativeBetweenFactor(
    const Eigen::Matrix<Scalar, 3, 1>& lambdas_x, const sym::Pose3<Scalar>& x0,
    const sym::Pose3<Scalar>& x1, const sym::Pose3<Scalar>& x2, const sym::Pose3<Scalar>& x3,
    const sym::Pose3<Scalar>& x_T_a, const sym::Pose3<Scalar>& a_T_b,
    const Eigen::Matrix<Scalar, 3, 1>& lambdas_y, const sym::Pose3<Scalar>& y0,
    const sym::Pose3<Scalar>& y1, const sym::Pose3<Scalar>& y2, const sym::Pose3<Scalar>& y3,
    const sym::Pose3<Scalar>& y_T_b, const Eigen::Matrix<Scalar, 6, 6>& sqrt_info,
    const Scalar epsilon) {
  // Total ops: 999

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_a = x_T_a.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _a_T_b = a_T_b.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y0 = y0.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y1 = y1.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y2 = y2.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y3 = y3.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y_T_b = y_T_b.Data();

  // Intermediate terms (212)
  const Scalar _tmp0 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp1 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp2 = 1 - epsilon;
  const Scalar _tmp3 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp4 = _x1[3] * _x2[3];
  const Scalar _tmp5 = std::min<Scalar>(_tmp2, std::fabs(_tmp3 - _tmp4));
  const Scalar _tmp6 = 1 - std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 =
      2 * std::min<Scalar>(0, (((-_tmp3 + _tmp4) > 0) - ((-_tmp3 + _tmp4) < 0))) + 1;
  const Scalar _tmp8 = std::acos(_tmp5);
  const Scalar _tmp9 = 4 * std::pow(_tmp7, Scalar(2)) * std::pow(_tmp8, Scalar(2)) *
                       std::pow(lambdas_x(1, 0), Scalar(2)) / _tmp6;
  const Scalar _tmp10 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp11 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp12 =
      std::sqrt(Scalar(std::pow(_tmp0, Scalar(2)) * _tmp9 + std::pow(_tmp1, Scalar(2)) * _tmp9 +
                       std::pow(_tmp10, Scalar(2)) * _tmp9 + _tmp11));
  const Scalar _tmp13 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp14 =
      _tmp7 * _tmp8 * lambdas_x(1, 0) * std::sin(_tmp13) / (_tmp12 * std::sqrt(_tmp6));
  const Scalar _tmp15 = _x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1];
  const Scalar _tmp16 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp17 = _x2[3] * _x3[3];
  const Scalar _tmp18 = std::min<Scalar>(_tmp2, std::fabs(_tmp16 - _tmp17));
  const Scalar _tmp19 = 1 - std::pow(_tmp18, Scalar(2));
  const Scalar _tmp20 =
      2 * std::min<Scalar>(0, (((-_tmp16 + _tmp17) > 0) - ((-_tmp16 + _tmp17) < 0))) + 1;
  const Scalar _tmp21 = std::acos(_tmp18);
  const Scalar _tmp22 = 4 * std::pow(_tmp20, Scalar(2)) * std::pow(_tmp21, Scalar(2)) *
                        std::pow(lambdas_x(2, 0), Scalar(2)) / _tmp19;
  const Scalar _tmp23 = -_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0];
  const Scalar _tmp24 = -_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2];
  const Scalar _tmp25 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp15, Scalar(2)) * _tmp22 +
                                         _tmp22 * std::pow(_tmp23, Scalar(2)) +
                                         _tmp22 * std::pow(_tmp24, Scalar(2))));
  const Scalar _tmp26 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp27 = std::cos(_tmp26);
  const Scalar _tmp28 = 2 * _tmp27;
  const Scalar _tmp29 = _tmp14 * _tmp28;
  const Scalar _tmp30 =
      _tmp20 * _tmp21 * lambdas_x(2, 0) * std::sin(_tmp26) / (std::sqrt(_tmp19) * _tmp25);
  const Scalar _tmp31 = std::cos(_tmp13);
  const Scalar _tmp32 = 2 * _tmp31;
  const Scalar _tmp33 = _tmp30 * _tmp32;
  const Scalar _tmp34 = _tmp1 * _tmp14;
  const Scalar _tmp35 = 4 * _tmp30;
  const Scalar _tmp36 = _tmp34 * _tmp35;
  const Scalar _tmp37 = _tmp23 * _tmp30;
  const Scalar _tmp38 = 4 * _tmp37;
  const Scalar _tmp39 = _tmp10 * _tmp14;
  const Scalar _tmp40 = _tmp0 * _tmp29 + _tmp15 * _tmp33 - _tmp24 * _tmp36 + _tmp38 * _tmp39;
  const Scalar _tmp41 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp42 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp43 = _x0[3] * _x1[3];
  const Scalar _tmp44 =
      2 * std::min<Scalar>(0, (((-_tmp42 + _tmp43) > 0) - ((-_tmp42 + _tmp43) < 0))) + 1;
  const Scalar _tmp45 = std::min<Scalar>(_tmp2, std::fabs(_tmp42 - _tmp43));
  const Scalar _tmp46 = std::acos(_tmp45);
  const Scalar _tmp47 = 1 - std::pow(_tmp45, Scalar(2));
  const Scalar _tmp48 = 4 * std::pow(_tmp44, Scalar(2)) * std::pow(_tmp46, Scalar(2)) *
                        std::pow(lambdas_x(0, 0), Scalar(2)) / _tmp47;
  const Scalar _tmp49 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp50 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp51 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp41, Scalar(2)) * _tmp48 +
                                         _tmp48 * std::pow(_tmp49, Scalar(2)) +
                                         _tmp48 * std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) * _tmp51;
  const Scalar _tmp53 = std::cos(_tmp52);
  const Scalar _tmp54 = _tmp35 * _tmp39;
  const Scalar _tmp55 = _tmp0 * _tmp14;
  const Scalar _tmp56 = _tmp35 * _tmp55;
  const Scalar _tmp57 = -_tmp15 * _tmp54 + _tmp24 * _tmp56 + _tmp28 * _tmp34 + _tmp32 * _tmp37;
  const Scalar _tmp58 =
      2 * _tmp44 * _tmp46 * lambdas_x(0, 0) * std::sin(_tmp52) / (std::sqrt(_tmp47) * _tmp51);
  const Scalar _tmp59 = _tmp50 * _tmp58;
  const Scalar _tmp60 = _tmp10 * _tmp29 + _tmp15 * _tmp36 + _tmp24 * _tmp33 - _tmp38 * _tmp55;
  const Scalar _tmp61 = _tmp58 * _tmp60;
  const Scalar _tmp62 = -_tmp15 * _tmp56 - _tmp24 * _tmp54 + _tmp27 * _tmp31 - _tmp34 * _tmp38;
  const Scalar _tmp63 = _tmp58 * _tmp62;
  const Scalar _tmp64 = _tmp40 * _tmp53 - _tmp41 * _tmp61 + _tmp49 * _tmp63 + _tmp57 * _tmp59;
  const Scalar _tmp65 = -_tmp40 * _tmp59 + _tmp41 * _tmp63 + _tmp49 * _tmp61 + _tmp53 * _tmp57;
  const Scalar _tmp66 = _tmp49 * _tmp58;
  const Scalar _tmp67 = _tmp41 * _tmp58;
  const Scalar _tmp68 = _tmp40 * _tmp67 + _tmp50 * _tmp63 + _tmp53 * _tmp60 - _tmp57 * _tmp66;
  const Scalar _tmp69 = -_tmp40 * _tmp66 + _tmp53 * _tmp62 - _tmp57 * _tmp67 - _tmp59 * _tmp60;
  const Scalar _tmp70 = -_tmp64 * _x0[1] - _tmp65 * _x0[0] - _tmp68 * _x0[2] + _tmp69 * _x0[3];
  const Scalar _tmp71 = -_tmp64 * _x0[2] + _tmp65 * _x0[3] + _tmp68 * _x0[1] + _tmp69 * _x0[0];
  const Scalar _tmp72 = _tmp64 * _x0[0] - _tmp65 * _x0[1] + _tmp68 * _x0[3] + _tmp69 * _x0[2];
  const Scalar _tmp73 = _tmp64 * _x0[3] + _tmp65 * _x0[2] - _tmp68 * _x0[0] + _tmp69 * _x0[1];
  const Scalar _tmp74 =
      _tmp70 * _x_T_a[1] - _tmp71 * _x_T_a[2] + _tmp72 * _x_T_a[0] + _tmp73 * _x_T_a[3];
  const Scalar _tmp75 = -_y2[0] * _y3[1] + _y2[1] * _y3[0] - _y2[2] * _y3[3] + _y2[3] * _y3[2];
  const Scalar _tmp76 = -_y2[0] * _y3[0] - _y2[1] * _y3[1] - _y2[2] * _y3[2];
  const Scalar _tmp77 = _y2[3] * _y3[3];
  const Scalar _tmp78 =
      2 * std::min<Scalar>(0, (((-_tmp76 + _tmp77) > 0) - ((-_tmp76 + _tmp77) < 0))) + 1;
  const Scalar _tmp79 = std::min<Scalar>(_tmp2, std::fabs(_tmp76 - _tmp77));
  const Scalar _tmp80 = std::acos(_tmp79);
  const Scalar _tmp81 = -_y2[0] * _y3[3] - _y2[1] * _y3[2] + _y2[2] * _y3[1] + _y2[3] * _y3[0];
  const Scalar _tmp82 = 1 - std::pow(_tmp79, Scalar(2));
  const Scalar _tmp83 = 4 * std::pow(_tmp78, Scalar(2)) * std::pow(_tmp80, Scalar(2)) *
                        std::pow(lambdas_y(2, 0), Scalar(2)) / _tmp82;
  const Scalar _tmp84 = _y2[0] * _y3[2] - _y2[1] * _y3[3] - _y2[2] * _y3[0] + _y2[3] * _y3[1];
  const Scalar _tmp85 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp75, Scalar(2)) * _tmp83 +
                                         std::pow(_tmp81, Scalar(2)) * _tmp83 +
                                         _tmp83 * std::pow(_tmp84, Scalar(2))));
  const Scalar _tmp86 = (Scalar(1) / Scalar(2)) * _tmp85;
  const Scalar _tmp87 =
      _tmp78 * _tmp80 * lambdas_y(2, 0) * std::sin(_tmp86) / (std::sqrt(_tmp82) * _tmp85);
  const Scalar _tmp88 = _tmp75 * _tmp87;
  const Scalar _tmp89 = -_y1[0] * _y2[1] + _y1[1] * _y2[0] - _y1[2] * _y2[3] + _y1[3] * _y2[2];
  const Scalar _tmp90 = -_y1[0] * _y2[0] - _y1[1] * _y2[1] - _y1[2] * _y2[2];
  const Scalar _tmp91 = _y1[3] * _y2[3];
  const Scalar _tmp92 =
      2 * std::min<Scalar>(0, (((-_tmp90 + _tmp91) > 0) - ((-_tmp90 + _tmp91) < 0))) + 1;
  const Scalar _tmp93 = std::min<Scalar>(_tmp2, std::fabs(_tmp90 - _tmp91));
  const Scalar _tmp94 = 1 - std::pow(_tmp93, Scalar(2));
  const Scalar _tmp95 = std::acos(_tmp93);
  const Scalar _tmp96 = 4 * std::pow(_tmp92, Scalar(2)) * std::pow(_tmp95, Scalar(2)) *
                        std::pow(lambdas_y(1, 0), Scalar(2)) / _tmp94;
  const Scalar _tmp97 = -_y1[0] * _y2[3] - _y1[1] * _y2[2] + _y1[2] * _y2[1] + _y1[3] * _y2[0];
  const Scalar _tmp98 = _y1[0] * _y2[2] - _y1[1] * _y2[3] - _y1[2] * _y2[0] + _y1[3] * _y2[1];
  const Scalar _tmp99 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp89, Scalar(2)) * _tmp96 +
                                         _tmp96 * std::pow(_tmp97, Scalar(2)) +
                                         _tmp96 * std::pow(_tmp98, Scalar(2))));
  const Scalar _tmp100 = (Scalar(1) / Scalar(2)) * _tmp99;
  const Scalar _tmp101 =
      _tmp92 * _tmp95 * lambdas_y(1, 0) * std::sin(_tmp100) / (std::sqrt(_tmp94) * _tmp99);
  const Scalar _tmp102 = 4 * _tmp101;
  const Scalar _tmp103 = _tmp102 * _tmp89;
  const Scalar _tmp104 = _tmp81 * _tmp87;
  const Scalar _tmp105 = _tmp101 * _tmp97;
  const Scalar _tmp106 = 4 * _tmp105;
  const Scalar _tmp107 = _tmp102 * _tmp98;
  const Scalar _tmp108 = _tmp84 * _tmp87;
  const Scalar _tmp109 = std::cos(_tmp100);
  const Scalar _tmp110 = std::cos(_tmp86);
  const Scalar _tmp111 =
      -_tmp103 * _tmp88 - _tmp104 * _tmp106 - _tmp107 * _tmp108 + _tmp109 * _tmp110;
  const Scalar _tmp112 = -_y0[0] * _y1[3] - _y0[1] * _y1[2] + _y0[2] * _y1[1] + _y0[3] * _y1[0];
  const Scalar _tmp113 = -_y0[0] * _y1[1] + _y0[1] * _y1[0] - _y0[2] * _y1[3] + _y0[3] * _y1[2];
  const Scalar _tmp114 = _y0[0] * _y1[0] + _y0[1] * _y1[1] + _y0[2] * _y1[2] + _y0[3] * _y1[3];
  const Scalar _tmp115 = 2 * std::min<Scalar>(0, (((_tmp114) > 0) - ((_tmp114) < 0))) + 1;
  const Scalar _tmp116 = std::min<Scalar>(_tmp2, std::fabs(_tmp114));
  const Scalar _tmp117 = std::acos(_tmp116);
  const Scalar _tmp118 = 1 - std::pow(_tmp116, Scalar(2));
  const Scalar _tmp119 = 4 * std::pow(_tmp115, Scalar(2)) * std::pow(_tmp117, Scalar(2)) *
                         std::pow(lambdas_y(0, 0), Scalar(2)) / _tmp118;
  const Scalar _tmp120 = _y0[0] * _y1[2] - _y0[1] * _y1[3] - _y0[2] * _y1[0] + _y0[3] * _y1[1];
  const Scalar _tmp121 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp112, Scalar(2)) * _tmp119 +
                                          std::pow(_tmp113, Scalar(2)) * _tmp119 +
                                          _tmp119 * std::pow(_tmp120, Scalar(2))));
  const Scalar _tmp122 = (Scalar(1) / Scalar(2)) * _tmp121;
  const Scalar _tmp123 =
      2 * _tmp115 * _tmp117 * lambdas_y(0, 0) * std::sin(_tmp122) / (std::sqrt(_tmp118) * _tmp121);
  const Scalar _tmp124 = _tmp112 * _tmp123;
  const Scalar _tmp125 = 2 * _tmp110;
  const Scalar _tmp126 = 2 * _tmp109;
  const Scalar _tmp127 = _tmp126 * _tmp87;
  const Scalar _tmp128 =
      -_tmp103 * _tmp108 + _tmp105 * _tmp125 + _tmp107 * _tmp88 + _tmp127 * _tmp81;
  const Scalar _tmp129 = std::cos(_tmp122);
  const Scalar _tmp130 = _tmp101 * _tmp125;
  const Scalar _tmp131 =
      -_tmp104 * _tmp107 + _tmp106 * _tmp108 + _tmp126 * _tmp88 + _tmp130 * _tmp89;
  const Scalar _tmp132 = _tmp120 * _tmp123;
  const Scalar _tmp133 = _tmp103 * _tmp104 - _tmp106 * _tmp88 + _tmp127 * _tmp84 + _tmp130 * _tmp98;
  const Scalar _tmp134 = _tmp113 * _tmp123;
  const Scalar _tmp135 =
      _tmp111 * _tmp124 + _tmp128 * _tmp129 + _tmp131 * _tmp132 - _tmp133 * _tmp134;
  const Scalar _tmp136 =
      _tmp111 * _tmp132 - _tmp124 * _tmp131 + _tmp128 * _tmp134 + _tmp129 * _tmp133;
  const Scalar _tmp137 =
      _tmp111 * _tmp134 + _tmp124 * _tmp133 - _tmp128 * _tmp132 + _tmp129 * _tmp131;
  const Scalar _tmp138 =
      _tmp111 * _tmp129 - _tmp124 * _tmp128 - _tmp131 * _tmp134 - _tmp132 * _tmp133;
  const Scalar _tmp139 = -_tmp135 * _y0[0] - _tmp136 * _y0[1] - _tmp137 * _y0[2] + _tmp138 * _y0[3];
  const Scalar _tmp140 = _tmp135 * _y0[3] - _tmp136 * _y0[2] + _tmp137 * _y0[1] + _tmp138 * _y0[0];
  const Scalar _tmp141 = -_tmp135 * _y0[1] + _tmp136 * _y0[0] + _tmp137 * _y0[3] + _tmp138 * _y0[2];
  const Scalar _tmp142 = _tmp135 * _y0[2] + _tmp136 * _y0[3] - _tmp137 * _y0[0] + _tmp138 * _y0[1];
  const Scalar _tmp143 =
      _tmp139 * _y_T_b[2] + _tmp140 * _y_T_b[1] + _tmp141 * _y_T_b[3] - _tmp142 * _y_T_b[0];
  const Scalar _tmp144 =
      _tmp70 * _x_T_a[2] + _tmp71 * _x_T_a[1] + _tmp72 * _x_T_a[3] - _tmp73 * _x_T_a[0];
  const Scalar _tmp145 =
      _tmp139 * _y_T_b[1] - _tmp140 * _y_T_b[2] + _tmp141 * _y_T_b[0] + _tmp142 * _y_T_b[3];
  const Scalar _tmp146 =
      _tmp139 * _y_T_b[0] + _tmp140 * _y_T_b[3] - _tmp141 * _y_T_b[1] + _tmp142 * _y_T_b[2];
  const Scalar _tmp147 =
      _tmp70 * _x_T_a[3] - _tmp71 * _x_T_a[0] - _tmp72 * _x_T_a[2] - _tmp73 * _x_T_a[1];
  const Scalar _tmp148 =
      _tmp70 * _x_T_a[0] + _tmp71 * _x_T_a[3] - _tmp72 * _x_T_a[1] + _tmp73 * _x_T_a[2];
  const Scalar _tmp149 =
      _tmp139 * _y_T_b[3] - _tmp140 * _y_T_b[0] - _tmp141 * _y_T_b[2] - _tmp142 * _y_T_b[1];
  const Scalar _tmp150 =
      -_tmp143 * _tmp74 + _tmp144 * _tmp145 + _tmp146 * _tmp147 - _tmp148 * _tmp149;
  const Scalar _tmp151 =
      _tmp143 * _tmp147 - _tmp144 * _tmp149 - _tmp145 * _tmp148 + _tmp146 * _tmp74;
  const Scalar _tmp152 =
      _tmp143 * _tmp144 + _tmp145 * _tmp74 + _tmp146 * _tmp148 + _tmp147 * _tmp149;
  const Scalar _tmp153 =
      _tmp143 * _tmp148 - _tmp144 * _tmp146 + _tmp145 * _tmp147 - _tmp149 * _tmp74;
  const Scalar _tmp154 = -_a_T_b[0] * _tmp150 - _a_T_b[1] * _tmp153 - _a_T_b[2] * _tmp151;
  const Scalar _tmp155 = _a_T_b[3] * _tmp152;
  const Scalar _tmp156 = std::min<Scalar>(_tmp2, std::fabs(_tmp154 - _tmp155));
  const Scalar _tmp157 =
      2 * (2 * std::min<Scalar>(0, (((-_tmp154 + _tmp155) > 0) - ((-_tmp154 + _tmp155) < 0))) + 1) *
      std::acos(_tmp156) / std::sqrt(Scalar(1 - std::pow(_tmp156, Scalar(2))));
  const Scalar _tmp158 = _tmp157 * (-_a_T_b[0] * _tmp152 - _a_T_b[1] * _tmp151 +
                                    _a_T_b[2] * _tmp153 + _a_T_b[3] * _tmp150);
  const Scalar _tmp159 = -2 * std::pow(_tmp144, Scalar(2));
  const Scalar _tmp160 = -2 * std::pow(_tmp148, Scalar(2));
  const Scalar _tmp161 = _tmp159 + _tmp160 + 1;
  const Scalar _tmp162 = 2 * _tmp141;
  const Scalar _tmp163 = _tmp142 * _tmp162;
  const Scalar _tmp164 = 2 * _tmp140;
  const Scalar _tmp165 = _tmp139 * _tmp164;
  const Scalar _tmp166 = _tmp142 * _tmp164;
  const Scalar _tmp167 = _tmp139 * _tmp162;
  const Scalar _tmp168 = -2 * std::pow(_tmp141, Scalar(2));
  const Scalar _tmp169 = 1 - 2 * std::pow(_tmp140, Scalar(2));
  const Scalar _tmp170 =
      _y0[5] + _y_T_b[4] * (_tmp166 + _tmp167) + _y_T_b[5] * (_tmp168 + _tmp169) +
      _y_T_b[6] * (_tmp163 - _tmp165) + lambdas_y(0, 0) * (-_y0[5] + _y1[5]) +
      lambdas_y(1, 0) * (-_y1[5] + _y2[5]) + lambdas_y(2, 0) * (-_y2[5] + _y3[5]);
  const Scalar _tmp171 = 2 * _tmp70;
  const Scalar _tmp172 = _tmp171 * _tmp73;
  const Scalar _tmp173 = 2 * _tmp71 * _tmp72;
  const Scalar _tmp174 = _tmp171 * _tmp71;
  const Scalar _tmp175 = 2 * _tmp73;
  const Scalar _tmp176 = _tmp175 * _tmp72;
  const Scalar _tmp177 = -2 * std::pow(_tmp71, Scalar(2));
  const Scalar _tmp178 = 1 - 2 * std::pow(_tmp73, Scalar(2));
  const Scalar _tmp179 =
      _x0[6] + _x_T_a[4] * (-_tmp172 + _tmp173) + _x_T_a[5] * (_tmp174 + _tmp176) +
      _x_T_a[6] * (_tmp177 + _tmp178) + lambdas_x(0, 0) * (-_x0[6] + _x1[6]) +
      lambdas_x(1, 0) * (-_x1[6] + _x2[6]) + lambdas_x(2, 0) * (-_x2[6] + _x3[6]);
  const Scalar _tmp180 = 2 * _tmp147;
  const Scalar _tmp181 = _tmp148 * _tmp180;
  const Scalar _tmp182 = 2 * _tmp144;
  const Scalar _tmp183 = _tmp182 * _tmp74;
  const Scalar _tmp184 = _tmp181 + _tmp183;
  const Scalar _tmp185 = _tmp171 * _tmp72;
  const Scalar _tmp186 = _tmp175 * _tmp71;
  const Scalar _tmp187 = -2 * std::pow(_tmp72, Scalar(2));
  const Scalar _tmp188 =
      _x0[4] + _x_T_a[4] * (_tmp178 + _tmp187) + _x_T_a[5] * (-_tmp185 + _tmp186) +
      _x_T_a[6] * (_tmp172 + _tmp173) + lambdas_x(0, 0) * (-_x0[4] + _x1[4]) +
      lambdas_x(1, 0) * (-_x1[4] + _x2[4]) + lambdas_x(2, 0) * (-_x2[4] + _x3[4]);
  const Scalar _tmp189 = _tmp144 * _tmp180;
  const Scalar _tmp190 = 2 * _tmp148 * _tmp74;
  const Scalar _tmp191 = -_tmp189 + _tmp190;
  const Scalar _tmp192 =
      _x0[5] + _x_T_a[4] * (_tmp185 + _tmp186) + _x_T_a[5] * (_tmp177 + _tmp187 + 1) +
      _x_T_a[6] * (-_tmp174 + _tmp176) + lambdas_x(0, 0) * (-_x0[5] + _x1[5]) +
      lambdas_x(1, 0) * (-_x1[5] + _x2[5]) + lambdas_x(2, 0) * (-_x2[5] + _x3[5]);
  const Scalar _tmp193 = _tmp140 * _tmp162;
  const Scalar _tmp194 = 2 * _tmp139 * _tmp142;
  const Scalar _tmp195 = -2 * std::pow(_tmp142, Scalar(2));
  const Scalar _tmp196 =
      _y0[6] + _y_T_b[4] * (_tmp193 - _tmp194) + _y_T_b[5] * (_tmp163 + _tmp165) +
      _y_T_b[6] * (_tmp169 + _tmp195) + lambdas_y(0, 0) * (-_y0[6] + _y1[6]) +
      lambdas_y(1, 0) * (-_y1[6] + _y2[6]) + lambdas_y(2, 0) * (-_y2[6] + _y3[6]);
  const Scalar _tmp197 =
      _y0[4] + _y_T_b[4] * (_tmp168 + _tmp195 + 1) + _y_T_b[5] * (_tmp166 - _tmp167) +
      _y_T_b[6] * (_tmp193 + _tmp194) + lambdas_y(0, 0) * (-_y0[4] + _y1[4]) +
      lambdas_y(1, 0) * (-_y1[4] + _y2[4]) + lambdas_y(2, 0) * (-_y2[4] + _y3[4]);
  const Scalar _tmp198 = -_a_T_b[5] + _tmp161 * _tmp170 - _tmp161 * _tmp192 - _tmp179 * _tmp184 +
                         _tmp184 * _tmp196 - _tmp188 * _tmp191 + _tmp191 * _tmp197;
  const Scalar _tmp199 = _tmp157 * (-_a_T_b[0] * _tmp153 + _a_T_b[1] * _tmp150 -
                                    _a_T_b[2] * _tmp152 + _a_T_b[3] * _tmp151);
  const Scalar _tmp200 = _tmp157 * (_a_T_b[0] * _tmp151 - _a_T_b[1] * _tmp152 -
                                    _a_T_b[2] * _tmp150 + _a_T_b[3] * _tmp153);
  const Scalar _tmp201 = _tmp180 * _tmp74;
  const Scalar _tmp202 = _tmp148 * _tmp182;
  const Scalar _tmp203 = -_tmp201 + _tmp202;
  const Scalar _tmp204 = _tmp189 + _tmp190;
  const Scalar _tmp205 = 1 - 2 * std::pow(_tmp74, Scalar(2));
  const Scalar _tmp206 = _tmp159 + _tmp205;
  const Scalar _tmp207 = -_a_T_b[4] + _tmp170 * _tmp204 - _tmp179 * _tmp203 - _tmp188 * _tmp206 -
                         _tmp192 * _tmp204 + _tmp196 * _tmp203 + _tmp197 * _tmp206;
  const Scalar _tmp208 = -_tmp181 + _tmp183;
  const Scalar _tmp209 = _tmp201 + _tmp202;
  const Scalar _tmp210 = _tmp160 + _tmp205;
  const Scalar _tmp211 = -_a_T_b[6] + _tmp170 * _tmp208 - _tmp179 * _tmp210 - _tmp188 * _tmp209 -
                         _tmp192 * _tmp208 + _tmp196 * _tmp210 + _tmp197 * _tmp209;

  // Output terms (1)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = _tmp158 * sqrt_info(0, 0) + _tmp198 * sqrt_info(0, 4) + _tmp199 * sqrt_info(0, 2) +
               _tmp200 * sqrt_info(0, 1) + _tmp207 * sqrt_info(0, 3) + _tmp211 * sqrt_info(0, 5);
  _res(1, 0) = _tmp158 * sqrt_info(1, 0) + _tmp198 * sqrt_info(1, 4) + _tmp199 * sqrt_info(1, 2) +
               _tmp200 * sqrt_info(1, 1) + _tmp207 * sqrt_info(1, 3) + _tmp211 * sqrt_info(1, 5);
  _res(2, 0) = _tmp158 * sqrt_info(2, 0) + _tmp198 * sqrt_info(2, 4) + _tmp199 * sqrt_info(2, 2) +
               _tmp200 * sqrt_info(2, 1) + _tmp207 * sqrt_info(2, 3) + _tmp211 * sqrt_info(2, 5);
  _res(3, 0) = _tmp158 * sqrt_info(3, 0) + _tmp198 * sqrt_info(3, 4) + _tmp199 * sqrt_info(3, 2) +
               _tmp200 * sqrt_info(3, 1) + _tmp207 * sqrt_info(3, 3) + _tmp211 * sqrt_info(3, 5);
  _res(4, 0) = _tmp158 * sqrt_info(4, 0) + _tmp198 * sqrt_info(4, 4) + _tmp199 * sqrt_info(4, 2) +
               _tmp200 * sqrt_info(4, 1) + _tmp207 * sqrt_info(4, 3) + _tmp211 * sqrt_info(4, 5);
  _res(5, 0) = _tmp158 * sqrt_info(5, 0) + _tmp198 * sqrt_info(5, 4) + _tmp199 * sqrt_info(5, 2) +
               _tmp200 * sqrt_info(5, 1) + _tmp207 * sqrt_info(5, 3) + _tmp211 * sqrt_info(5, 5);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
