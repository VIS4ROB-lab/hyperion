// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose2.h>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline3_pose2_sensor_relative_between_factor
 *
 * Args:
 *     lambdas_x: Matrix31
 *     x0: Pose2
 *     x1: Pose2
 *     x2: Pose2
 *     x3: Pose2
 *     x_T_a: Pose2
 *     a_T_b: Pose2
 *     lambdas_y: Matrix31
 *     y0: Pose2
 *     y1: Pose2
 *     y2: Pose2
 *     y3: Pose2
 *     y_T_b: Pose2
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 *     res_D_x0: (3x3) jacobian of res (3) wrt arg x0 (3)
 *     res_D_x1: (3x3) jacobian of res (3) wrt arg x1 (3)
 *     res_D_x2: (3x3) jacobian of res (3) wrt arg x2 (3)
 *     res_D_x3: (3x3) jacobian of res (3) wrt arg x3 (3)
 *     res_D_x_T_a: (3x3) jacobian of res (3) wrt arg x_T_a (3)
 *     res_D_y0: (3x3) jacobian of res (3) wrt arg y0 (3)
 *     res_D_y1: (3x3) jacobian of res (3) wrt arg y1 (3)
 *     res_D_y2: (3x3) jacobian of res (3) wrt arg y2 (3)
 *     res_D_y3: (3x3) jacobian of res (3) wrt arg y3 (3)
 *     res_D_y_T_b: (3x3) jacobian of res (3) wrt arg y_T_b (3)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline3Pose2SensorRelativeBetweenFactorWithJacobians1234589101112(
    const Eigen::Matrix<Scalar, 3, 1>& lambdas_x, const sym::Pose2<Scalar>& x0,
    const sym::Pose2<Scalar>& x1, const sym::Pose2<Scalar>& x2, const sym::Pose2<Scalar>& x3,
    const sym::Pose2<Scalar>& x_T_a, const sym::Pose2<Scalar>& a_T_b,
    const Eigen::Matrix<Scalar, 3, 1>& lambdas_y, const sym::Pose2<Scalar>& y0,
    const sym::Pose2<Scalar>& y1, const sym::Pose2<Scalar>& y2, const sym::Pose2<Scalar>& y3,
    const sym::Pose2<Scalar>& y_T_b, const Eigen::Matrix<Scalar, 3, 3>& sqrt_info,
    const Scalar epsilon, Scalar* const res_D_x0 = nullptr, Scalar* const res_D_x1 = nullptr,
    Scalar* const res_D_x2 = nullptr, Scalar* const res_D_x3 = nullptr,
    Scalar* const res_D_x_T_a = nullptr, Scalar* const res_D_y0 = nullptr,
    Scalar* const res_D_y1 = nullptr, Scalar* const res_D_y2 = nullptr,
    Scalar* const res_D_y3 = nullptr, Scalar* const res_D_y_T_b = nullptr) {
  // Total ops: 1289

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x_T_a = x_T_a.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _a_T_b = a_T_b.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y0 = y0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y1 = y1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y2 = y2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y3 = y3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y_T_b = y_T_b.Data();

  // Intermediate terms (343)
  const Scalar _tmp0 = _x2[1] * _x3[0];
  const Scalar _tmp1 = _x2[0] * _x3[1];
  const Scalar _tmp2 = -_tmp0 + _tmp1;
  const Scalar _tmp3 = _x2[0] * _x3[0];
  const Scalar _tmp4 = _x2[1] * _x3[1];
  const Scalar _tmp5 = _tmp3 + _tmp4;
  const Scalar _tmp6 = _tmp5 + epsilon * ((((_tmp5) > 0) - ((_tmp5) < 0)) + Scalar(0.5));
  const Scalar _tmp7 = lambdas_x(2, 0) * std::atan2(_tmp2, _tmp6);
  const Scalar _tmp8 = std::cos(_tmp7);
  const Scalar _tmp9 = _x1[0] * _x2[1];
  const Scalar _tmp10 = _x1[1] * _x2[0];
  const Scalar _tmp11 = -_tmp10 + _tmp9;
  const Scalar _tmp12 = _x1[0] * _x2[0];
  const Scalar _tmp13 = _x1[1] * _x2[1];
  const Scalar _tmp14 = _tmp12 + _tmp13;
  const Scalar _tmp15 = _tmp14 + epsilon * ((((_tmp14) > 0) - ((_tmp14) < 0)) + Scalar(0.5));
  const Scalar _tmp16 = lambdas_x(1, 0) * std::atan2(_tmp11, _tmp15);
  const Scalar _tmp17 = std::cos(_tmp16);
  const Scalar _tmp18 = _tmp17 * _tmp8;
  const Scalar _tmp19 = std::sin(_tmp7);
  const Scalar _tmp20 = std::sin(_tmp16);
  const Scalar _tmp21 = _tmp19 * _tmp20;
  const Scalar _tmp22 = _tmp18 - _tmp21;
  const Scalar _tmp23 = _x0[1] * _x1[0];
  const Scalar _tmp24 = _x0[0] * _x1[1];
  const Scalar _tmp25 = -_tmp23 + _tmp24;
  const Scalar _tmp26 = _x0[1] * _x1[1];
  const Scalar _tmp27 = _x0[0] * _x1[0];
  const Scalar _tmp28 = _tmp26 + _tmp27;
  const Scalar _tmp29 = _tmp28 + epsilon * ((((_tmp28) > 0) - ((_tmp28) < 0)) + Scalar(0.5));
  const Scalar _tmp30 = lambdas_x(0, 0) * std::atan2(_tmp25, _tmp29);
  const Scalar _tmp31 = std::cos(_tmp30);
  const Scalar _tmp32 = _tmp22 * _tmp31;
  const Scalar _tmp33 = _tmp20 * _tmp8;
  const Scalar _tmp34 = _tmp17 * _tmp19;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = std::sin(_tmp30);
  const Scalar _tmp37 = _tmp35 * _tmp36;
  const Scalar _tmp38 = _tmp32 - _tmp37;
  const Scalar _tmp39 = _tmp38 * _x0[1];
  const Scalar _tmp40 = _tmp22 * _tmp36;
  const Scalar _tmp41 = _tmp31 * _tmp35;
  const Scalar _tmp42 = _tmp40 + _tmp41;
  const Scalar _tmp43 = _tmp42 * _x0[0];
  const Scalar _tmp44 = _tmp39 + _tmp43;
  const Scalar _tmp45 = _tmp38 * _x0[0] - _tmp42 * _x0[1];
  const Scalar _tmp46 = -_tmp44 * _x_T_a[1] + _tmp45 * _x_T_a[0];
  const Scalar _tmp47 = _tmp44 * _x_T_a[2] + _tmp45 * _x_T_a[3] + _x0[3] +
                        lambdas_x(0, 0) * (-_x0[3] + _x1[3]) +
                        lambdas_x(1, 0) * (-_x1[3] + _x2[3]) + lambdas_x(2, 0) * (-_x2[3] + _x3[3]);
  const Scalar _tmp48 = _tmp46 * _tmp47;
  const Scalar _tmp49 = _tmp44 * _x_T_a[0];
  const Scalar _tmp50 = _tmp45 * _x_T_a[1];
  const Scalar _tmp51 = _tmp49 + _tmp50;
  const Scalar _tmp52 = -_tmp44 * _x_T_a[3] + _tmp45 * _x_T_a[2] + _x0[2] +
                        lambdas_x(0, 0) * (-_x0[2] + _x1[2]) +
                        lambdas_x(1, 0) * (-_x1[2] + _x2[2]) + lambdas_x(2, 0) * (-_x2[2] + _x3[2]);
  const Scalar _tmp53 = _y1[0] * _y2[1];
  const Scalar _tmp54 = _y1[1] * _y2[0];
  const Scalar _tmp55 = _tmp53 - _tmp54;
  const Scalar _tmp56 = _y1[0] * _y2[0];
  const Scalar _tmp57 = _y1[1] * _y2[1];
  const Scalar _tmp58 = _tmp56 + _tmp57;
  const Scalar _tmp59 = _tmp58 + epsilon * ((((_tmp58) > 0) - ((_tmp58) < 0)) + Scalar(0.5));
  const Scalar _tmp60 = lambdas_y(1, 0) * std::atan2(_tmp55, _tmp59);
  const Scalar _tmp61 = std::sin(_tmp60);
  const Scalar _tmp62 = _y2[0] * _y3[1];
  const Scalar _tmp63 = _y2[1] * _y3[0];
  const Scalar _tmp64 = _tmp62 - _tmp63;
  const Scalar _tmp65 = _y2[1] * _y3[1];
  const Scalar _tmp66 = _y2[0] * _y3[0];
  const Scalar _tmp67 = _tmp65 + _tmp66;
  const Scalar _tmp68 = _tmp67 + epsilon * ((((_tmp67) > 0) - ((_tmp67) < 0)) + Scalar(0.5));
  const Scalar _tmp69 = lambdas_y(2, 0) * std::atan2(_tmp64, _tmp68);
  const Scalar _tmp70 = std::cos(_tmp69);
  const Scalar _tmp71 = _tmp61 * _tmp70;
  const Scalar _tmp72 = std::cos(_tmp60);
  const Scalar _tmp73 = std::sin(_tmp69);
  const Scalar _tmp74 = _tmp72 * _tmp73;
  const Scalar _tmp75 = _tmp71 + _tmp74;
  const Scalar _tmp76 = _y0[1] * _y1[0];
  const Scalar _tmp77 = _y0[0] * _y1[1];
  const Scalar _tmp78 = -_tmp76 + _tmp77;
  const Scalar _tmp79 = _y0[0] * _y1[0];
  const Scalar _tmp80 = _y0[1] * _y1[1];
  const Scalar _tmp81 = _tmp79 + _tmp80;
  const Scalar _tmp82 = _tmp81 + epsilon * ((((_tmp81) > 0) - ((_tmp81) < 0)) + Scalar(0.5));
  const Scalar _tmp83 = lambdas_y(0, 0) * std::atan2(_tmp78, _tmp82);
  const Scalar _tmp84 = std::sin(_tmp83);
  const Scalar _tmp85 = _tmp75 * _tmp84;
  const Scalar _tmp86 = _tmp61 * _tmp73;
  const Scalar _tmp87 = _tmp70 * _tmp72;
  const Scalar _tmp88 = -_tmp86 + _tmp87;
  const Scalar _tmp89 = std::cos(_tmp83);
  const Scalar _tmp90 = _tmp88 * _tmp89;
  const Scalar _tmp91 = -_tmp85 + _tmp90;
  const Scalar _tmp92 = _tmp91 * _y0[1];
  const Scalar _tmp93 = _tmp75 * _tmp89;
  const Scalar _tmp94 = _tmp84 * _tmp88;
  const Scalar _tmp95 = _tmp93 + _tmp94;
  const Scalar _tmp96 = _tmp95 * _y0[0];
  const Scalar _tmp97 = _tmp92 + _tmp96;
  const Scalar _tmp98 = _tmp91 * _y0[0] - _tmp95 * _y0[1];
  const Scalar _tmp99 = _tmp97 * _y_T_b[2] + _tmp98 * _y_T_b[3] + _y0[3] +
                        lambdas_y(0, 0) * (-_y0[3] + _y1[3]) +
                        lambdas_y(1, 0) * (-_y1[3] + _y2[3]) + lambdas_y(2, 0) * (-_y2[3] + _y3[3]);
  const Scalar _tmp100 = _tmp46 * _tmp99;
  const Scalar _tmp101 =
      -_tmp97 * _y_T_b[3] + _tmp98 * _y_T_b[2] + _y0[2] + lambdas_y(0, 0) * (-_y0[2] + _y1[2]) +
      lambdas_y(1, 0) * (-_y1[2] + _y2[2]) + lambdas_y(2, 0) * (-_y2[2] + _y3[2]);
  const Scalar _tmp102 = -_a_T_b[3] + _tmp100 - _tmp101 * _tmp51 - _tmp48 + _tmp51 * _tmp52;
  const Scalar _tmp103 = _tmp46 * _tmp52;
  const Scalar _tmp104 = _tmp101 * _tmp46;
  const Scalar _tmp105 = -_a_T_b[2] - _tmp103 + _tmp104 - _tmp47 * _tmp51 + _tmp51 * _tmp99;
  const Scalar _tmp106 = _tmp97 * _y_T_b[0];
  const Scalar _tmp107 = _tmp98 * _y_T_b[1];
  const Scalar _tmp108 = _tmp106 + _tmp107;
  const Scalar _tmp109 = -_tmp97 * _y_T_b[1] + _tmp98 * _y_T_b[0];
  const Scalar _tmp110 = _tmp109 * _tmp46;
  const Scalar _tmp111 = _tmp108 * _tmp51 + _tmp110;
  const Scalar _tmp112 = _tmp109 * _tmp51;
  const Scalar _tmp113 = _tmp108 * _tmp46;
  const Scalar _tmp114 = -_tmp112 + _tmp113;
  const Scalar _tmp115 = _a_T_b[0] * _tmp114 - _a_T_b[1] * _tmp111;
  const Scalar _tmp116 = _a_T_b[0] * _tmp111 + _a_T_b[1] * _tmp114;
  const Scalar _tmp117 = _tmp116 + epsilon * ((((_tmp116) > 0) - ((_tmp116) < 0)) + Scalar(0.5));
  const Scalar _tmp118 = std::atan2(_tmp115, _tmp117);
  const Scalar _tmp119 = std::pow(_tmp25, Scalar(2));
  const Scalar _tmp120 = std::pow(_tmp29, Scalar(2));
  const Scalar _tmp121 = Scalar(1.0) / (_tmp120);
  const Scalar _tmp122 = Scalar(1.0) / (_tmp29);
  const Scalar _tmp123 = -_tmp119 * _tmp121 + _tmp122 * (-_tmp26 - _tmp27);
  const Scalar _tmp124 = _tmp120 * lambdas_x(0, 0) / (_tmp119 + _tmp120);
  const Scalar _tmp125 = _tmp124 * _tmp40;
  const Scalar _tmp126 = _tmp124 * _tmp41;
  const Scalar _tmp127 = -_tmp123 * _tmp125 - _tmp123 * _tmp126;
  const Scalar _tmp128 = _tmp124 * _tmp32;
  const Scalar _tmp129 = _tmp124 * _tmp37;
  const Scalar _tmp130 = _tmp123 * _tmp128 - _tmp123 * _tmp129;
  const Scalar _tmp131 = _tmp127 * _x0[1] + _tmp130 * _x0[0] + _tmp45;
  const Scalar _tmp132 = -_tmp39 - _tmp43;
  const Scalar _tmp133 = _tmp127 * _x0[0] - _tmp130 * _x0[1] + _tmp132;
  const Scalar _tmp134 = _tmp131 * _x_T_a[0] + _tmp133 * _x_T_a[1];
  const Scalar _tmp135 = -_tmp131 * _x_T_a[1] + _tmp133 * _x_T_a[0];
  const Scalar _tmp136 = _tmp108 * _tmp135 - _tmp109 * _tmp134;
  const Scalar _tmp137 = _tmp108 * _tmp134 + _tmp109 * _tmp135;
  const Scalar _tmp138 = std::pow(_tmp117, Scalar(2));
  const Scalar _tmp139 = _tmp115 / _tmp138;
  const Scalar _tmp140 = Scalar(1.0) / (_tmp117);
  const Scalar _tmp141 = _tmp138 / (std::pow(_tmp115, Scalar(2)) + _tmp138);
  const Scalar _tmp142 = _tmp141 * (-_tmp139 * (_a_T_b[0] * _tmp137 + _a_T_b[1] * _tmp136) +
                                    _tmp140 * (_a_T_b[0] * _tmp136 - _a_T_b[1] * _tmp137));
  const Scalar _tmp143 = -_tmp131 * _x_T_a[3] + _tmp133 * _x_T_a[2];
  const Scalar _tmp144 = _tmp131 * _x_T_a[2] + _tmp133 * _x_T_a[3];
  const Scalar _tmp145 = _tmp101 * _tmp135 - _tmp134 * _tmp47 + _tmp134 * _tmp99 -
                         _tmp135 * _tmp52 - _tmp143 * _tmp46 - _tmp144 * _tmp51;
  const Scalar _tmp146 = -_tmp101 * _tmp134 + _tmp134 * _tmp52 - _tmp135 * _tmp47 +
                         _tmp135 * _tmp99 + _tmp143 * _tmp51 - _tmp144 * _tmp46;
  const Scalar _tmp147 = 1 - lambdas_x(0, 0);
  const Scalar _tmp148 = _tmp147 * _tmp46;
  const Scalar _tmp149 = _tmp147 * _tmp51;
  const Scalar _tmp150 = _tmp51 * sqrt_info(1, 2);
  const Scalar _tmp151 = _tmp46 * sqrt_info(2, 1);
  const Scalar _tmp152 = _tmp51 * sqrt_info(2, 2);
  const Scalar _tmp153 = _tmp46 * sqrt_info(0, 2);
  const Scalar _tmp154 = _tmp51 * sqrt_info(0, 1);
  const Scalar _tmp155 = _tmp51 * sqrt_info(1, 1);
  const Scalar _tmp156 = std::pow(_tmp11, Scalar(2));
  const Scalar _tmp157 = std::pow(_tmp15, Scalar(2));
  const Scalar _tmp158 = Scalar(1.0) / (_tmp157);
  const Scalar _tmp159 = Scalar(1.0) / (_tmp15);
  const Scalar _tmp160 = _tmp157 * lambdas_x(1, 0) / (_tmp156 + _tmp157);
  const Scalar _tmp161 = _tmp160 * (-_tmp156 * _tmp158 + _tmp159 * (-_tmp12 - _tmp13));
  const Scalar _tmp162 = -_tmp161 * _tmp33 - _tmp161 * _tmp34;
  const Scalar _tmp163 = -_tmp121 * _tmp25 * (_tmp23 - _tmp24) + _tmp122 * _tmp28;
  const Scalar _tmp164 = _tmp161 * _tmp18 - _tmp161 * _tmp21;
  const Scalar _tmp165 =
      _tmp128 * _tmp163 - _tmp129 * _tmp163 + _tmp162 * _tmp36 + _tmp164 * _tmp31;
  const Scalar _tmp166 =
      -_tmp125 * _tmp163 - _tmp126 * _tmp163 + _tmp162 * _tmp31 - _tmp164 * _tmp36;
  const Scalar _tmp167 = _tmp165 * _x0[0] + _tmp166 * _x0[1];
  const Scalar _tmp168 = -_tmp165 * _x0[1] + _tmp166 * _x0[0];
  const Scalar _tmp169 = _tmp167 * _x_T_a[0] + _tmp168 * _x_T_a[1];
  const Scalar _tmp170 = -_tmp167 * _x_T_a[1] + _tmp168 * _x_T_a[0];
  const Scalar _tmp171 = _tmp167 * _x_T_a[2] + _tmp168 * _x_T_a[3];
  const Scalar _tmp172 = -_tmp167 * _x_T_a[3] + _tmp168 * _x_T_a[2];
  const Scalar _tmp173 = _tmp101 * _tmp170 - _tmp169 * _tmp47 + _tmp169 * _tmp99 -
                         _tmp170 * _tmp52 - _tmp171 * _tmp51 - _tmp172 * _tmp46;
  const Scalar _tmp174 = _tmp108 * _tmp170 - _tmp109 * _tmp169;
  const Scalar _tmp175 = _tmp108 * _tmp169 + _tmp109 * _tmp170;
  const Scalar _tmp176 = -_tmp139 * (_a_T_b[0] * _tmp175 + _a_T_b[1] * _tmp174) +
                         _tmp140 * (_a_T_b[0] * _tmp174 - _a_T_b[1] * _tmp175);
  const Scalar _tmp177 = _tmp141 * _tmp176;
  const Scalar _tmp178 = -_tmp101 * _tmp169 + _tmp169 * _tmp52 - _tmp170 * _tmp47 +
                         _tmp170 * _tmp99 - _tmp171 * _tmp46 + _tmp172 * _tmp51;
  const Scalar _tmp179 = _tmp141 * sqrt_info(2, 0);
  const Scalar _tmp180 = lambdas_x(0, 0) - lambdas_x(1, 0);
  const Scalar _tmp181 = _tmp46 * sqrt_info(0, 1);
  const Scalar _tmp182 = _tmp180 * _tmp51;
  const Scalar _tmp183 = _tmp46 * sqrt_info(1, 1);
  const Scalar _tmp184 = _tmp46 * sqrt_info(1, 2);
  const Scalar _tmp185 = _tmp46 * sqrt_info(2, 2);
  const Scalar _tmp186 = _tmp160 * (-_tmp11 * _tmp158 * (_tmp10 - _tmp9) + _tmp14 * _tmp159);
  const Scalar _tmp187 = std::pow(_tmp2, Scalar(2));
  const Scalar _tmp188 = std::pow(_tmp6, Scalar(2));
  const Scalar _tmp189 = Scalar(1.0) / (_tmp188);
  const Scalar _tmp190 = Scalar(1.0) / (_tmp6);
  const Scalar _tmp191 = _tmp188 * lambdas_x(2, 0) / (_tmp187 + _tmp188);
  const Scalar _tmp192 = _tmp191 * (-_tmp187 * _tmp189 + _tmp190 * (-_tmp3 - _tmp4));
  const Scalar _tmp193 = -_tmp186 * _tmp33 - _tmp186 * _tmp34 - _tmp192 * _tmp33 - _tmp192 * _tmp34;
  const Scalar _tmp194 = _tmp18 * _tmp186 + _tmp18 * _tmp192 - _tmp186 * _tmp21 - _tmp192 * _tmp21;
  const Scalar _tmp195 = _tmp193 * _tmp31 - _tmp194 * _tmp36;
  const Scalar _tmp196 = _tmp193 * _tmp36 + _tmp194 * _tmp31;
  const Scalar _tmp197 = _tmp195 * _x0[0] - _tmp196 * _x0[1];
  const Scalar _tmp198 = _tmp195 * _x0[1] + _tmp196 * _x0[0];
  const Scalar _tmp199 = _tmp197 * _x_T_a[0] - _tmp198 * _x_T_a[1];
  const Scalar _tmp200 = _tmp197 * _x_T_a[1] + _tmp198 * _x_T_a[0];
  const Scalar _tmp201 = _tmp197 * _x_T_a[3] + _tmp198 * _x_T_a[2];
  const Scalar _tmp202 = _tmp197 * _x_T_a[2] - _tmp198 * _x_T_a[3];
  const Scalar _tmp203 = -_tmp101 * _tmp200 - _tmp199 * _tmp47 + _tmp199 * _tmp99 +
                         _tmp200 * _tmp52 - _tmp201 * _tmp46 + _tmp202 * _tmp51;
  const Scalar _tmp204 = _tmp101 * _tmp199 - _tmp199 * _tmp52 - _tmp200 * _tmp47 +
                         _tmp200 * _tmp99 - _tmp201 * _tmp51 - _tmp202 * _tmp46;
  const Scalar _tmp205 = _tmp108 * _tmp200 + _tmp109 * _tmp199;
  const Scalar _tmp206 = _tmp108 * _tmp199 - _tmp109 * _tmp200;
  const Scalar _tmp207 = -_tmp139 * (_a_T_b[0] * _tmp205 + _a_T_b[1] * _tmp206) +
                         _tmp140 * (_a_T_b[0] * _tmp206 - _a_T_b[1] * _tmp205);
  const Scalar _tmp208 = _tmp141 * sqrt_info(0, 0);
  const Scalar _tmp209 = _tmp141 * sqrt_info(1, 0);
  const Scalar _tmp210 = lambdas_x(1, 0) - lambdas_x(2, 0);
  const Scalar _tmp211 = _tmp210 * _tmp51;
  const Scalar _tmp212 = _tmp210 * _tmp46;
  const Scalar _tmp213 = _tmp191 * (-_tmp189 * _tmp2 * (_tmp0 - _tmp1) + _tmp190 * _tmp5);
  const Scalar _tmp214 = _tmp18 * _tmp213 - _tmp21 * _tmp213;
  const Scalar _tmp215 = -_tmp213 * _tmp33 - _tmp213 * _tmp34;
  const Scalar _tmp216 = -_tmp214 * _tmp36 + _tmp215 * _tmp31;
  const Scalar _tmp217 = _tmp214 * _tmp31 + _tmp215 * _tmp36;
  const Scalar _tmp218 = _tmp216 * _x0[0] - _tmp217 * _x0[1];
  const Scalar _tmp219 = _tmp216 * _x0[1] + _tmp217 * _x0[0];
  const Scalar _tmp220 = _tmp218 * _x_T_a[1] + _tmp219 * _x_T_a[0];
  const Scalar _tmp221 = _tmp218 * _x_T_a[0] - _tmp219 * _x_T_a[1];
  const Scalar _tmp222 = _tmp218 * _x_T_a[3] + _tmp219 * _x_T_a[2];
  const Scalar _tmp223 = _tmp218 * _x_T_a[2] - _tmp219 * _x_T_a[3];
  const Scalar _tmp224 = _tmp101 * _tmp221 - _tmp220 * _tmp47 + _tmp220 * _tmp99 -
                         _tmp221 * _tmp52 - _tmp222 * _tmp51 - _tmp223 * _tmp46;
  const Scalar _tmp225 = -_tmp101 * _tmp220 + _tmp220 * _tmp52 - _tmp221 * _tmp47 +
                         _tmp221 * _tmp99 - _tmp222 * _tmp46 + _tmp223 * _tmp51;
  const Scalar _tmp226 = _tmp108 * _tmp221 - _tmp109 * _tmp220;
  const Scalar _tmp227 = _tmp108 * _tmp220 + _tmp109 * _tmp221;
  const Scalar _tmp228 = -_tmp139 * (_a_T_b[0] * _tmp227 + _a_T_b[1] * _tmp226) +
                         _tmp140 * (_a_T_b[0] * _tmp226 - _a_T_b[1] * _tmp227);
  const Scalar _tmp229 = _tmp46 * lambdas_x(2, 0);
  const Scalar _tmp230 = _tmp51 * lambdas_x(2, 0);
  const Scalar _tmp231 = -_tmp49 - _tmp50;
  const Scalar _tmp232 = _tmp103 - _tmp104 - _tmp231 * _tmp47 + _tmp231 * _tmp99;
  const Scalar _tmp233 = _tmp109 * _tmp231 + _tmp113;
  const Scalar _tmp234 = _tmp108 * _tmp231 - _tmp110;
  const Scalar _tmp235 = -_tmp139 * (_a_T_b[0] * _tmp233 + _a_T_b[1] * _tmp234) +
                         _tmp140 * (_a_T_b[0] * _tmp234 - _a_T_b[1] * _tmp233);
  const Scalar _tmp236 = _tmp100 + _tmp101 * _tmp231 - _tmp231 * _tmp52 - _tmp48;
  const Scalar _tmp237 = -_tmp45 * _tmp46;
  const Scalar _tmp238 = _tmp237 - _tmp44 * _tmp51;
  const Scalar _tmp239 = _tmp45 * _tmp51;
  const Scalar _tmp240 = _tmp239 - _tmp44 * _tmp46;
  const Scalar _tmp241 = _tmp132 * _tmp51 + _tmp237;
  const Scalar _tmp242 = -_tmp132 * _tmp46 - _tmp239;
  const Scalar _tmp243 = std::pow(_tmp78, Scalar(2));
  const Scalar _tmp244 = std::pow(_tmp82, Scalar(2));
  const Scalar _tmp245 = Scalar(1.0) / (_tmp244);
  const Scalar _tmp246 = Scalar(1.0) / (_tmp82);
  const Scalar _tmp247 = _tmp244 * lambdas_y(0, 0) / (_tmp243 + _tmp244);
  const Scalar _tmp248 = _tmp247 * (-_tmp243 * _tmp245 + _tmp246 * (-_tmp79 - _tmp80));
  const Scalar _tmp249 = -_tmp248 * _tmp93 - _tmp248 * _tmp94;
  const Scalar _tmp250 = -_tmp248 * _tmp85 + _tmp248 * _tmp90;
  const Scalar _tmp251 = -_tmp92 - _tmp96;
  const Scalar _tmp252 = _tmp249 * _y0[0] - _tmp250 * _y0[1] + _tmp251;
  const Scalar _tmp253 = _tmp249 * _y0[1] + _tmp250 * _y0[0] + _tmp98;
  const Scalar _tmp254 = _tmp252 * _y_T_b[0] - _tmp253 * _y_T_b[1];
  const Scalar _tmp255 = _tmp252 * _y_T_b[1] + _tmp253 * _y_T_b[0];
  const Scalar _tmp256 = -_tmp254 * _tmp51 + _tmp255 * _tmp46;
  const Scalar _tmp257 = _tmp254 * _tmp46 + _tmp255 * _tmp51;
  const Scalar _tmp258 = -_tmp139 * (_a_T_b[0] * _tmp257 + _a_T_b[1] * _tmp256) +
                         _tmp140 * (_a_T_b[0] * _tmp256 - _a_T_b[1] * _tmp257);
  const Scalar _tmp259 = _tmp252 * _y_T_b[2] - _tmp253 * _y_T_b[3];
  const Scalar _tmp260 = _tmp252 * _y_T_b[3] + _tmp253 * _y_T_b[2];
  const Scalar _tmp261 = -_tmp259 * _tmp51 + _tmp260 * _tmp46;
  const Scalar _tmp262 = _tmp259 * _tmp46 + _tmp260 * _tmp51;
  const Scalar _tmp263 = 1 - lambdas_y(0, 0);
  const Scalar _tmp264 = _tmp263 * _tmp46;
  const Scalar _tmp265 = _tmp263 * _tmp51;
  const Scalar _tmp266 = Scalar(1.0) / (_tmp59);
  const Scalar _tmp267 = std::pow(_tmp55, Scalar(2));
  const Scalar _tmp268 = std::pow(_tmp59, Scalar(2));
  const Scalar _tmp269 = Scalar(1.0) / (_tmp268);
  const Scalar _tmp270 = _tmp268 * lambdas_y(1, 0) / (_tmp267 + _tmp268);
  const Scalar _tmp271 = _tmp270 * (_tmp266 * (-_tmp56 - _tmp57) - _tmp267 * _tmp269);
  const Scalar _tmp272 = -_tmp271 * _tmp71 - _tmp271 * _tmp74;
  const Scalar _tmp273 = -_tmp271 * _tmp86 + _tmp271 * _tmp87;
  const Scalar _tmp274 = _tmp247 * (-_tmp245 * _tmp78 * (_tmp76 - _tmp77) + _tmp246 * _tmp81);
  const Scalar _tmp275 = _tmp272 * _tmp84 + _tmp273 * _tmp89 - _tmp274 * _tmp85 + _tmp274 * _tmp90;
  const Scalar _tmp276 = _tmp272 * _tmp89 - _tmp273 * _tmp84 - _tmp274 * _tmp93 - _tmp274 * _tmp94;
  const Scalar _tmp277 = -_tmp275 * _y0[1] + _tmp276 * _y0[0];
  const Scalar _tmp278 = _tmp275 * _y0[0] + _tmp276 * _y0[1];
  const Scalar _tmp279 = _tmp277 * _y_T_b[1] + _tmp278 * _y_T_b[0];
  const Scalar _tmp280 = _tmp277 * _y_T_b[0] - _tmp278 * _y_T_b[1];
  const Scalar _tmp281 = _tmp279 * _tmp46 - _tmp280 * _tmp51;
  const Scalar _tmp282 = _tmp279 * _tmp51 + _tmp280 * _tmp46;
  const Scalar _tmp283 = -_tmp139 * (_a_T_b[0] * _tmp282 + _a_T_b[1] * _tmp281) +
                         _tmp140 * (_a_T_b[0] * _tmp281 - _a_T_b[1] * _tmp282);
  const Scalar _tmp284 = _tmp277 * _y_T_b[2] - _tmp278 * _y_T_b[3];
  const Scalar _tmp285 = _tmp277 * _y_T_b[3] + _tmp278 * _y_T_b[2];
  const Scalar _tmp286 = _tmp284 * _tmp46 + _tmp285 * _tmp51;
  const Scalar _tmp287 = -_tmp284 * _tmp51 + _tmp285 * _tmp46;
  const Scalar _tmp288 = lambdas_y(0, 0) - lambdas_y(1, 0);
  const Scalar _tmp289 = _tmp288 * _tmp51;
  const Scalar _tmp290 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp291 = std::pow(_tmp64, Scalar(2));
  const Scalar _tmp292 = std::pow(_tmp68, Scalar(2));
  const Scalar _tmp293 = Scalar(1.0) / (_tmp292);
  const Scalar _tmp294 = _tmp292 * lambdas_y(2, 0) / (_tmp291 + _tmp292);
  const Scalar _tmp295 = _tmp294 * (_tmp290 * (-_tmp65 - _tmp66) - _tmp291 * _tmp293);
  const Scalar _tmp296 = _tmp270 * (_tmp266 * _tmp58 - _tmp269 * _tmp55 * (-_tmp53 + _tmp54));
  const Scalar _tmp297 = -_tmp295 * _tmp86 + _tmp295 * _tmp87 - _tmp296 * _tmp86 + _tmp296 * _tmp87;
  const Scalar _tmp298 = -_tmp295 * _tmp71 - _tmp295 * _tmp74 - _tmp296 * _tmp71 - _tmp296 * _tmp74;
  const Scalar _tmp299 = _tmp297 * _tmp89 + _tmp298 * _tmp84;
  const Scalar _tmp300 = -_tmp297 * _tmp84 + _tmp298 * _tmp89;
  const Scalar _tmp301 = _tmp299 * _y0[0] + _tmp300 * _y0[1];
  const Scalar _tmp302 = -_tmp299 * _y0[1] + _tmp300 * _y0[0];
  const Scalar _tmp303 = -_tmp301 * _y_T_b[3] + _tmp302 * _y_T_b[2];
  const Scalar _tmp304 = _tmp301 * _y_T_b[2] + _tmp302 * _y_T_b[3];
  const Scalar _tmp305 = -_tmp303 * _tmp51 + _tmp304 * _tmp46;
  const Scalar _tmp306 = _tmp303 * _tmp46 + _tmp304 * _tmp51;
  const Scalar _tmp307 = _tmp301 * _y_T_b[0] + _tmp302 * _y_T_b[1];
  const Scalar _tmp308 = -_tmp301 * _y_T_b[1] + _tmp302 * _y_T_b[0];
  const Scalar _tmp309 = _tmp307 * _tmp51 + _tmp308 * _tmp46;
  const Scalar _tmp310 = _tmp307 * _tmp46 - _tmp308 * _tmp51;
  const Scalar _tmp311 = -_tmp139 * (_a_T_b[0] * _tmp309 + _a_T_b[1] * _tmp310) +
                         _tmp140 * (_a_T_b[0] * _tmp310 - _a_T_b[1] * _tmp309);
  const Scalar _tmp312 = lambdas_y(1, 0) - lambdas_y(2, 0);
  const Scalar _tmp313 = _tmp312 * _tmp51;
  const Scalar _tmp314 = _tmp312 * _tmp46;
  const Scalar _tmp315 = _tmp294 * (_tmp290 * _tmp67 - _tmp293 * _tmp64 * (-_tmp62 + _tmp63));
  const Scalar _tmp316 = -_tmp315 * _tmp71 - _tmp315 * _tmp74;
  const Scalar _tmp317 = -_tmp315 * _tmp86 + _tmp315 * _tmp87;
  const Scalar _tmp318 = _tmp316 * _tmp84 + _tmp317 * _tmp89;
  const Scalar _tmp319 = _tmp316 * _tmp89 - _tmp317 * _tmp84;
  const Scalar _tmp320 = -_tmp318 * _y0[1] + _tmp319 * _y0[0];
  const Scalar _tmp321 = _tmp318 * _y0[0] + _tmp319 * _y0[1];
  const Scalar _tmp322 = _tmp320 * _y_T_b[2] - _tmp321 * _y_T_b[3];
  const Scalar _tmp323 = _tmp320 * _y_T_b[3] + _tmp321 * _y_T_b[2];
  const Scalar _tmp324 = _tmp322 * _tmp46 + _tmp323 * _tmp51;
  const Scalar _tmp325 = -_tmp322 * _tmp51 + _tmp323 * _tmp46;
  const Scalar _tmp326 = _tmp320 * _y_T_b[0] - _tmp321 * _y_T_b[1];
  const Scalar _tmp327 = _tmp320 * _y_T_b[1] + _tmp321 * _y_T_b[0];
  const Scalar _tmp328 = _tmp326 * _tmp46 + _tmp327 * _tmp51;
  const Scalar _tmp329 = -_tmp326 * _tmp51 + _tmp327 * _tmp46;
  const Scalar _tmp330 = -_tmp139 * (_a_T_b[0] * _tmp328 + _a_T_b[1] * _tmp329) +
                         _tmp140 * (_a_T_b[0] * _tmp329 - _a_T_b[1] * _tmp328);
  const Scalar _tmp331 = _tmp46 * lambdas_y(2, 0);
  const Scalar _tmp332 = _tmp51 * lambdas_y(2, 0);
  const Scalar _tmp333 = -_tmp106 - _tmp107;
  const Scalar _tmp334 = _tmp110 - _tmp333 * _tmp51;
  const Scalar _tmp335 = _tmp112 + _tmp333 * _tmp46;
  const Scalar _tmp336 = -_tmp139 * (_a_T_b[0] * _tmp335 + _a_T_b[1] * _tmp334) +
                         _tmp140 * (_a_T_b[0] * _tmp334 - _a_T_b[1] * _tmp335);
  const Scalar _tmp337 = _tmp46 * _tmp98;
  const Scalar _tmp338 = _tmp337 + _tmp51 * _tmp97;
  const Scalar _tmp339 = _tmp51 * _tmp98;
  const Scalar _tmp340 = -_tmp339 + _tmp46 * _tmp97;
  const Scalar _tmp341 = -_tmp251 * _tmp51 + _tmp337;
  const Scalar _tmp342 = _tmp251 * _tmp46 + _tmp339;

  // Output terms (11)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp102 * sqrt_info(0, 2) + _tmp105 * sqrt_info(0, 1) + _tmp118 * sqrt_info(0, 0);
  _res(1, 0) = _tmp102 * sqrt_info(1, 2) + _tmp105 * sqrt_info(1, 1) + _tmp118 * sqrt_info(1, 0);
  _res(2, 0) = _tmp102 * sqrt_info(2, 2) + _tmp105 * sqrt_info(2, 1) + _tmp118 * sqrt_info(2, 0);

  if (res_D_x0 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x0{res_D_x0};

    _res_D_x0(0, 0) =
        _tmp142 * sqrt_info(0, 0) + _tmp145 * sqrt_info(0, 1) + _tmp146 * sqrt_info(0, 2);
    _res_D_x0(1, 0) =
        _tmp142 * sqrt_info(1, 0) + _tmp145 * sqrt_info(1, 1) + _tmp146 * sqrt_info(1, 2);
    _res_D_x0(2, 0) =
        _tmp142 * sqrt_info(2, 0) + _tmp145 * sqrt_info(2, 1) + _tmp146 * sqrt_info(2, 2);
    _res_D_x0(0, 1) = -_tmp148 * sqrt_info(0, 1) + _tmp149 * sqrt_info(0, 2);
    _res_D_x0(1, 1) = _tmp147 * _tmp150 - _tmp148 * sqrt_info(1, 1);
    _res_D_x0(2, 1) = -_tmp147 * _tmp151 + _tmp147 * _tmp152;
    _res_D_x0(0, 2) = -_tmp147 * _tmp153 - _tmp147 * _tmp154;
    _res_D_x0(1, 2) = -_tmp147 * _tmp155 - _tmp148 * sqrt_info(1, 2);
    _res_D_x0(2, 2) = -_tmp148 * sqrt_info(2, 2) - _tmp149 * sqrt_info(2, 1);
  }

  if (res_D_x1 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x1{res_D_x1};

    _res_D_x1(0, 0) =
        _tmp173 * sqrt_info(0, 1) + _tmp177 * sqrt_info(0, 0) + _tmp178 * sqrt_info(0, 2);
    _res_D_x1(1, 0) =
        _tmp173 * sqrt_info(1, 1) + _tmp177 * sqrt_info(1, 0) + _tmp178 * sqrt_info(1, 2);
    _res_D_x1(2, 0) = _tmp173 * sqrt_info(2, 1) + _tmp176 * _tmp179 + _tmp178 * sqrt_info(2, 2);
    _res_D_x1(0, 1) = -_tmp180 * _tmp181 + _tmp182 * sqrt_info(0, 2);
    _res_D_x1(1, 1) = -_tmp180 * _tmp183 + _tmp182 * sqrt_info(1, 2);
    _res_D_x1(2, 1) = -_tmp151 * _tmp180 + _tmp182 * sqrt_info(2, 2);
    _res_D_x1(0, 2) = -_tmp153 * _tmp180 - _tmp182 * sqrt_info(0, 1);
    _res_D_x1(1, 2) = -_tmp155 * _tmp180 - _tmp180 * _tmp184;
    _res_D_x1(2, 2) = -_tmp180 * _tmp185 - _tmp182 * sqrt_info(2, 1);
  }

  if (res_D_x2 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x2{res_D_x2};

    _res_D_x2(0, 0) = _tmp203 * sqrt_info(0, 2) + _tmp204 * sqrt_info(0, 1) + _tmp207 * _tmp208;
    _res_D_x2(1, 0) = _tmp203 * sqrt_info(1, 2) + _tmp204 * sqrt_info(1, 1) + _tmp207 * _tmp209;
    _res_D_x2(2, 0) = _tmp179 * _tmp207 + _tmp203 * sqrt_info(2, 2) + _tmp204 * sqrt_info(2, 1);
    _res_D_x2(0, 1) = _tmp211 * sqrt_info(0, 2) - _tmp212 * sqrt_info(0, 1);
    _res_D_x2(1, 1) = _tmp150 * _tmp210 - _tmp212 * sqrt_info(1, 1);
    _res_D_x2(2, 1) = _tmp152 * _tmp210 - _tmp212 * sqrt_info(2, 1);
    _res_D_x2(0, 2) = -_tmp154 * _tmp210 - _tmp212 * sqrt_info(0, 2);
    _res_D_x2(1, 2) = -_tmp155 * _tmp210 - _tmp212 * sqrt_info(1, 2);
    _res_D_x2(2, 2) = -_tmp211 * sqrt_info(2, 1) - _tmp212 * sqrt_info(2, 2);
  }

  if (res_D_x3 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x3{res_D_x3};

    _res_D_x3(0, 0) = _tmp208 * _tmp228 + _tmp224 * sqrt_info(0, 1) + _tmp225 * sqrt_info(0, 2);
    _res_D_x3(1, 0) = _tmp209 * _tmp228 + _tmp224 * sqrt_info(1, 1) + _tmp225 * sqrt_info(1, 2);
    _res_D_x3(2, 0) = _tmp179 * _tmp228 + _tmp224 * sqrt_info(2, 1) + _tmp225 * sqrt_info(2, 2);
    _res_D_x3(0, 1) = -_tmp229 * sqrt_info(0, 1) + _tmp230 * sqrt_info(0, 2);
    _res_D_x3(1, 1) = _tmp150 * lambdas_x(2, 0) - _tmp229 * sqrt_info(1, 1);
    _res_D_x3(2, 1) = -_tmp151 * lambdas_x(2, 0) + _tmp152 * lambdas_x(2, 0);
    _res_D_x3(0, 2) = -_tmp153 * lambdas_x(2, 0) - _tmp154 * lambdas_x(2, 0);
    _res_D_x3(1, 2) = -_tmp155 * lambdas_x(2, 0) - _tmp229 * sqrt_info(1, 2);
    _res_D_x3(2, 2) = -_tmp185 * lambdas_x(2, 0) - _tmp230 * sqrt_info(2, 1);
  }

  if (res_D_x_T_a != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x_T_a{res_D_x_T_a};

    _res_D_x_T_a(0, 0) = _tmp208 * _tmp235 + _tmp232 * sqrt_info(0, 2) + _tmp236 * sqrt_info(0, 1);
    _res_D_x_T_a(1, 0) = _tmp209 * _tmp235 + _tmp232 * sqrt_info(1, 2) + _tmp236 * sqrt_info(1, 1);
    _res_D_x_T_a(2, 0) = _tmp179 * _tmp235 + _tmp232 * sqrt_info(2, 2) + _tmp236 * sqrt_info(2, 1);
    _res_D_x_T_a(0, 1) = _tmp238 * sqrt_info(0, 1) + _tmp240 * sqrt_info(0, 2);
    _res_D_x_T_a(1, 1) = _tmp238 * sqrt_info(1, 1) + _tmp240 * sqrt_info(1, 2);
    _res_D_x_T_a(2, 1) = _tmp238 * sqrt_info(2, 1) + _tmp240 * sqrt_info(2, 2);
    _res_D_x_T_a(0, 2) = _tmp241 * sqrt_info(0, 2) + _tmp242 * sqrt_info(0, 1);
    _res_D_x_T_a(1, 2) = _tmp241 * sqrt_info(1, 2) + _tmp242 * sqrt_info(1, 1);
    _res_D_x_T_a(2, 2) = _tmp241 * sqrt_info(2, 2) + _tmp242 * sqrt_info(2, 1);
  }

  if (res_D_y0 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_y0{res_D_y0};

    _res_D_y0(0, 0) = _tmp208 * _tmp258 + _tmp261 * sqrt_info(0, 2) + _tmp262 * sqrt_info(0, 1);
    _res_D_y0(1, 0) = _tmp209 * _tmp258 + _tmp261 * sqrt_info(1, 2) + _tmp262 * sqrt_info(1, 1);
    _res_D_y0(2, 0) = _tmp179 * _tmp258 + _tmp261 * sqrt_info(2, 2) + _tmp262 * sqrt_info(2, 1);
    _res_D_y0(0, 1) = _tmp264 * sqrt_info(0, 1) - _tmp265 * sqrt_info(0, 2);
    _res_D_y0(1, 1) = -_tmp150 * _tmp263 + _tmp264 * sqrt_info(1, 1);
    _res_D_y0(2, 1) = _tmp151 * _tmp263 - _tmp152 * _tmp263;
    _res_D_y0(0, 2) = _tmp153 * _tmp263 + _tmp154 * _tmp263;
    _res_D_y0(1, 2) = _tmp155 * _tmp263 + _tmp264 * sqrt_info(1, 2);
    _res_D_y0(2, 2) = _tmp264 * sqrt_info(2, 2) + _tmp265 * sqrt_info(2, 1);
  }

  if (res_D_y1 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_y1{res_D_y1};

    _res_D_y1(0, 0) = _tmp208 * _tmp283 + _tmp286 * sqrt_info(0, 1) + _tmp287 * sqrt_info(0, 2);
    _res_D_y1(1, 0) = _tmp209 * _tmp283 + _tmp286 * sqrt_info(1, 1) + _tmp287 * sqrt_info(1, 2);
    _res_D_y1(2, 0) = _tmp179 * _tmp283 + _tmp286 * sqrt_info(2, 1) + _tmp287 * sqrt_info(2, 2);
    _res_D_y1(0, 1) = _tmp181 * _tmp288 - _tmp289 * sqrt_info(0, 2);
    _res_D_y1(1, 1) = _tmp183 * _tmp288 - _tmp289 * sqrt_info(1, 2);
    _res_D_y1(2, 1) = _tmp151 * _tmp288 - _tmp289 * sqrt_info(2, 2);
    _res_D_y1(0, 2) = _tmp153 * _tmp288 + _tmp154 * _tmp288;
    _res_D_y1(1, 2) = _tmp155 * _tmp288 + _tmp184 * _tmp288;
    _res_D_y1(2, 2) = _tmp185 * _tmp288 + _tmp289 * sqrt_info(2, 1);
  }

  if (res_D_y2 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_y2{res_D_y2};

    _res_D_y2(0, 0) = _tmp208 * _tmp311 + _tmp305 * sqrt_info(0, 2) + _tmp306 * sqrt_info(0, 1);
    _res_D_y2(1, 0) = _tmp209 * _tmp311 + _tmp305 * sqrt_info(1, 2) + _tmp306 * sqrt_info(1, 1);
    _res_D_y2(2, 0) = _tmp179 * _tmp311 + _tmp305 * sqrt_info(2, 2) + _tmp306 * sqrt_info(2, 1);
    _res_D_y2(0, 1) = -_tmp313 * sqrt_info(0, 2) + _tmp314 * sqrt_info(0, 1);
    _res_D_y2(1, 1) = -_tmp150 * _tmp312 + _tmp314 * sqrt_info(1, 1);
    _res_D_y2(2, 1) = _tmp151 * _tmp312 - _tmp152 * _tmp312;
    _res_D_y2(0, 2) = _tmp154 * _tmp312 + _tmp314 * sqrt_info(0, 2);
    _res_D_y2(1, 2) = _tmp155 * _tmp312 + _tmp314 * sqrt_info(1, 2);
    _res_D_y2(2, 2) = _tmp313 * sqrt_info(2, 1) + _tmp314 * sqrt_info(2, 2);
  }

  if (res_D_y3 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_y3{res_D_y3};

    _res_D_y3(0, 0) = _tmp208 * _tmp330 + _tmp324 * sqrt_info(0, 1) + _tmp325 * sqrt_info(0, 2);
    _res_D_y3(1, 0) = _tmp209 * _tmp330 + _tmp324 * sqrt_info(1, 1) + _tmp325 * sqrt_info(1, 2);
    _res_D_y3(2, 0) = _tmp179 * _tmp330 + _tmp324 * sqrt_info(2, 1) + _tmp325 * sqrt_info(2, 2);
    _res_D_y3(0, 1) = _tmp331 * sqrt_info(0, 1) - _tmp332 * sqrt_info(0, 2);
    _res_D_y3(1, 1) = -_tmp150 * lambdas_y(2, 0) + _tmp331 * sqrt_info(1, 1);
    _res_D_y3(2, 1) = _tmp151 * lambdas_y(2, 0) - _tmp152 * lambdas_y(2, 0);
    _res_D_y3(0, 2) = _tmp153 * lambdas_y(2, 0) + _tmp154 * lambdas_y(2, 0);
    _res_D_y3(1, 2) = _tmp155 * lambdas_y(2, 0) + _tmp331 * sqrt_info(1, 2);
    _res_D_y3(2, 2) = _tmp331 * sqrt_info(2, 2) + _tmp332 * sqrt_info(2, 1);
  }

  if (res_D_y_T_b != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_y_T_b{res_D_y_T_b};

    _res_D_y_T_b(0, 0) = _tmp208 * _tmp336;
    _res_D_y_T_b(1, 0) = _tmp209 * _tmp336;
    _res_D_y_T_b(2, 0) = _tmp179 * _tmp336;
    _res_D_y_T_b(0, 1) = _tmp338 * sqrt_info(0, 1) + _tmp340 * sqrt_info(0, 2);
    _res_D_y_T_b(1, 1) = _tmp338 * sqrt_info(1, 1) + _tmp340 * sqrt_info(1, 2);
    _res_D_y_T_b(2, 1) = _tmp338 * sqrt_info(2, 1) + _tmp340 * sqrt_info(2, 2);
    _res_D_y_T_b(0, 2) = _tmp341 * sqrt_info(0, 2) + _tmp342 * sqrt_info(0, 1);
    _res_D_y_T_b(1, 2) = _tmp341 * sqrt_info(1, 2) + _tmp342 * sqrt_info(1, 1);
    _res_D_y_T_b(2, 2) = _tmp341 * sqrt_info(2, 2) + _tmp342 * sqrt_info(2, 1);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion