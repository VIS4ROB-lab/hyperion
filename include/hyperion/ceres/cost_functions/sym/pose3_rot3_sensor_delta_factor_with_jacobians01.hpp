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
 * Symbolic function: pose3_rot3_sensor_delta_factor
 *
 * Args:
 *     x: Pose3
 *     x_T_y: Rot3
 *     y: Pose3
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 *     res_D_x: (3x7) jacobian (result_dim x storage_dim) of res (3) wrt arg x (7) (row-major)
 *     res_D_x_T_y: (3x4) jacobian (result_dim x storage_dim) of res (3) wrt arg x_T_y (4)
 * (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Pose3Rot3SensorDeltaFactorWithJacobians01(
    const sym::Pose3<Scalar>& x, const sym::Rot3<Scalar>& x_T_y, const sym::Pose3<Scalar>& y,
    const Eigen::Matrix<Scalar, 3, 3>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr, Scalar* const res_D_x_T_y = nullptr) {
  // Total ops: 803

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x_T_y = x_T_y.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (224)
  const Scalar _tmp0 = _x[2] * _y[2];
  const Scalar _tmp1 = _x[1] * _y[1];
  const Scalar _tmp2 = _x[0] * _y[0];
  const Scalar _tmp3 = _x[3] * _y[3];
  const Scalar _tmp4 = _tmp0 + _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp5 = _tmp4 * _x_T_y[0];
  const Scalar _tmp6 = _x[3] * _y[0];
  const Scalar _tmp7 = _x[2] * _y[1];
  const Scalar _tmp8 = _x[1] * _y[2];
  const Scalar _tmp9 = _x[0] * _y[3];
  const Scalar _tmp10 = _tmp6 + _tmp7 - _tmp8 - _tmp9;
  const Scalar _tmp11 = _tmp10 * _x_T_y[3];
  const Scalar _tmp12 = _x[3] * _y[1];
  const Scalar _tmp13 = _x[2] * _y[0];
  const Scalar _tmp14 = _x[1] * _y[3];
  const Scalar _tmp15 = _x[0] * _y[2];
  const Scalar _tmp16 = _tmp12 - _tmp13 - _tmp14 + _tmp15;
  const Scalar _tmp17 = _tmp16 * _x_T_y[2];
  const Scalar _tmp18 = _x[3] * _y[2];
  const Scalar _tmp19 = _x[2] * _y[3];
  const Scalar _tmp20 = _x[1] * _y[0];
  const Scalar _tmp21 = _x[0] * _y[1];
  const Scalar _tmp22 = _tmp18 - _tmp19 + _tmp20 - _tmp21;
  const Scalar _tmp23 = _tmp22 * _x_T_y[1];
  const Scalar _tmp24 = _tmp11 + _tmp17 - _tmp23 - _tmp5;
  const Scalar _tmp25 = _tmp4 * _x_T_y[3];
  const Scalar _tmp26 = _tmp10 * _x_T_y[0];
  const Scalar _tmp27 = _tmp16 * _x_T_y[1];
  const Scalar _tmp28 = _tmp22 * _x_T_y[2];
  const Scalar _tmp29 = -_tmp26 - _tmp27 - _tmp28;
  const Scalar _tmp30 = _tmp25 - _tmp29;
  const Scalar _tmp31 = 2 * std::min<Scalar>(0, (((_tmp30) > 0) - ((_tmp30) < 0))) + 1;
  const Scalar _tmp32 = 2 * _tmp31;
  const Scalar _tmp33 = 1 - epsilon;
  const Scalar _tmp34 = std::min<Scalar>(_tmp33, std::fabs(_tmp30));
  const Scalar _tmp35 = std::acos(_tmp34) / std::sqrt(Scalar(1 - std::pow(_tmp34, Scalar(2))));
  const Scalar _tmp36 = _tmp32 * _tmp35;
  const Scalar _tmp37 = _tmp24 * _tmp36;
  const Scalar _tmp38 = _tmp4 * _x_T_y[1];
  const Scalar _tmp39 = _tmp10 * _x_T_y[2];
  const Scalar _tmp40 = _tmp16 * _x_T_y[3];
  const Scalar _tmp41 = _tmp22 * _x_T_y[0];
  const Scalar _tmp42 = -_tmp38 - _tmp39 + _tmp40 + _tmp41;
  const Scalar _tmp43 = _tmp36 * _tmp42;
  const Scalar _tmp44 = _tmp4 * _x_T_y[2];
  const Scalar _tmp45 = _tmp10 * _x_T_y[1];
  const Scalar _tmp46 = _tmp16 * _x_T_y[0];
  const Scalar _tmp47 = _tmp22 * _x_T_y[3];
  const Scalar _tmp48 = -_tmp44 + _tmp45 - _tmp46 + _tmp47;
  const Scalar _tmp49 = _tmp36 * _tmp48;
  const Scalar _tmp50 = _tmp32 * sqrt_info(2, 0);
  const Scalar _tmp51 = _tmp48 * sqrt_info(2, 2);
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) * _tmp19;
  const Scalar _tmp54 = (Scalar(1) / Scalar(2)) * _tmp20;
  const Scalar _tmp55 = (Scalar(1) / Scalar(2)) * _tmp21;
  const Scalar _tmp56 = -_tmp52 + _tmp53 - _tmp54 + _tmp55;
  const Scalar _tmp57 = -Scalar(1) / Scalar(2) * _tmp0 - Scalar(1) / Scalar(2) * _tmp1 -
                        Scalar(1) / Scalar(2) * _tmp2 - Scalar(1) / Scalar(2) * _tmp3;
  const Scalar _tmp58 = _tmp57 * _x_T_y[2];
  const Scalar _tmp59 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp60 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp61 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp62 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp63 = _tmp59 + _tmp60 - _tmp61 - _tmp62;
  const Scalar _tmp64 = -_tmp63 * _x_T_y[1];
  const Scalar _tmp65 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp66 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp67 = (Scalar(1) / Scalar(2)) * _tmp14;
  const Scalar _tmp68 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp69 = _tmp65 - _tmp66 - _tmp67 + _tmp68;
  const Scalar _tmp70 = -_tmp69 * _x_T_y[0];
  const Scalar _tmp71 = _tmp56 * _x_T_y[3] + _tmp58 + _tmp64 + _tmp70;
  const Scalar _tmp72 = std::fabs(_tmp25 + _tmp26 + _tmp27 + _tmp28);
  const Scalar _tmp73 = std::min<Scalar>(_tmp33, _tmp72);
  const Scalar _tmp74 = 1 - std::pow(_tmp73, Scalar(2));
  const Scalar _tmp75 = std::acos(_tmp73);
  const Scalar _tmp76 = _tmp75 / std::sqrt(_tmp74);
  const Scalar _tmp77 = _tmp32 * _tmp76;
  const Scalar _tmp78 = _tmp71 * _tmp77;
  const Scalar _tmp79 = _tmp48 * sqrt_info(0, 2);
  const Scalar _tmp80 = _tmp57 * _x_T_y[1];
  const Scalar _tmp81 = -_tmp80;
  const Scalar _tmp82 = -_tmp63 * _x_T_y[2];
  const Scalar _tmp83 = _tmp69 * _x_T_y[3];
  const Scalar _tmp84 = -_tmp56 * _x_T_y[0] + _tmp81 + _tmp82 - _tmp83;
  const Scalar _tmp85 = _tmp31 * ((((_tmp33 - _tmp72) > 0) - ((_tmp33 - _tmp72) < 0)) + 1) *
                        (((-_tmp25 + _tmp29) > 0) - ((-_tmp25 + _tmp29) < 0));
  const Scalar _tmp86 = _tmp85 / _tmp74;
  const Scalar _tmp87 = _tmp84 * _tmp86;
  const Scalar _tmp88 = _tmp57 * _x_T_y[3];
  const Scalar _tmp89 = _tmp63 * _x_T_y[0];
  const Scalar _tmp90 = _tmp69 * _x_T_y[1];
  const Scalar _tmp91 = -_tmp56 * _x_T_y[2] + _tmp88 + _tmp89 - _tmp90;
  const Scalar _tmp92 = _tmp77 * _tmp91;
  const Scalar _tmp93 = _tmp57 * _x_T_y[0];
  const Scalar _tmp94 = -_tmp93;
  const Scalar _tmp95 = _tmp63 * _x_T_y[3];
  const Scalar _tmp96 = -_tmp69 * _x_T_y[2];
  const Scalar _tmp97 = _tmp77 * (_tmp56 * _x_T_y[1] + _tmp94 + _tmp95 + _tmp96);
  const Scalar _tmp98 = _tmp73 * _tmp75 * _tmp85 / (_tmp74 * std::sqrt(_tmp74));
  const Scalar _tmp99 = _tmp42 * _tmp98;
  const Scalar _tmp100 = _tmp84 * _tmp99;
  const Scalar _tmp101 = _tmp84 * _tmp98;
  const Scalar _tmp102 = _tmp101 * _tmp24;
  const Scalar _tmp103 = _tmp24 * _tmp86;
  const Scalar _tmp104 = _tmp103 * _tmp84;
  const Scalar _tmp105 = _tmp42 * _tmp86;
  const Scalar _tmp106 = _tmp105 * _tmp84;
  const Scalar _tmp107 = _tmp100 * sqrt_info(0, 1) + _tmp101 * _tmp79 + _tmp102 * sqrt_info(0, 0) -
                         _tmp104 * sqrt_info(0, 0) - _tmp106 * sqrt_info(0, 1) +
                         _tmp78 * sqrt_info(0, 0) - _tmp79 * _tmp87 + _tmp92 * sqrt_info(0, 1) +
                         _tmp97 * sqrt_info(0, 2);
  const Scalar _tmp108 = 2 * _tmp107;
  const Scalar _tmp109 = _tmp52 - _tmp53 + _tmp54 - _tmp55;
  const Scalar _tmp110 = -_tmp109 * _x_T_y[1];
  const Scalar _tmp111 = -_tmp65 + _tmp66 + _tmp67 - _tmp68;
  const Scalar _tmp112 = _tmp110 - _tmp111 * _x_T_y[2] + _tmp94 - _tmp95;
  const Scalar _tmp113 = _tmp112 * _tmp86;
  const Scalar _tmp114 = _tmp112 * _tmp99;
  const Scalar _tmp115 = _tmp105 * _tmp112;
  const Scalar _tmp116 = _tmp109 * _x_T_y[3];
  const Scalar _tmp117 = -_tmp58;
  const Scalar _tmp118 = _tmp111 * _x_T_y[0] + _tmp116 + _tmp117 + _tmp64;
  const Scalar _tmp119 = _tmp118 * _tmp77;
  const Scalar _tmp120 = _tmp24 * _tmp98;
  const Scalar _tmp121 = _tmp120 * sqrt_info(0, 0);
  const Scalar _tmp122 = _tmp79 * _tmp98;
  const Scalar _tmp123 = -_tmp109 * _x_T_y[0];
  const Scalar _tmp124 = _tmp77 * (_tmp111 * _x_T_y[3] + _tmp123 + _tmp80 + _tmp82);
  const Scalar _tmp125 = _tmp103 * _tmp112;
  const Scalar _tmp126 = _tmp109 * _x_T_y[2];
  const Scalar _tmp127 = -_tmp111 * _x_T_y[1] + _tmp126 + _tmp88 - _tmp89;
  const Scalar _tmp128 = _tmp77 * sqrt_info(0, 0);
  const Scalar _tmp129 = _tmp112 * _tmp121 + _tmp112 * _tmp122 - _tmp113 * _tmp79 +
                         _tmp114 * sqrt_info(0, 1) - _tmp115 * sqrt_info(0, 1) +
                         _tmp119 * sqrt_info(0, 1) + _tmp124 * sqrt_info(0, 2) -
                         _tmp125 * sqrt_info(0, 0) + _tmp127 * _tmp128;
  const Scalar _tmp130 = 2 * _x[3];
  const Scalar _tmp131 = -_tmp59 - _tmp60 + _tmp61 + _tmp62;
  const Scalar _tmp132 = -_tmp116 + _tmp117 - _tmp131 * _x_T_y[1] + _tmp70;
  const Scalar _tmp133 = _tmp132 * _tmp98;
  const Scalar _tmp134 = _tmp123 + _tmp131 * _x_T_y[2] + _tmp81 + _tmp83;
  const Scalar _tmp135 = _tmp132 * _tmp99;
  const Scalar _tmp136 = _tmp110 + _tmp131 * _x_T_y[3] + _tmp93 + _tmp96;
  const Scalar _tmp137 = _tmp77 * sqrt_info(0, 1);
  const Scalar _tmp138 = _tmp103 * _tmp132;
  const Scalar _tmp139 = _tmp105 * _tmp132;
  const Scalar _tmp140 = _tmp133 * _tmp24;
  const Scalar _tmp141 = _tmp132 * _tmp86;
  const Scalar _tmp142 = _tmp77 * (-_tmp126 - _tmp131 * _x_T_y[0] + _tmp88 + _tmp90);
  const Scalar _tmp143 = _tmp128 * _tmp134 + _tmp133 * _tmp79 + _tmp135 * sqrt_info(0, 1) +
                         _tmp136 * _tmp137 - _tmp138 * sqrt_info(0, 0) - _tmp139 * sqrt_info(0, 1) +
                         _tmp140 * sqrt_info(0, 0) - _tmp141 * _tmp79 + _tmp142 * sqrt_info(0, 2);
  const Scalar _tmp144 = 2 * _x[1];
  const Scalar _tmp145 = _tmp48 * sqrt_info(1, 2);
  const Scalar _tmp146 = _tmp105 * sqrt_info(1, 1);
  const Scalar _tmp147 = _tmp100 * sqrt_info(1, 1) + _tmp101 * _tmp145 + _tmp102 * sqrt_info(1, 0) -
                         _tmp104 * sqrt_info(1, 0) - _tmp145 * _tmp87 - _tmp146 * _tmp84 +
                         _tmp78 * sqrt_info(1, 0) + _tmp92 * sqrt_info(1, 1) +
                         _tmp97 * sqrt_info(1, 2);
  const Scalar _tmp148 = 2 * _tmp147;
  const Scalar _tmp149 = _tmp112 * _tmp120;
  const Scalar _tmp150 = _tmp145 * _tmp98;
  const Scalar _tmp151 = _tmp77 * sqrt_info(1, 0);
  const Scalar _tmp152 = _tmp112 * _tmp150 - _tmp113 * _tmp145 + _tmp114 * sqrt_info(1, 1) -
                         _tmp115 * sqrt_info(1, 1) + _tmp119 * sqrt_info(1, 1) +
                         _tmp124 * sqrt_info(1, 2) - _tmp125 * sqrt_info(1, 0) + _tmp127 * _tmp151 +
                         _tmp149 * sqrt_info(1, 0);
  const Scalar _tmp153 = _tmp77 * sqrt_info(1, 1);
  const Scalar _tmp154 =
      2 * _tmp133 * _tmp145 + 2 * _tmp134 * _tmp151 + 2 * _tmp135 * sqrt_info(1, 1) +
      2 * _tmp136 * _tmp153 - 2 * _tmp138 * sqrt_info(1, 0) - 2 * _tmp139 * sqrt_info(1, 1) +
      2 * _tmp140 * sqrt_info(1, 0) - 2 * _tmp141 * _tmp145 + 2 * _tmp142 * sqrt_info(1, 2);
  const Scalar _tmp155 = _tmp50 * _tmp76;
  const Scalar _tmp156 = _tmp77 * sqrt_info(2, 1);
  const Scalar _tmp157 = _tmp99 * sqrt_info(2, 1);
  const Scalar _tmp158 = _tmp101 * _tmp51 + _tmp102 * sqrt_info(2, 0) - _tmp104 * sqrt_info(2, 0) -
                         _tmp106 * sqrt_info(2, 1) + _tmp155 * _tmp71 + _tmp156 * _tmp91 +
                         _tmp157 * _tmp84 - _tmp51 * _tmp87 + _tmp97 * sqrt_info(2, 2);
  const Scalar _tmp159 = 2 * _tmp158;
  const Scalar _tmp160 = _tmp51 * _tmp98;
  const Scalar _tmp161 = _tmp112 * _tmp160 - _tmp113 * _tmp51 + _tmp114 * sqrt_info(2, 1) -
                         _tmp115 * sqrt_info(2, 1) + _tmp118 * _tmp156 + _tmp124 * sqrt_info(2, 2) -
                         _tmp125 * sqrt_info(2, 0) + _tmp127 * _tmp155 + _tmp149 * sqrt_info(2, 0);
  const Scalar _tmp162 = _tmp133 * _tmp51 + _tmp134 * _tmp155 + _tmp135 * sqrt_info(2, 1) +
                         _tmp136 * _tmp156 - _tmp138 * sqrt_info(2, 0) - _tmp139 * sqrt_info(2, 1) +
                         _tmp140 * sqrt_info(2, 0) - _tmp141 * _tmp51 + _tmp142 * sqrt_info(2, 2);
  const Scalar _tmp163 = 2 * _tmp129;
  const Scalar _tmp164 = 2 * _tmp143;
  const Scalar _tmp165 = 2 * _tmp152;
  const Scalar _tmp166 = 2 * _tmp161;
  const Scalar _tmp167 = 2 * _tmp162;
  const Scalar _tmp168 = (Scalar(1) / Scalar(2)) * _tmp5;
  const Scalar _tmp169 = (Scalar(1) / Scalar(2)) * _tmp11;
  const Scalar _tmp170 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp171 = (Scalar(1) / Scalar(2)) * _tmp23;
  const Scalar _tmp172 = _tmp168 - _tmp169 - _tmp170 + _tmp171;
  const Scalar _tmp173 = _tmp172 * _tmp98;
  const Scalar _tmp174 = _tmp173 * _tmp24;
  const Scalar _tmp175 = _tmp172 * _tmp99;
  const Scalar _tmp176 = _tmp103 * _tmp172;
  const Scalar _tmp177 = (Scalar(1) / Scalar(2)) * _tmp38;
  const Scalar _tmp178 = (Scalar(1) / Scalar(2)) * _tmp39;
  const Scalar _tmp179 = (Scalar(1) / Scalar(2)) * _tmp40;
  const Scalar _tmp180 = (Scalar(1) / Scalar(2)) * _tmp41;
  const Scalar _tmp181 = _tmp177 + _tmp178 - _tmp179 - _tmp180;
  const Scalar _tmp182 = _tmp181 * _tmp77;
  const Scalar _tmp183 = _tmp105 * _tmp172;
  const Scalar _tmp184 = -Scalar(1) / Scalar(2) * _tmp25 - Scalar(1) / Scalar(2) * _tmp26 -
                         Scalar(1) / Scalar(2) * _tmp27 - Scalar(1) / Scalar(2) * _tmp28;
  const Scalar _tmp185 = _tmp184 * _tmp77;
  const Scalar _tmp186 = _tmp79 * _tmp86;
  const Scalar _tmp187 = (Scalar(1) / Scalar(2)) * _tmp44;
  const Scalar _tmp188 = (Scalar(1) / Scalar(2)) * _tmp45;
  const Scalar _tmp189 = (Scalar(1) / Scalar(2)) * _tmp46;
  const Scalar _tmp190 = (Scalar(1) / Scalar(2)) * _tmp47;
  const Scalar _tmp191 = -_tmp187 + _tmp188 - _tmp189 + _tmp190;
  const Scalar _tmp192 = _tmp137 * _tmp191 - _tmp172 * _tmp186 + _tmp173 * _tmp79 +
                         _tmp174 * sqrt_info(0, 0) + _tmp175 * sqrt_info(0, 1) -
                         _tmp176 * sqrt_info(0, 0) + _tmp182 * sqrt_info(0, 2) -
                         _tmp183 * sqrt_info(0, 1) + _tmp185 * sqrt_info(0, 0);
  const Scalar _tmp193 = 2 * _x_T_y[3];
  const Scalar _tmp194 = _tmp187 - _tmp188 + _tmp189 - _tmp190;
  const Scalar _tmp195 = _tmp194 * _tmp99;
  const Scalar _tmp196 = _tmp194 * _tmp86;
  const Scalar _tmp197 = _tmp196 * _tmp24;
  const Scalar _tmp198 = -_tmp177 - _tmp178 + _tmp179 + _tmp180;
  const Scalar _tmp199 = _tmp105 * _tmp194;
  const Scalar _tmp200 = _tmp121 * _tmp194 + _tmp122 * _tmp194 + _tmp128 * _tmp198 +
                         _tmp137 * _tmp172 + _tmp185 * sqrt_info(0, 2) + _tmp195 * sqrt_info(0, 1) -
                         _tmp196 * _tmp79 - _tmp197 * sqrt_info(0, 0) - _tmp199 * sqrt_info(0, 1);
  const Scalar _tmp201 = 2 * _x_T_y[1];
  const Scalar _tmp202 = _tmp77 * (-_tmp168 + _tmp169 + _tmp170 - _tmp171);
  const Scalar _tmp203 = _tmp103 * _tmp181;
  const Scalar _tmp204 = _tmp181 * _tmp99;
  const Scalar _tmp205 = _tmp105 * _tmp181;
  const Scalar _tmp206 = 2 * _tmp121 * _tmp181 + 2 * _tmp122 * _tmp181 + 2 * _tmp128 * _tmp194 -
                         2 * _tmp181 * _tmp186 + 2 * _tmp185 * sqrt_info(0, 1) +
                         2 * _tmp202 * sqrt_info(0, 2) - 2 * _tmp203 * sqrt_info(0, 0) +
                         2 * _tmp204 * sqrt_info(0, 1) - 2 * _tmp205 * sqrt_info(0, 1);
  const Scalar _tmp207 = _tmp172 * _tmp86;
  const Scalar _tmp208 = _tmp145 * _tmp173 - _tmp145 * _tmp207 - _tmp146 * _tmp172 +
                         _tmp153 * _tmp191 + _tmp174 * sqrt_info(1, 0) + _tmp175 * sqrt_info(1, 1) -
                         _tmp176 * sqrt_info(1, 0) + _tmp182 * sqrt_info(1, 2) +
                         _tmp185 * sqrt_info(1, 0);
  const Scalar _tmp209 = _tmp120 * _tmp194;
  const Scalar _tmp210 = -_tmp145 * _tmp196 - _tmp146 * _tmp194 + _tmp150 * _tmp194 +
                         _tmp151 * _tmp198 + _tmp153 * _tmp172 + _tmp185 * sqrt_info(1, 2) +
                         _tmp195 * sqrt_info(1, 1) - _tmp197 * sqrt_info(1, 0) +
                         _tmp209 * sqrt_info(1, 0);
  const Scalar _tmp211 = _tmp120 * _tmp181;
  const Scalar _tmp212 = _tmp181 * _tmp86;
  const Scalar _tmp213 = -2 * _tmp145 * _tmp212 - 2 * _tmp146 * _tmp181 + 2 * _tmp150 * _tmp181 +
                         2 * _tmp151 * _tmp194 + 2 * _tmp185 * sqrt_info(1, 1) +
                         2 * _tmp202 * sqrt_info(1, 2) - 2 * _tmp203 * sqrt_info(1, 0) +
                         2 * _tmp204 * sqrt_info(1, 1) + 2 * _tmp211 * sqrt_info(1, 0);
  const Scalar _tmp214 = _tmp155 * _tmp184 + _tmp156 * _tmp191 + _tmp157 * _tmp172 +
                         _tmp173 * _tmp51 + _tmp174 * sqrt_info(2, 0) - _tmp176 * sqrt_info(2, 0) +
                         _tmp182 * sqrt_info(2, 2) - _tmp183 * sqrt_info(2, 1) - _tmp207 * _tmp51;
  const Scalar _tmp215 = _tmp155 * _tmp198 + _tmp156 * _tmp172 + _tmp157 * _tmp194 +
                         _tmp160 * _tmp194 + _tmp185 * sqrt_info(2, 2) - _tmp196 * _tmp51 -
                         _tmp197 * sqrt_info(2, 0) - _tmp199 * sqrt_info(2, 1) +
                         _tmp209 * sqrt_info(2, 0);
  const Scalar _tmp216 = _tmp155 * _tmp194 + _tmp156 * _tmp184 + _tmp157 * _tmp181 +
                         _tmp160 * _tmp181 + _tmp202 * sqrt_info(2, 2) - _tmp203 * sqrt_info(2, 0) -
                         _tmp205 * sqrt_info(2, 1) + _tmp211 * sqrt_info(2, 0) - _tmp212 * _tmp51;
  const Scalar _tmp217 = 2 * _tmp216;
  const Scalar _tmp218 = 2 * _tmp192;
  const Scalar _tmp219 = 2 * _tmp200;
  const Scalar _tmp220 = 2 * _tmp208;
  const Scalar _tmp221 = 2 * _tmp210;
  const Scalar _tmp222 = 2 * _tmp214;
  const Scalar _tmp223 = 2 * _tmp215;

  // Output terms (3)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp37 * sqrt_info(0, 0) + _tmp43 * sqrt_info(0, 1) + _tmp49 * sqrt_info(0, 2);
  _res(1, 0) = _tmp37 * sqrt_info(1, 0) + _tmp43 * sqrt_info(1, 1) + _tmp49 * sqrt_info(1, 2);
  _res(2, 0) = _tmp24 * _tmp35 * _tmp50 + _tmp36 * _tmp51 + _tmp43 * sqrt_info(2, 1);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 7, Eigen::RowMajor>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = -_tmp108 * _x[2] + _tmp129 * _tmp130 + _tmp143 * _tmp144;
    _res_D_x(1, 0) = _tmp130 * _tmp152 - _tmp148 * _x[2] + _tmp154 * _x[1];
    _res_D_x(2, 0) = _tmp130 * _tmp161 + _tmp144 * _tmp162 - _tmp159 * _x[2];
    _res_D_x(0, 1) = _tmp107 * _tmp130 + _tmp163 * _x[2] - _tmp164 * _x[0];
    _res_D_x(1, 1) = _tmp130 * _tmp147 - _tmp154 * _x[0] + _tmp165 * _x[2];
    _res_D_x(2, 1) = _tmp130 * _tmp158 + _tmp166 * _x[2] - _tmp167 * _x[0];
    _res_D_x(0, 2) = _tmp108 * _x[0] - _tmp129 * _tmp144 + _tmp130 * _tmp143;
    _res_D_x(1, 2) = -_tmp144 * _tmp152 + _tmp148 * _x[0] + _tmp154 * _x[3];
    _res_D_x(2, 2) = -_tmp144 * _tmp161 + _tmp159 * _x[0] + _tmp167 * _x[3];
    _res_D_x(0, 3) = -_tmp107 * _tmp144 - _tmp163 * _x[0] - _tmp164 * _x[2];
    _res_D_x(1, 3) = -_tmp144 * _tmp147 - _tmp154 * _x[2] - _tmp165 * _x[0];
    _res_D_x(2, 3) = -_tmp144 * _tmp158 - _tmp166 * _x[0] - _tmp167 * _x[2];
    _res_D_x(0, 4) = 0;
    _res_D_x(1, 4) = 0;
    _res_D_x(2, 4) = 0;
    _res_D_x(0, 5) = 0;
    _res_D_x(1, 5) = 0;
    _res_D_x(2, 5) = 0;
    _res_D_x(0, 6) = 0;
    _res_D_x(1, 6) = 0;
    _res_D_x(2, 6) = 0;
  }

  if (res_D_x_T_y != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>> _res_D_x_T_y{res_D_x_T_y};

    _res_D_x_T_y(0, 0) = _tmp192 * _tmp193 + _tmp200 * _tmp201 - _tmp206 * _x_T_y[2];
    _res_D_x_T_y(1, 0) = _tmp193 * _tmp208 + _tmp201 * _tmp210 - _tmp213 * _x_T_y[2];
    _res_D_x_T_y(2, 0) = _tmp193 * _tmp214 + _tmp201 * _tmp215 - _tmp217 * _x_T_y[2];
    _res_D_x_T_y(0, 1) = _tmp206 * _x_T_y[3] + _tmp218 * _x_T_y[2] - _tmp219 * _x_T_y[0];
    _res_D_x_T_y(1, 1) = _tmp213 * _x_T_y[3] + _tmp220 * _x_T_y[2] - _tmp221 * _x_T_y[0];
    _res_D_x_T_y(2, 1) = _tmp193 * _tmp216 + _tmp222 * _x_T_y[2] - _tmp223 * _x_T_y[0];
    _res_D_x_T_y(0, 2) = -_tmp192 * _tmp201 + _tmp193 * _tmp200 + _tmp206 * _x_T_y[0];
    _res_D_x_T_y(1, 2) = _tmp193 * _tmp210 - _tmp201 * _tmp208 + _tmp213 * _x_T_y[0];
    _res_D_x_T_y(2, 2) = _tmp193 * _tmp215 - _tmp201 * _tmp214 + _tmp217 * _x_T_y[0];
    _res_D_x_T_y(0, 3) = -_tmp206 * _x_T_y[1] - _tmp218 * _x_T_y[0] - _tmp219 * _x_T_y[2];
    _res_D_x_T_y(1, 3) = -_tmp213 * _x_T_y[1] - _tmp220 * _x_T_y[0] - _tmp221 * _x_T_y[2];
    _res_D_x_T_y(2, 3) = -_tmp201 * _tmp216 - _tmp222 * _x_T_y[0] - _tmp223 * _x_T_y[2];
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres