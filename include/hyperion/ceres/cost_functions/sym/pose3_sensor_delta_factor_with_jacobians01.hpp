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
 *     res_D_x: (6x7) jacobian (result_dim x storage_dim) of res (6) wrt arg x (7) (row-major)
 *     res_D_x_T_y: (6x7) jacobian (result_dim x storage_dim) of res (6) wrt arg x_T_y (7)
 * (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> Pose3SensorDeltaFactorWithJacobians01(
    const sym::Pose3<Scalar>& x, const sym::Pose3<Scalar>& x_T_y, const sym::Pose3<Scalar>& y,
    const Eigen::Matrix<Scalar, 6, 6>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr, Scalar* const res_D_x_T_y = nullptr) {
  // Total ops: 1615

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_y = x_T_y.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (340)
  const Scalar _tmp0 = _x[3] * _y[2];
  const Scalar _tmp1 = _x[2] * _y[3];
  const Scalar _tmp2 = _x[1] * _y[0];
  const Scalar _tmp3 = _x[0] * _y[1];
  const Scalar _tmp4 = _tmp0 - _tmp1 + _tmp2 - _tmp3;
  const Scalar _tmp5 = _tmp4 * _x_T_y[2];
  const Scalar _tmp6 = _x[3] * _y[0];
  const Scalar _tmp7 = _x[2] * _y[1];
  const Scalar _tmp8 = _x[1] * _y[2];
  const Scalar _tmp9 = _x[0] * _y[3];
  const Scalar _tmp10 = _tmp6 + _tmp7 - _tmp8 - _tmp9;
  const Scalar _tmp11 = _tmp10 * _x_T_y[0];
  const Scalar _tmp12 = _x[3] * _y[1];
  const Scalar _tmp13 = _x[2] * _y[0];
  const Scalar _tmp14 = _x[1] * _y[3];
  const Scalar _tmp15 = _x[0] * _y[2];
  const Scalar _tmp16 = _tmp12 - _tmp13 - _tmp14 + _tmp15;
  const Scalar _tmp17 = _tmp16 * _x_T_y[1];
  const Scalar _tmp18 = -_tmp11 - _tmp17 - _tmp5;
  const Scalar _tmp19 = _x[2] * _y[2];
  const Scalar _tmp20 = _x[1] * _y[1];
  const Scalar _tmp21 = _x[0] * _y[0];
  const Scalar _tmp22 = _x[3] * _y[3];
  const Scalar _tmp23 = _tmp19 + _tmp20 + _tmp21 + _tmp22;
  const Scalar _tmp24 = _tmp23 * _x_T_y[3];
  const Scalar _tmp25 =
      2 * std::min<Scalar>(0, (((-_tmp18 + _tmp24) > 0) - ((-_tmp18 + _tmp24) < 0))) + 1;
  const Scalar _tmp26 = 2 * _tmp25;
  const Scalar _tmp27 = _tmp23 * _x_T_y[0];
  const Scalar _tmp28 = _tmp4 * _x_T_y[1];
  const Scalar _tmp29 = _tmp10 * _x_T_y[3];
  const Scalar _tmp30 = _tmp16 * _x_T_y[2];
  const Scalar _tmp31 = -_tmp27 - _tmp28 + _tmp29 + _tmp30;
  const Scalar _tmp32 = 1 - epsilon;
  const Scalar _tmp33 = std::min<Scalar>(_tmp32, std::fabs(_tmp18 - _tmp24));
  const Scalar _tmp34 = std::acos(_tmp33) / std::sqrt(Scalar(1 - std::pow(_tmp33, Scalar(2))));
  const Scalar _tmp35 = _tmp31 * _tmp34;
  const Scalar _tmp36 = _tmp26 * _tmp35;
  const Scalar _tmp37 = std::pow(_x[2], Scalar(2));
  const Scalar _tmp38 = 2 * _tmp37;
  const Scalar _tmp39 = -_tmp38;
  const Scalar _tmp40 = std::pow(_x[0], Scalar(2));
  const Scalar _tmp41 = 2 * _tmp40;
  const Scalar _tmp42 = 1 - _tmp41;
  const Scalar _tmp43 = _tmp39 + _tmp42;
  const Scalar _tmp44 = 2 * _x[1];
  const Scalar _tmp45 = _tmp44 * _x[0];
  const Scalar _tmp46 = 2 * _x[3];
  const Scalar _tmp47 = _tmp46 * _x[2];
  const Scalar _tmp48 = -_tmp47;
  const Scalar _tmp49 = _tmp45 + _tmp48;
  const Scalar _tmp50 = _tmp49 * _x[4];
  const Scalar _tmp51 = _tmp46 * _x[0];
  const Scalar _tmp52 = _tmp44 * _x[2];
  const Scalar _tmp53 = _tmp51 + _tmp52;
  const Scalar _tmp54 = _tmp53 * _x[6];
  const Scalar _tmp55 = _tmp49 * _y[4] + _tmp53 * _y[6];
  const Scalar _tmp56 = -_tmp43 * _x[5] + _tmp43 * _y[5] - _tmp50 - _tmp54 + _tmp55 - _x_T_y[5];
  const Scalar _tmp57 = _tmp23 * _x_T_y[1];
  const Scalar _tmp58 = _tmp4 * _x_T_y[0];
  const Scalar _tmp59 = _tmp10 * _x_T_y[2];
  const Scalar _tmp60 = _tmp16 * _x_T_y[3];
  const Scalar _tmp61 = -_tmp57 + _tmp58 - _tmp59 + _tmp60;
  const Scalar _tmp62 = _tmp34 * _tmp61;
  const Scalar _tmp63 = _tmp26 * _tmp62;
  const Scalar _tmp64 = std::pow(_x[1], Scalar(2));
  const Scalar _tmp65 = 2 * _tmp64;
  const Scalar _tmp66 = -_tmp65;
  const Scalar _tmp67 = _tmp42 + _tmp66;
  const Scalar _tmp68 = 2 * _x[0];
  const Scalar _tmp69 = _tmp68 * _x[2];
  const Scalar _tmp70 = _tmp44 * _x[3];
  const Scalar _tmp71 = _tmp69 + _tmp70;
  const Scalar _tmp72 = _tmp71 * _x[4];
  const Scalar _tmp73 = -_tmp51;
  const Scalar _tmp74 = _tmp52 + _tmp73;
  const Scalar _tmp75 = _tmp74 * _x[5];
  const Scalar _tmp76 = _tmp71 * _y[4] + _tmp74 * _y[5];
  const Scalar _tmp77 = -_tmp67 * _x[6] + _tmp67 * _y[6] - _tmp72 - _tmp75 + _tmp76 - _x_T_y[6];
  const Scalar _tmp78 = _tmp39 + _tmp66 + 1;
  const Scalar _tmp79 = _tmp45 + _tmp47;
  const Scalar _tmp80 = _tmp79 * _x[5];
  const Scalar _tmp81 = -_tmp70;
  const Scalar _tmp82 = _tmp69 + _tmp81;
  const Scalar _tmp83 = _tmp82 * _x[6];
  const Scalar _tmp84 = _tmp79 * _y[5] + _tmp82 * _y[6];
  const Scalar _tmp85 = -_tmp78 * _x[4] + _tmp78 * _y[4] - _tmp80 - _tmp83 + _tmp84 - _x_T_y[4];
  const Scalar _tmp86 = _tmp26 * sqrt_info(0, 2);
  const Scalar _tmp87 = _tmp23 * _x_T_y[2];
  const Scalar _tmp88 = _tmp4 * _x_T_y[3];
  const Scalar _tmp89 = _tmp10 * _x_T_y[1];
  const Scalar _tmp90 = _tmp16 * _x_T_y[0];
  const Scalar _tmp91 = -_tmp87 + _tmp88 + _tmp89 - _tmp90;
  const Scalar _tmp92 = _tmp34 * _tmp91;
  const Scalar _tmp93 = _tmp26 * sqrt_info(1, 2);
  const Scalar _tmp94 = _tmp26 * _tmp92;
  const Scalar _tmp95 = _tmp26 * sqrt_info(3, 2);
  const Scalar _tmp96 = _tmp26 * sqrt_info(4, 0);
  const Scalar _tmp97 = _tmp26 * sqrt_info(4, 1);
  const Scalar _tmp98 = -Scalar(1) / Scalar(2) * _tmp19 - Scalar(1) / Scalar(2) * _tmp20 -
                        Scalar(1) / Scalar(2) * _tmp21 - Scalar(1) / Scalar(2) * _tmp22;
  const Scalar _tmp99 = _tmp98 * _x_T_y[2];
  const Scalar _tmp100 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp101 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp102 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp103 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp104 = _tmp100 + _tmp101 - _tmp102 - _tmp103;
  const Scalar _tmp105 = -_tmp104 * _x_T_y[1];
  const Scalar _tmp106 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp107 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp108 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp109 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp110 = _tmp106 - _tmp107 + _tmp108 - _tmp109;
  const Scalar _tmp111 = _tmp110 * _x_T_y[3];
  const Scalar _tmp112 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp113 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp114 = (Scalar(1) / Scalar(2)) * _tmp14;
  const Scalar _tmp115 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp116 = -_tmp112 + _tmp113 + _tmp114 - _tmp115;
  const Scalar _tmp117 = _tmp11 + _tmp17 + _tmp24 + _tmp5;
  const Scalar _tmp118 = std::fabs(_tmp117);
  const Scalar _tmp119 = std::min<Scalar>(_tmp118, _tmp32);
  const Scalar _tmp120 = std::acos(_tmp119);
  const Scalar _tmp121 = 1 - std::pow(_tmp119, Scalar(2));
  const Scalar _tmp122 = std::pow(_tmp121, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp123 = _tmp120 * _tmp122;
  const Scalar _tmp124 = _tmp123 * (_tmp105 + _tmp111 + _tmp116 * _x_T_y[0] - _tmp99);
  const Scalar _tmp125 = _tmp124 * _tmp26;
  const Scalar _tmp126 = _tmp98 * _x_T_y[0];
  const Scalar _tmp127 = _tmp104 * _x_T_y[3];
  const Scalar _tmp128 = _tmp110 * _x_T_y[1];
  const Scalar _tmp129 = _tmp116 * _x_T_y[2] + _tmp126 + _tmp127 + _tmp128;
  const Scalar _tmp130 = _tmp25 * ((((-_tmp118 + _tmp32) > 0) - ((-_tmp118 + _tmp32) < 0)) + 1) *
                         (((_tmp117) > 0) - ((_tmp117) < 0));
  const Scalar _tmp131 = _tmp119 * _tmp130 / (_tmp121 * std::sqrt(_tmp121));
  const Scalar _tmp132 = _tmp120 * _tmp131;
  const Scalar _tmp133 = _tmp129 * _tmp132;
  const Scalar _tmp134 = _tmp133 * _tmp91;
  const Scalar _tmp135 = _tmp130 / _tmp121;
  const Scalar _tmp136 = _tmp135 * _tmp91;
  const Scalar _tmp137 = _tmp129 * _tmp136;
  const Scalar _tmp138 = _tmp135 * _tmp31;
  const Scalar _tmp139 = _tmp138 * sqrt_info(0, 0);
  const Scalar _tmp140 = _tmp98 * _x_T_y[3];
  const Scalar _tmp141 = _tmp104 * _x_T_y[0];
  const Scalar _tmp142 = _tmp110 * _x_T_y[2];
  const Scalar _tmp143 = -_tmp116 * _x_T_y[1] + _tmp140 - _tmp141 + _tmp142;
  const Scalar _tmp144 = _tmp123 * _tmp26;
  const Scalar _tmp145 = _tmp143 * _tmp144;
  const Scalar _tmp146 = _tmp129 * _tmp135;
  const Scalar _tmp147 = _tmp146 * _tmp61;
  const Scalar _tmp148 = _tmp98 * _x_T_y[1];
  const Scalar _tmp149 = _tmp104 * _x_T_y[2];
  const Scalar _tmp150 = -_tmp110 * _x_T_y[0];
  const Scalar _tmp151 = _tmp123 * (_tmp116 * _x_T_y[3] + _tmp148 - _tmp149 + _tmp150);
  const Scalar _tmp152 = _tmp31 * sqrt_info(0, 0);
  const Scalar _tmp153 = _tmp61 * sqrt_info(0, 1);
  const Scalar _tmp154 = -_tmp45;
  const Scalar _tmp155 = _tmp154 + _tmp47;
  const Scalar _tmp156 = -_tmp64;
  const Scalar _tmp157 = _tmp156 + _tmp37;
  const Scalar _tmp158 = std::pow(_x[3], Scalar(2));
  const Scalar _tmp159 = -_tmp158;
  const Scalar _tmp160 = _tmp159 + _tmp40;
  const Scalar _tmp161 = _tmp157 + _tmp160;
  const Scalar _tmp162 = -_tmp52;
  const Scalar _tmp163 = _tmp162 + _tmp73;
  const Scalar _tmp164 = -_tmp155 * _x[4] + _tmp155 * _y[4] - _tmp161 * _x[5] + _tmp161 * _y[5] -
                         _tmp163 * _x[6] + _tmp163 * _y[6];
  const Scalar _tmp165 = -_tmp40;
  const Scalar _tmp166 = _tmp158 + _tmp165;
  const Scalar _tmp167 = _tmp157 + _tmp166;
  const Scalar _tmp168 = -_tmp167 * _x[6] + _tmp167 * _y[6] - _tmp72 - _tmp75 + _tmp76;
  const Scalar _tmp169 = _tmp125 * sqrt_info(0, 1) - _tmp129 * _tmp139 + _tmp133 * _tmp152 +
                         _tmp133 * _tmp153 + _tmp134 * sqrt_info(0, 2) - _tmp137 * sqrt_info(0, 2) +
                         _tmp145 * sqrt_info(0, 0) - _tmp147 * sqrt_info(0, 1) + _tmp151 * _tmp86 +
                         _tmp164 * sqrt_info(0, 5) + _tmp168 * sqrt_info(0, 4);
  const Scalar _tmp170 = -_tmp100 - _tmp101 + _tmp102 + _tmp103;
  const Scalar _tmp171 = _tmp112 - _tmp113 - _tmp114 + _tmp115;
  const Scalar _tmp172 = _tmp171 * _x_T_y[0];
  const Scalar _tmp173 = _tmp111 + _tmp170 * _x_T_y[1] + _tmp172 + _tmp99;
  const Scalar _tmp174 = _tmp132 * _tmp173;
  const Scalar _tmp175 = _tmp174 * _tmp91;
  const Scalar _tmp176 = _tmp171 * _x_T_y[3];
  const Scalar _tmp177 = -_tmp148 + _tmp150 + _tmp170 * _x_T_y[2] + _tmp176;
  const Scalar _tmp178 = _tmp144 * _tmp177;
  const Scalar _tmp179 = _tmp136 * _tmp173;
  const Scalar _tmp180 = _tmp174 * _tmp61;
  const Scalar _tmp181 = _tmp135 * _tmp173;
  const Scalar _tmp182 = _tmp181 * _tmp61;
  const Scalar _tmp183 = _tmp171 * _x_T_y[1];
  const Scalar _tmp184 = _tmp140 - _tmp142 - _tmp170 * _x_T_y[0] + _tmp183;
  const Scalar _tmp185 = _tmp123 * _tmp184;
  const Scalar _tmp186 = -_tmp171 * _x_T_y[2];
  const Scalar _tmp187 = _tmp126 - _tmp128 + _tmp170 * _x_T_y[3] + _tmp186;
  const Scalar _tmp188 = _tmp144 * _tmp187;
  const Scalar _tmp189 = -_tmp37;
  const Scalar _tmp190 = _tmp189 + _tmp64;
  const Scalar _tmp191 = _tmp166 + _tmp190;
  const Scalar _tmp192 = -_tmp191 * _x[5] + _tmp191 * _y[5] - _tmp50 - _tmp54 + _tmp55;
  const Scalar _tmp193 = _tmp154 + _tmp48;
  const Scalar _tmp194 = -_tmp69;
  const Scalar _tmp195 = _tmp194 + _tmp70;
  const Scalar _tmp196 = _tmp159 + _tmp165 + _tmp37 + _tmp64;
  const Scalar _tmp197 = -_tmp193 * _x[5] + _tmp193 * _y[5] - _tmp195 * _x[6] + _tmp195 * _y[6] -
                         _tmp196 * _x[4] + _tmp196 * _y[4];
  const Scalar _tmp198 = -_tmp139 * _tmp173 + _tmp152 * _tmp174 + _tmp175 * sqrt_info(0, 2) +
                         _tmp178 * sqrt_info(0, 0) - _tmp179 * sqrt_info(0, 2) +
                         _tmp180 * sqrt_info(0, 1) - _tmp182 * sqrt_info(0, 1) + _tmp185 * _tmp86 +
                         _tmp188 * sqrt_info(0, 1) + _tmp192 * sqrt_info(0, 3) +
                         _tmp197 * sqrt_info(0, 4);
  const Scalar _tmp199 = -_tmp106 + _tmp107 - _tmp108 + _tmp109;
  const Scalar _tmp200 = -_tmp126 + _tmp127 + _tmp186 + _tmp199 * _x_T_y[1];
  const Scalar _tmp201 = _tmp123 * _tmp200;
  const Scalar _tmp202 = _tmp105 - _tmp172 + _tmp199 * _x_T_y[3] + _tmp99;
  const Scalar _tmp203 = _tmp144 * _tmp202;
  const Scalar _tmp204 = _tmp148 + _tmp149 + _tmp176 + _tmp199 * _x_T_y[0];
  const Scalar _tmp205 = _tmp132 * _tmp204;
  const Scalar _tmp206 = _tmp205 * _tmp91;
  const Scalar _tmp207 = _tmp136 * _tmp204;
  const Scalar _tmp208 = _tmp135 * _tmp204;
  const Scalar _tmp209 = _tmp208 * _tmp61;
  const Scalar _tmp210 = _tmp140 + _tmp141 - _tmp183 - _tmp199 * _x_T_y[2];
  const Scalar _tmp211 = _tmp144 * _tmp210;
  const Scalar _tmp212 = _tmp138 * _tmp204;
  const Scalar _tmp213 = _tmp194 + _tmp81;
  const Scalar _tmp214 = _tmp160 + _tmp190;
  const Scalar _tmp215 = _tmp162 + _tmp51;
  const Scalar _tmp216 = -_tmp213 * _x[4] + _tmp213 * _y[4] - _tmp214 * _x[6] + _tmp214 * _y[6] -
                         _tmp215 * _x[5] + _tmp215 * _y[5];
  const Scalar _tmp217 = _tmp156 + _tmp158 + _tmp189 + _tmp40;
  const Scalar _tmp218 = -_tmp217 * _x[4] + _tmp217 * _y[4] - _tmp80 - _tmp83 + _tmp84;
  const Scalar _tmp219 = _tmp152 * _tmp205 + _tmp153 * _tmp205 + _tmp201 * _tmp86 +
                         _tmp203 * sqrt_info(0, 0) + _tmp206 * sqrt_info(0, 2) -
                         _tmp207 * sqrt_info(0, 2) - _tmp209 * sqrt_info(0, 1) +
                         _tmp211 * sqrt_info(0, 1) - _tmp212 * sqrt_info(0, 0) +
                         _tmp216 * sqrt_info(0, 3) + _tmp218 * sqrt_info(0, 5);
  const Scalar _tmp220 = 2 * _x[2];
  const Scalar _tmp221 = _tmp91 * sqrt_info(1, 2);
  const Scalar _tmp222 = _tmp129 * _tmp138;
  const Scalar _tmp223 = _tmp151 * _tmp26;
  const Scalar _tmp224 = _tmp133 * _tmp31;
  const Scalar _tmp225 = _tmp61 * sqrt_info(1, 1);
  const Scalar _tmp226 = _tmp125 * sqrt_info(1, 1) + _tmp133 * _tmp221 + _tmp133 * _tmp225 -
                         _tmp137 * sqrt_info(1, 2) + _tmp145 * sqrt_info(1, 0) -
                         _tmp147 * sqrt_info(1, 1) + _tmp164 * sqrt_info(1, 5) +
                         _tmp168 * sqrt_info(1, 4) - _tmp222 * sqrt_info(1, 0) +
                         _tmp223 * sqrt_info(1, 2) + _tmp224 * sqrt_info(1, 0);
  const Scalar _tmp227 = _tmp174 * _tmp31;
  const Scalar _tmp228 = _tmp138 * _tmp173;
  const Scalar _tmp229 = _tmp174 * _tmp221 + _tmp174 * _tmp225 + _tmp178 * sqrt_info(1, 0) -
                         _tmp179 * sqrt_info(1, 2) - _tmp182 * sqrt_info(1, 1) + _tmp185 * _tmp93 +
                         _tmp188 * sqrt_info(1, 1) + _tmp192 * sqrt_info(1, 3) +
                         _tmp197 * sqrt_info(1, 4) + _tmp227 * sqrt_info(1, 0) -
                         _tmp228 * sqrt_info(1, 0);
  const Scalar _tmp230 = _tmp205 * _tmp31;
  const Scalar _tmp231 = _tmp201 * _tmp93 + _tmp203 * sqrt_info(1, 0) + _tmp205 * _tmp221 +
                         _tmp205 * _tmp225 - _tmp207 * sqrt_info(1, 2) - _tmp209 * sqrt_info(1, 1) +
                         _tmp211 * sqrt_info(1, 1) - _tmp212 * sqrt_info(1, 0) +
                         _tmp216 * sqrt_info(1, 3) + _tmp218 * sqrt_info(1, 5) +
                         _tmp230 * sqrt_info(1, 0);
  const Scalar _tmp232 = _tmp61 * sqrt_info(2, 1);
  const Scalar _tmp233 = _tmp31 * sqrt_info(2, 0);
  const Scalar _tmp234 = _tmp125 * sqrt_info(2, 1) + _tmp133 * _tmp232 + _tmp133 * _tmp233 +
                         _tmp134 * sqrt_info(2, 2) - _tmp137 * sqrt_info(2, 2) +
                         _tmp145 * sqrt_info(2, 0) - _tmp146 * _tmp232 + _tmp164 * sqrt_info(2, 5) +
                         _tmp168 * sqrt_info(2, 4) - _tmp222 * sqrt_info(2, 0) +
                         _tmp223 * sqrt_info(2, 2);
  const Scalar _tmp235 = _tmp144 * _tmp184;
  const Scalar _tmp236 = _tmp174 * _tmp232 + _tmp174 * _tmp233 + _tmp175 * sqrt_info(2, 2) +
                         _tmp178 * sqrt_info(2, 0) - _tmp179 * sqrt_info(2, 2) - _tmp181 * _tmp232 +
                         _tmp188 * sqrt_info(2, 1) + _tmp192 * sqrt_info(2, 3) +
                         _tmp197 * sqrt_info(2, 4) - _tmp228 * sqrt_info(2, 0) +
                         _tmp235 * sqrt_info(2, 2);
  const Scalar _tmp237 = _tmp144 * sqrt_info(2, 2);
  const Scalar _tmp238 =
      2 * _tmp200 * _tmp237 + 2 * _tmp203 * sqrt_info(2, 0) + 2 * _tmp205 * _tmp232 +
      2 * _tmp205 * _tmp233 + 2 * _tmp206 * sqrt_info(2, 2) - 2 * _tmp207 * sqrt_info(2, 2) -
      2 * _tmp208 * _tmp232 + 2 * _tmp211 * sqrt_info(2, 1) - 2 * _tmp212 * sqrt_info(2, 0) +
      2 * _tmp216 * sqrt_info(2, 3) + 2 * _tmp218 * sqrt_info(2, 5);
  const Scalar _tmp239 = _tmp91 * sqrt_info(3, 2);
  const Scalar _tmp240 = _tmp61 * sqrt_info(3, 1);
  const Scalar _tmp241 = _tmp125 * sqrt_info(3, 1) + _tmp133 * _tmp239 + _tmp133 * _tmp240 -
                         _tmp137 * sqrt_info(3, 2) + _tmp145 * sqrt_info(3, 0) - _tmp146 * _tmp240 +
                         _tmp151 * _tmp95 + _tmp164 * sqrt_info(3, 5) + _tmp168 * sqrt_info(3, 4) -
                         _tmp222 * sqrt_info(3, 0) + _tmp224 * sqrt_info(3, 0);
  const Scalar _tmp242 = _tmp144 * sqrt_info(3, 1);
  const Scalar _tmp243 = _tmp174 * _tmp239 + _tmp174 * _tmp240 + _tmp178 * sqrt_info(3, 0) -
                         _tmp179 * sqrt_info(3, 2) - _tmp182 * sqrt_info(3, 1) + _tmp185 * _tmp95 +
                         _tmp187 * _tmp242 + _tmp192 * sqrt_info(3, 3) + _tmp197 * sqrt_info(3, 4) +
                         _tmp227 * sqrt_info(3, 0) - _tmp228 * sqrt_info(3, 0);
  const Scalar _tmp244 = _tmp201 * _tmp95 + _tmp203 * sqrt_info(3, 0) + _tmp205 * _tmp239 +
                         _tmp205 * _tmp240 - _tmp207 * sqrt_info(3, 2) - _tmp208 * _tmp240 +
                         _tmp210 * _tmp242 - _tmp212 * sqrt_info(3, 0) + _tmp216 * sqrt_info(3, 3) +
                         _tmp218 * sqrt_info(3, 5) + _tmp230 * sqrt_info(3, 0);
  const Scalar _tmp245 = 2 * _tmp244;
  const Scalar _tmp246 = _tmp91 * sqrt_info(4, 2);
  const Scalar _tmp247 = _tmp123 * _tmp96;
  const Scalar _tmp248 = _tmp61 * sqrt_info(4, 1);
  const Scalar _tmp249 = _tmp124 * _tmp97 + _tmp133 * _tmp246 + _tmp133 * _tmp248 -
                         _tmp137 * sqrt_info(4, 2) + _tmp143 * _tmp247 - _tmp147 * sqrt_info(4, 1) +
                         _tmp164 * sqrt_info(4, 5) + _tmp168 * sqrt_info(4, 4) -
                         _tmp222 * sqrt_info(4, 0) + _tmp223 * sqrt_info(4, 2) +
                         _tmp224 * sqrt_info(4, 0);
  const Scalar _tmp250 = _tmp123 * _tmp97;
  const Scalar _tmp251 = _tmp174 * _tmp246 + _tmp177 * _tmp247 - _tmp179 * sqrt_info(4, 2) +
                         _tmp180 * sqrt_info(4, 1) - _tmp182 * sqrt_info(4, 1) + _tmp187 * _tmp250 +
                         _tmp192 * sqrt_info(4, 3) + _tmp197 * sqrt_info(4, 4) +
                         _tmp227 * sqrt_info(4, 0) - _tmp228 * sqrt_info(4, 0) +
                         _tmp235 * sqrt_info(4, 2);
  const Scalar _tmp252 = _tmp144 * _tmp200;
  const Scalar _tmp253 = _tmp202 * _tmp247 + _tmp205 * _tmp246 + _tmp205 * _tmp248 -
                         _tmp207 * sqrt_info(4, 2) - _tmp209 * sqrt_info(4, 1) + _tmp210 * _tmp250 -
                         _tmp212 * sqrt_info(4, 0) + _tmp216 * sqrt_info(4, 3) +
                         _tmp218 * sqrt_info(4, 5) + _tmp230 * sqrt_info(4, 0) +
                         _tmp252 * sqrt_info(4, 2);
  const Scalar _tmp254 = 2 * _tmp253;
  const Scalar _tmp255 = _tmp91 * sqrt_info(5, 2);
  const Scalar _tmp256 = _tmp61 * sqrt_info(5, 1);
  const Scalar _tmp257 = _tmp125 * sqrt_info(5, 1) + _tmp133 * _tmp255 + _tmp133 * _tmp256 -
                         _tmp137 * sqrt_info(5, 2) + _tmp145 * sqrt_info(5, 0) - _tmp146 * _tmp256 +
                         _tmp164 * sqrt_info(5, 5) + _tmp168 * sqrt_info(5, 4) -
                         _tmp222 * sqrt_info(5, 0) + _tmp223 * sqrt_info(5, 2) +
                         _tmp224 * sqrt_info(5, 0);
  const Scalar _tmp258 = _tmp174 * _tmp255 + _tmp174 * _tmp256 + _tmp178 * sqrt_info(5, 0) -
                         _tmp179 * sqrt_info(5, 2) - _tmp181 * _tmp256 + _tmp188 * sqrt_info(5, 1) +
                         _tmp192 * sqrt_info(5, 3) + _tmp197 * sqrt_info(5, 4) +
                         _tmp227 * sqrt_info(5, 0) - _tmp228 * sqrt_info(5, 0) +
                         _tmp235 * sqrt_info(5, 2);
  const Scalar _tmp259 = _tmp203 * sqrt_info(5, 0) + _tmp205 * _tmp255 + _tmp205 * _tmp256 -
                         _tmp207 * sqrt_info(5, 2) - _tmp208 * _tmp256 + _tmp211 * sqrt_info(5, 1) -
                         _tmp212 * sqrt_info(5, 0) + _tmp216 * sqrt_info(5, 3) +
                         _tmp218 * sqrt_info(5, 5) + _tmp230 * sqrt_info(5, 0) +
                         _tmp252 * sqrt_info(5, 2);
  const Scalar _tmp260 = _tmp38 + _tmp65 - 1;
  const Scalar _tmp261 = _tmp41 - 1;
  const Scalar _tmp262 = _tmp261 + _tmp38;
  const Scalar _tmp263 = _tmp261 + _tmp65;
  const Scalar _tmp264 = (Scalar(1) / Scalar(2)) * _tmp87;
  const Scalar _tmp265 = (Scalar(1) / Scalar(2)) * _tmp88;
  const Scalar _tmp266 = (Scalar(1) / Scalar(2)) * _tmp89;
  const Scalar _tmp267 = (Scalar(1) / Scalar(2)) * _tmp90;
  const Scalar _tmp268 = -_tmp264 + _tmp265 + _tmp266 - _tmp267;
  const Scalar _tmp269 = _tmp135 * _tmp268;
  const Scalar _tmp270 = _tmp269 * _tmp61;
  const Scalar _tmp271 = _tmp120 * _tmp268;
  const Scalar _tmp272 = _tmp131 * _tmp271;
  const Scalar _tmp273 = _tmp272 * _tmp91;
  const Scalar _tmp274 = (Scalar(1) / Scalar(2)) * _tmp27;
  const Scalar _tmp275 = (Scalar(1) / Scalar(2)) * _tmp28;
  const Scalar _tmp276 = (Scalar(1) / Scalar(2)) * _tmp29;
  const Scalar _tmp277 = (Scalar(1) / Scalar(2)) * _tmp30;
  const Scalar _tmp278 = _tmp274 + _tmp275 - _tmp276 - _tmp277;
  const Scalar _tmp279 = _tmp144 * _tmp278;
  const Scalar _tmp280 = (Scalar(1) / Scalar(2)) * _tmp57;
  const Scalar _tmp281 = (Scalar(1) / Scalar(2)) * _tmp58;
  const Scalar _tmp282 = (Scalar(1) / Scalar(2)) * _tmp59;
  const Scalar _tmp283 = (Scalar(1) / Scalar(2)) * _tmp60;
  const Scalar _tmp284 = -_tmp280 + _tmp281 - _tmp282 + _tmp283;
  const Scalar _tmp285 = _tmp144 * _tmp284;
  const Scalar _tmp286 = _tmp138 * _tmp268;
  const Scalar _tmp287 = _tmp136 * _tmp268;
  const Scalar _tmp288 = _tmp272 * _tmp61;
  const Scalar _tmp289 = -Scalar(1) / Scalar(2) * _tmp11 - Scalar(1) / Scalar(2) * _tmp17 -
                         Scalar(1) / Scalar(2) * _tmp24 - Scalar(1) / Scalar(2) * _tmp5;
  const Scalar _tmp290 = _tmp123 * _tmp289;
  const Scalar _tmp291 = _tmp152 * _tmp272 - _tmp270 * sqrt_info(0, 1) + _tmp273 * sqrt_info(0, 2) +
                         _tmp279 * sqrt_info(0, 1) + _tmp285 * sqrt_info(0, 0) -
                         _tmp286 * sqrt_info(0, 0) - _tmp287 * sqrt_info(0, 2) +
                         _tmp288 * sqrt_info(0, 1) + _tmp290 * _tmp86;
  const Scalar _tmp292 = 2 * _x_T_y[1];
  const Scalar _tmp293 = _tmp122 * _tmp271;
  const Scalar _tmp294 = _tmp26 * _tmp293;
  const Scalar _tmp295 = -_tmp274 - _tmp275 + _tmp276 + _tmp277;
  const Scalar _tmp296 = _tmp135 * _tmp295;
  const Scalar _tmp297 = _tmp296 * _tmp61;
  const Scalar _tmp298 = _tmp296 * _tmp31;
  const Scalar _tmp299 = _tmp132 * _tmp295;
  const Scalar _tmp300 = _tmp299 * _tmp61;
  const Scalar _tmp301 = _tmp299 * _tmp91;
  const Scalar _tmp302 = _tmp296 * _tmp91;
  const Scalar _tmp303 = _tmp280 - _tmp281 + _tmp282 - _tmp283;
  const Scalar _tmp304 = _tmp123 * _tmp303;
  const Scalar _tmp305 = _tmp144 * _tmp289;
  const Scalar _tmp306 = _tmp152 * _tmp299 + _tmp294 * sqrt_info(0, 1) - _tmp297 * sqrt_info(0, 1) -
                         _tmp298 * sqrt_info(0, 0) + _tmp300 * sqrt_info(0, 1) +
                         _tmp301 * sqrt_info(0, 2) - _tmp302 * sqrt_info(0, 2) + _tmp304 * _tmp86 +
                         _tmp305 * sqrt_info(0, 0);
  const Scalar _tmp307 = 2 * _x_T_y[3];
  const Scalar _tmp308 = _tmp132 * _tmp284;
  const Scalar _tmp309 = _tmp308 * _tmp91;
  const Scalar _tmp310 = _tmp264 - _tmp265 - _tmp266 + _tmp267;
  const Scalar _tmp311 = _tmp144 * _tmp310;
  const Scalar _tmp312 = _tmp135 * _tmp284;
  const Scalar _tmp313 = _tmp312 * _tmp61;
  const Scalar _tmp314 = _tmp123 * _tmp295;
  const Scalar _tmp315 = _tmp136 * _tmp284;
  const Scalar _tmp316 = -_tmp139 * _tmp284 + _tmp152 * _tmp308 + _tmp153 * _tmp308 +
                         _tmp305 * sqrt_info(0, 1) + _tmp309 * sqrt_info(0, 2) +
                         _tmp311 * sqrt_info(0, 0) - _tmp313 * sqrt_info(0, 1) + _tmp314 * _tmp86 -
                         _tmp315 * sqrt_info(0, 2);
  const Scalar _tmp317 = 2 * _x_T_y[2];
  const Scalar _tmp318 = _tmp272 * _tmp31;
  const Scalar _tmp319 = _tmp221 * _tmp272 - _tmp225 * _tmp269 + _tmp225 * _tmp272 +
                         _tmp279 * sqrt_info(1, 1) + _tmp285 * sqrt_info(1, 0) -
                         _tmp286 * sqrt_info(1, 0) - _tmp287 * sqrt_info(1, 2) + _tmp290 * _tmp93 +
                         _tmp318 * sqrt_info(1, 0);
  const Scalar _tmp320 = _tmp299 * _tmp31;
  const Scalar _tmp321 = _tmp225 * _tmp299 + _tmp294 * sqrt_info(1, 1) - _tmp297 * sqrt_info(1, 1) -
                         _tmp298 * sqrt_info(1, 0) + _tmp301 * sqrt_info(1, 2) -
                         _tmp302 * sqrt_info(1, 2) + _tmp304 * _tmp93 + _tmp305 * sqrt_info(1, 0) +
                         _tmp320 * sqrt_info(1, 0);
  const Scalar _tmp322 = _tmp308 * _tmp31;
  const Scalar _tmp323 = _tmp138 * _tmp284;
  const Scalar _tmp324 = _tmp221 * _tmp308 + _tmp225 * _tmp308 + _tmp305 * sqrt_info(1, 1) +
                         _tmp311 * sqrt_info(1, 0) - _tmp313 * sqrt_info(1, 1) + _tmp314 * _tmp93 -
                         _tmp315 * sqrt_info(1, 2) + _tmp322 * sqrt_info(1, 0) -
                         _tmp323 * sqrt_info(1, 0);
  const Scalar _tmp325 = -_tmp232 * _tmp269 + _tmp232 * _tmp272 + _tmp233 * _tmp272 +
                         _tmp273 * sqrt_info(2, 2) + _tmp279 * sqrt_info(2, 1) +
                         _tmp285 * sqrt_info(2, 0) - _tmp286 * sqrt_info(2, 0) -
                         _tmp287 * sqrt_info(2, 2) + _tmp305 * sqrt_info(2, 2);
  const Scalar _tmp326 = _tmp232 * _tmp299 + _tmp237 * _tmp303 + _tmp294 * sqrt_info(2, 1) -
                         _tmp297 * sqrt_info(2, 1) - _tmp298 * sqrt_info(2, 0) +
                         _tmp301 * sqrt_info(2, 2) - _tmp302 * sqrt_info(2, 2) +
                         _tmp305 * sqrt_info(2, 0) + _tmp320 * sqrt_info(2, 0);
  const Scalar _tmp327 = _tmp232 * _tmp308 - _tmp232 * _tmp312 + _tmp233 * _tmp308 +
                         _tmp237 * _tmp295 + _tmp305 * sqrt_info(2, 1) + _tmp309 * sqrt_info(2, 2) +
                         _tmp311 * sqrt_info(2, 0) - _tmp315 * sqrt_info(2, 2) -
                         _tmp323 * sqrt_info(2, 0);
  const Scalar _tmp328 = _tmp239 * _tmp272 - _tmp240 * _tmp269 + _tmp240 * _tmp272 +
                         _tmp242 * _tmp278 + _tmp285 * sqrt_info(3, 0) - _tmp286 * sqrt_info(3, 0) -
                         _tmp287 * sqrt_info(3, 2) + _tmp290 * _tmp95 + _tmp318 * sqrt_info(3, 0);
  const Scalar _tmp329 = _tmp240 * _tmp299 + _tmp294 * sqrt_info(3, 1) - _tmp297 * sqrt_info(3, 1) -
                         _tmp298 * sqrt_info(3, 0) + _tmp301 * sqrt_info(3, 2) -
                         _tmp302 * sqrt_info(3, 2) + _tmp304 * _tmp95 + _tmp305 * sqrt_info(3, 0) +
                         _tmp320 * sqrt_info(3, 0);
  const Scalar _tmp330 = _tmp239 * _tmp308 + _tmp240 * _tmp308 - _tmp240 * _tmp312 +
                         _tmp305 * sqrt_info(3, 1) + _tmp311 * sqrt_info(3, 0) + _tmp314 * _tmp95 -
                         _tmp315 * sqrt_info(3, 2) + _tmp322 * sqrt_info(3, 0) -
                         _tmp323 * sqrt_info(3, 0);
  const Scalar _tmp331 = _tmp246 * _tmp272 + _tmp247 * _tmp284 + _tmp250 * _tmp278 -
                         _tmp270 * sqrt_info(4, 1) - _tmp286 * sqrt_info(4, 0) -
                         _tmp287 * sqrt_info(4, 2) + _tmp288 * sqrt_info(4, 1) +
                         _tmp305 * sqrt_info(4, 2) + _tmp318 * sqrt_info(4, 0);
  const Scalar _tmp332 = _tmp144 * _tmp303;
  const Scalar _tmp333 = _tmp246 * _tmp299 + _tmp247 * _tmp289 + _tmp293 * _tmp97 -
                         _tmp297 * sqrt_info(4, 1) - _tmp298 * sqrt_info(4, 0) +
                         _tmp300 * sqrt_info(4, 1) - _tmp302 * sqrt_info(4, 2) +
                         _tmp320 * sqrt_info(4, 0) + _tmp332 * sqrt_info(4, 2);
  const Scalar _tmp334 = _tmp144 * _tmp295;
  const Scalar _tmp335 = _tmp246 * _tmp308 + _tmp247 * _tmp310 + _tmp248 * _tmp308 +
                         _tmp290 * _tmp97 - _tmp313 * sqrt_info(4, 1) - _tmp315 * sqrt_info(4, 2) +
                         _tmp322 * sqrt_info(4, 0) - _tmp323 * sqrt_info(4, 0) +
                         _tmp334 * sqrt_info(4, 2);
  const Scalar _tmp336 = _tmp255 * _tmp272 - _tmp256 * _tmp269 + _tmp256 * _tmp272 +
                         _tmp279 * sqrt_info(5, 1) + _tmp285 * sqrt_info(5, 0) -
                         _tmp286 * sqrt_info(5, 0) - _tmp287 * sqrt_info(5, 2) +
                         _tmp305 * sqrt_info(5, 2) + _tmp318 * sqrt_info(5, 0);
  const Scalar _tmp337 = _tmp255 * _tmp299 - _tmp256 * _tmp296 + _tmp256 * _tmp299 +
                         _tmp294 * sqrt_info(5, 1) - _tmp298 * sqrt_info(5, 0) -
                         _tmp302 * sqrt_info(5, 2) + _tmp305 * sqrt_info(5, 0) +
                         _tmp320 * sqrt_info(5, 0) + _tmp332 * sqrt_info(5, 2);
  const Scalar _tmp338 = _tmp255 * _tmp308 + _tmp256 * _tmp308 - _tmp256 * _tmp312 +
                         _tmp305 * sqrt_info(5, 1) + _tmp311 * sqrt_info(5, 0) -
                         _tmp315 * sqrt_info(5, 2) + _tmp322 * sqrt_info(5, 0) -
                         _tmp323 * sqrt_info(5, 0) + _tmp334 * sqrt_info(5, 2);
  const Scalar _tmp339 = 2 * _x_T_y[0];

  // Output terms (3)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = _tmp36 * sqrt_info(0, 0) + _tmp56 * sqrt_info(0, 4) + _tmp63 * sqrt_info(0, 1) +
               _tmp77 * sqrt_info(0, 5) + _tmp85 * sqrt_info(0, 3) + _tmp86 * _tmp92;
  _res(1, 0) = _tmp36 * sqrt_info(1, 0) + _tmp56 * sqrt_info(1, 4) + _tmp63 * sqrt_info(1, 1) +
               _tmp77 * sqrt_info(1, 5) + _tmp85 * sqrt_info(1, 3) + _tmp92 * _tmp93;
  _res(2, 0) = _tmp36 * sqrt_info(2, 0) + _tmp56 * sqrt_info(2, 4) + _tmp63 * sqrt_info(2, 1) +
               _tmp77 * sqrt_info(2, 5) + _tmp85 * sqrt_info(2, 3) + _tmp94 * sqrt_info(2, 2);
  _res(3, 0) = _tmp36 * sqrt_info(3, 0) + _tmp56 * sqrt_info(3, 4) + _tmp63 * sqrt_info(3, 1) +
               _tmp77 * sqrt_info(3, 5) + _tmp85 * sqrt_info(3, 3) + _tmp92 * _tmp95;
  _res(4, 0) = _tmp35 * _tmp96 + _tmp56 * sqrt_info(4, 4) + _tmp62 * _tmp97 +
               _tmp77 * sqrt_info(4, 5) + _tmp85 * sqrt_info(4, 3) + _tmp94 * sqrt_info(4, 2);
  _res(5, 0) = _tmp36 * sqrt_info(5, 0) + _tmp56 * sqrt_info(5, 4) + _tmp63 * sqrt_info(5, 1) +
               _tmp77 * sqrt_info(5, 5) + _tmp85 * sqrt_info(5, 3) + _tmp94 * sqrt_info(5, 2);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 6, 7, Eigen::RowMajor>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = _tmp169 * _tmp46 + _tmp198 * _tmp44 - _tmp219 * _tmp220;
    _res_D_x(1, 0) = -_tmp220 * _tmp231 + _tmp226 * _tmp46 + _tmp229 * _tmp44;
    _res_D_x(2, 0) = _tmp234 * _tmp46 + _tmp236 * _tmp44 - _tmp238 * _x[2];
    _res_D_x(3, 0) = _tmp241 * _tmp46 + _tmp243 * _tmp44 - _tmp245 * _x[2];
    _res_D_x(4, 0) = _tmp249 * _tmp46 + _tmp251 * _tmp44 - _tmp254 * _x[2];
    _res_D_x(5, 0) = -_tmp220 * _tmp259 + _tmp257 * _tmp46 + _tmp258 * _tmp44;
    _res_D_x(0, 1) = _tmp169 * _tmp220 - _tmp198 * _tmp68 + _tmp219 * _tmp46;
    _res_D_x(1, 1) = _tmp220 * _tmp226 - _tmp229 * _tmp68 + _tmp231 * _tmp46;
    _res_D_x(2, 1) = _tmp220 * _tmp234 - _tmp236 * _tmp68 + _tmp238 * _x[3];
    _res_D_x(3, 1) = _tmp220 * _tmp241 - _tmp243 * _tmp68 + _tmp244 * _tmp46;
    _res_D_x(4, 1) = _tmp220 * _tmp249 - _tmp251 * _tmp68 + _tmp253 * _tmp46;
    _res_D_x(5, 1) = _tmp220 * _tmp257 - _tmp258 * _tmp68 + _tmp259 * _tmp46;
    _res_D_x(0, 2) = -_tmp169 * _tmp44 + _tmp198 * _tmp46 + _tmp219 * _tmp68;
    _res_D_x(1, 2) = -_tmp226 * _tmp44 + _tmp229 * _tmp46 + _tmp231 * _tmp68;
    _res_D_x(2, 2) = -_tmp234 * _tmp44 + _tmp236 * _tmp46 + _tmp238 * _x[0];
    _res_D_x(3, 2) = -_tmp241 * _tmp44 + _tmp243 * _tmp46 + _tmp245 * _x[0];
    _res_D_x(4, 2) = -_tmp249 * _tmp44 + _tmp251 * _tmp46 + _tmp254 * _x[0];
    _res_D_x(5, 2) = -_tmp257 * _tmp44 + _tmp258 * _tmp46 + _tmp259 * _tmp68;
    _res_D_x(0, 3) = -_tmp169 * _tmp68 - _tmp198 * _tmp220 - _tmp219 * _tmp44;
    _res_D_x(1, 3) = -_tmp220 * _tmp229 - _tmp226 * _tmp68 - _tmp231 * _tmp44;
    _res_D_x(2, 3) = -_tmp220 * _tmp236 - _tmp234 * _tmp68 - _tmp238 * _x[1];
    _res_D_x(3, 3) = -_tmp220 * _tmp243 - _tmp241 * _tmp68 - _tmp244 * _tmp44;
    _res_D_x(4, 3) = -_tmp220 * _tmp251 - _tmp249 * _tmp68 - _tmp253 * _tmp44;
    _res_D_x(5, 3) = -_tmp220 * _tmp258 - _tmp257 * _tmp68 - _tmp259 * _tmp44;
    _res_D_x(0, 4) =
        _tmp155 * sqrt_info(0, 4) + _tmp213 * sqrt_info(0, 5) + _tmp260 * sqrt_info(0, 3);
    _res_D_x(1, 4) =
        _tmp155 * sqrt_info(1, 4) + _tmp213 * sqrt_info(1, 5) + _tmp260 * sqrt_info(1, 3);
    _res_D_x(2, 4) =
        _tmp155 * sqrt_info(2, 4) + _tmp213 * sqrt_info(2, 5) + _tmp260 * sqrt_info(2, 3);
    _res_D_x(3, 4) =
        _tmp155 * sqrt_info(3, 4) + _tmp213 * sqrt_info(3, 5) + _tmp260 * sqrt_info(3, 3);
    _res_D_x(4, 4) =
        _tmp155 * sqrt_info(4, 4) + _tmp213 * sqrt_info(4, 5) + _tmp260 * sqrt_info(4, 3);
    _res_D_x(5, 4) =
        _tmp155 * sqrt_info(5, 4) + _tmp213 * sqrt_info(5, 5) + _tmp260 * sqrt_info(5, 3);
    _res_D_x(0, 5) =
        _tmp193 * sqrt_info(0, 3) + _tmp215 * sqrt_info(0, 5) + _tmp262 * sqrt_info(0, 4);
    _res_D_x(1, 5) =
        _tmp193 * sqrt_info(1, 3) + _tmp215 * sqrt_info(1, 5) + _tmp262 * sqrt_info(1, 4);
    _res_D_x(2, 5) =
        _tmp193 * sqrt_info(2, 3) + _tmp215 * sqrt_info(2, 5) + _tmp262 * sqrt_info(2, 4);
    _res_D_x(3, 5) =
        _tmp193 * sqrt_info(3, 3) + _tmp215 * sqrt_info(3, 5) + _tmp262 * sqrt_info(3, 4);
    _res_D_x(4, 5) =
        _tmp193 * sqrt_info(4, 3) + _tmp215 * sqrt_info(4, 5) + _tmp262 * sqrt_info(4, 4);
    _res_D_x(5, 5) =
        _tmp193 * sqrt_info(5, 3) + _tmp215 * sqrt_info(5, 5) + _tmp262 * sqrt_info(5, 4);
    _res_D_x(0, 6) =
        _tmp163 * sqrt_info(0, 4) + _tmp195 * sqrt_info(0, 3) + _tmp263 * sqrt_info(0, 5);
    _res_D_x(1, 6) =
        _tmp163 * sqrt_info(1, 4) + _tmp195 * sqrt_info(1, 3) + _tmp263 * sqrt_info(1, 5);
    _res_D_x(2, 6) =
        _tmp163 * sqrt_info(2, 4) + _tmp195 * sqrt_info(2, 3) + _tmp263 * sqrt_info(2, 5);
    _res_D_x(3, 6) =
        _tmp163 * sqrt_info(3, 4) + _tmp195 * sqrt_info(3, 3) + _tmp263 * sqrt_info(3, 5);
    _res_D_x(4, 6) =
        _tmp163 * sqrt_info(4, 4) + _tmp195 * sqrt_info(4, 3) + _tmp263 * sqrt_info(4, 5);
    _res_D_x(5, 6) =
        _tmp163 * sqrt_info(5, 4) + _tmp195 * sqrt_info(5, 3) + _tmp263 * sqrt_info(5, 5);
  }

  if (res_D_x_T_y != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 6, 7, Eigen::RowMajor>> _res_D_x_T_y{res_D_x_T_y};

    _res_D_x_T_y(0, 0) = _tmp291 * _tmp292 + _tmp306 * _tmp307 - _tmp316 * _tmp317;
    _res_D_x_T_y(1, 0) = _tmp292 * _tmp319 + _tmp307 * _tmp321 - _tmp317 * _tmp324;
    _res_D_x_T_y(2, 0) = _tmp292 * _tmp325 + _tmp307 * _tmp326 - _tmp317 * _tmp327;
    _res_D_x_T_y(3, 0) = _tmp292 * _tmp328 + _tmp307 * _tmp329 - _tmp317 * _tmp330;
    _res_D_x_T_y(4, 0) = _tmp292 * _tmp331 + _tmp307 * _tmp333 - _tmp317 * _tmp335;
    _res_D_x_T_y(5, 0) = _tmp292 * _tmp336 + _tmp307 * _tmp337 - _tmp317 * _tmp338;
    _res_D_x_T_y(0, 1) = -_tmp291 * _tmp339 + _tmp306 * _tmp317 + _tmp307 * _tmp316;
    _res_D_x_T_y(1, 1) = _tmp307 * _tmp324 + _tmp317 * _tmp321 - _tmp319 * _tmp339;
    _res_D_x_T_y(2, 1) = _tmp307 * _tmp327 + _tmp317 * _tmp326 - _tmp325 * _tmp339;
    _res_D_x_T_y(3, 1) = _tmp307 * _tmp330 + _tmp317 * _tmp329 - _tmp328 * _tmp339;
    _res_D_x_T_y(4, 1) = _tmp307 * _tmp335 + _tmp317 * _tmp333 - _tmp331 * _tmp339;
    _res_D_x_T_y(5, 1) = _tmp307 * _tmp338 + _tmp317 * _tmp337 - _tmp336 * _tmp339;
    _res_D_x_T_y(0, 2) = _tmp291 * _tmp307 - _tmp292 * _tmp306 + _tmp316 * _tmp339;
    _res_D_x_T_y(1, 2) = -_tmp292 * _tmp321 + _tmp307 * _tmp319 + _tmp324 * _tmp339;
    _res_D_x_T_y(2, 2) = -_tmp292 * _tmp326 + _tmp307 * _tmp325 + _tmp327 * _tmp339;
    _res_D_x_T_y(3, 2) = -_tmp292 * _tmp329 + _tmp307 * _tmp328 + _tmp330 * _tmp339;
    _res_D_x_T_y(4, 2) = -_tmp292 * _tmp333 + _tmp307 * _tmp331 + _tmp335 * _tmp339;
    _res_D_x_T_y(5, 2) = -_tmp292 * _tmp337 + _tmp307 * _tmp336 + _tmp338 * _tmp339;
    _res_D_x_T_y(0, 3) = -_tmp291 * _tmp317 - _tmp292 * _tmp316 - _tmp306 * _tmp339;
    _res_D_x_T_y(1, 3) = -_tmp292 * _tmp324 - _tmp317 * _tmp319 - _tmp321 * _tmp339;
    _res_D_x_T_y(2, 3) = -_tmp292 * _tmp327 - _tmp317 * _tmp325 - _tmp326 * _tmp339;
    _res_D_x_T_y(3, 3) = -_tmp292 * _tmp330 - _tmp317 * _tmp328 - _tmp329 * _tmp339;
    _res_D_x_T_y(4, 3) = -_tmp292 * _tmp335 - _tmp317 * _tmp331 - _tmp333 * _tmp339;
    _res_D_x_T_y(5, 3) = -_tmp292 * _tmp338 - _tmp317 * _tmp336 - _tmp337 * _tmp339;
    _res_D_x_T_y(0, 4) = -sqrt_info(0, 3);
    _res_D_x_T_y(1, 4) = -sqrt_info(1, 3);
    _res_D_x_T_y(2, 4) = -sqrt_info(2, 3);
    _res_D_x_T_y(3, 4) = -sqrt_info(3, 3);
    _res_D_x_T_y(4, 4) = -sqrt_info(4, 3);
    _res_D_x_T_y(5, 4) = -sqrt_info(5, 3);
    _res_D_x_T_y(0, 5) = -sqrt_info(0, 4);
    _res_D_x_T_y(1, 5) = -sqrt_info(1, 4);
    _res_D_x_T_y(2, 5) = -sqrt_info(2, 4);
    _res_D_x_T_y(3, 5) = -sqrt_info(3, 4);
    _res_D_x_T_y(4, 5) = -sqrt_info(4, 4);
    _res_D_x_T_y(5, 5) = -sqrt_info(5, 4);
    _res_D_x_T_y(0, 6) = -sqrt_info(0, 5);
    _res_D_x_T_y(1, 6) = -sqrt_info(1, 5);
    _res_D_x_T_y(2, 6) = -sqrt_info(2, 5);
    _res_D_x_T_y(3, 6) = -sqrt_info(3, 5);
    _res_D_x_T_y(4, 6) = -sqrt_info(4, 5);
    _res_D_x_T_y(5, 6) = -sqrt_info(5, 5);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
