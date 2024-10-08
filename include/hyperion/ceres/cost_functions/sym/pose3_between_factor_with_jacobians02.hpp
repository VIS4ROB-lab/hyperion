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
 * Symbolic function: pose3_between_factor
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
 *     res_D_y: (6x7) jacobian (result_dim x storage_dim) of res (6) wrt arg y (7) (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> Pose3BetweenFactorWithJacobians02(
    const sym::Pose3<Scalar>& x, const sym::Pose3<Scalar>& x_T_y, const sym::Pose3<Scalar>& y,
    const Eigen::Matrix<Scalar, 6, 6>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr, Scalar* const res_D_y = nullptr) {
  // Total ops: 1682

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x_T_y = x_T_y.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (338)
  const Scalar _tmp0 = _x[2] * _y[2];
  const Scalar _tmp1 = _x[1] * _y[1];
  const Scalar _tmp2 = _x[0] * _y[0];
  const Scalar _tmp3 = _x[3] * _y[3];
  const Scalar _tmp4 = _tmp0 + _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp5 = _x[3] * _y[2];
  const Scalar _tmp6 = _x[2] * _y[3];
  const Scalar _tmp7 = _x[1] * _y[0];
  const Scalar _tmp8 = _x[0] * _y[1];
  const Scalar _tmp9 = _tmp5 - _tmp6 + _tmp7 - _tmp8;
  const Scalar _tmp10 = _x[3] * _y[0];
  const Scalar _tmp11 = _x[2] * _y[1];
  const Scalar _tmp12 = _x[1] * _y[2];
  const Scalar _tmp13 = _x[0] * _y[3];
  const Scalar _tmp14 = _tmp10 + _tmp11 - _tmp12 - _tmp13;
  const Scalar _tmp15 = _x[3] * _y[1];
  const Scalar _tmp16 = _x[2] * _y[0];
  const Scalar _tmp17 = _x[1] * _y[3];
  const Scalar _tmp18 = _x[0] * _y[2];
  const Scalar _tmp19 = _tmp15 - _tmp16 - _tmp17 + _tmp18;
  const Scalar _tmp20 =
      _tmp14 * _x_T_y[3] + _tmp19 * _x_T_y[2] - _tmp4 * _x_T_y[0] - _tmp9 * _x_T_y[1];
  const Scalar _tmp21 = _tmp9 * _x_T_y[2];
  const Scalar _tmp22 = _tmp14 * _x_T_y[0];
  const Scalar _tmp23 = _tmp19 * _x_T_y[1];
  const Scalar _tmp24 = -_tmp21 - _tmp22 - _tmp23;
  const Scalar _tmp25 = _tmp4 * _x_T_y[3];
  const Scalar _tmp26 =
      2 * std::min<Scalar>(0, (((-_tmp24 + _tmp25) > 0) - ((-_tmp24 + _tmp25) < 0))) + 1;
  const Scalar _tmp27 = 2 * _tmp26;
  const Scalar _tmp28 = 1 - epsilon;
  const Scalar _tmp29 = std::min<Scalar>(_tmp28, std::fabs(_tmp24 - _tmp25));
  const Scalar _tmp30 = std::acos(_tmp29) / std::sqrt(Scalar(1 - std::pow(_tmp29, Scalar(2))));
  const Scalar _tmp31 = _tmp27 * _tmp30;
  const Scalar _tmp32 = _tmp20 * _tmp31;
  const Scalar _tmp33 = std::pow(_x[2], Scalar(2));
  const Scalar _tmp34 = 2 * _tmp33;
  const Scalar _tmp35 = -_tmp34;
  const Scalar _tmp36 = std::pow(_x[0], Scalar(2));
  const Scalar _tmp37 = 2 * _tmp36;
  const Scalar _tmp38 = 1 - _tmp37;
  const Scalar _tmp39 = _tmp35 + _tmp38;
  const Scalar _tmp40 = 2 * _x[0];
  const Scalar _tmp41 = _tmp40 * _x[1];
  const Scalar _tmp42 = 2 * _x[3];
  const Scalar _tmp43 = _tmp42 * _x[2];
  const Scalar _tmp44 = -_tmp43;
  const Scalar _tmp45 = _tmp41 + _tmp44;
  const Scalar _tmp46 = _tmp45 * _x[4];
  const Scalar _tmp47 = _tmp40 * _x[3];
  const Scalar _tmp48 = 2 * _x[2];
  const Scalar _tmp49 = _tmp48 * _x[1];
  const Scalar _tmp50 = _tmp47 + _tmp49;
  const Scalar _tmp51 = _tmp50 * _x[6];
  const Scalar _tmp52 = _tmp45 * _y[4] + _tmp50 * _y[6];
  const Scalar _tmp53 = -_tmp39 * _x[5] + _tmp39 * _y[5] - _tmp46 - _tmp51 + _tmp52 - _x_T_y[5];
  const Scalar _tmp54 =
      -_tmp14 * _x_T_y[2] + _tmp19 * _x_T_y[3] - _tmp4 * _x_T_y[1] + _tmp9 * _x_T_y[0];
  const Scalar _tmp55 = _tmp54 * sqrt_info(0, 1);
  const Scalar _tmp56 = std::pow(_x[1], Scalar(2));
  const Scalar _tmp57 = 2 * _tmp56;
  const Scalar _tmp58 = -_tmp57;
  const Scalar _tmp59 = _tmp38 + _tmp58;
  const Scalar _tmp60 = _tmp40 * _x[2];
  const Scalar _tmp61 = _tmp42 * _x[1];
  const Scalar _tmp62 = _tmp60 + _tmp61;
  const Scalar _tmp63 = _tmp62 * _x[4];
  const Scalar _tmp64 = -_tmp47;
  const Scalar _tmp65 = _tmp49 + _tmp64;
  const Scalar _tmp66 = _tmp65 * _x[5];
  const Scalar _tmp67 = _tmp62 * _y[4] + _tmp65 * _y[5];
  const Scalar _tmp68 = -_tmp59 * _x[6] + _tmp59 * _y[6] - _tmp63 - _tmp66 + _tmp67 - _x_T_y[6];
  const Scalar _tmp69 = _tmp35 + _tmp58 + 1;
  const Scalar _tmp70 = _tmp41 + _tmp43;
  const Scalar _tmp71 = _tmp70 * _x[5];
  const Scalar _tmp72 = -_tmp61;
  const Scalar _tmp73 = _tmp60 + _tmp72;
  const Scalar _tmp74 = _tmp73 * _x[6];
  const Scalar _tmp75 = _tmp70 * _y[5] + _tmp73 * _y[6];
  const Scalar _tmp76 = -_tmp69 * _x[4] + _tmp69 * _y[4] - _tmp71 - _tmp74 + _tmp75 - _x_T_y[4];
  const Scalar _tmp77 =
      _tmp14 * _x_T_y[1] - _tmp19 * _x_T_y[0] - _tmp4 * _x_T_y[2] + _tmp9 * _x_T_y[3];
  const Scalar _tmp78 = _tmp31 * _tmp77;
  const Scalar _tmp79 = _tmp31 * _tmp54;
  const Scalar _tmp80 = _tmp27 * sqrt_info(4, 1);
  const Scalar _tmp81 = (Scalar(1) / Scalar(2)) * _tmp10;
  const Scalar _tmp82 = (Scalar(1) / Scalar(2)) * _tmp11;
  const Scalar _tmp83 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp84 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp85 = _tmp81 + _tmp82 - _tmp83 - _tmp84;
  const Scalar _tmp86 = -_tmp85 * _x_T_y[1];
  const Scalar _tmp87 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp88 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp89 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp90 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp91 = -_tmp87 - _tmp88 - _tmp89 - _tmp90;
  const Scalar _tmp92 = _tmp91 * _x_T_y[2];
  const Scalar _tmp93 = (Scalar(1) / Scalar(2)) * _tmp5;
  const Scalar _tmp94 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp95 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp96 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp97 = _tmp93 - _tmp94 + _tmp95 - _tmp96;
  const Scalar _tmp98 = _tmp97 * _x_T_y[3];
  const Scalar _tmp99 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp100 = (Scalar(1) / Scalar(2)) * _tmp16;
  const Scalar _tmp101 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp102 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp103 = _tmp100 + _tmp101 - _tmp102 - _tmp99;
  const Scalar _tmp104 = _tmp103 * _x_T_y[0];
  const Scalar _tmp105 = _tmp104 + _tmp98;
  const Scalar _tmp106 = _tmp105 + _tmp86 - _tmp92;
  const Scalar _tmp107 = _tmp21 + _tmp22 + _tmp23 + _tmp25;
  const Scalar _tmp108 = std::fabs(_tmp107);
  const Scalar _tmp109 = std::min<Scalar>(_tmp108, _tmp28);
  const Scalar _tmp110 = std::acos(_tmp109);
  const Scalar _tmp111 = 1 - std::pow(_tmp109, Scalar(2));
  const Scalar _tmp112 = _tmp110 / std::sqrt(_tmp111);
  const Scalar _tmp113 = _tmp112 * _tmp27;
  const Scalar _tmp114 = _tmp106 * _tmp113;
  const Scalar _tmp115 = _tmp91 * _x_T_y[0];
  const Scalar _tmp116 = _tmp85 * _x_T_y[3];
  const Scalar _tmp117 = _tmp97 * _x_T_y[1];
  const Scalar _tmp118 = _tmp103 * _x_T_y[2];
  const Scalar _tmp119 = _tmp117 + _tmp118;
  const Scalar _tmp120 = _tmp115 + _tmp116 + _tmp119;
  const Scalar _tmp121 = _tmp26 * ((((-_tmp108 + _tmp28) > 0) - ((-_tmp108 + _tmp28) < 0)) + 1) *
                         (((_tmp107) > 0) - ((_tmp107) < 0));
  const Scalar _tmp122 = _tmp109 * _tmp110 * _tmp121 / (_tmp111 * std::sqrt(_tmp111));
  const Scalar _tmp123 = _tmp122 * _tmp77;
  const Scalar _tmp124 = _tmp120 * _tmp123;
  const Scalar _tmp125 = _tmp121 / _tmp111;
  const Scalar _tmp126 = _tmp125 * _tmp77;
  const Scalar _tmp127 = _tmp120 * _tmp126;
  const Scalar _tmp128 = _tmp125 * _tmp20;
  const Scalar _tmp129 = _tmp120 * _tmp128;
  const Scalar _tmp130 = _tmp91 * _x_T_y[3];
  const Scalar _tmp131 = _tmp85 * _x_T_y[0];
  const Scalar _tmp132 = _tmp97 * _x_T_y[2];
  const Scalar _tmp133 = -_tmp103 * _x_T_y[1];
  const Scalar _tmp134 = _tmp130 - _tmp131 + _tmp132 + _tmp133;
  const Scalar _tmp135 = _tmp113 * _tmp134;
  const Scalar _tmp136 = _tmp120 * _tmp125;
  const Scalar _tmp137 = _tmp91 * _x_T_y[1];
  const Scalar _tmp138 = _tmp85 * _x_T_y[2];
  const Scalar _tmp139 = -_tmp97 * _x_T_y[0];
  const Scalar _tmp140 = _tmp103 * _x_T_y[3];
  const Scalar _tmp141 = _tmp113 * (_tmp137 - _tmp138 + _tmp139 + _tmp140);
  const Scalar _tmp142 = _tmp122 * _tmp20;
  const Scalar _tmp143 = _tmp120 * _tmp142;
  const Scalar _tmp144 = _tmp122 * _tmp55;
  const Scalar _tmp145 = -_tmp41;
  const Scalar _tmp146 = _tmp145 + _tmp43;
  const Scalar _tmp147 = std::pow(_x[3], Scalar(2));
  const Scalar _tmp148 = -_tmp147;
  const Scalar _tmp149 = _tmp148 + _tmp36;
  const Scalar _tmp150 = -_tmp56;
  const Scalar _tmp151 = _tmp150 + _tmp33;
  const Scalar _tmp152 = _tmp149 + _tmp151;
  const Scalar _tmp153 = -_tmp49;
  const Scalar _tmp154 = _tmp153 + _tmp64;
  const Scalar _tmp155 = -_tmp146 * _x[4] + _tmp146 * _y[4] - _tmp152 * _x[5] + _tmp152 * _y[5] -
                         _tmp154 * _x[6] + _tmp154 * _y[6];
  const Scalar _tmp156 = -_tmp36;
  const Scalar _tmp157 = _tmp147 + _tmp156;
  const Scalar _tmp158 = _tmp151 + _tmp157;
  const Scalar _tmp159 = -_tmp158 * _x[6] + _tmp158 * _y[6] - _tmp63 - _tmp66 + _tmp67;
  const Scalar _tmp160 = _tmp114 * sqrt_info(0, 1) + _tmp120 * _tmp144 + _tmp124 * sqrt_info(0, 2) -
                         _tmp127 * sqrt_info(0, 2) - _tmp129 * sqrt_info(0, 0) +
                         _tmp135 * sqrt_info(0, 0) - _tmp136 * _tmp55 + _tmp141 * sqrt_info(0, 2) +
                         _tmp143 * sqrt_info(0, 0) + _tmp155 * sqrt_info(0, 5) +
                         _tmp159 * sqrt_info(0, 4);
  const Scalar _tmp161 = -_tmp81 - _tmp82 + _tmp83 + _tmp84;
  const Scalar _tmp162 = _tmp161 * _x_T_y[1];
  const Scalar _tmp163 = -_tmp100 - _tmp101 + _tmp102 + _tmp99;
  const Scalar _tmp164 = _tmp163 * _x_T_y[0];
  const Scalar _tmp165 = _tmp162 + _tmp164;
  const Scalar _tmp166 = _tmp165 + _tmp92 + _tmp98;
  const Scalar _tmp167 = _tmp123 * _tmp166;
  const Scalar _tmp168 = _tmp161 * _x_T_y[2];
  const Scalar _tmp169 = _tmp163 * _x_T_y[3];
  const Scalar _tmp170 = _tmp168 + _tmp169;
  const Scalar _tmp171 = -_tmp137 + _tmp139 + _tmp170;
  const Scalar _tmp172 = _tmp113 * _tmp171;
  const Scalar _tmp173 = _tmp126 * _tmp166;
  const Scalar _tmp174 = _tmp142 * _tmp166;
  const Scalar _tmp175 = _tmp125 * _tmp166;
  const Scalar _tmp176 = -_tmp161 * _x_T_y[0];
  const Scalar _tmp177 = _tmp163 * _x_T_y[1];
  const Scalar _tmp178 = _tmp176 + _tmp177;
  const Scalar _tmp179 = _tmp113 * (_tmp130 - _tmp132 + _tmp178);
  const Scalar _tmp180 = _tmp128 * _tmp166;
  const Scalar _tmp181 = _tmp161 * _x_T_y[3];
  const Scalar _tmp182 = -_tmp163 * _x_T_y[2];
  const Scalar _tmp183 = _tmp181 + _tmp182;
  const Scalar _tmp184 = _tmp115 - _tmp117 + _tmp183;
  const Scalar _tmp185 = _tmp113 * _tmp184;
  const Scalar _tmp186 = -_tmp33;
  const Scalar _tmp187 = _tmp186 + _tmp56;
  const Scalar _tmp188 = _tmp157 + _tmp187;
  const Scalar _tmp189 = -_tmp188 * _x[5] + _tmp188 * _y[5] - _tmp46 - _tmp51 + _tmp52;
  const Scalar _tmp190 = _tmp145 + _tmp44;
  const Scalar _tmp191 = -_tmp60;
  const Scalar _tmp192 = _tmp191 + _tmp61;
  const Scalar _tmp193 = _tmp148 + _tmp156 + _tmp33 + _tmp56;
  const Scalar _tmp194 = -_tmp190 * _x[5] + _tmp190 * _y[5] - _tmp192 * _x[6] + _tmp192 * _y[6] -
                         _tmp193 * _x[4] + _tmp193 * _y[4];
  const Scalar _tmp195 = _tmp144 * _tmp166 + _tmp167 * sqrt_info(0, 2) + _tmp172 * sqrt_info(0, 0) -
                         _tmp173 * sqrt_info(0, 2) + _tmp174 * sqrt_info(0, 0) - _tmp175 * _tmp55 +
                         _tmp179 * sqrt_info(0, 2) - _tmp180 * sqrt_info(0, 0) +
                         _tmp185 * sqrt_info(0, 1) + _tmp189 * sqrt_info(0, 3) +
                         _tmp194 * sqrt_info(0, 4);
  const Scalar _tmp196 = 2 * _x[1];
  const Scalar _tmp197 = -_tmp93 + _tmp94 - _tmp95 + _tmp96;
  const Scalar _tmp198 = _tmp197 * _x_T_y[1];
  const Scalar _tmp199 = _tmp116 + _tmp198;
  const Scalar _tmp200 = -_tmp115 + _tmp182 + _tmp199;
  const Scalar _tmp201 = _tmp113 * sqrt_info(0, 2);
  const Scalar _tmp202 = _tmp197 * _x_T_y[3];
  const Scalar _tmp203 = _tmp202 + _tmp86;
  const Scalar _tmp204 = -_tmp164 + _tmp203 + _tmp92;
  const Scalar _tmp205 = _tmp113 * _tmp204;
  const Scalar _tmp206 = _tmp197 * _x_T_y[0];
  const Scalar _tmp207 = _tmp138 + _tmp206;
  const Scalar _tmp208 = _tmp137 + _tmp169 + _tmp207;
  const Scalar _tmp209 = _tmp123 * _tmp208;
  const Scalar _tmp210 = _tmp126 * _tmp208;
  const Scalar _tmp211 = _tmp142 * _tmp208;
  const Scalar _tmp212 = _tmp125 * _tmp208;
  const Scalar _tmp213 = -_tmp197 * _x_T_y[2];
  const Scalar _tmp214 = _tmp131 + _tmp213;
  const Scalar _tmp215 = _tmp130 - _tmp177 + _tmp214;
  const Scalar _tmp216 = _tmp113 * _tmp215;
  const Scalar _tmp217 = _tmp128 * _tmp208;
  const Scalar _tmp218 = _tmp191 + _tmp72;
  const Scalar _tmp219 = _tmp149 + _tmp187;
  const Scalar _tmp220 = _tmp153 + _tmp47;
  const Scalar _tmp221 = -_tmp218 * _x[4] + _tmp218 * _y[4] - _tmp219 * _x[6] + _tmp219 * _y[6] -
                         _tmp220 * _x[5] + _tmp220 * _y[5];
  const Scalar _tmp222 = _tmp147 + _tmp150 + _tmp186 + _tmp36;
  const Scalar _tmp223 = -_tmp222 * _x[4] + _tmp222 * _y[4] - _tmp71 - _tmp74 + _tmp75;
  const Scalar _tmp224 = _tmp144 * _tmp208 + _tmp200 * _tmp201 + _tmp205 * sqrt_info(0, 0) +
                         _tmp209 * sqrt_info(0, 2) - _tmp210 * sqrt_info(0, 2) +
                         _tmp211 * sqrt_info(0, 0) - _tmp212 * _tmp55 + _tmp216 * sqrt_info(0, 1) -
                         _tmp217 * sqrt_info(0, 0) + _tmp221 * sqrt_info(0, 3) +
                         _tmp223 * sqrt_info(0, 5);
  const Scalar _tmp225 = _tmp125 * _tmp54;
  const Scalar _tmp226 = _tmp120 * _tmp225;
  const Scalar _tmp227 = _tmp122 * _tmp54;
  const Scalar _tmp228 = _tmp227 * sqrt_info(1, 1);
  const Scalar _tmp229 = _tmp114 * sqrt_info(1, 1) + _tmp120 * _tmp228 + _tmp124 * sqrt_info(1, 2) -
                         _tmp127 * sqrt_info(1, 2) - _tmp129 * sqrt_info(1, 0) +
                         _tmp135 * sqrt_info(1, 0) + _tmp141 * sqrt_info(1, 2) +
                         _tmp143 * sqrt_info(1, 0) + _tmp155 * sqrt_info(1, 5) +
                         _tmp159 * sqrt_info(1, 4) - _tmp226 * sqrt_info(1, 1);
  const Scalar _tmp230 = _tmp166 * _tmp225;
  const Scalar _tmp231 = _tmp166 * _tmp228 + _tmp167 * sqrt_info(1, 2) + _tmp172 * sqrt_info(1, 0) -
                         _tmp173 * sqrt_info(1, 2) + _tmp174 * sqrt_info(1, 0) +
                         _tmp179 * sqrt_info(1, 2) - _tmp180 * sqrt_info(1, 0) +
                         _tmp185 * sqrt_info(1, 1) + _tmp189 * sqrt_info(1, 3) +
                         _tmp194 * sqrt_info(1, 4) - _tmp230 * sqrt_info(1, 1);
  const Scalar _tmp232 = _tmp113 * _tmp200;
  const Scalar _tmp233 = _tmp208 * _tmp225;
  const Scalar _tmp234 = _tmp205 * sqrt_info(1, 0) + _tmp208 * _tmp228 + _tmp209 * sqrt_info(1, 2) -
                         _tmp210 * sqrt_info(1, 2) + _tmp211 * sqrt_info(1, 0) +
                         _tmp216 * sqrt_info(1, 1) - _tmp217 * sqrt_info(1, 0) +
                         _tmp221 * sqrt_info(1, 3) + _tmp223 * sqrt_info(1, 5) +
                         _tmp232 * sqrt_info(1, 2) - _tmp233 * sqrt_info(1, 1);
  const Scalar _tmp235 = _tmp54 * sqrt_info(2, 1);
  const Scalar _tmp236 = _tmp122 * _tmp235;
  const Scalar _tmp237 = _tmp114 * sqrt_info(2, 1) + _tmp120 * _tmp236 + _tmp124 * sqrt_info(2, 2) -
                         _tmp127 * sqrt_info(2, 2) - _tmp129 * sqrt_info(2, 0) +
                         _tmp135 * sqrt_info(2, 0) - _tmp136 * _tmp235 + _tmp141 * sqrt_info(2, 2) +
                         _tmp143 * sqrt_info(2, 0) + _tmp155 * sqrt_info(2, 5) +
                         _tmp159 * sqrt_info(2, 4);
  const Scalar _tmp238 = _tmp126 * sqrt_info(2, 2);
  const Scalar _tmp239 = _tmp166 * _tmp236 - _tmp166 * _tmp238 + _tmp167 * sqrt_info(2, 2) +
                         _tmp172 * sqrt_info(2, 0) + _tmp174 * sqrt_info(2, 0) - _tmp175 * _tmp235 +
                         _tmp179 * sqrt_info(2, 2) - _tmp180 * sqrt_info(2, 0) +
                         _tmp185 * sqrt_info(2, 1) + _tmp189 * sqrt_info(2, 3) +
                         _tmp194 * sqrt_info(2, 4);
  const Scalar _tmp240 = _tmp205 * sqrt_info(2, 0) + _tmp208 * _tmp236 - _tmp208 * _tmp238 +
                         _tmp209 * sqrt_info(2, 2) + _tmp211 * sqrt_info(2, 0) - _tmp212 * _tmp235 +
                         _tmp216 * sqrt_info(2, 1) - _tmp217 * sqrt_info(2, 0) +
                         _tmp221 * sqrt_info(2, 3) + _tmp223 * sqrt_info(2, 5) +
                         _tmp232 * sqrt_info(2, 2);
  const Scalar _tmp241 = _tmp225 * sqrt_info(3, 1);
  const Scalar _tmp242 = _tmp227 * sqrt_info(3, 1);
  const Scalar _tmp243 = _tmp114 * sqrt_info(3, 1) - _tmp120 * _tmp241 + _tmp120 * _tmp242 +
                         _tmp124 * sqrt_info(3, 2) - _tmp127 * sqrt_info(3, 2) -
                         _tmp129 * sqrt_info(3, 0) + _tmp135 * sqrt_info(3, 0) +
                         _tmp141 * sqrt_info(3, 2) + _tmp143 * sqrt_info(3, 0) +
                         _tmp155 * sqrt_info(3, 5) + _tmp159 * sqrt_info(3, 4);
  const Scalar _tmp244 = _tmp126 * sqrt_info(3, 2);
  const Scalar _tmp245 = -_tmp166 * _tmp241 + _tmp166 * _tmp242 - _tmp166 * _tmp244 +
                         _tmp167 * sqrt_info(3, 2) + _tmp172 * sqrt_info(3, 0) +
                         _tmp174 * sqrt_info(3, 0) + _tmp179 * sqrt_info(3, 2) -
                         _tmp180 * sqrt_info(3, 0) + _tmp185 * sqrt_info(3, 1) +
                         _tmp189 * sqrt_info(3, 3) + _tmp194 * sqrt_info(3, 4);
  const Scalar _tmp246 = _tmp205 * sqrt_info(3, 0) - _tmp208 * _tmp241 + _tmp208 * _tmp242 -
                         _tmp208 * _tmp244 + _tmp209 * sqrt_info(3, 2) + _tmp211 * sqrt_info(3, 0) +
                         _tmp216 * sqrt_info(3, 1) - _tmp217 * sqrt_info(3, 0) +
                         _tmp221 * sqrt_info(3, 3) + _tmp223 * sqrt_info(3, 5) +
                         _tmp232 * sqrt_info(3, 2);
  const Scalar _tmp247 = _tmp112 * _tmp80;
  const Scalar _tmp248 = _tmp126 * sqrt_info(4, 2);
  const Scalar _tmp249 = _tmp227 * sqrt_info(4, 1);
  const Scalar _tmp250 = 2 * _tmp106 * _tmp247 - 2 * _tmp120 * _tmp248 + 2 * _tmp120 * _tmp249 +
                         2 * _tmp124 * sqrt_info(4, 2) - 2 * _tmp129 * sqrt_info(4, 0) +
                         2 * _tmp135 * sqrt_info(4, 0) + 2 * _tmp141 * sqrt_info(4, 2) +
                         2 * _tmp143 * sqrt_info(4, 0) + 2 * _tmp155 * sqrt_info(4, 5) +
                         2 * _tmp159 * sqrt_info(4, 4) - 2 * _tmp226 * sqrt_info(4, 1);
  const Scalar _tmp251 = -_tmp166 * _tmp248 + _tmp166 * _tmp249 + _tmp167 * sqrt_info(4, 2) +
                         _tmp172 * sqrt_info(4, 0) + _tmp174 * sqrt_info(4, 0) +
                         _tmp179 * sqrt_info(4, 2) - _tmp180 * sqrt_info(4, 0) + _tmp184 * _tmp247 +
                         _tmp189 * sqrt_info(4, 3) + _tmp194 * sqrt_info(4, 4) -
                         _tmp230 * sqrt_info(4, 1);
  const Scalar _tmp252 = _tmp205 * sqrt_info(4, 0) - _tmp208 * _tmp248 + _tmp208 * _tmp249 +
                         _tmp209 * sqrt_info(4, 2) + _tmp211 * sqrt_info(4, 0) + _tmp215 * _tmp247 -
                         _tmp217 * sqrt_info(4, 0) + _tmp221 * sqrt_info(4, 3) +
                         _tmp223 * sqrt_info(4, 5) + _tmp232 * sqrt_info(4, 2) -
                         _tmp233 * sqrt_info(4, 1);
  const Scalar _tmp253 = _tmp126 * sqrt_info(5, 2);
  const Scalar _tmp254 = _tmp113 * sqrt_info(5, 0);
  const Scalar _tmp255 = _tmp227 * sqrt_info(5, 1);
  const Scalar _tmp256 =
      2 * _tmp114 * sqrt_info(5, 1) - 2 * _tmp120 * _tmp253 + 2 * _tmp120 * _tmp255 +
      2 * _tmp124 * sqrt_info(5, 2) - 2 * _tmp129 * sqrt_info(5, 0) + 2 * _tmp134 * _tmp254 +
      2 * _tmp141 * sqrt_info(5, 2) + 2 * _tmp143 * sqrt_info(5, 0) +
      2 * _tmp155 * sqrt_info(5, 5) + 2 * _tmp159 * sqrt_info(5, 4) - 2 * _tmp226 * sqrt_info(5, 1);
  const Scalar _tmp257 = -_tmp166 * _tmp253 + _tmp166 * _tmp255 + _tmp167 * sqrt_info(5, 2) +
                         _tmp171 * _tmp254 + _tmp174 * sqrt_info(5, 0) + _tmp179 * sqrt_info(5, 2) -
                         _tmp180 * sqrt_info(5, 0) + _tmp185 * sqrt_info(5, 1) +
                         _tmp189 * sqrt_info(5, 3) + _tmp194 * sqrt_info(5, 4) -
                         _tmp230 * sqrt_info(5, 1);
  const Scalar _tmp258 = _tmp204 * _tmp254 - _tmp208 * _tmp253 + _tmp208 * _tmp255 +
                         _tmp209 * sqrt_info(5, 2) + _tmp211 * sqrt_info(5, 0) +
                         _tmp216 * sqrt_info(5, 1) - _tmp217 * sqrt_info(5, 0) +
                         _tmp221 * sqrt_info(5, 3) + _tmp223 * sqrt_info(5, 5) +
                         _tmp232 * sqrt_info(5, 2) - _tmp233 * sqrt_info(5, 1);
  const Scalar _tmp259 = _tmp34 + _tmp57 - 1;
  const Scalar _tmp260 = _tmp37 - 1;
  const Scalar _tmp261 = _tmp260 + _tmp34;
  const Scalar _tmp262 = _tmp260 + _tmp57;
  const Scalar _tmp263 = _tmp87 + _tmp88 + _tmp89 + _tmp90;
  const Scalar _tmp264 = _tmp263 * _x_T_y[2];
  const Scalar _tmp265 = _tmp165 + _tmp202 + _tmp264;
  const Scalar _tmp266 = _tmp125 * _tmp265;
  const Scalar _tmp267 = _tmp263 * _x_T_y[3];
  const Scalar _tmp268 = _tmp178 + _tmp213 + _tmp267;
  const Scalar _tmp269 = _tmp263 * _x_T_y[1];
  const Scalar _tmp270 = _tmp170 - _tmp206 - _tmp269;
  const Scalar _tmp271 = _tmp113 * _tmp270;
  const Scalar _tmp272 = _tmp122 * _tmp265;
  const Scalar _tmp273 = _tmp142 * _tmp265;
  const Scalar _tmp274 = _tmp126 * _tmp265;
  const Scalar _tmp275 = _tmp263 * _x_T_y[0];
  const Scalar _tmp276 = _tmp183 - _tmp198 + _tmp275;
  const Scalar _tmp277 = _tmp113 * _tmp276;
  const Scalar _tmp278 = _tmp128 * _tmp265;
  const Scalar _tmp279 = _tmp123 * _tmp265;
  const Scalar _tmp280 = _tmp201 * _tmp268 - _tmp266 * _tmp55 + _tmp271 * sqrt_info(0, 0) +
                         _tmp272 * _tmp55 + _tmp273 * sqrt_info(0, 0) - _tmp274 * sqrt_info(0, 2) +
                         _tmp277 * sqrt_info(0, 1) - _tmp278 * sqrt_info(0, 0) +
                         _tmp279 * sqrt_info(0, 2);
  const Scalar _tmp281 = 2 * _y[1];
  const Scalar _tmp282 = _tmp119 + _tmp181 + _tmp275;
  const Scalar _tmp283 = _tmp122 * _tmp282;
  const Scalar _tmp284 = _tmp123 * _tmp282;
  const Scalar _tmp285 = _tmp142 * _tmp282;
  const Scalar _tmp286 = _tmp126 * _tmp282;
  const Scalar _tmp287 = _tmp128 * _tmp282;
  const Scalar _tmp288 = _tmp140 + _tmp269;
  const Scalar _tmp289 = _tmp139 - _tmp168 + _tmp288;
  const Scalar _tmp290 = _tmp133 + _tmp267;
  const Scalar _tmp291 = _tmp132 + _tmp176 + _tmp290;
  const Scalar _tmp292 = _tmp113 * _tmp291;
  const Scalar _tmp293 = _tmp125 * _tmp282;
  const Scalar _tmp294 = _tmp105 - _tmp162 - _tmp264;
  const Scalar _tmp295 = _tmp113 * _tmp294;
  const Scalar _tmp296 = _tmp201 * _tmp289 + _tmp283 * _tmp55 + _tmp284 * sqrt_info(0, 2) +
                         _tmp285 * sqrt_info(0, 0) - _tmp286 * sqrt_info(0, 2) -
                         _tmp287 * sqrt_info(0, 0) + _tmp292 * sqrt_info(0, 0) - _tmp293 * _tmp55 +
                         _tmp295 * sqrt_info(0, 1);
  const Scalar _tmp297 = 2 * _y[3];
  const Scalar _tmp298 = _tmp207 + _tmp288;
  const Scalar _tmp299 = _tmp125 * _tmp298;
  const Scalar _tmp300 = _tmp20 * _tmp299;
  const Scalar _tmp301 = -_tmp104 + _tmp203 + _tmp264;
  const Scalar _tmp302 = _tmp113 * _tmp301;
  const Scalar _tmp303 = -_tmp118 + _tmp199 - _tmp275;
  const Scalar _tmp304 = _tmp112 * (_tmp214 + _tmp290);
  const Scalar _tmp305 = _tmp27 * _tmp304;
  const Scalar _tmp306 = _tmp126 * _tmp298;
  const Scalar _tmp307 = _tmp142 * _tmp298;
  const Scalar _tmp308 = _tmp123 * _tmp298;
  const Scalar _tmp309 = _tmp144 * _tmp298 + _tmp201 * _tmp303 - _tmp299 * _tmp55 -
                         _tmp300 * sqrt_info(0, 0) + _tmp302 * sqrt_info(0, 0) +
                         _tmp305 * sqrt_info(0, 1) - _tmp306 * sqrt_info(0, 2) +
                         _tmp307 * sqrt_info(0, 0) + _tmp308 * sqrt_info(0, 2);
  const Scalar _tmp310 = 2 * _y[2];
  const Scalar _tmp311 = _tmp225 * _tmp265;
  const Scalar _tmp312 = _tmp113 * _tmp268;
  const Scalar _tmp313 = _tmp228 * _tmp265 + _tmp271 * sqrt_info(1, 0) + _tmp273 * sqrt_info(1, 0) -
                         _tmp274 * sqrt_info(1, 2) + _tmp277 * sqrt_info(1, 1) -
                         _tmp278 * sqrt_info(1, 0) + _tmp279 * sqrt_info(1, 2) -
                         _tmp311 * sqrt_info(1, 1) + _tmp312 * sqrt_info(1, 2);
  const Scalar _tmp314 = _tmp227 * _tmp282;
  const Scalar _tmp315 = _tmp113 * _tmp289;
  const Scalar _tmp316 = _tmp225 * _tmp282;
  const Scalar _tmp317 =
      _tmp284 * sqrt_info(1, 2) + _tmp285 * sqrt_info(1, 0) - _tmp286 * sqrt_info(1, 2) -
      _tmp287 * sqrt_info(1, 0) + _tmp292 * sqrt_info(1, 0) + _tmp295 * sqrt_info(1, 1) +
      _tmp314 * sqrt_info(1, 1) + _tmp315 * sqrt_info(1, 2) - _tmp316 * sqrt_info(1, 1);
  const Scalar _tmp318 = 2 * _tmp317;
  const Scalar _tmp319 = _tmp113 * _tmp303;
  const Scalar _tmp320 = _tmp299 * _tmp54;
  const Scalar _tmp321 = _tmp228 * _tmp298 - _tmp300 * sqrt_info(1, 0) + _tmp302 * sqrt_info(1, 0) +
                         _tmp305 * sqrt_info(1, 1) - _tmp306 * sqrt_info(1, 2) +
                         _tmp307 * sqrt_info(1, 0) + _tmp308 * sqrt_info(1, 2) +
                         _tmp319 * sqrt_info(1, 2) - _tmp320 * sqrt_info(1, 1);
  const Scalar _tmp322 = -_tmp235 * _tmp266 + _tmp235 * _tmp272 - _tmp238 * _tmp265 +
                         _tmp271 * sqrt_info(2, 0) + _tmp273 * sqrt_info(2, 0) +
                         _tmp277 * sqrt_info(2, 1) - _tmp278 * sqrt_info(2, 0) +
                         _tmp279 * sqrt_info(2, 2) + _tmp312 * sqrt_info(2, 2);
  const Scalar _tmp323 = _tmp235 * _tmp283 - _tmp235 * _tmp293 + _tmp284 * sqrt_info(2, 2) +
                         _tmp285 * sqrt_info(2, 0) - _tmp286 * sqrt_info(2, 2) -
                         _tmp287 * sqrt_info(2, 0) + _tmp292 * sqrt_info(2, 0) +
                         _tmp295 * sqrt_info(2, 1) + _tmp315 * sqrt_info(2, 2);
  const Scalar _tmp324 = -_tmp235 * _tmp299 + _tmp236 * _tmp298 - _tmp238 * _tmp298 -
                         _tmp300 * sqrt_info(2, 0) + _tmp302 * sqrt_info(2, 0) +
                         _tmp305 * sqrt_info(2, 1) + _tmp307 * sqrt_info(2, 0) +
                         _tmp308 * sqrt_info(2, 2) + _tmp319 * sqrt_info(2, 2);
  const Scalar _tmp325 = -_tmp241 * _tmp265 + _tmp242 * _tmp265 - _tmp244 * _tmp265 +
                         _tmp271 * sqrt_info(3, 0) + _tmp273 * sqrt_info(3, 0) +
                         _tmp277 * sqrt_info(3, 1) - _tmp278 * sqrt_info(3, 0) +
                         _tmp279 * sqrt_info(3, 2) + _tmp312 * sqrt_info(3, 2);
  const Scalar _tmp326 =
      -_tmp241 * _tmp282 + _tmp284 * sqrt_info(3, 2) + _tmp285 * sqrt_info(3, 0) -
      _tmp286 * sqrt_info(3, 2) - _tmp287 * sqrt_info(3, 0) + _tmp292 * sqrt_info(3, 0) +
      _tmp295 * sqrt_info(3, 1) + _tmp314 * sqrt_info(3, 1) + _tmp315 * sqrt_info(3, 2);
  const Scalar _tmp327 = 2 * _tmp326;
  const Scalar _tmp328 = _tmp242 * _tmp298 - _tmp244 * _tmp298 - _tmp300 * sqrt_info(3, 0) +
                         _tmp302 * sqrt_info(3, 0) + _tmp305 * sqrt_info(3, 1) +
                         _tmp307 * sqrt_info(3, 0) + _tmp308 * sqrt_info(3, 2) +
                         _tmp319 * sqrt_info(3, 2) - _tmp320 * sqrt_info(3, 1);
  const Scalar _tmp329 = _tmp247 * _tmp276 - _tmp248 * _tmp265 + _tmp249 * _tmp265 +
                         _tmp271 * sqrt_info(4, 0) + _tmp273 * sqrt_info(4, 0) -
                         _tmp278 * sqrt_info(4, 0) + _tmp279 * sqrt_info(4, 2) -
                         _tmp311 * sqrt_info(4, 1) + _tmp312 * sqrt_info(4, 2);
  const Scalar _tmp330 = _tmp247 * _tmp294 - _tmp248 * _tmp282 + _tmp249 * _tmp282 +
                         _tmp284 * sqrt_info(4, 2) + _tmp285 * sqrt_info(4, 0) -
                         _tmp287 * sqrt_info(4, 0) + _tmp292 * sqrt_info(4, 0) +
                         _tmp315 * sqrt_info(4, 2) - _tmp316 * sqrt_info(4, 1);
  const Scalar _tmp331 = 2 * _tmp330;
  const Scalar _tmp332 = -_tmp248 * _tmp298 + _tmp249 * _tmp298 - _tmp300 * sqrt_info(4, 0) +
                         _tmp302 * sqrt_info(4, 0) + _tmp304 * _tmp80 + _tmp307 * sqrt_info(4, 0) +
                         _tmp308 * sqrt_info(4, 2) + _tmp319 * sqrt_info(4, 2) -
                         _tmp320 * sqrt_info(4, 1);
  const Scalar _tmp333 = -_tmp253 * _tmp265 + _tmp254 * _tmp270 + _tmp255 * _tmp265 +
                         _tmp273 * sqrt_info(5, 0) + _tmp277 * sqrt_info(5, 1) -
                         _tmp278 * sqrt_info(5, 0) + _tmp279 * sqrt_info(5, 2) -
                         _tmp311 * sqrt_info(5, 1) + _tmp312 * sqrt_info(5, 2);
  const Scalar _tmp334 = -_tmp253 * _tmp282 + _tmp254 * _tmp291 + _tmp284 * sqrt_info(5, 2) +
                         _tmp285 * sqrt_info(5, 0) - _tmp287 * sqrt_info(5, 0) +
                         _tmp295 * sqrt_info(5, 1) + _tmp314 * sqrt_info(5, 1) +
                         _tmp315 * sqrt_info(5, 2) - _tmp316 * sqrt_info(5, 1);
  const Scalar _tmp335 = 2 * _tmp334;
  const Scalar _tmp336 = -_tmp253 * _tmp298 + _tmp254 * _tmp301 + _tmp255 * _tmp298 -
                         _tmp300 * sqrt_info(5, 0) + _tmp305 * sqrt_info(5, 1) +
                         _tmp307 * sqrt_info(5, 0) + _tmp308 * sqrt_info(5, 2) +
                         _tmp319 * sqrt_info(5, 2) - _tmp320 * sqrt_info(5, 1);
  const Scalar _tmp337 = 2 * _y[0];

  // Output terms (3)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = _tmp31 * _tmp55 + _tmp32 * sqrt_info(0, 0) + _tmp53 * sqrt_info(0, 4) +
               _tmp68 * sqrt_info(0, 5) + _tmp76 * sqrt_info(0, 3) + _tmp78 * sqrt_info(0, 2);
  _res(1, 0) = _tmp32 * sqrt_info(1, 0) + _tmp53 * sqrt_info(1, 4) + _tmp68 * sqrt_info(1, 5) +
               _tmp76 * sqrt_info(1, 3) + _tmp78 * sqrt_info(1, 2) + _tmp79 * sqrt_info(1, 1);
  _res(2, 0) = _tmp32 * sqrt_info(2, 0) + _tmp53 * sqrt_info(2, 4) + _tmp68 * sqrt_info(2, 5) +
               _tmp76 * sqrt_info(2, 3) + _tmp78 * sqrt_info(2, 2) + _tmp79 * sqrt_info(2, 1);
  _res(3, 0) = _tmp32 * sqrt_info(3, 0) + _tmp53 * sqrt_info(3, 4) + _tmp68 * sqrt_info(3, 5) +
               _tmp76 * sqrt_info(3, 3) + _tmp78 * sqrt_info(3, 2) + _tmp79 * sqrt_info(3, 1);
  _res(4, 0) = _tmp30 * _tmp54 * _tmp80 + _tmp32 * sqrt_info(4, 0) + _tmp53 * sqrt_info(4, 4) +
               _tmp68 * sqrt_info(4, 5) + _tmp76 * sqrt_info(4, 3) + _tmp78 * sqrt_info(4, 2);
  _res(5, 0) = _tmp32 * sqrt_info(5, 0) + _tmp53 * sqrt_info(5, 4) + _tmp68 * sqrt_info(5, 5) +
               _tmp76 * sqrt_info(5, 3) + _tmp78 * sqrt_info(5, 2) + _tmp79 * sqrt_info(5, 1);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 6, 7, Eigen::RowMajor>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = _tmp160 * _tmp42 + _tmp195 * _tmp196 - _tmp224 * _tmp48;
    _res_D_x(1, 0) = _tmp196 * _tmp231 + _tmp229 * _tmp42 - _tmp234 * _tmp48;
    _res_D_x(2, 0) = _tmp196 * _tmp239 + _tmp237 * _tmp42 - _tmp240 * _tmp48;
    _res_D_x(3, 0) = _tmp196 * _tmp245 + _tmp243 * _tmp42 - _tmp246 * _tmp48;
    _res_D_x(4, 0) = _tmp196 * _tmp251 + _tmp250 * _x[3] - _tmp252 * _tmp48;
    _res_D_x(5, 0) = _tmp196 * _tmp257 + _tmp256 * _x[3] - _tmp258 * _tmp48;
    _res_D_x(0, 1) = _tmp160 * _tmp48 - _tmp195 * _tmp40 + _tmp224 * _tmp42;
    _res_D_x(1, 1) = _tmp229 * _tmp48 - _tmp231 * _tmp40 + _tmp234 * _tmp42;
    _res_D_x(2, 1) = _tmp237 * _tmp48 - _tmp239 * _tmp40 + _tmp240 * _tmp42;
    _res_D_x(3, 1) = _tmp243 * _tmp48 - _tmp245 * _tmp40 + _tmp246 * _tmp42;
    _res_D_x(4, 1) = _tmp250 * _x[2] - _tmp251 * _tmp40 + _tmp252 * _tmp42;
    _res_D_x(5, 1) = _tmp256 * _x[2] - _tmp257 * _tmp40 + _tmp258 * _tmp42;
    _res_D_x(0, 2) = -_tmp160 * _tmp196 + _tmp195 * _tmp42 + _tmp224 * _tmp40;
    _res_D_x(1, 2) = -_tmp196 * _tmp229 + _tmp231 * _tmp42 + _tmp234 * _tmp40;
    _res_D_x(2, 2) = -_tmp196 * _tmp237 + _tmp239 * _tmp42 + _tmp240 * _tmp40;
    _res_D_x(3, 2) = -_tmp196 * _tmp243 + _tmp245 * _tmp42 + _tmp246 * _tmp40;
    _res_D_x(4, 2) = -_tmp250 * _x[1] + _tmp251 * _tmp42 + _tmp252 * _tmp40;
    _res_D_x(5, 2) = -_tmp256 * _x[1] + _tmp257 * _tmp42 + _tmp258 * _tmp40;
    _res_D_x(0, 3) = -_tmp160 * _tmp40 - _tmp195 * _tmp48 - _tmp196 * _tmp224;
    _res_D_x(1, 3) = -_tmp196 * _tmp234 - _tmp229 * _tmp40 - _tmp231 * _tmp48;
    _res_D_x(2, 3) = -_tmp196 * _tmp240 - _tmp237 * _tmp40 - _tmp239 * _tmp48;
    _res_D_x(3, 3) = -_tmp196 * _tmp246 - _tmp243 * _tmp40 - _tmp245 * _tmp48;
    _res_D_x(4, 3) = -_tmp196 * _tmp252 - _tmp250 * _x[0] - _tmp251 * _tmp48;
    _res_D_x(5, 3) = -_tmp196 * _tmp258 - _tmp256 * _x[0] - _tmp257 * _tmp48;
    _res_D_x(0, 4) =
        _tmp146 * sqrt_info(0, 4) + _tmp218 * sqrt_info(0, 5) + _tmp259 * sqrt_info(0, 3);
    _res_D_x(1, 4) =
        _tmp146 * sqrt_info(1, 4) + _tmp218 * sqrt_info(1, 5) + _tmp259 * sqrt_info(1, 3);
    _res_D_x(2, 4) =
        _tmp146 * sqrt_info(2, 4) + _tmp218 * sqrt_info(2, 5) + _tmp259 * sqrt_info(2, 3);
    _res_D_x(3, 4) =
        _tmp146 * sqrt_info(3, 4) + _tmp218 * sqrt_info(3, 5) + _tmp259 * sqrt_info(3, 3);
    _res_D_x(4, 4) =
        _tmp146 * sqrt_info(4, 4) + _tmp218 * sqrt_info(4, 5) + _tmp259 * sqrt_info(4, 3);
    _res_D_x(5, 4) =
        _tmp146 * sqrt_info(5, 4) + _tmp218 * sqrt_info(5, 5) + _tmp259 * sqrt_info(5, 3);
    _res_D_x(0, 5) =
        _tmp190 * sqrt_info(0, 3) + _tmp220 * sqrt_info(0, 5) + _tmp261 * sqrt_info(0, 4);
    _res_D_x(1, 5) =
        _tmp190 * sqrt_info(1, 3) + _tmp220 * sqrt_info(1, 5) + _tmp261 * sqrt_info(1, 4);
    _res_D_x(2, 5) =
        _tmp190 * sqrt_info(2, 3) + _tmp220 * sqrt_info(2, 5) + _tmp261 * sqrt_info(2, 4);
    _res_D_x(3, 5) =
        _tmp190 * sqrt_info(3, 3) + _tmp220 * sqrt_info(3, 5) + _tmp261 * sqrt_info(3, 4);
    _res_D_x(4, 5) =
        _tmp190 * sqrt_info(4, 3) + _tmp220 * sqrt_info(4, 5) + _tmp261 * sqrt_info(4, 4);
    _res_D_x(5, 5) =
        _tmp190 * sqrt_info(5, 3) + _tmp220 * sqrt_info(5, 5) + _tmp261 * sqrt_info(5, 4);
    _res_D_x(0, 6) =
        _tmp154 * sqrt_info(0, 4) + _tmp192 * sqrt_info(0, 3) + _tmp262 * sqrt_info(0, 5);
    _res_D_x(1, 6) =
        _tmp154 * sqrt_info(1, 4) + _tmp192 * sqrt_info(1, 3) + _tmp262 * sqrt_info(1, 5);
    _res_D_x(2, 6) =
        _tmp154 * sqrt_info(2, 4) + _tmp192 * sqrt_info(2, 3) + _tmp262 * sqrt_info(2, 5);
    _res_D_x(3, 6) =
        _tmp154 * sqrt_info(3, 4) + _tmp192 * sqrt_info(3, 3) + _tmp262 * sqrt_info(3, 5);
    _res_D_x(4, 6) =
        _tmp154 * sqrt_info(4, 4) + _tmp192 * sqrt_info(4, 3) + _tmp262 * sqrt_info(4, 5);
    _res_D_x(5, 6) =
        _tmp154 * sqrt_info(5, 4) + _tmp192 * sqrt_info(5, 3) + _tmp262 * sqrt_info(5, 5);
  }

  if (res_D_y != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 6, 7, Eigen::RowMajor>> _res_D_y{res_D_y};

    _res_D_y(0, 0) = _tmp280 * _tmp281 + _tmp296 * _tmp297 - _tmp309 * _tmp310;
    _res_D_y(1, 0) = _tmp281 * _tmp313 - _tmp310 * _tmp321 + _tmp318 * _y[3];
    _res_D_y(2, 0) = _tmp281 * _tmp322 + _tmp297 * _tmp323 - _tmp310 * _tmp324;
    _res_D_y(3, 0) = _tmp281 * _tmp325 - _tmp310 * _tmp328 + _tmp327 * _y[3];
    _res_D_y(4, 0) = _tmp281 * _tmp329 - _tmp310 * _tmp332 + _tmp331 * _y[3];
    _res_D_y(5, 0) = _tmp281 * _tmp333 - _tmp310 * _tmp336 + _tmp335 * _y[3];
    _res_D_y(0, 1) = -_tmp280 * _tmp337 + _tmp296 * _tmp310 + _tmp297 * _tmp309;
    _res_D_y(1, 1) = _tmp297 * _tmp321 + _tmp310 * _tmp317 - _tmp313 * _tmp337;
    _res_D_y(2, 1) = _tmp297 * _tmp324 + _tmp310 * _tmp323 - _tmp322 * _tmp337;
    _res_D_y(3, 1) = _tmp297 * _tmp328 + _tmp310 * _tmp326 - _tmp325 * _tmp337;
    _res_D_y(4, 1) = _tmp297 * _tmp332 + _tmp310 * _tmp330 - _tmp329 * _tmp337;
    _res_D_y(5, 1) = _tmp297 * _tmp336 + _tmp310 * _tmp334 - _tmp333 * _tmp337;
    _res_D_y(0, 2) = _tmp280 * _tmp297 - _tmp281 * _tmp296 + _tmp309 * _tmp337;
    _res_D_y(1, 2) = -_tmp281 * _tmp317 + _tmp297 * _tmp313 + _tmp321 * _tmp337;
    _res_D_y(2, 2) = -_tmp281 * _tmp323 + _tmp297 * _tmp322 + _tmp324 * _tmp337;
    _res_D_y(3, 2) = -_tmp281 * _tmp326 + _tmp297 * _tmp325 + _tmp328 * _tmp337;
    _res_D_y(4, 2) = -_tmp281 * _tmp330 + _tmp297 * _tmp329 + _tmp332 * _tmp337;
    _res_D_y(5, 2) = -_tmp281 * _tmp334 + _tmp297 * _tmp333 + _tmp336 * _tmp337;
    _res_D_y(0, 3) = -_tmp280 * _tmp310 - _tmp281 * _tmp309 - _tmp296 * _tmp337;
    _res_D_y(1, 3) = -_tmp281 * _tmp321 - _tmp310 * _tmp313 - _tmp318 * _y[0];
    _res_D_y(2, 3) = -_tmp281 * _tmp324 - _tmp310 * _tmp322 - _tmp323 * _tmp337;
    _res_D_y(3, 3) = -_tmp281 * _tmp328 - _tmp310 * _tmp325 - _tmp327 * _y[0];
    _res_D_y(4, 3) = -_tmp281 * _tmp332 - _tmp310 * _tmp329 - _tmp331 * _y[0];
    _res_D_y(5, 3) = -_tmp281 * _tmp336 - _tmp310 * _tmp333 - _tmp335 * _y[0];
    _res_D_y(0, 4) = _tmp45 * sqrt_info(0, 4) + _tmp62 * sqrt_info(0, 5) + _tmp69 * sqrt_info(0, 3);
    _res_D_y(1, 4) = _tmp45 * sqrt_info(1, 4) + _tmp62 * sqrt_info(1, 5) + _tmp69 * sqrt_info(1, 3);
    _res_D_y(2, 4) = _tmp45 * sqrt_info(2, 4) + _tmp62 * sqrt_info(2, 5) + _tmp69 * sqrt_info(2, 3);
    _res_D_y(3, 4) = _tmp45 * sqrt_info(3, 4) + _tmp62 * sqrt_info(3, 5) + _tmp69 * sqrt_info(3, 3);
    _res_D_y(4, 4) = _tmp45 * sqrt_info(4, 4) + _tmp62 * sqrt_info(4, 5) + _tmp69 * sqrt_info(4, 3);
    _res_D_y(5, 4) = _tmp45 * sqrt_info(5, 4) + _tmp62 * sqrt_info(5, 5) + _tmp69 * sqrt_info(5, 3);
    _res_D_y(0, 5) = _tmp39 * sqrt_info(0, 4) + _tmp65 * sqrt_info(0, 5) + _tmp70 * sqrt_info(0, 3);
    _res_D_y(1, 5) = _tmp39 * sqrt_info(1, 4) + _tmp65 * sqrt_info(1, 5) + _tmp70 * sqrt_info(1, 3);
    _res_D_y(2, 5) = _tmp39 * sqrt_info(2, 4) + _tmp65 * sqrt_info(2, 5) + _tmp70 * sqrt_info(2, 3);
    _res_D_y(3, 5) = _tmp39 * sqrt_info(3, 4) + _tmp65 * sqrt_info(3, 5) + _tmp70 * sqrt_info(3, 3);
    _res_D_y(4, 5) = _tmp39 * sqrt_info(4, 4) + _tmp65 * sqrt_info(4, 5) + _tmp70 * sqrt_info(4, 3);
    _res_D_y(5, 5) = _tmp39 * sqrt_info(5, 4) + _tmp65 * sqrt_info(5, 5) + _tmp70 * sqrt_info(5, 3);
    _res_D_y(0, 6) = _tmp50 * sqrt_info(0, 4) + _tmp59 * sqrt_info(0, 5) + _tmp73 * sqrt_info(0, 3);
    _res_D_y(1, 6) = _tmp50 * sqrt_info(1, 4) + _tmp59 * sqrt_info(1, 5) + _tmp73 * sqrt_info(1, 3);
    _res_D_y(2, 6) = _tmp50 * sqrt_info(2, 4) + _tmp59 * sqrt_info(2, 5) + _tmp73 * sqrt_info(2, 3);
    _res_D_y(3, 6) = _tmp50 * sqrt_info(3, 4) + _tmp59 * sqrt_info(3, 5) + _tmp73 * sqrt_info(3, 3);
    _res_D_y(4, 6) = _tmp50 * sqrt_info(4, 4) + _tmp59 * sqrt_info(4, 5) + _tmp73 * sqrt_info(4, 3);
    _res_D_y(5, 6) = _tmp50 * sqrt_info(5, 4) + _tmp59 * sqrt_info(5, 5) + _tmp73 * sqrt_info(5, 3);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
