// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/linear_camera_cal.h>
#include <sym/pose3.h>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: pose3_camera_linear_bearing_factor
 *
 * Args:
 *     w_T_b: Pose3
 *     b_T_c: Pose3
 *     calibration: LinearCameraCal
 *     l_w: Matrix31
 *     pixel: Matrix21
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 *     res_D_w_T_b: (1x6) jacobian of res (1) wrt arg w_T_b (6)
 *     res_D_b_T_c: (1x6) jacobian of res (1) wrt arg b_T_c (6)
 *     res_D_calibration: (1x4) jacobian of res (1) wrt arg calibration (4)
 *     res_D_l_w: (1x3) jacobian of res (1) wrt arg l_w (3)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Pose3CameraLinearBearingFactorWithJacobians0123(
    const sym::Pose3<Scalar>& w_T_b, const sym::Pose3<Scalar>& b_T_c,
    const sym::LinearCameraCal<Scalar>& calibration, const Eigen::Matrix<Scalar, 3, 1>& l_w,
    const Eigen::Matrix<Scalar, 2, 1>& pixel, const Eigen::Matrix<Scalar, 1, 1>& sqrt_info,
    const Scalar epsilon, Scalar* const res_D_w_T_b = nullptr, Scalar* const res_D_b_T_c = nullptr,
    Scalar* const res_D_calibration = nullptr, Scalar* const res_D_l_w = nullptr) {
  // Total ops: 1226

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _w_T_b = w_T_b.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _b_T_c = b_T_c.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _calibration = calibration.Data();

  // Intermediate terms (415)
  const Scalar _tmp0 = _b_T_c[1] * _w_T_b[2];
  const Scalar _tmp1 = _b_T_c[2] * _w_T_b[1];
  const Scalar _tmp2 = _b_T_c[3] * _w_T_b[0];
  const Scalar _tmp3 = _b_T_c[0] * _w_T_b[3];
  const Scalar _tmp4 = -_tmp0 + _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp5 = 2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = -_tmp5;
  const Scalar _tmp7 = _b_T_c[1] * _w_T_b[3];
  const Scalar _tmp8 = _b_T_c[2] * _w_T_b[0];
  const Scalar _tmp9 = _b_T_c[3] * _w_T_b[1];
  const Scalar _tmp10 = _b_T_c[0] * _w_T_b[2];
  const Scalar _tmp11 = _tmp10 + _tmp7 - _tmp8 + _tmp9;
  const Scalar _tmp12 = 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 = 1 - _tmp12;
  const Scalar _tmp14 = _tmp13 + _tmp6;
  const Scalar _tmp15 = _b_T_c[1] * _w_T_b[0];
  const Scalar _tmp16 = _b_T_c[2] * _w_T_b[3];
  const Scalar _tmp17 = _b_T_c[3] * _w_T_b[2];
  const Scalar _tmp18 = _b_T_c[0] * _w_T_b[1];
  const Scalar _tmp19 = _tmp15 + _tmp16 + _tmp17 - _tmp18;
  const Scalar _tmp20 = 2 * _tmp19;
  const Scalar _tmp21 = _tmp20 * _tmp4;
  const Scalar _tmp22 = _b_T_c[1] * _w_T_b[1];
  const Scalar _tmp23 = _b_T_c[2] * _w_T_b[2];
  const Scalar _tmp24 = _b_T_c[0] * _w_T_b[0];
  const Scalar _tmp25 = _b_T_c[3] * _w_T_b[3];
  const Scalar _tmp26 = -_tmp22 - _tmp23 - _tmp24 + _tmp25;
  const Scalar _tmp27 = 2 * _tmp11;
  const Scalar _tmp28 = _tmp26 * _tmp27;
  const Scalar _tmp29 = _tmp21 + _tmp28;
  const Scalar _tmp30 = _tmp19 * _tmp27;
  const Scalar _tmp31 = 2 * _tmp4;
  const Scalar _tmp32 = _tmp26 * _tmp31;
  const Scalar _tmp33 = -_tmp32;
  const Scalar _tmp34 = _tmp30 + _tmp33;
  const Scalar _tmp35 = std::pow(_w_T_b[0], Scalar(2));
  const Scalar _tmp36 = -2 * _tmp35;
  const Scalar _tmp37 = std::pow(_w_T_b[2], Scalar(2));
  const Scalar _tmp38 = 1 - 2 * _tmp37;
  const Scalar _tmp39 = _tmp36 + _tmp38;
  const Scalar _tmp40 = 2 * _w_T_b[0];
  const Scalar _tmp41 = _tmp40 * _w_T_b[1];
  const Scalar _tmp42 = 2 * _w_T_b[2];
  const Scalar _tmp43 = _tmp42 * _w_T_b[3];
  const Scalar _tmp44 = _tmp41 + _tmp43;
  const Scalar _tmp45 = _tmp40 * _w_T_b[3];
  const Scalar _tmp46 = -_tmp45;
  const Scalar _tmp47 = _tmp42 * _w_T_b[1];
  const Scalar _tmp48 = _tmp46 + _tmp47;
  const Scalar _tmp49 = _b_T_c[4] * _tmp44 + _b_T_c[5] * _tmp39 + _b_T_c[6] * _tmp48 + _w_T_b[5];
  const Scalar _tmp50 = std::pow(_w_T_b[1], Scalar(2));
  const Scalar _tmp51 = -2 * _tmp50;
  const Scalar _tmp52 = _tmp38 + _tmp51;
  const Scalar _tmp53 = -_tmp43;
  const Scalar _tmp54 = _tmp41 + _tmp53;
  const Scalar _tmp55 = _tmp42 * _w_T_b[0];
  const Scalar _tmp56 = 2 * _w_T_b[1] * _w_T_b[3];
  const Scalar _tmp57 = _tmp55 + _tmp56;
  const Scalar _tmp58 = _b_T_c[4] * _tmp52 + _b_T_c[5] * _tmp54 + _b_T_c[6] * _tmp57 + _w_T_b[4];
  const Scalar _tmp59 = _tmp36 + _tmp51 + 1;
  const Scalar _tmp60 = -_tmp56;
  const Scalar _tmp61 = _tmp55 + _tmp60;
  const Scalar _tmp62 = _tmp45 + _tmp47;
  const Scalar _tmp63 = _b_T_c[4] * _tmp61 + _b_T_c[5] * _tmp62 + _b_T_c[6] * _tmp59 + _w_T_b[6];
  const Scalar _tmp64 = -_tmp14 * _tmp63 + _tmp14 * l_w(2, 0) - _tmp29 * _tmp58 +
                        _tmp29 * l_w(0, 0) - _tmp34 * _tmp49 + _tmp34 * l_w(1, 0);
  const Scalar _tmp65 = -_calibration[3] + pixel(1, 0);
  const Scalar _tmp66 = Scalar(1.0) / (_calibration[1]);
  const Scalar _tmp67 = _tmp65 * _tmp66;
  const Scalar _tmp68 = 2 * std::pow(_tmp19, Scalar(2));
  const Scalar _tmp69 = -_tmp68;
  const Scalar _tmp70 = _tmp6 + _tmp69 + 1;
  const Scalar _tmp71 = _tmp27 * _tmp4;
  const Scalar _tmp72 = _tmp20 * _tmp26;
  const Scalar _tmp73 = -_tmp72;
  const Scalar _tmp74 = _tmp71 + _tmp73;
  const Scalar _tmp75 = _tmp30 + _tmp32;
  const Scalar _tmp76 = -_tmp49 * _tmp70 - _tmp58 * _tmp74 - _tmp63 * _tmp75 + _tmp70 * l_w(1, 0) +
                        _tmp74 * l_w(0, 0) + _tmp75 * l_w(2, 0);
  const Scalar _tmp77 = _tmp64 * _tmp67 - _tmp76;
  const Scalar _tmp78 = -_calibration[2] + pixel(0, 0);
  const Scalar _tmp79 = Scalar(1.0) / (_calibration[0]);
  const Scalar _tmp80 = _tmp78 * _tmp79;
  const Scalar _tmp81 = _tmp13 + _tmp69;
  const Scalar _tmp82 = -_tmp28;
  const Scalar _tmp83 = _tmp21 + _tmp82;
  const Scalar _tmp84 = _tmp71 + _tmp72;
  const Scalar _tmp85 = -_tmp49 * _tmp84 - _tmp58 * _tmp81 - _tmp63 * _tmp83 + _tmp81 * l_w(0, 0) +
                        _tmp83 * l_w(2, 0) + _tmp84 * l_w(1, 0);
  const Scalar _tmp86 = -_tmp64 * _tmp80 + _tmp85;
  const Scalar _tmp87 = -_tmp67 * _tmp85 + _tmp76 * _tmp80;
  const Scalar _tmp88 = std::pow(_tmp77, Scalar(2)) + std::pow(_tmp86, Scalar(2)) +
                        std::pow(_tmp87, Scalar(2)) + epsilon;
  const Scalar _tmp89 = std::sqrt(_tmp88);
  const Scalar _tmp90 = _tmp64 + _tmp67 * _tmp76 + _tmp80 * _tmp85;
  const Scalar _tmp91 = _tmp90 + epsilon * ((((_tmp90) > 0) - ((_tmp90) < 0)) + Scalar(0.5));
  const Scalar _tmp92 = (Scalar(1) / Scalar(2)) * _tmp24;
  const Scalar _tmp93 = -_tmp92;
  const Scalar _tmp94 = (Scalar(1) / Scalar(2)) * _tmp23;
  const Scalar _tmp95 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp96 = (Scalar(1) / Scalar(2)) * _tmp22;
  const Scalar _tmp97 = _tmp95 + _tmp96;
  const Scalar _tmp98 = _tmp93 + _tmp94 + _tmp97;
  const Scalar _tmp99 = 4 * _tmp4;
  const Scalar _tmp100 = -_tmp98 * _tmp99;
  const Scalar _tmp101 = (Scalar(1) / Scalar(2)) * _tmp16;
  const Scalar _tmp102 = -_tmp101;
  const Scalar _tmp103 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp104 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp105 = -_tmp104;
  const Scalar _tmp106 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp107 = -_tmp106;
  const Scalar _tmp108 = _tmp105 + _tmp107;
  const Scalar _tmp109 = _tmp102 + _tmp103 + _tmp108;
  const Scalar _tmp110 = 4 * _tmp11;
  const Scalar _tmp111 = -_tmp109 * _tmp110;
  const Scalar _tmp112 = _tmp100 + _tmp111;
  const Scalar _tmp113 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp114 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp115 = -_tmp114;
  const Scalar _tmp116 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp117 = -_tmp116;
  const Scalar _tmp118 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp119 = -_tmp118;
  const Scalar _tmp120 = _tmp117 + _tmp119;
  const Scalar _tmp121 = _tmp113 + _tmp115 + _tmp120;
  const Scalar _tmp122 = _tmp121 * _tmp27;
  const Scalar _tmp123 = 2 * _tmp26;
  const Scalar _tmp124 = _tmp109 * _tmp123;
  const Scalar _tmp125 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp126 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp127 = -_tmp126;
  const Scalar _tmp128 = _tmp125 + _tmp127;
  const Scalar _tmp129 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp130 = -_tmp129;
  const Scalar _tmp131 = (Scalar(1) / Scalar(2)) * _tmp10;
  const Scalar _tmp132 = -_tmp131;
  const Scalar _tmp133 = _tmp130 + _tmp132;
  const Scalar _tmp134 = _tmp128 + _tmp133;
  const Scalar _tmp135 = _tmp134 * _tmp31 + _tmp20 * _tmp98;
  const Scalar _tmp136 = _tmp122 + _tmp124 + _tmp135;
  const Scalar _tmp137 = _tmp121 * _tmp31;
  const Scalar _tmp138 = _tmp123 * _tmp98;
  const Scalar _tmp139 = _tmp109 * _tmp20 + _tmp134 * _tmp27;
  const Scalar _tmp140 = -_tmp137 - _tmp138 + _tmp139;
  const Scalar _tmp141 = -_tmp41;
  const Scalar _tmp142 = _b_T_c[5] * _tmp57 + _b_T_c[6] * (_tmp141 + _tmp43);
  const Scalar _tmp143 = -_tmp35;
  const Scalar _tmp144 = -_tmp50;
  const Scalar _tmp145 = std::pow(_w_T_b[3], Scalar(2));
  const Scalar _tmp146 = -_tmp47;
  const Scalar _tmp147 =
      _b_T_c[5] * (_tmp143 + _tmp144 + _tmp145 + _tmp37) + _b_T_c[6] * (_tmp146 + _tmp46);
  const Scalar _tmp148 = -_tmp145;
  const Scalar _tmp149 = _tmp148 + _tmp37;
  const Scalar _tmp150 = _tmp144 + _tmp35;
  const Scalar _tmp151 = _b_T_c[5] * _tmp48 + _b_T_c[6] * (_tmp149 + _tmp150);
  const Scalar _tmp152 = -_tmp112 * _tmp63 + _tmp112 * l_w(2, 0) - _tmp136 * _tmp58 +
                         _tmp136 * l_w(0, 0) - _tmp14 * _tmp147 - _tmp140 * _tmp49 +
                         _tmp140 * l_w(1, 0) - _tmp142 * _tmp29 - _tmp151 * _tmp34;
  const Scalar _tmp153 = -_tmp122 - _tmp124 + _tmp135;
  const Scalar _tmp154 = 4 * _tmp19;
  const Scalar _tmp155 = -_tmp134 * _tmp154;
  const Scalar _tmp156 = _tmp111 + _tmp155;
  const Scalar _tmp157 = _tmp121 * _tmp20;
  const Scalar _tmp158 = _tmp123 * _tmp134;
  const Scalar _tmp159 = _tmp109 * _tmp31 + _tmp27 * _tmp98;
  const Scalar _tmp160 = _tmp157 + _tmp158 + _tmp159;
  const Scalar _tmp161 = -_tmp142 * _tmp81 - _tmp147 * _tmp83 - _tmp151 * _tmp84 -
                         _tmp153 * _tmp63 + _tmp153 * l_w(2, 0) - _tmp156 * _tmp58 +
                         _tmp156 * l_w(0, 0) - _tmp160 * _tmp49 + _tmp160 * l_w(1, 0);
  const Scalar _tmp162 = 2 * _tmp86;
  const Scalar _tmp163 = _tmp137 + _tmp138 + _tmp139;
  const Scalar _tmp164 = _tmp163 * _tmp63;
  const Scalar _tmp165 = _tmp100 + _tmp155;
  const Scalar _tmp166 = _tmp165 * _tmp49;
  const Scalar _tmp167 = _tmp163 * l_w(2, 0);
  const Scalar _tmp168 = -_tmp157 - _tmp158 + _tmp159;
  const Scalar _tmp169 = _tmp168 * l_w(0, 0);
  const Scalar _tmp170 = _tmp142 * _tmp74;
  const Scalar _tmp171 = _tmp168 * _tmp58;
  const Scalar _tmp172 = _tmp165 * l_w(1, 0);
  const Scalar _tmp173 = _tmp151 * _tmp70;
  const Scalar _tmp174 = _tmp147 * _tmp75;
  const Scalar _tmp175 =
      -_tmp164 - _tmp166 + _tmp167 + _tmp169 - _tmp170 - _tmp171 + _tmp172 - _tmp173 - _tmp174;
  const Scalar _tmp176 = 2 * _tmp87;
  const Scalar _tmp177 = 2 * _tmp77;
  const Scalar _tmp178 = (Scalar(1) / Scalar(2)) / (_tmp89 * _tmp91);
  const Scalar _tmp179 = std::pow(_tmp91, Scalar(2));
  const Scalar _tmp180 = _tmp89 / _tmp179;
  const Scalar _tmp181 = _tmp179 * sqrt_info(0, 0) / (_tmp179 + _tmp88);
  const Scalar _tmp182 = -_tmp125;
  const Scalar _tmp183 = _tmp127 + _tmp182;
  const Scalar _tmp184 = _tmp130 + _tmp131 + _tmp183;
  const Scalar _tmp185 = _tmp184 * _tmp27;
  const Scalar _tmp186 = _tmp95 - _tmp96;
  const Scalar _tmp187 = _tmp186 + _tmp92 + _tmp94;
  const Scalar _tmp188 = _tmp123 * _tmp187;
  const Scalar _tmp189 = _tmp117 + _tmp118;
  const Scalar _tmp190 = -_tmp113;
  const Scalar _tmp191 = _tmp115 + _tmp190;
  const Scalar _tmp192 = _tmp189 + _tmp191;
  const Scalar _tmp193 = -_tmp103;
  const Scalar _tmp194 = _tmp101 + _tmp108 + _tmp193;
  const Scalar _tmp195 = _tmp192 * _tmp31 + _tmp194 * _tmp20;
  const Scalar _tmp196 = -_tmp185 - _tmp188 + _tmp195;
  const Scalar _tmp197 = -_tmp110 * _tmp187;
  const Scalar _tmp198 = -_tmp154 * _tmp192;
  const Scalar _tmp199 = _tmp197 + _tmp198;
  const Scalar _tmp200 = _tmp184 * _tmp20;
  const Scalar _tmp201 = _tmp123 * _tmp192;
  const Scalar _tmp202 = _tmp187 * _tmp31 + _tmp194 * _tmp27;
  const Scalar _tmp203 = _tmp200 + _tmp201 + _tmp202;
  const Scalar _tmp204 = _b_T_c[4] * (_tmp146 + _tmp45) + _b_T_c[6] * _tmp44;
  const Scalar _tmp205 = -_tmp37;
  const Scalar _tmp206 = _b_T_c[4] * (_tmp148 + _tmp205 + _tmp35 + _tmp50) + _b_T_c[6] * _tmp61;
  const Scalar _tmp207 = _tmp145 + _tmp205;
  const Scalar _tmp208 = -_tmp55;
  const Scalar _tmp209 = _b_T_c[4] * (_tmp208 + _tmp60) + _b_T_c[6] * (_tmp150 + _tmp207);
  const Scalar _tmp210 = -_tmp196 * _tmp63 + _tmp196 * l_w(2, 0) - _tmp199 * _tmp58 +
                         _tmp199 * l_w(0, 0) - _tmp203 * _tmp49 + _tmp203 * l_w(1, 0) -
                         _tmp204 * _tmp84 - _tmp206 * _tmp83 - _tmp209 * _tmp81;
  const Scalar _tmp211 = -_tmp194 * _tmp99;
  const Scalar _tmp212 = _tmp198 + _tmp211;
  const Scalar _tmp213 = _tmp212 * _tmp49;
  const Scalar _tmp214 = _tmp184 * _tmp31;
  const Scalar _tmp215 = _tmp123 * _tmp194;
  const Scalar _tmp216 = _tmp187 * _tmp20 + _tmp192 * _tmp27;
  const Scalar _tmp217 = _tmp214 + _tmp215 + _tmp216;
  const Scalar _tmp218 = _tmp217 * _tmp63;
  const Scalar _tmp219 = _tmp217 * l_w(2, 0);
  const Scalar _tmp220 = -_tmp200 - _tmp201 + _tmp202;
  const Scalar _tmp221 = _tmp220 * l_w(0, 0);
  const Scalar _tmp222 = _tmp209 * _tmp74;
  const Scalar _tmp223 = _tmp212 * l_w(1, 0);
  const Scalar _tmp224 = _tmp206 * _tmp75;
  const Scalar _tmp225 = _tmp204 * _tmp70;
  const Scalar _tmp226 = _tmp220 * _tmp58;
  const Scalar _tmp227 =
      -_tmp213 - _tmp218 + _tmp219 + _tmp221 - _tmp222 + _tmp223 - _tmp224 - _tmp225 - _tmp226;
  const Scalar _tmp228 = _tmp197 + _tmp211;
  const Scalar _tmp229 = -_tmp214 - _tmp215 + _tmp216;
  const Scalar _tmp230 = _tmp185 + _tmp188 + _tmp195;
  const Scalar _tmp231 = -_tmp14 * _tmp206 - _tmp204 * _tmp34 - _tmp209 * _tmp29 -
                         _tmp228 * _tmp63 + _tmp228 * l_w(2, 0) - _tmp229 * _tmp49 +
                         _tmp229 * l_w(1, 0) - _tmp230 * _tmp58 + _tmp230 * l_w(0, 0);
  const Scalar _tmp232 = _b_T_c[4] * _tmp62 + _b_T_c[5] * (_tmp208 + _tmp56);
  const Scalar _tmp233 = _tmp102 + _tmp193;
  const Scalar _tmp234 = _tmp104 + _tmp107;
  const Scalar _tmp235 = _tmp233 + _tmp234;
  const Scalar _tmp236 = _tmp235 * _tmp27;
  const Scalar _tmp237 = _tmp114 + _tmp120 + _tmp190;
  const Scalar _tmp238 = _tmp123 * _tmp237;
  const Scalar _tmp239 = -_tmp94;
  const Scalar _tmp240 = _tmp239 + _tmp92 + _tmp97;
  const Scalar _tmp241 = _tmp129 + _tmp132 + _tmp183;
  const Scalar _tmp242 = _tmp20 * _tmp241 + _tmp240 * _tmp31;
  const Scalar _tmp243 = _tmp236 + _tmp238 + _tmp242;
  const Scalar _tmp244 = -_tmp241 * _tmp99;
  const Scalar _tmp245 = -_tmp110 * _tmp237;
  const Scalar _tmp246 = _tmp244 + _tmp245;
  const Scalar _tmp247 = _tmp235 * _tmp31;
  const Scalar _tmp248 = _tmp123 * _tmp241;
  const Scalar _tmp249 = _tmp20 * _tmp237 + _tmp240 * _tmp27;
  const Scalar _tmp250 = -_tmp247 - _tmp248 + _tmp249;
  const Scalar _tmp251 = _tmp143 + _tmp50;
  const Scalar _tmp252 = _b_T_c[4] * _tmp54 + _b_T_c[5] * (_tmp149 + _tmp251);
  const Scalar _tmp253 = _b_T_c[4] * (_tmp207 + _tmp251) + _b_T_c[5] * (_tmp141 + _tmp53);
  const Scalar _tmp254 = -_tmp14 * _tmp232 - _tmp243 * _tmp58 + _tmp243 * l_w(0, 0) -
                         _tmp246 * _tmp63 + _tmp246 * l_w(2, 0) - _tmp250 * _tmp49 +
                         _tmp250 * l_w(1, 0) - _tmp252 * _tmp29 - _tmp253 * _tmp34;
  const Scalar _tmp255 = -_tmp236 - _tmp238 + _tmp242;
  const Scalar _tmp256 = _tmp20 * _tmp235;
  const Scalar _tmp257 = _tmp123 * _tmp240;
  const Scalar _tmp258 = _tmp237 * _tmp31 + _tmp241 * _tmp27;
  const Scalar _tmp259 = _tmp256 + _tmp257 + _tmp258;
  const Scalar _tmp260 = -_tmp154 * _tmp240;
  const Scalar _tmp261 = _tmp245 + _tmp260;
  const Scalar _tmp262 = -_tmp232 * _tmp83 - _tmp252 * _tmp81 - _tmp253 * _tmp84 -
                         _tmp255 * _tmp63 + _tmp255 * l_w(2, 0) - _tmp259 * _tmp49 +
                         _tmp259 * l_w(1, 0) - _tmp261 * _tmp58 + _tmp261 * l_w(0, 0);
  const Scalar _tmp263 = _tmp244 + _tmp260;
  const Scalar _tmp264 = _tmp263 * _tmp49;
  const Scalar _tmp265 = _tmp247 + _tmp248 + _tmp249;
  const Scalar _tmp266 = _tmp265 * _tmp63;
  const Scalar _tmp267 = _tmp253 * _tmp70;
  const Scalar _tmp268 = -_tmp256 - _tmp257 + _tmp258;
  const Scalar _tmp269 = _tmp268 * l_w(0, 0);
  const Scalar _tmp270 = _tmp265 * l_w(2, 0);
  const Scalar _tmp271 = _tmp232 * _tmp75;
  const Scalar _tmp272 = _tmp252 * _tmp74;
  const Scalar _tmp273 = _tmp263 * l_w(1, 0);
  const Scalar _tmp274 = _tmp268 * _tmp58;
  const Scalar _tmp275 =
      -_tmp264 - _tmp266 - _tmp267 + _tmp269 + _tmp270 - _tmp271 - _tmp272 + _tmp273 - _tmp274;
  const Scalar _tmp276 = -_tmp21;
  const Scalar _tmp277 = _tmp276 + _tmp82;
  const Scalar _tmp278 = _tmp68 - 1;
  const Scalar _tmp279 = _tmp12 + _tmp278;
  const Scalar _tmp280 = -_tmp71;
  const Scalar _tmp281 = _tmp280 + _tmp72;
  const Scalar _tmp282 = -_tmp30;
  const Scalar _tmp283 = _tmp282 + _tmp32;
  const Scalar _tmp284 = _tmp280 + _tmp73;
  const Scalar _tmp285 = _tmp278 + _tmp5;
  const Scalar _tmp286 = _tmp276 + _tmp28;
  const Scalar _tmp287 = _tmp282 + _tmp33;
  const Scalar _tmp288 = _tmp12 + _tmp5 - 1;
  const Scalar _tmp289 = _tmp126 + _tmp133 + _tmp182;
  const Scalar _tmp290 = _tmp27 * _tmp289;
  const Scalar _tmp291 = _tmp101 + _tmp103 + _tmp234;
  const Scalar _tmp292 = _tmp20 * _tmp291 + _tmp290;
  const Scalar _tmp293 = _tmp186 + _tmp239 + _tmp93;
  const Scalar _tmp294 = _tmp123 * _tmp293;
  const Scalar _tmp295 = _tmp116 + _tmp119 + _tmp191;
  const Scalar _tmp296 = _tmp295 * _tmp31;
  const Scalar _tmp297 = _tmp294 + _tmp296;
  const Scalar _tmp298 = _tmp292 + _tmp297;
  const Scalar _tmp299 = _tmp298 * _tmp63;
  const Scalar _tmp300 = -_tmp293 * _tmp99;
  const Scalar _tmp301 = -_tmp154 * _tmp289;
  const Scalar _tmp302 = _tmp300 + _tmp301;
  const Scalar _tmp303 = _tmp302 * _tmp49;
  const Scalar _tmp304 = _tmp298 * l_w(2, 0);
  const Scalar _tmp305 = _tmp20 * _tmp295;
  const Scalar _tmp306 = _tmp123 * _tmp289;
  const Scalar _tmp307 = _tmp27 * _tmp293;
  const Scalar _tmp308 = _tmp291 * _tmp31 + _tmp307;
  const Scalar _tmp309 = -_tmp305 - _tmp306 + _tmp308;
  const Scalar _tmp310 = _tmp309 * l_w(0, 0);
  const Scalar _tmp311 = _tmp309 * _tmp58;
  const Scalar _tmp312 = _tmp302 * l_w(1, 0);
  const Scalar _tmp313 = -_tmp299 - _tmp303 + _tmp304 + _tmp310 - _tmp311 + _tmp312;
  const Scalar _tmp314 = _tmp27 * _tmp295;
  const Scalar _tmp315 = _tmp123 * _tmp291;
  const Scalar _tmp316 = _tmp289 * _tmp31;
  const Scalar _tmp317 = _tmp20 * _tmp293;
  const Scalar _tmp318 = _tmp316 + _tmp317;
  const Scalar _tmp319 = -_tmp314 - _tmp315 + _tmp318;
  const Scalar _tmp320 = -_tmp110 * _tmp291;
  const Scalar _tmp321 = _tmp301 + _tmp320;
  const Scalar _tmp322 = _tmp305 + _tmp306 + _tmp308;
  const Scalar _tmp323 = -_tmp319 * _tmp63 + _tmp319 * l_w(2, 0) - _tmp321 * _tmp58 +
                         _tmp321 * l_w(0, 0) - _tmp322 * _tmp49 + _tmp322 * l_w(1, 0);
  const Scalar _tmp324 = _tmp300 + _tmp320;
  const Scalar _tmp325 = _tmp314 + _tmp315 + _tmp318;
  const Scalar _tmp326 = -_tmp294;
  const Scalar _tmp327 = _tmp292 - _tmp296 + _tmp326;
  const Scalar _tmp328 = -_tmp324 * _tmp63 + _tmp324 * l_w(2, 0) - _tmp325 * _tmp58 +
                         _tmp325 * l_w(0, 0) - _tmp327 * _tmp49 + _tmp327 * l_w(1, 0);
  const Scalar _tmp329 = _tmp105 + _tmp106 + _tmp233;
  const Scalar _tmp330 = -_tmp329 * _tmp99;
  const Scalar _tmp331 = _tmp113 + _tmp114 + _tmp189;
  const Scalar _tmp332 = -_tmp154 * _tmp331;
  const Scalar _tmp333 = _tmp330 + _tmp332;
  const Scalar _tmp334 = _tmp333 * _tmp49;
  const Scalar _tmp335 = _tmp27 * _tmp331;
  const Scalar _tmp336 = _tmp123 * _tmp329;
  const Scalar _tmp337 = _tmp318 + _tmp335 + _tmp336;
  const Scalar _tmp338 = _tmp337 * _tmp63;
  const Scalar _tmp339 = _tmp337 * l_w(2, 0);
  const Scalar _tmp340 = _tmp20 * _tmp289;
  const Scalar _tmp341 = _tmp123 * _tmp331;
  const Scalar _tmp342 = _tmp293 * _tmp31;
  const Scalar _tmp343 = _tmp27 * _tmp329;
  const Scalar _tmp344 = _tmp342 + _tmp343;
  const Scalar _tmp345 = -_tmp340 - _tmp341 + _tmp344;
  const Scalar _tmp346 = _tmp345 * l_w(0, 0);
  const Scalar _tmp347 = -_tmp110 * _tmp293;
  const Scalar _tmp348 = _tmp330 + _tmp347;
  const Scalar _tmp349 = -_tmp316 + _tmp317 + _tmp335 - _tmp336;
  const Scalar _tmp350 = _tmp20 * _tmp329;
  const Scalar _tmp351 = _tmp31 * _tmp331 + _tmp350;
  const Scalar _tmp352 = _tmp290 + _tmp294 + _tmp351;
  const Scalar _tmp353 = -_tmp348 * _tmp63 + _tmp348 * l_w(2, 0) - _tmp349 * _tmp49 +
                         _tmp349 * l_w(1, 0) - _tmp352 * _tmp58 + _tmp352 * l_w(0, 0);
  const Scalar _tmp354 = _tmp333 * l_w(1, 0);
  const Scalar _tmp355 = _tmp345 * _tmp58;
  const Scalar _tmp356 = -_tmp290 + _tmp326 + _tmp351;
  const Scalar _tmp357 = _tmp332 + _tmp347;
  const Scalar _tmp358 = _tmp340 + _tmp341 + _tmp344;
  const Scalar _tmp359 = -_tmp356 * _tmp63 + _tmp356 * l_w(2, 0) - _tmp357 * _tmp58 +
                         _tmp357 * l_w(0, 0) - _tmp358 * _tmp49 + _tmp358 * l_w(1, 0);
  const Scalar _tmp360 = -_tmp334 - _tmp338 + _tmp339 + _tmp346 + _tmp354 - _tmp355;
  const Scalar _tmp361 = _tmp128 + _tmp129 + _tmp131;
  const Scalar _tmp362 = _tmp20 * _tmp361;
  const Scalar _tmp363 = _tmp123 * _tmp295;
  const Scalar _tmp364 = _tmp342 - _tmp343 + _tmp362 - _tmp363;
  const Scalar _tmp365 = _tmp27 * _tmp361;
  const Scalar _tmp366 = _tmp297 + _tmp350 + _tmp365;
  const Scalar _tmp367 = -_tmp110 * _tmp295;
  const Scalar _tmp368 = -_tmp154 * _tmp293;
  const Scalar _tmp369 = _tmp367 + _tmp368;
  const Scalar _tmp370 = -_tmp364 * _tmp63 + _tmp364 * l_w(2, 0) - _tmp366 * _tmp49 +
                         _tmp366 * l_w(1, 0) - _tmp369 * _tmp58 + _tmp369 * l_w(0, 0);
  const Scalar _tmp371 = -_tmp361 * _tmp99;
  const Scalar _tmp372 = _tmp368 + _tmp371;
  const Scalar _tmp373 = _tmp372 * _tmp49;
  const Scalar _tmp374 = _tmp31 * _tmp329;
  const Scalar _tmp375 = _tmp123 * _tmp361;
  const Scalar _tmp376 = _tmp305 + _tmp307;
  const Scalar _tmp377 = _tmp374 + _tmp375 + _tmp376;
  const Scalar _tmp378 = _tmp377 * _tmp63;
  const Scalar _tmp379 = _tmp296 + _tmp326 - _tmp350 + _tmp365;
  const Scalar _tmp380 = _tmp379 * l_w(0, 0);
  const Scalar _tmp381 = _tmp377 * l_w(2, 0);
  const Scalar _tmp382 = _tmp372 * l_w(1, 0);
  const Scalar _tmp383 = _tmp379 * _tmp58;
  const Scalar _tmp384 = -_tmp373 - _tmp378 + _tmp380 + _tmp381 + _tmp382 - _tmp383;
  const Scalar _tmp385 = _tmp344 + _tmp362 + _tmp363;
  const Scalar _tmp386 = _tmp367 + _tmp371;
  const Scalar _tmp387 = -_tmp374 - _tmp375 + _tmp376;
  const Scalar _tmp388 = -_tmp385 * _tmp58 + _tmp385 * l_w(0, 0) - _tmp386 * _tmp63 +
                         _tmp386 * l_w(2, 0) - _tmp387 * _tmp49 + _tmp387 * l_w(1, 0);
  const Scalar _tmp389 = -_tmp44 * _tmp84 - _tmp52 * _tmp81 - _tmp61 * _tmp83;
  const Scalar _tmp390 = _tmp44 * _tmp70;
  const Scalar _tmp391 = _tmp52 * _tmp74;
  const Scalar _tmp392 = _tmp61 * _tmp75;
  const Scalar _tmp393 = -_tmp390 - _tmp391 - _tmp392;
  const Scalar _tmp394 = -_tmp14 * _tmp61 - _tmp29 * _tmp52 - _tmp34 * _tmp44;
  const Scalar _tmp395 = _tmp39 * _tmp70;
  const Scalar _tmp396 = _tmp54 * _tmp74;
  const Scalar _tmp397 = _tmp62 * _tmp75;
  const Scalar _tmp398 = -_tmp395 - _tmp396 - _tmp397;
  const Scalar _tmp399 = -_tmp39 * _tmp84 - _tmp54 * _tmp81 - _tmp62 * _tmp83;
  const Scalar _tmp400 = -_tmp14 * _tmp62 - _tmp29 * _tmp54 - _tmp34 * _tmp39;
  const Scalar _tmp401 = _tmp48 * _tmp70;
  const Scalar _tmp402 = _tmp57 * _tmp74;
  const Scalar _tmp403 = _tmp59 * _tmp75;
  const Scalar _tmp404 = -_tmp401 - _tmp402 - _tmp403;
  const Scalar _tmp405 = -_tmp48 * _tmp84 - _tmp57 * _tmp81 - _tmp59 * _tmp83;
  const Scalar _tmp406 = -_tmp14 * _tmp59 - _tmp29 * _tmp57 - _tmp34 * _tmp48;
  const Scalar _tmp407 = _tmp180 * _tmp85;
  const Scalar _tmp408 = _tmp78 / std::pow(_calibration[0], Scalar(2));
  const Scalar _tmp409 = _tmp176 * _tmp76;
  const Scalar _tmp410 = _tmp162 * _tmp64;
  const Scalar _tmp411 = _tmp177 * _tmp64;
  const Scalar _tmp412 = _tmp65 / std::pow(_calibration[1], Scalar(2));
  const Scalar _tmp413 = _tmp176 * _tmp85;
  const Scalar _tmp414 = _tmp180 * _tmp76;

  // Output terms (5)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) * std::atan2(_tmp89, _tmp91);

  if (res_D_w_T_b != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 6>> _res_D_w_T_b{res_D_w_T_b};

    _res_D_w_T_b(0, 0) =
        _tmp181 * (_tmp178 * (_tmp162 * (-_tmp152 * _tmp80 + _tmp161) +
                              _tmp176 * (-_tmp161 * _tmp67 + _tmp175 * _tmp80) +
                              _tmp177 * (_tmp152 * _tmp67 + _tmp164 + _tmp166 - _tmp167 - _tmp169 +
                                         _tmp170 + _tmp171 - _tmp172 + _tmp173 + _tmp174)) -
                   _tmp180 * (_tmp152 + _tmp161 * _tmp80 + _tmp175 * _tmp67));
    _res_D_w_T_b(0, 1) =
        _tmp181 * (_tmp178 * (_tmp162 * (_tmp210 - _tmp231 * _tmp80) +
                              _tmp176 * (-_tmp210 * _tmp67 + _tmp227 * _tmp80) +
                              _tmp177 * (_tmp213 + _tmp218 - _tmp219 - _tmp221 + _tmp222 - _tmp223 +
                                         _tmp224 + _tmp225 + _tmp226 + _tmp231 * _tmp67)) -
                   _tmp180 * (_tmp210 * _tmp80 + _tmp227 * _tmp67 + _tmp231));
    _res_D_w_T_b(0, 2) =
        _tmp181 * (_tmp178 * (_tmp162 * (-_tmp254 * _tmp80 + _tmp262) +
                              _tmp176 * (-_tmp262 * _tmp67 + _tmp275 * _tmp80) +
                              _tmp177 * (_tmp254 * _tmp67 + _tmp264 + _tmp266 + _tmp267 - _tmp269 -
                                         _tmp270 + _tmp271 + _tmp272 - _tmp273 + _tmp274)) -
                   _tmp180 * (_tmp254 + _tmp262 * _tmp80 + _tmp275 * _tmp67));
    _res_D_w_T_b(0, 3) = _tmp181 * (_tmp178 * (_tmp162 * (-_tmp277 * _tmp80 + _tmp279) +
                                               _tmp176 * (-_tmp279 * _tmp67 + _tmp281 * _tmp80) +
                                               _tmp177 * (_tmp277 * _tmp67 + _tmp74)) -
                                    _tmp180 * (_tmp277 + _tmp279 * _tmp80 + _tmp281 * _tmp67));
    _res_D_w_T_b(0, 4) = _tmp181 * (_tmp178 * (_tmp162 * (-_tmp283 * _tmp80 + _tmp284) +
                                               _tmp176 * (-_tmp284 * _tmp67 + _tmp285 * _tmp80) +
                                               _tmp177 * (_tmp283 * _tmp67 + _tmp70)) -
                                    _tmp180 * (_tmp283 + _tmp284 * _tmp80 + _tmp285 * _tmp67));
    _res_D_w_T_b(0, 5) = _tmp181 * (_tmp178 * (_tmp162 * (_tmp286 - _tmp288 * _tmp80) +
                                               _tmp176 * (-_tmp286 * _tmp67 + _tmp287 * _tmp80) +
                                               _tmp177 * (_tmp288 * _tmp67 + _tmp75)) -
                                    _tmp180 * (_tmp286 * _tmp80 + _tmp287 * _tmp67 + _tmp288));
  }

  if (res_D_b_T_c != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 6>> _res_D_b_T_c{res_D_b_T_c};

    _res_D_b_T_c(0, 0) = _tmp181 * (_tmp178 * (_tmp162 * (_tmp323 - _tmp328 * _tmp80) +
                                               _tmp176 * (_tmp313 * _tmp80 - _tmp323 * _tmp67) +
                                               _tmp177 * (_tmp299 + _tmp303 - _tmp304 - _tmp310 +
                                                          _tmp311 - _tmp312 + _tmp328 * _tmp67)) -
                                    _tmp180 * (_tmp313 * _tmp67 + _tmp323 * _tmp80 + _tmp328));
    _res_D_b_T_c(0, 1) = _tmp181 * (_tmp178 * (_tmp162 * (-_tmp353 * _tmp80 + _tmp359) +
                                               _tmp176 * (-_tmp359 * _tmp67 + _tmp360 * _tmp80) +
                                               _tmp177 * (_tmp334 + _tmp338 - _tmp339 - _tmp346 +
                                                          _tmp353 * _tmp67 - _tmp354 + _tmp355)) -
                                    _tmp180 * (_tmp353 + _tmp359 * _tmp80 + _tmp360 * _tmp67));
    _res_D_b_T_c(0, 2) = _tmp181 * (_tmp178 * (_tmp162 * (_tmp370 - _tmp388 * _tmp80) +
                                               _tmp176 * (-_tmp370 * _tmp67 + _tmp384 * _tmp80) +
                                               _tmp177 * (_tmp373 + _tmp378 - _tmp380 - _tmp381 -
                                                          _tmp382 + _tmp383 + _tmp388 * _tmp67)) -
                                    _tmp180 * (_tmp370 * _tmp80 + _tmp384 * _tmp67 + _tmp388));
    _res_D_b_T_c(0, 3) =
        _tmp181 * (_tmp178 * (_tmp162 * (_tmp389 - _tmp394 * _tmp80) +
                              _tmp176 * (-_tmp389 * _tmp67 + _tmp393 * _tmp80) +
                              _tmp177 * (_tmp390 + _tmp391 + _tmp392 + _tmp394 * _tmp67)) -
                   _tmp180 * (_tmp389 * _tmp80 + _tmp393 * _tmp67 + _tmp394));
    _res_D_b_T_c(0, 4) =
        _tmp181 * (_tmp178 * (_tmp162 * (_tmp399 - _tmp400 * _tmp80) +
                              _tmp176 * (_tmp398 * _tmp80 - _tmp399 * _tmp67) +
                              _tmp177 * (_tmp395 + _tmp396 + _tmp397 + _tmp400 * _tmp67)) -
                   _tmp180 * (_tmp398 * _tmp67 + _tmp399 * _tmp80 + _tmp400));
    _res_D_b_T_c(0, 5) =
        _tmp181 * (_tmp178 * (_tmp162 * (_tmp405 - _tmp406 * _tmp80) +
                              _tmp176 * (_tmp404 * _tmp80 - _tmp405 * _tmp67) +
                              _tmp177 * (_tmp401 + _tmp402 + _tmp403 + _tmp406 * _tmp67)) -
                   _tmp180 * (_tmp404 * _tmp67 + _tmp405 * _tmp80 + _tmp406));
  }

  if (res_D_calibration != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 4>> _res_D_calibration{res_D_calibration};

    _res_D_calibration(0, 0) =
        _tmp181 * (_tmp178 * (-_tmp408 * _tmp409 + _tmp408 * _tmp410) + _tmp407 * _tmp408);
    _res_D_calibration(0, 1) =
        _tmp181 * (_tmp178 * (-_tmp411 * _tmp412 + _tmp412 * _tmp413) + _tmp412 * _tmp414);
    _res_D_calibration(0, 2) =
        _tmp181 * (_tmp178 * (-_tmp409 * _tmp79 + _tmp410 * _tmp79) + _tmp407 * _tmp79);
    _res_D_calibration(0, 3) =
        _tmp181 * (_tmp178 * (-_tmp411 * _tmp66 + _tmp413 * _tmp66) + _tmp414 * _tmp66);
  }

  if (res_D_l_w != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 3>> _res_D_l_w{res_D_l_w};

    _res_D_l_w(0, 0) = _tmp181 * (_tmp178 * (_tmp162 * (-_tmp29 * _tmp80 + _tmp81) +
                                             _tmp176 * (-_tmp67 * _tmp81 + _tmp74 * _tmp80) +
                                             _tmp177 * (_tmp281 + _tmp29 * _tmp67)) -
                                  _tmp180 * (_tmp29 + _tmp67 * _tmp74 + _tmp80 * _tmp81));
    _res_D_l_w(0, 1) = _tmp181 * (_tmp178 * (_tmp162 * (-_tmp34 * _tmp80 + _tmp84) +
                                             _tmp176 * (-_tmp67 * _tmp84 + _tmp70 * _tmp80) +
                                             _tmp177 * (_tmp285 + _tmp34 * _tmp67)) -
                                  _tmp180 * (_tmp34 + _tmp67 * _tmp70 + _tmp80 * _tmp84));
    _res_D_l_w(0, 2) = _tmp181 * (_tmp178 * (_tmp162 * (-_tmp14 * _tmp80 + _tmp83) +
                                             _tmp176 * (-_tmp67 * _tmp83 + _tmp75 * _tmp80) +
                                             _tmp177 * (_tmp14 * _tmp67 + _tmp287)) -
                                  _tmp180 * (_tmp14 + _tmp67 * _tmp75 + _tmp80 * _tmp83));
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion