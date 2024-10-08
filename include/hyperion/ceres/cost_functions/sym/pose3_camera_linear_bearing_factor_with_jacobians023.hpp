// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/linear_camera_cal.h>
#include <sym/pose3.h>

namespace sym_ceres {

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
 *     res_D_w_T_b: (1x7) jacobian (result_dim x storage_dim) of res (1) wrt arg w_T_b (7)
 * (row-major) res_D_calibration: (1x4) jacobian (result_dim x storage_dim) of res (1) wrt arg
 * calibration (4) (row-major) res_D_l_w: (1x3) jacobian (result_dim x storage_dim) of res (1) wrt
 * arg l_w (3) (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Pose3CameraLinearBearingFactorWithJacobians023(
    const sym::Pose3<Scalar>& w_T_b, const sym::Pose3<Scalar>& b_T_c,
    const sym::LinearCameraCal<Scalar>& calibration, const Eigen::Matrix<Scalar, 3, 1>& l_w,
    const Eigen::Matrix<Scalar, 2, 1>& pixel, const Eigen::Matrix<Scalar, 1, 1>& sqrt_info,
    const Scalar epsilon, Scalar* const res_D_w_T_b = nullptr,
    Scalar* const res_D_calibration = nullptr, Scalar* const res_D_l_w = nullptr) {
  // Total ops: 837

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _w_T_b = w_T_b.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _b_T_c = b_T_c.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _calibration = calibration.Data();

  // Intermediate terms (284)
  const Scalar _tmp0 = _b_T_c[1] * _w_T_b[3];
  const Scalar _tmp1 = _b_T_c[2] * _w_T_b[0];
  const Scalar _tmp2 = _b_T_c[3] * _w_T_b[1];
  const Scalar _tmp3 = _b_T_c[0] * _w_T_b[2];
  const Scalar _tmp4 = _tmp0 - _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp5 = 2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = -_tmp5;
  const Scalar _tmp7 = _b_T_c[1] * _w_T_b[2];
  const Scalar _tmp8 = _b_T_c[2] * _w_T_b[1];
  const Scalar _tmp9 = _b_T_c[3] * _w_T_b[0];
  const Scalar _tmp10 = _b_T_c[0] * _w_T_b[3];
  const Scalar _tmp11 = _tmp10 - _tmp7 + _tmp8 + _tmp9;
  const Scalar _tmp12 = 2 * std::pow(_tmp11, Scalar(2));
  const Scalar _tmp13 = -_tmp12;
  const Scalar _tmp14 = _tmp13 + _tmp6 + 1;
  const Scalar _tmp15 = _b_T_c[1] * _w_T_b[0];
  const Scalar _tmp16 = _b_T_c[2] * _w_T_b[3];
  const Scalar _tmp17 = _b_T_c[3] * _w_T_b[2];
  const Scalar _tmp18 = _b_T_c[0] * _w_T_b[1];
  const Scalar _tmp19 = _tmp15 + _tmp16 + _tmp17 - _tmp18;
  const Scalar _tmp20 = 2 * _tmp19;
  const Scalar _tmp21 = _tmp11 * _tmp20;
  const Scalar _tmp22 = _b_T_c[1] * _w_T_b[1];
  const Scalar _tmp23 = _b_T_c[2] * _w_T_b[2];
  const Scalar _tmp24 = _b_T_c[0] * _w_T_b[0];
  const Scalar _tmp25 = _b_T_c[3] * _w_T_b[3];
  const Scalar _tmp26 = -_tmp22 - _tmp23 - _tmp24 + _tmp25;
  const Scalar _tmp27 = 2 * _tmp4;
  const Scalar _tmp28 = _tmp26 * _tmp27;
  const Scalar _tmp29 = _tmp21 + _tmp28;
  const Scalar _tmp30 = _tmp20 * _tmp4;
  const Scalar _tmp31 = 2 * _tmp11;
  const Scalar _tmp32 = _tmp26 * _tmp31;
  const Scalar _tmp33 = -_tmp32;
  const Scalar _tmp34 = _tmp30 + _tmp33;
  const Scalar _tmp35 = std::pow(_w_T_b[2], Scalar(2));
  const Scalar _tmp36 = -2 * _tmp35;
  const Scalar _tmp37 = std::pow(_w_T_b[0], Scalar(2));
  const Scalar _tmp38 = 1 - 2 * _tmp37;
  const Scalar _tmp39 = 2 * _w_T_b[0];
  const Scalar _tmp40 = _tmp39 * _w_T_b[1];
  const Scalar _tmp41 = 2 * _w_T_b[3];
  const Scalar _tmp42 = _tmp41 * _w_T_b[2];
  const Scalar _tmp43 = _tmp40 + _tmp42;
  const Scalar _tmp44 = _tmp39 * _w_T_b[3];
  const Scalar _tmp45 = -_tmp44;
  const Scalar _tmp46 = 2 * _w_T_b[2];
  const Scalar _tmp47 = _tmp46 * _w_T_b[1];
  const Scalar _tmp48 = _tmp45 + _tmp47;
  const Scalar _tmp49 =
      _b_T_c[4] * _tmp43 + _b_T_c[5] * (_tmp36 + _tmp38) + _b_T_c[6] * _tmp48 + _w_T_b[5];
  const Scalar _tmp50 = std::pow(_w_T_b[1], Scalar(2));
  const Scalar _tmp51 = -2 * _tmp50;
  const Scalar _tmp52 = -_tmp42;
  const Scalar _tmp53 = _tmp40 + _tmp52;
  const Scalar _tmp54 = _tmp39 * _w_T_b[2];
  const Scalar _tmp55 = _tmp41 * _w_T_b[1];
  const Scalar _tmp56 = _tmp54 + _tmp55;
  const Scalar _tmp57 =
      _b_T_c[4] * (_tmp36 + _tmp51 + 1) + _b_T_c[5] * _tmp53 + _b_T_c[6] * _tmp56 + _w_T_b[4];
  const Scalar _tmp58 = -_tmp55;
  const Scalar _tmp59 = _tmp54 + _tmp58;
  const Scalar _tmp60 = _tmp44 + _tmp47;
  const Scalar _tmp61 =
      _b_T_c[4] * _tmp59 + _b_T_c[5] * _tmp60 + _b_T_c[6] * (_tmp38 + _tmp51) + _w_T_b[6];
  const Scalar _tmp62 = -_tmp14 * _tmp61 + _tmp14 * l_w(2, 0) - _tmp29 * _tmp57 +
                        _tmp29 * l_w(0, 0) - _tmp34 * _tmp49 + _tmp34 * l_w(1, 0);
  const Scalar _tmp63 = -_calibration[3] + pixel(1, 0);
  const Scalar _tmp64 = Scalar(1.0) / (_calibration[1]);
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = 2 * std::pow(_tmp19, Scalar(2));
  const Scalar _tmp67 = 1 - _tmp66;
  const Scalar _tmp68 = _tmp13 + _tmp67;
  const Scalar _tmp69 = _tmp31 * _tmp4;
  const Scalar _tmp70 = _tmp20 * _tmp26;
  const Scalar _tmp71 = -_tmp70;
  const Scalar _tmp72 = _tmp69 + _tmp71;
  const Scalar _tmp73 = _tmp30 + _tmp32;
  const Scalar _tmp74 = -_tmp49 * _tmp68 - _tmp57 * _tmp72 - _tmp61 * _tmp73 + _tmp68 * l_w(1, 0) +
                        _tmp72 * l_w(0, 0) + _tmp73 * l_w(2, 0);
  const Scalar _tmp75 = _tmp62 * _tmp65 - _tmp74;
  const Scalar _tmp76 = -_calibration[2] + pixel(0, 0);
  const Scalar _tmp77 = Scalar(1.0) / (_calibration[0]);
  const Scalar _tmp78 = _tmp76 * _tmp77;
  const Scalar _tmp79 = _tmp6 + _tmp67;
  const Scalar _tmp80 = -_tmp28;
  const Scalar _tmp81 = _tmp21 + _tmp80;
  const Scalar _tmp82 = _tmp69 + _tmp70;
  const Scalar _tmp83 = -_tmp49 * _tmp82 - _tmp57 * _tmp79 - _tmp61 * _tmp81 + _tmp79 * l_w(0, 0) +
                        _tmp81 * l_w(2, 0) + _tmp82 * l_w(1, 0);
  const Scalar _tmp84 = -_tmp62 * _tmp78 + _tmp83;
  const Scalar _tmp85 = -_tmp65 * _tmp83 + _tmp74 * _tmp78;
  const Scalar _tmp86 = std::pow(_tmp75, Scalar(2)) + std::pow(_tmp84, Scalar(2)) +
                        std::pow(_tmp85, Scalar(2)) + epsilon;
  const Scalar _tmp87 = std::sqrt(_tmp86);
  const Scalar _tmp88 = _tmp62 + _tmp65 * _tmp74 + _tmp78 * _tmp83;
  const Scalar _tmp89 = _tmp88 + epsilon * ((((_tmp88) > 0) - ((_tmp88) < 0)) + Scalar(0.5));
  const Scalar _tmp90 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp91 = -_tmp90;
  const Scalar _tmp92 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp93 = -Scalar(1) / Scalar(2) * _tmp1;
  const Scalar _tmp94 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp95 = _tmp93 - _tmp94;
  const Scalar _tmp96 = _tmp91 + _tmp92 + _tmp95;
  const Scalar _tmp97 = _tmp27 * _tmp96;
  const Scalar _tmp98 = (Scalar(1) / Scalar(2)) * _tmp22;
  const Scalar _tmp99 = (Scalar(1) / Scalar(2)) * _tmp23;
  const Scalar _tmp100 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp101 = (Scalar(1) / Scalar(2)) * _tmp24;
  const Scalar _tmp102 = _tmp100 + _tmp101 - _tmp98 + _tmp99;
  const Scalar _tmp103 = 2 * _tmp26;
  const Scalar _tmp104 = _tmp102 * _tmp103;
  const Scalar _tmp105 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp106 = -_tmp105;
  const Scalar _tmp107 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp108 = -Scalar(1) / Scalar(2) * _tmp7;
  const Scalar _tmp109 = (Scalar(1) / Scalar(2)) * _tmp10;
  const Scalar _tmp110 = _tmp108 - _tmp109;
  const Scalar _tmp111 = _tmp106 + _tmp107 + _tmp110;
  const Scalar _tmp112 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp113 = -_tmp112;
  const Scalar _tmp114 = (Scalar(1) / Scalar(2)) * _tmp16;
  const Scalar _tmp115 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp116 = -_tmp115;
  const Scalar _tmp117 = -Scalar(1) / Scalar(2) * _tmp18;
  const Scalar _tmp118 = _tmp113 + _tmp114 + _tmp116 + _tmp117;
  const Scalar _tmp119 = _tmp111 * _tmp31 + _tmp118 * _tmp20;
  const Scalar _tmp120 = -_tmp104 + _tmp119 - _tmp97;
  const Scalar _tmp121 = 4 * _tmp4;
  const Scalar _tmp122 = -_tmp102 * _tmp121;
  const Scalar _tmp123 = 4 * _tmp19;
  const Scalar _tmp124 = -_tmp111 * _tmp123;
  const Scalar _tmp125 = _tmp122 + _tmp124;
  const Scalar _tmp126 = _tmp20 * _tmp96;
  const Scalar _tmp127 = _tmp103 * _tmp111;
  const Scalar _tmp128 = _tmp102 * _tmp31 + _tmp118 * _tmp27;
  const Scalar _tmp129 = _tmp126 + _tmp127 + _tmp128;
  const Scalar _tmp130 = -_tmp47;
  const Scalar _tmp131 = _b_T_c[4] * (_tmp130 + _tmp44) + _b_T_c[6] * _tmp43;
  const Scalar _tmp132 = -_tmp35;
  const Scalar _tmp133 = _tmp132 + _tmp37;
  const Scalar _tmp134 = std::pow(_w_T_b[3], Scalar(2));
  const Scalar _tmp135 = -_tmp134;
  const Scalar _tmp136 = _tmp135 + _tmp50;
  const Scalar _tmp137 = _b_T_c[4] * (_tmp133 + _tmp136) + _b_T_c[6] * _tmp59;
  const Scalar _tmp138 = -_tmp50;
  const Scalar _tmp139 = _tmp134 + _tmp138;
  const Scalar _tmp140 = -_tmp54;
  const Scalar _tmp141 = _b_T_c[4] * (_tmp140 + _tmp58) + _b_T_c[6] * (_tmp133 + _tmp139);
  const Scalar _tmp142 = -_tmp120 * _tmp61 + _tmp120 * l_w(2, 0) - _tmp125 * _tmp57 +
                         _tmp125 * l_w(0, 0) - _tmp129 * _tmp49 + _tmp129 * l_w(1, 0) -
                         _tmp131 * _tmp82 - _tmp137 * _tmp81 - _tmp141 * _tmp79;
  const Scalar _tmp143 = 4 * _tmp11;
  const Scalar _tmp144 = -_tmp118 * _tmp143;
  const Scalar _tmp145 = _tmp124 + _tmp144;
  const Scalar _tmp146 = _tmp145 * _tmp49;
  const Scalar _tmp147 = _tmp31 * _tmp96;
  const Scalar _tmp148 = _tmp103 * _tmp118;
  const Scalar _tmp149 = _tmp102 * _tmp20 + _tmp111 * _tmp27;
  const Scalar _tmp150 = _tmp147 + _tmp148 + _tmp149;
  const Scalar _tmp151 = _tmp150 * _tmp61;
  const Scalar _tmp152 = _tmp150 * l_w(2, 0);
  const Scalar _tmp153 = -_tmp126 - _tmp127 + _tmp128;
  const Scalar _tmp154 = _tmp153 * l_w(0, 0);
  const Scalar _tmp155 = _tmp141 * _tmp72;
  const Scalar _tmp156 = _tmp145 * l_w(1, 0);
  const Scalar _tmp157 = _tmp137 * _tmp73;
  const Scalar _tmp158 = _tmp131 * _tmp68;
  const Scalar _tmp159 = _tmp153 * _tmp57;
  const Scalar _tmp160 =
      -_tmp146 - _tmp151 + _tmp152 + _tmp154 - _tmp155 + _tmp156 - _tmp157 - _tmp158 - _tmp159;
  const Scalar _tmp161 = 2 * _tmp85;
  const Scalar _tmp162 = _tmp122 + _tmp144;
  const Scalar _tmp163 = -_tmp147 - _tmp148 + _tmp149;
  const Scalar _tmp164 = _tmp104 + _tmp119 + _tmp97;
  const Scalar _tmp165 = -_tmp131 * _tmp34 - _tmp137 * _tmp14 - _tmp141 * _tmp29 -
                         _tmp162 * _tmp61 + _tmp162 * l_w(2, 0) - _tmp163 * _tmp49 +
                         _tmp163 * l_w(1, 0) - _tmp164 * _tmp57 + _tmp164 * l_w(0, 0);
  const Scalar _tmp166 = 2 * _tmp75;
  const Scalar _tmp167 = 2 * _tmp84;
  const Scalar _tmp168 = (Scalar(1) / Scalar(2)) / (_tmp87 * _tmp89);
  const Scalar _tmp169 = std::pow(_tmp89, Scalar(2));
  const Scalar _tmp170 = _tmp87 / _tmp169;
  const Scalar _tmp171 = _tmp169 * sqrt_info(0, 0) / (_tmp169 + _tmp86);
  const Scalar _tmp172 =
      _tmp171 * (_tmp168 * (_tmp161 * (-_tmp142 * _tmp65 + _tmp160 * _tmp78) +
                            _tmp166 * (_tmp146 + _tmp151 - _tmp152 - _tmp154 + _tmp155 - _tmp156 +
                                       _tmp157 + _tmp158 + _tmp159 + _tmp165 * _tmp65) +
                            _tmp167 * (_tmp142 - _tmp165 * _tmp78)) -
                 _tmp170 * (_tmp142 * _tmp78 + _tmp160 * _tmp65 + _tmp165));
  const Scalar _tmp173 = _b_T_c[4] * _tmp60 + _b_T_c[5] * (_tmp140 + _tmp55);
  const Scalar _tmp174 = -_tmp114 + _tmp117;
  const Scalar _tmp175 = _tmp112 + _tmp116 + _tmp174;
  const Scalar _tmp176 = _tmp175 * _tmp27;
  const Scalar _tmp177 = -_tmp107;
  const Scalar _tmp178 = _tmp106 + _tmp108 + _tmp109 + _tmp177;
  const Scalar _tmp179 = _tmp103 * _tmp178;
  const Scalar _tmp180 = _tmp100 + _tmp98;
  const Scalar _tmp181 = _tmp101 + _tmp180 - _tmp99;
  const Scalar _tmp182 = -_tmp92;
  const Scalar _tmp183 = _tmp182 + _tmp91 + _tmp93 + _tmp94;
  const Scalar _tmp184 = _tmp181 * _tmp31 + _tmp183 * _tmp20;
  const Scalar _tmp185 = _tmp176 + _tmp179 + _tmp184;
  const Scalar _tmp186 = -_tmp143 * _tmp183;
  const Scalar _tmp187 = -_tmp121 * _tmp178;
  const Scalar _tmp188 = _tmp186 + _tmp187;
  const Scalar _tmp189 = _tmp175 * _tmp31;
  const Scalar _tmp190 = _tmp103 * _tmp183;
  const Scalar _tmp191 = _tmp178 * _tmp20 + _tmp181 * _tmp27;
  const Scalar _tmp192 = -_tmp189 - _tmp190 + _tmp191;
  const Scalar _tmp193 = -_tmp37;
  const Scalar _tmp194 = _tmp193 + _tmp35;
  const Scalar _tmp195 = _b_T_c[4] * _tmp53 + _b_T_c[5] * (_tmp136 + _tmp194);
  const Scalar _tmp196 = -_tmp40;
  const Scalar _tmp197 =
      _b_T_c[4] * (_tmp132 + _tmp134 + _tmp193 + _tmp50) + _b_T_c[5] * (_tmp196 + _tmp52);
  const Scalar _tmp198 = -_tmp14 * _tmp173 - _tmp185 * _tmp57 + _tmp185 * l_w(0, 0) -
                         _tmp188 * _tmp61 + _tmp188 * l_w(2, 0) - _tmp192 * _tmp49 +
                         _tmp192 * l_w(1, 0) - _tmp195 * _tmp29 - _tmp197 * _tmp34;
  const Scalar _tmp199 = -_tmp176 - _tmp179 + _tmp184;
  const Scalar _tmp200 = _tmp175 * _tmp20;
  const Scalar _tmp201 = _tmp103 * _tmp181;
  const Scalar _tmp202 = _tmp178 * _tmp31 + _tmp183 * _tmp27;
  const Scalar _tmp203 = _tmp200 + _tmp201 + _tmp202;
  const Scalar _tmp204 = -_tmp123 * _tmp181;
  const Scalar _tmp205 = _tmp187 + _tmp204;
  const Scalar _tmp206 = -_tmp173 * _tmp81 - _tmp195 * _tmp79 - _tmp197 * _tmp82 -
                         _tmp199 * _tmp61 + _tmp199 * l_w(2, 0) - _tmp203 * _tmp49 +
                         _tmp203 * l_w(1, 0) - _tmp205 * _tmp57 + _tmp205 * l_w(0, 0);
  const Scalar _tmp207 = _tmp186 + _tmp204;
  const Scalar _tmp208 = _tmp207 * _tmp49;
  const Scalar _tmp209 = _tmp189 + _tmp190 + _tmp191;
  const Scalar _tmp210 = _tmp209 * _tmp61;
  const Scalar _tmp211 = _tmp197 * _tmp68;
  const Scalar _tmp212 = -_tmp200 - _tmp201 + _tmp202;
  const Scalar _tmp213 = _tmp212 * l_w(0, 0);
  const Scalar _tmp214 = _tmp209 * l_w(2, 0);
  const Scalar _tmp215 = _tmp173 * _tmp73;
  const Scalar _tmp216 = _tmp195 * _tmp72;
  const Scalar _tmp217 = _tmp207 * l_w(1, 0);
  const Scalar _tmp218 = _tmp212 * _tmp57;
  const Scalar _tmp219 =
      -_tmp208 - _tmp210 - _tmp211 + _tmp213 + _tmp214 - _tmp215 - _tmp216 + _tmp217 - _tmp218;
  const Scalar _tmp220 =
      _tmp171 * (_tmp168 * (_tmp161 * (-_tmp206 * _tmp65 + _tmp219 * _tmp78) +
                            _tmp166 * (_tmp198 * _tmp65 + _tmp208 + _tmp210 + _tmp211 - _tmp213 -
                                       _tmp214 + _tmp215 + _tmp216 - _tmp217 + _tmp218) +
                            _tmp167 * (-_tmp198 * _tmp78 + _tmp206)) -
                 _tmp170 * (_tmp198 + _tmp206 * _tmp78 + _tmp219 * _tmp65));
  const Scalar _tmp221 = 2 * _w_T_b[1];
  const Scalar _tmp222 = -_tmp101 + _tmp180 + _tmp99;
  const Scalar _tmp223 = -_tmp143 * _tmp222;
  const Scalar _tmp224 = _tmp113 + _tmp115 + _tmp174;
  const Scalar _tmp225 = -_tmp121 * _tmp224;
  const Scalar _tmp226 = _tmp223 + _tmp225;
  const Scalar _tmp227 = _tmp105 + _tmp110 + _tmp177;
  const Scalar _tmp228 = _tmp227 * _tmp27;
  const Scalar _tmp229 = _tmp103 * _tmp224;
  const Scalar _tmp230 = _tmp182 + _tmp90 + _tmp95;
  const Scalar _tmp231 = _tmp20 * _tmp222 + _tmp230 * _tmp31;
  const Scalar _tmp232 = _tmp228 + _tmp229 + _tmp231;
  const Scalar _tmp233 = _tmp227 * _tmp31;
  const Scalar _tmp234 = _tmp103 * _tmp222;
  const Scalar _tmp235 = _tmp20 * _tmp224 + _tmp230 * _tmp27;
  const Scalar _tmp236 = -_tmp233 - _tmp234 + _tmp235;
  const Scalar _tmp237 = _b_T_c[5] * _tmp56 + _b_T_c[6] * (_tmp196 + _tmp42);
  const Scalar _tmp238 = _b_T_c[5] * (_tmp139 + _tmp194) + _b_T_c[6] * (_tmp130 + _tmp45);
  const Scalar _tmp239 = _b_T_c[5] * _tmp48 + _b_T_c[6] * (_tmp135 + _tmp138 + _tmp35 + _tmp37);
  const Scalar _tmp240 = -_tmp14 * _tmp238 - _tmp226 * _tmp61 + _tmp226 * l_w(2, 0) -
                         _tmp232 * _tmp57 + _tmp232 * l_w(0, 0) - _tmp236 * _tmp49 +
                         _tmp236 * l_w(1, 0) - _tmp237 * _tmp29 - _tmp239 * _tmp34;
  const Scalar _tmp241 = -_tmp228 - _tmp229 + _tmp231;
  const Scalar _tmp242 = -_tmp123 * _tmp230;
  const Scalar _tmp243 = _tmp225 + _tmp242;
  const Scalar _tmp244 = _tmp20 * _tmp227;
  const Scalar _tmp245 = _tmp103 * _tmp230;
  const Scalar _tmp246 = _tmp222 * _tmp27 + _tmp224 * _tmp31;
  const Scalar _tmp247 = _tmp244 + _tmp245 + _tmp246;
  const Scalar _tmp248 = -_tmp237 * _tmp79 - _tmp238 * _tmp81 - _tmp239 * _tmp82 -
                         _tmp241 * _tmp61 + _tmp241 * l_w(2, 0) - _tmp243 * _tmp57 +
                         _tmp243 * l_w(0, 0) - _tmp247 * _tmp49 + _tmp247 * l_w(1, 0);
  const Scalar _tmp249 = _tmp233 + _tmp234 + _tmp235;
  const Scalar _tmp250 = _tmp249 * _tmp61;
  const Scalar _tmp251 = _tmp223 + _tmp242;
  const Scalar _tmp252 = _tmp251 * _tmp49;
  const Scalar _tmp253 = _tmp249 * l_w(2, 0);
  const Scalar _tmp254 = -_tmp244 - _tmp245 + _tmp246;
  const Scalar _tmp255 = _tmp254 * l_w(0, 0);
  const Scalar _tmp256 = _tmp237 * _tmp72;
  const Scalar _tmp257 = _tmp254 * _tmp57;
  const Scalar _tmp258 = _tmp251 * l_w(1, 0);
  const Scalar _tmp259 = _tmp239 * _tmp68;
  const Scalar _tmp260 = _tmp238 * _tmp73;
  const Scalar _tmp261 =
      -_tmp250 - _tmp252 + _tmp253 + _tmp255 - _tmp256 - _tmp257 + _tmp258 - _tmp259 - _tmp260;
  const Scalar _tmp262 =
      _tmp171 * (_tmp168 * (_tmp161 * (-_tmp248 * _tmp65 + _tmp261 * _tmp78) +
                            _tmp166 * (_tmp240 * _tmp65 + _tmp250 + _tmp252 - _tmp253 - _tmp255 +
                                       _tmp256 + _tmp257 - _tmp258 + _tmp259 + _tmp260) +
                            _tmp167 * (-_tmp240 * _tmp78 + _tmp248)) -
                 _tmp170 * (_tmp240 + _tmp248 * _tmp78 + _tmp261 * _tmp65));
  const Scalar _tmp263 = -_tmp21;
  const Scalar _tmp264 = _tmp263 + _tmp80;
  const Scalar _tmp265 = _tmp66 - 1;
  const Scalar _tmp266 = _tmp265 + _tmp5;
  const Scalar _tmp267 = -_tmp69;
  const Scalar _tmp268 = _tmp267 + _tmp70;
  const Scalar _tmp269 = -_tmp30;
  const Scalar _tmp270 = _tmp269 + _tmp32;
  const Scalar _tmp271 = _tmp267 + _tmp71;
  const Scalar _tmp272 = _tmp12 + _tmp265;
  const Scalar _tmp273 = _tmp263 + _tmp28;
  const Scalar _tmp274 = _tmp269 + _tmp33;
  const Scalar _tmp275 = _tmp12 + _tmp5 - 1;
  const Scalar _tmp276 = _tmp170 * _tmp83;
  const Scalar _tmp277 = _tmp76 / std::pow(_calibration[0], Scalar(2));
  const Scalar _tmp278 = _tmp161 * _tmp74;
  const Scalar _tmp279 = _tmp167 * _tmp62;
  const Scalar _tmp280 = _tmp166 * _tmp62;
  const Scalar _tmp281 = _tmp63 / std::pow(_calibration[1], Scalar(2));
  const Scalar _tmp282 = _tmp161 * _tmp83;
  const Scalar _tmp283 = _tmp170 * _tmp74;

  // Output terms (4)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) * std::atan2(_tmp87, _tmp89);

  if (res_D_w_T_b != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 7>> _res_D_w_T_b{res_D_w_T_b};

    _res_D_w_T_b(0, 0) = -_tmp172 * _tmp46 + _tmp220 * _tmp221 + _tmp262 * _tmp41;
    _res_D_w_T_b(0, 1) = _tmp172 * _tmp41 - _tmp220 * _tmp39 + _tmp262 * _tmp46;
    _res_D_w_T_b(0, 2) = _tmp172 * _tmp39 + _tmp220 * _tmp41 - _tmp221 * _tmp262;
    _res_D_w_T_b(0, 3) = -_tmp172 * _tmp221 - _tmp220 * _tmp46 - _tmp262 * _tmp39;
    _res_D_w_T_b(0, 4) = _tmp171 * (_tmp168 * (_tmp161 * (-_tmp266 * _tmp65 + _tmp268 * _tmp78) +
                                               _tmp166 * (_tmp264 * _tmp65 + _tmp72) +
                                               _tmp167 * (-_tmp264 * _tmp78 + _tmp266)) -
                                    _tmp170 * (_tmp264 + _tmp266 * _tmp78 + _tmp268 * _tmp65));
    _res_D_w_T_b(0, 5) = _tmp171 * (_tmp168 * (_tmp161 * (-_tmp271 * _tmp65 + _tmp272 * _tmp78) +
                                               _tmp166 * (_tmp270 * _tmp65 + _tmp68) +
                                               _tmp167 * (-_tmp270 * _tmp78 + _tmp271)) -
                                    _tmp170 * (_tmp270 + _tmp271 * _tmp78 + _tmp272 * _tmp65));
    _res_D_w_T_b(0, 6) = _tmp171 * (_tmp168 * (_tmp161 * (-_tmp273 * _tmp65 + _tmp274 * _tmp78) +
                                               _tmp166 * (_tmp275 * _tmp65 + _tmp73) +
                                               _tmp167 * (_tmp273 - _tmp275 * _tmp78)) -
                                    _tmp170 * (_tmp273 * _tmp78 + _tmp274 * _tmp65 + _tmp275));
  }

  if (res_D_calibration != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 4>> _res_D_calibration{res_D_calibration};

    _res_D_calibration(0, 0) =
        _tmp171 * (_tmp168 * (-_tmp277 * _tmp278 + _tmp277 * _tmp279) + _tmp276 * _tmp277);
    _res_D_calibration(0, 1) =
        _tmp171 * (_tmp168 * (-_tmp280 * _tmp281 + _tmp281 * _tmp282) + _tmp281 * _tmp283);
    _res_D_calibration(0, 2) =
        _tmp171 * (_tmp168 * (-_tmp278 * _tmp77 + _tmp279 * _tmp77) + _tmp276 * _tmp77);
    _res_D_calibration(0, 3) =
        _tmp171 * (_tmp168 * (-_tmp280 * _tmp64 + _tmp282 * _tmp64) + _tmp283 * _tmp64);
  }

  if (res_D_l_w != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 3>> _res_D_l_w{res_D_l_w};

    _res_D_l_w(0, 0) = _tmp171 * (_tmp168 * (_tmp161 * (-_tmp65 * _tmp79 + _tmp72 * _tmp78) +
                                             _tmp166 * (_tmp268 + _tmp29 * _tmp65) +
                                             _tmp167 * (-_tmp29 * _tmp78 + _tmp79)) -
                                  _tmp170 * (_tmp29 + _tmp65 * _tmp72 + _tmp78 * _tmp79));
    _res_D_l_w(0, 1) = _tmp171 * (_tmp168 * (_tmp161 * (-_tmp65 * _tmp82 + _tmp68 * _tmp78) +
                                             _tmp166 * (_tmp272 + _tmp34 * _tmp65) +
                                             _tmp167 * (-_tmp34 * _tmp78 + _tmp82)) -
                                  _tmp170 * (_tmp34 + _tmp65 * _tmp68 + _tmp78 * _tmp82));
    _res_D_l_w(0, 2) = _tmp171 * (_tmp168 * (_tmp161 * (-_tmp65 * _tmp81 + _tmp73 * _tmp78) +
                                             _tmp166 * (_tmp14 * _tmp65 + _tmp274) +
                                             _tmp167 * (-_tmp14 * _tmp78 + _tmp81)) -
                                  _tmp170 * (_tmp14 + _tmp65 * _tmp73 + _tmp78 * _tmp81));
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
