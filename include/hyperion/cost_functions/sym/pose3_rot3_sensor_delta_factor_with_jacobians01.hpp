// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose3.h>
#include <sym/rot3.h>

namespace sym_hyperion {

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
 *     res_D_x: (3x6) jacobian of res (3) wrt arg x (6)
 *     res_D_x_T_y: (3x3) jacobian of res (3) wrt arg x_T_y (3)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Pose3Rot3SensorDeltaFactorWithJacobians01(
    const sym::Pose3<Scalar>& x, const sym::Rot3<Scalar>& x_T_y, const sym::Pose3<Scalar>& y,
    const Eigen::Matrix<Scalar, 3, 3>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr, Scalar* const res_D_x_T_y = nullptr) {
  // Total ops: 624

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x_T_y = x_T_y.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (178)
  const Scalar _tmp0 = _x[2] * _y[2];
  const Scalar _tmp1 = _x[1] * _y[1];
  const Scalar _tmp2 = _x[0] * _y[0];
  const Scalar _tmp3 = _x[3] * _y[3];
  const Scalar _tmp4 = _tmp0 + _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp5 = _tmp4 * _x_T_y[3];
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
  const Scalar _tmp18 = _x[3] * _y[2];
  const Scalar _tmp19 = _x[2] * _y[3];
  const Scalar _tmp20 = _x[1] * _y[0];
  const Scalar _tmp21 = _x[0] * _y[1];
  const Scalar _tmp22 = _tmp18 - _tmp19 + _tmp20 - _tmp21;
  const Scalar _tmp23 = _tmp22 * _x_T_y[2];
  const Scalar _tmp24 = -_tmp11 - _tmp17 - _tmp23;
  const Scalar _tmp25 = -_tmp24 + _tmp5;
  const Scalar _tmp26 = 1 - epsilon;
  const Scalar _tmp27 = std::min<Scalar>(_tmp26, std::fabs(_tmp25));
  const Scalar _tmp28 = std::acos(_tmp27);
  const Scalar _tmp29 =
      std::pow(Scalar(1 - std::pow(_tmp27, Scalar(2))), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp30 = 2 * std::min<Scalar>(0, (((_tmp25) > 0) - ((_tmp25) < 0))) + 1;
  const Scalar _tmp31 = 2 * _tmp30;
  const Scalar _tmp32 = _tmp28 * _tmp29 * _tmp31;
  const Scalar _tmp33 = _tmp4 * _x_T_y[0];
  const Scalar _tmp34 = _tmp10 * _x_T_y[3];
  const Scalar _tmp35 = _tmp16 * _x_T_y[2];
  const Scalar _tmp36 = _tmp22 * _x_T_y[1];
  const Scalar _tmp37 = -_tmp33 + _tmp34 + _tmp35 - _tmp36;
  const Scalar _tmp38 = _tmp37 * sqrt_info(0, 0);
  const Scalar _tmp39 = _tmp4 * _x_T_y[1];
  const Scalar _tmp40 = _tmp10 * _x_T_y[2];
  const Scalar _tmp41 = _tmp16 * _x_T_y[3];
  const Scalar _tmp42 = _tmp22 * _x_T_y[0];
  const Scalar _tmp43 = -_tmp39 - _tmp40 + _tmp41 + _tmp42;
  const Scalar _tmp44 = _tmp32 * _tmp43;
  const Scalar _tmp45 = _tmp4 * _x_T_y[2];
  const Scalar _tmp46 = _tmp10 * _x_T_y[1];
  const Scalar _tmp47 = _tmp16 * _x_T_y[0];
  const Scalar _tmp48 = _tmp22 * _x_T_y[3];
  const Scalar _tmp49 = -_tmp45 + _tmp46 - _tmp47 + _tmp48;
  const Scalar _tmp50 = _tmp32 * _tmp49;
  const Scalar _tmp51 = _tmp31 * sqrt_info(1, 0);
  const Scalar _tmp52 = _tmp37 * sqrt_info(2, 0);
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp54 = (Scalar(1) / Scalar(2)) * _tmp19;
  const Scalar _tmp55 = (Scalar(1) / Scalar(2)) * _tmp20;
  const Scalar _tmp56 = (Scalar(1) / Scalar(2)) * _tmp21;
  const Scalar _tmp57 = _tmp53 - _tmp54 + _tmp55 - _tmp56;
  const Scalar _tmp58 = -_tmp57 * _x_T_y[1];
  const Scalar _tmp59 = -Scalar(1) / Scalar(2) * _tmp0 - Scalar(1) / Scalar(2) * _tmp1 -
                        Scalar(1) / Scalar(2) * _tmp2 - Scalar(1) / Scalar(2) * _tmp3;
  const Scalar _tmp60 = _tmp59 * _x_T_y[0];
  const Scalar _tmp61 = -_tmp60;
  const Scalar _tmp62 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp63 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp64 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp65 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp66 = _tmp62 + _tmp63 - _tmp64 - _tmp65;
  const Scalar _tmp67 = _tmp66 * _x_T_y[3];
  const Scalar _tmp68 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp69 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp70 = (Scalar(1) / Scalar(2)) * _tmp14;
  const Scalar _tmp71 = (Scalar(1) / Scalar(2)) * _tmp15;
  const Scalar _tmp72 = -_tmp68 + _tmp69 + _tmp70 - _tmp71;
  const Scalar _tmp73 = _tmp58 + _tmp61 - _tmp67 - _tmp72 * _x_T_y[2];
  const Scalar _tmp74 = std::fabs(_tmp11 + _tmp17 + _tmp23 + _tmp5);
  const Scalar _tmp75 = std::min<Scalar>(_tmp26, _tmp74);
  const Scalar _tmp76 = 1 - std::pow(_tmp75, Scalar(2));
  const Scalar _tmp77 = _tmp30 * ((((_tmp26 - _tmp74) > 0) - ((_tmp26 - _tmp74) < 0)) + 1) *
                        (((_tmp24 - _tmp5) > 0) - ((_tmp24 - _tmp5) < 0));
  const Scalar _tmp78 = _tmp77 / _tmp76;
  const Scalar _tmp79 = _tmp73 * _tmp78;
  const Scalar _tmp80 = _tmp49 * sqrt_info(0, 2);
  const Scalar _tmp81 = std::acos(_tmp75);
  const Scalar _tmp82 = _tmp75 * _tmp77 * _tmp81 / (_tmp76 * std::sqrt(_tmp76));
  const Scalar _tmp83 = _tmp43 * _tmp82;
  const Scalar _tmp84 = _tmp73 * _tmp83;
  const Scalar _tmp85 = _tmp43 * _tmp79;
  const Scalar _tmp86 = _tmp57 * _x_T_y[3];
  const Scalar _tmp87 = _tmp59 * _x_T_y[2];
  const Scalar _tmp88 = -_tmp87;
  const Scalar _tmp89 = -_tmp66 * _x_T_y[1];
  const Scalar _tmp90 = _tmp81 / std::sqrt(_tmp76);
  const Scalar _tmp91 = _tmp31 * _tmp90;
  const Scalar _tmp92 = _tmp91 * (_tmp72 * _x_T_y[0] + _tmp86 + _tmp88 + _tmp89);
  const Scalar _tmp93 = _tmp73 * _tmp82;
  const Scalar _tmp94 = -_tmp57 * _x_T_y[0];
  const Scalar _tmp95 = _tmp59 * _x_T_y[1];
  const Scalar _tmp96 = -_tmp66 * _x_T_y[2];
  const Scalar _tmp97 = _tmp91 * (_tmp72 * _x_T_y[3] + _tmp94 + _tmp95 + _tmp96);
  const Scalar _tmp98 = _tmp57 * _x_T_y[2];
  const Scalar _tmp99 = _tmp59 * _x_T_y[3];
  const Scalar _tmp100 = _tmp66 * _x_T_y[0];
  const Scalar _tmp101 = -_tmp100 - _tmp72 * _x_T_y[1] + _tmp98 + _tmp99;
  const Scalar _tmp102 = _tmp101 * _tmp91;
  const Scalar _tmp103 = _tmp49 * sqrt_info(1, 2);
  const Scalar _tmp104 = _tmp37 * sqrt_info(1, 0);
  const Scalar _tmp105 = _tmp104 * _tmp82;
  const Scalar _tmp106 = _tmp51 * _tmp90;
  const Scalar _tmp107 = _tmp49 * sqrt_info(2, 2);
  const Scalar _tmp108 = _tmp83 * sqrt_info(2, 1);
  const Scalar _tmp109 = -_tmp53 + _tmp54 - _tmp55 + _tmp56;
  const Scalar _tmp110 = _tmp68 - _tmp69 - _tmp70 + _tmp71;
  const Scalar _tmp111 = -_tmp110 * _x_T_y[0];
  const Scalar _tmp112 = _tmp109 * _x_T_y[3] + _tmp111 + _tmp87 + _tmp89;
  const Scalar _tmp113 = _tmp112 * _tmp91;
  const Scalar _tmp114 = -_tmp95;
  const Scalar _tmp115 = _tmp110 * _x_T_y[3];
  const Scalar _tmp116 = -_tmp109 * _x_T_y[0] + _tmp114 - _tmp115 + _tmp96;
  const Scalar _tmp117 = _tmp116 * _tmp78;
  const Scalar _tmp118 = _tmp110 * _x_T_y[1];
  const Scalar _tmp119 = _tmp100 - _tmp109 * _x_T_y[2] - _tmp118 + _tmp99;
  const Scalar _tmp120 = _tmp91 * sqrt_info(0, 1);
  const Scalar _tmp121 = -_tmp110 * _x_T_y[2];
  const Scalar _tmp122 = _tmp91 * (_tmp109 * _x_T_y[1] + _tmp121 + _tmp61 + _tmp67);
  const Scalar _tmp123 = _tmp116 * _tmp83;
  const Scalar _tmp124 = _tmp116 * _tmp82;
  const Scalar _tmp125 = _tmp43 * _tmp78;
  const Scalar _tmp126 = _tmp125 * sqrt_info(0, 1);
  const Scalar _tmp127 = _tmp119 * _tmp91;
  const Scalar _tmp128 = _tmp116 * _tmp125;
  const Scalar _tmp129 = -_tmp62 - _tmp63 + _tmp64 + _tmp65;
  const Scalar _tmp130 = _tmp111 - _tmp129 * _x_T_y[1] - _tmp86 + _tmp88;
  const Scalar _tmp131 = _tmp130 * _tmp82;
  const Scalar _tmp132 = _tmp114 + _tmp115 + _tmp129 * _x_T_y[2] + _tmp94;
  const Scalar _tmp133 = _tmp132 * _tmp91;
  const Scalar _tmp134 = _tmp130 * _tmp83;
  const Scalar _tmp135 = _tmp121 + _tmp129 * _x_T_y[3] + _tmp58 + _tmp60;
  const Scalar _tmp136 = _tmp130 * _tmp78;
  const Scalar _tmp137 = _tmp91 * (_tmp118 - _tmp129 * _x_T_y[0] - _tmp98 + _tmp99);
  const Scalar _tmp138 = _tmp135 * _tmp91;
  const Scalar _tmp139 = _tmp104 * _tmp78;
  const Scalar _tmp140 = _tmp125 * sqrt_info(1, 1);
  const Scalar _tmp141 = _tmp125 * sqrt_info(2, 1);
  const Scalar _tmp142 = (Scalar(1) / Scalar(2)) * _tmp33;
  const Scalar _tmp143 = (Scalar(1) / Scalar(2)) * _tmp34;
  const Scalar _tmp144 = (Scalar(1) / Scalar(2)) * _tmp35;
  const Scalar _tmp145 = (Scalar(1) / Scalar(2)) * _tmp36;
  const Scalar _tmp146 = _tmp142 - _tmp143 - _tmp144 + _tmp145;
  const Scalar _tmp147 = _tmp146 * _tmp82;
  const Scalar _tmp148 = _tmp146 * _tmp83;
  const Scalar _tmp149 = _tmp146 * _tmp78;
  const Scalar _tmp150 = (Scalar(1) / Scalar(2)) * _tmp39;
  const Scalar _tmp151 = (Scalar(1) / Scalar(2)) * _tmp40;
  const Scalar _tmp152 = (Scalar(1) / Scalar(2)) * _tmp41;
  const Scalar _tmp153 = (Scalar(1) / Scalar(2)) * _tmp42;
  const Scalar _tmp154 = _tmp150 + _tmp151 - _tmp152 - _tmp153;
  const Scalar _tmp155 = _tmp154 * _tmp91;
  const Scalar _tmp156 = -Scalar(1) / Scalar(2) * _tmp11 - Scalar(1) / Scalar(2) * _tmp17 -
                         Scalar(1) / Scalar(2) * _tmp23 - Scalar(1) / Scalar(2) * _tmp5;
  const Scalar _tmp157 = _tmp156 * _tmp91;
  const Scalar _tmp158 = (Scalar(1) / Scalar(2)) * _tmp45;
  const Scalar _tmp159 = (Scalar(1) / Scalar(2)) * _tmp46;
  const Scalar _tmp160 = (Scalar(1) / Scalar(2)) * _tmp47;
  const Scalar _tmp161 = (Scalar(1) / Scalar(2)) * _tmp48;
  const Scalar _tmp162 = -_tmp158 + _tmp159 - _tmp160 + _tmp161;
  const Scalar _tmp163 = _tmp162 * _tmp91;
  const Scalar _tmp164 = _tmp91 * (-_tmp142 + _tmp143 + _tmp144 - _tmp145);
  const Scalar _tmp165 = _tmp154 * _tmp82;
  const Scalar _tmp166 = _tmp154 * _tmp78;
  const Scalar _tmp167 = _tmp154 * _tmp83;
  const Scalar _tmp168 = _tmp158 - _tmp159 + _tmp160 - _tmp161;
  const Scalar _tmp169 = _tmp168 * _tmp91;
  const Scalar _tmp170 = _tmp125 * _tmp154;
  const Scalar _tmp171 = _tmp168 * _tmp83;
  const Scalar _tmp172 = _tmp168 * _tmp82;
  const Scalar _tmp173 = _tmp168 * _tmp78;
  const Scalar _tmp174 = _tmp90 * (-_tmp150 - _tmp151 + _tmp152 + _tmp153);
  const Scalar _tmp175 = _tmp174 * _tmp31;
  const Scalar _tmp176 = _tmp146 * _tmp91;
  const Scalar _tmp177 = _tmp125 * _tmp168;

  // Output terms (3)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp32 * _tmp38 + _tmp44 * sqrt_info(0, 1) + _tmp50 * sqrt_info(0, 2);
  _res(1, 0) =
      _tmp28 * _tmp29 * _tmp37 * _tmp51 + _tmp44 * sqrt_info(1, 1) + _tmp50 * sqrt_info(1, 2);
  _res(2, 0) = _tmp32 * _tmp52 + _tmp44 * sqrt_info(2, 1) + _tmp50 * sqrt_info(2, 2);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 6>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = _tmp102 * sqrt_info(0, 0) - _tmp38 * _tmp79 + _tmp38 * _tmp93 -
                     _tmp79 * _tmp80 + _tmp80 * _tmp93 + _tmp84 * sqrt_info(0, 1) -
                     _tmp85 * sqrt_info(0, 1) + _tmp92 * sqrt_info(0, 1) + _tmp97 * sqrt_info(0, 2);
    _res_D_x(1, 0) = _tmp101 * _tmp106 - _tmp103 * _tmp79 + _tmp103 * _tmp93 - _tmp104 * _tmp79 +
                     _tmp105 * _tmp73 + _tmp84 * sqrt_info(1, 1) - _tmp85 * sqrt_info(1, 1) +
                     _tmp92 * sqrt_info(1, 1) + _tmp97 * sqrt_info(1, 2);
    _res_D_x(2, 0) = _tmp102 * sqrt_info(2, 0) - _tmp107 * _tmp79 + _tmp107 * _tmp93 +
                     _tmp108 * _tmp73 - _tmp52 * _tmp79 + _tmp52 * _tmp93 -
                     _tmp85 * sqrt_info(2, 1) + _tmp92 * sqrt_info(2, 1) + _tmp97 * sqrt_info(2, 2);
    _res_D_x(0, 1) = _tmp113 * sqrt_info(0, 0) - _tmp116 * _tmp126 - _tmp117 * _tmp38 -
                     _tmp117 * _tmp80 + _tmp119 * _tmp120 + _tmp122 * sqrt_info(0, 2) +
                     _tmp123 * sqrt_info(0, 1) + _tmp124 * _tmp38 + _tmp124 * _tmp80;
    _res_D_x(1, 1) = -_tmp103 * _tmp117 + _tmp103 * _tmp124 - _tmp104 * _tmp117 +
                     _tmp105 * _tmp116 + _tmp106 * _tmp112 + _tmp122 * sqrt_info(1, 2) +
                     _tmp123 * sqrt_info(1, 1) + _tmp127 * sqrt_info(1, 1) -
                     _tmp128 * sqrt_info(1, 1);
    _res_D_x(2, 1) = -_tmp107 * _tmp117 + _tmp107 * _tmp124 + _tmp108 * _tmp116 +
                     _tmp113 * sqrt_info(2, 0) - _tmp117 * _tmp52 + _tmp122 * sqrt_info(2, 2) +
                     _tmp124 * _tmp52 + _tmp127 * sqrt_info(2, 1) - _tmp128 * sqrt_info(2, 1);
    _res_D_x(0, 2) = _tmp120 * _tmp135 - _tmp126 * _tmp130 + _tmp131 * _tmp38 + _tmp131 * _tmp80 +
                     _tmp133 * sqrt_info(0, 0) + _tmp134 * sqrt_info(0, 1) - _tmp136 * _tmp38 -
                     _tmp136 * _tmp80 + _tmp137 * sqrt_info(0, 2);
    _res_D_x(1, 2) = _tmp103 * _tmp131 - _tmp103 * _tmp136 + _tmp105 * _tmp130 + _tmp106 * _tmp132 -
                     _tmp130 * _tmp139 - _tmp130 * _tmp140 + _tmp134 * sqrt_info(1, 1) +
                     _tmp137 * sqrt_info(1, 2) + _tmp138 * sqrt_info(1, 1);
    _res_D_x(2, 2) = _tmp107 * _tmp131 - _tmp107 * _tmp136 - _tmp130 * _tmp141 + _tmp131 * _tmp52 +
                     _tmp133 * sqrt_info(2, 0) + _tmp134 * sqrt_info(2, 1) - _tmp136 * _tmp52 +
                     _tmp137 * sqrt_info(2, 2) + _tmp138 * sqrt_info(2, 1);
    _res_D_x(0, 3) = 0;
    _res_D_x(1, 3) = 0;
    _res_D_x(2, 3) = 0;
    _res_D_x(0, 4) = 0;
    _res_D_x(1, 4) = 0;
    _res_D_x(2, 4) = 0;
    _res_D_x(0, 5) = 0;
    _res_D_x(1, 5) = 0;
    _res_D_x(2, 5) = 0;
  }

  if (res_D_x_T_y != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x_T_y{res_D_x_T_y};

    _res_D_x_T_y(0, 0) = _tmp120 * _tmp162 - _tmp126 * _tmp146 + _tmp147 * _tmp38 +
                         _tmp147 * _tmp80 + _tmp148 * sqrt_info(0, 1) - _tmp149 * _tmp38 -
                         _tmp149 * _tmp80 + _tmp155 * sqrt_info(0, 2) + _tmp157 * sqrt_info(0, 0);
    _res_D_x_T_y(1, 0) = _tmp103 * _tmp147 - _tmp103 * _tmp149 + _tmp105 * _tmp146 +
                         _tmp106 * _tmp156 - _tmp139 * _tmp146 - _tmp140 * _tmp146 +
                         _tmp148 * sqrt_info(1, 1) + _tmp155 * sqrt_info(1, 2) +
                         _tmp163 * sqrt_info(1, 1);
    _res_D_x_T_y(2, 0) = _tmp107 * _tmp147 - _tmp107 * _tmp149 - _tmp141 * _tmp146 +
                         _tmp147 * _tmp52 + _tmp148 * sqrt_info(2, 1) - _tmp149 * _tmp52 +
                         _tmp155 * sqrt_info(2, 2) + _tmp157 * sqrt_info(2, 0) +
                         _tmp163 * sqrt_info(2, 1);
    _res_D_x_T_y(0, 1) = _tmp120 * _tmp156 - _tmp126 * _tmp154 + _tmp164 * sqrt_info(0, 2) +
                         _tmp165 * _tmp38 + _tmp165 * _tmp80 - _tmp166 * _tmp38 - _tmp166 * _tmp80 +
                         _tmp167 * sqrt_info(0, 1) + _tmp169 * sqrt_info(0, 0);
    _res_D_x_T_y(1, 1) = _tmp103 * _tmp165 - _tmp103 * _tmp166 - _tmp104 * _tmp166 +
                         _tmp105 * _tmp154 + _tmp106 * _tmp168 + _tmp157 * sqrt_info(1, 1) +
                         _tmp164 * sqrt_info(1, 2) + _tmp167 * sqrt_info(1, 1) -
                         _tmp170 * sqrt_info(1, 1);
    _res_D_x_T_y(2, 1) = _tmp107 * _tmp165 - _tmp107 * _tmp166 + _tmp108 * _tmp154 +
                         _tmp157 * sqrt_info(2, 1) + _tmp164 * sqrt_info(2, 2) + _tmp165 * _tmp52 -
                         _tmp166 * _tmp52 + _tmp169 * sqrt_info(2, 0) - _tmp170 * sqrt_info(2, 1);
    _res_D_x_T_y(0, 2) = _tmp120 * _tmp146 - _tmp126 * _tmp168 + _tmp157 * sqrt_info(0, 2) +
                         _tmp171 * sqrt_info(0, 1) + _tmp172 * _tmp38 + _tmp172 * _tmp80 -
                         _tmp173 * _tmp38 - _tmp173 * _tmp80 + _tmp175 * sqrt_info(0, 0);
    _res_D_x_T_y(1, 2) = _tmp103 * _tmp172 - _tmp103 * _tmp173 + _tmp105 * _tmp168 -
                         _tmp139 * _tmp168 + _tmp157 * sqrt_info(1, 2) + _tmp171 * sqrt_info(1, 1) +
                         _tmp174 * _tmp51 + _tmp176 * sqrt_info(1, 1) - _tmp177 * sqrt_info(1, 1);
    _res_D_x_T_y(2, 2) = _tmp107 * _tmp172 - _tmp107 * _tmp173 + _tmp108 * _tmp168 +
                         _tmp157 * sqrt_info(2, 2) + _tmp172 * _tmp52 - _tmp173 * _tmp52 +
                         _tmp175 * sqrt_info(2, 0) + _tmp176 * sqrt_info(2, 1) -
                         _tmp177 * sqrt_info(2, 1);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
