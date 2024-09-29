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
 * Symbolic function: pose3_delta_factor
 *
 * Args:
 *     x: Pose3
 *     y: Pose3
 *     sqrt_info: Matrix66
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix61
 *     res_D_x: (6x6) jacobian of res (6) wrt arg x (6)
 *     res_D_y: (6x6) jacobian of res (6) wrt arg y (6)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> Pose3DeltaFactorWithJacobians01(
    const sym::Pose3<Scalar>& x, const sym::Pose3<Scalar>& y,
    const Eigen::Matrix<Scalar, 6, 6>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr, Scalar* const res_D_y = nullptr) {
  // Total ops: 827

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _y = y.Data();

  // Intermediate terms (156)
  const Scalar _tmp0 = _x[3] * _y[1];
  const Scalar _tmp1 = _x[2] * _y[0];
  const Scalar _tmp2 = _x[1] * _y[3];
  const Scalar _tmp3 = _x[0] * _y[2];
  const Scalar _tmp4 = _tmp0 - _tmp1 - _tmp2 + _tmp3;
  const Scalar _tmp5 = _x[2] * _y[2];
  const Scalar _tmp6 = _x[1] * _y[1];
  const Scalar _tmp7 = _x[0] * _y[0];
  const Scalar _tmp8 = -_tmp5 - _tmp6 - _tmp7;
  const Scalar _tmp9 = _x[3] * _y[3];
  const Scalar _tmp10 =
      2 * std::min<Scalar>(0, (((-_tmp8 + _tmp9) > 0) - ((-_tmp8 + _tmp9) < 0))) + 1;
  const Scalar _tmp11 = 2 * _tmp10;
  const Scalar _tmp12 = 1 - epsilon;
  const Scalar _tmp13 = std::min<Scalar>(_tmp12, std::fabs(_tmp8 - _tmp9));
  const Scalar _tmp14 = std::acos(_tmp13) / std::sqrt(Scalar(1 - std::pow(_tmp13, Scalar(2))));
  const Scalar _tmp15 = _tmp11 * _tmp14;
  const Scalar _tmp16 = _tmp15 * _tmp4;
  const Scalar _tmp17 = _x[3] * _y[0];
  const Scalar _tmp18 = _x[2] * _y[1];
  const Scalar _tmp19 = _x[1] * _y[2];
  const Scalar _tmp20 = _x[0] * _y[3];
  const Scalar _tmp21 = _tmp17 + _tmp18 - _tmp19 - _tmp20;
  const Scalar _tmp22 = _tmp14 * _tmp21;
  const Scalar _tmp23 = _tmp11 * _tmp22;
  const Scalar _tmp24 = _x[3] * _y[2];
  const Scalar _tmp25 = _x[2] * _y[3];
  const Scalar _tmp26 = _x[1] * _y[0];
  const Scalar _tmp27 = _x[0] * _y[1];
  const Scalar _tmp28 = _tmp24 - _tmp25 + _tmp26 - _tmp27;
  const Scalar _tmp29 = _tmp15 * _tmp28;
  const Scalar _tmp30 = -_x[4] + _y[4];
  const Scalar _tmp31 = -_x[6] + _y[6];
  const Scalar _tmp32 = -_x[5] + _y[5];
  const Scalar _tmp33 = _tmp11 * sqrt_info(3, 0);
  const Scalar _tmp34 = _tmp28 * sqrt_info(3, 2);
  const Scalar _tmp35 = (Scalar(1) / Scalar(2)) * _tmp9;
  const Scalar _tmp36 = (Scalar(1) / Scalar(2)) * _tmp5;
  const Scalar _tmp37 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp38 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp39 = -_tmp35 - _tmp36 - _tmp37 - _tmp38;
  const Scalar _tmp40 = _tmp5 + _tmp6 + _tmp7 + _tmp9;
  const Scalar _tmp41 = std::fabs(_tmp40);
  const Scalar _tmp42 = std::min<Scalar>(_tmp12, _tmp41);
  const Scalar _tmp43 = 1 - std::pow(_tmp42, Scalar(2));
  const Scalar _tmp44 = std::acos(_tmp42);
  const Scalar _tmp45 = _tmp44 / std::sqrt(_tmp43);
  const Scalar _tmp46 = _tmp11 * _tmp45;
  const Scalar _tmp47 = _tmp39 * _tmp46;
  const Scalar _tmp48 = _tmp28 * sqrt_info(0, 2);
  const Scalar _tmp49 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp50 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp51 = (Scalar(1) / Scalar(2)) * _tmp19;
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) * _tmp20;
  const Scalar _tmp53 = _tmp49 + _tmp50 - _tmp51 - _tmp52;
  const Scalar _tmp54 = _tmp10 * ((((_tmp12 - _tmp41) > 0) - ((_tmp12 - _tmp41) < 0)) + 1) *
                        (((_tmp40) > 0) - ((_tmp40) < 0));
  const Scalar _tmp55 = _tmp54 / _tmp43;
  const Scalar _tmp56 = _tmp53 * _tmp55;
  const Scalar _tmp57 = _tmp4 * sqrt_info(0, 1);
  const Scalar _tmp58 = _tmp42 * _tmp44 * _tmp54 / (_tmp43 * std::sqrt(_tmp43));
  const Scalar _tmp59 = _tmp21 * _tmp58;
  const Scalar _tmp60 = _tmp53 * _tmp59;
  const Scalar _tmp61 = _tmp21 * _tmp55;
  const Scalar _tmp62 = _tmp53 * _tmp61;
  const Scalar _tmp63 = _tmp53 * _tmp58;
  const Scalar _tmp64 = (Scalar(1) / Scalar(2)) * _tmp24;
  const Scalar _tmp65 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp66 = (Scalar(1) / Scalar(2)) * _tmp26;
  const Scalar _tmp67 = (Scalar(1) / Scalar(2)) * _tmp27;
  const Scalar _tmp68 = _tmp64 - _tmp65 + _tmp66 - _tmp67;
  const Scalar _tmp69 = _tmp46 * _tmp68;
  const Scalar _tmp70 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp71 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp72 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp73 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp74 = -_tmp70 + _tmp71 + _tmp72 - _tmp73;
  const Scalar _tmp75 = _tmp46 * _tmp74;
  const Scalar _tmp76 = _tmp69 * sqrt_info(0, 1) + _tmp75 * sqrt_info(0, 2);
  const Scalar _tmp77 = _tmp28 * _tmp55;
  const Scalar _tmp78 = _tmp77 * sqrt_info(1, 2);
  const Scalar _tmp79 = _tmp4 * _tmp56;
  const Scalar _tmp80 = _tmp4 * _tmp63;
  const Scalar _tmp81 = _tmp28 * _tmp63;
  const Scalar _tmp82 = _tmp69 * sqrt_info(1, 1) + _tmp75 * sqrt_info(1, 2);
  const Scalar _tmp83 = _tmp53 * _tmp77;
  const Scalar _tmp84 = _tmp4 * sqrt_info(2, 1);
  const Scalar _tmp85 = _tmp28 * sqrt_info(2, 2);
  const Scalar _tmp86 = _tmp69 * sqrt_info(2, 1) + _tmp75 * sqrt_info(2, 2);
  const Scalar _tmp87 = _tmp33 * _tmp45;
  const Scalar _tmp88 = _tmp4 * sqrt_info(3, 1);
  const Scalar _tmp89 = _tmp69 * sqrt_info(3, 1) + _tmp75 * sqrt_info(3, 2);
  const Scalar _tmp90 = _tmp69 * sqrt_info(4, 1) + _tmp75 * sqrt_info(4, 2);
  const Scalar _tmp91 = _tmp28 * sqrt_info(5, 2);
  const Scalar _tmp92 = _tmp4 * sqrt_info(5, 1);
  const Scalar _tmp93 = _tmp69 * sqrt_info(5, 1) + _tmp75 * sqrt_info(5, 2);
  const Scalar _tmp94 = _tmp70 - _tmp71 - _tmp72 + _tmp73;
  const Scalar _tmp95 = _tmp58 * _tmp94;
  const Scalar _tmp96 = _tmp55 * _tmp94;
  const Scalar _tmp97 = _tmp59 * _tmp94;
  const Scalar _tmp98 = _tmp61 * _tmp94;
  const Scalar _tmp99 = _tmp46 * _tmp53;
  const Scalar _tmp100 = -_tmp64 + _tmp65 - _tmp66 + _tmp67;
  const Scalar _tmp101 = _tmp100 * _tmp46;
  const Scalar _tmp102 = _tmp101 * sqrt_info(0, 0) + _tmp99 * sqrt_info(0, 2);
  const Scalar _tmp103 = _tmp28 * _tmp58;
  const Scalar _tmp104 = _tmp103 * sqrt_info(1, 2);
  const Scalar _tmp105 = _tmp4 * _tmp96;
  const Scalar _tmp106 = _tmp4 * _tmp95;
  const Scalar _tmp107 = _tmp101 * sqrt_info(1, 0) + _tmp99 * sqrt_info(1, 2);
  const Scalar _tmp108 = _tmp77 * _tmp94;
  const Scalar _tmp109 = _tmp101 * sqrt_info(2, 0) + _tmp99 * sqrt_info(2, 2);
  const Scalar _tmp110 = _tmp34 * _tmp58;
  const Scalar _tmp111 = _tmp100 * _tmp87 + _tmp99 * sqrt_info(3, 2);
  const Scalar _tmp112 = _tmp103 * sqrt_info(4, 2);
  const Scalar _tmp113 = _tmp101 * sqrt_info(4, 0) + _tmp99 * sqrt_info(4, 2);
  const Scalar _tmp114 = _tmp101 * sqrt_info(5, 0) + _tmp99 * sqrt_info(5, 2);
  const Scalar _tmp115 = _tmp59 * _tmp68;
  const Scalar _tmp116 = _tmp61 * _tmp68;
  const Scalar _tmp117 = _tmp58 * _tmp68;
  const Scalar _tmp118 = _tmp55 * _tmp68;
  const Scalar _tmp119 = _tmp46 * _tmp94;
  const Scalar _tmp120 = -_tmp49 - _tmp50 + _tmp51 + _tmp52;
  const Scalar _tmp121 = _tmp120 * _tmp46;
  const Scalar _tmp122 = _tmp119 * sqrt_info(0, 0) + _tmp121 * sqrt_info(0, 1);
  const Scalar _tmp123 = _tmp117 * _tmp4;
  const Scalar _tmp124 = _tmp118 * _tmp28;
  const Scalar _tmp125 = _tmp118 * _tmp4;
  const Scalar _tmp126 = _tmp119 * sqrt_info(1, 0) + _tmp121 * sqrt_info(1, 1);
  const Scalar _tmp127 = _tmp119 * sqrt_info(2, 0) + _tmp121 * sqrt_info(2, 1);
  const Scalar _tmp128 = _tmp121 * sqrt_info(3, 1) + _tmp87 * _tmp94;
  const Scalar _tmp129 = _tmp119 * sqrt_info(4, 0) + _tmp121 * sqrt_info(4, 1);
  const Scalar _tmp130 = _tmp119 * sqrt_info(5, 0) + _tmp121 * sqrt_info(5, 1);
  const Scalar _tmp131 = _tmp120 * _tmp55;
  const Scalar _tmp132 = _tmp120 * _tmp59;
  const Scalar _tmp133 = _tmp35 + _tmp36 + _tmp37 + _tmp38;
  const Scalar _tmp134 = _tmp133 * _tmp46;
  const Scalar _tmp135 = _tmp120 * _tmp58;
  const Scalar _tmp136 = _tmp120 * _tmp61;
  const Scalar _tmp137 = _tmp131 * _tmp4;
  const Scalar _tmp138 = _tmp135 * _tmp4;
  const Scalar _tmp139 = _tmp28 * sqrt_info(1, 2);
  const Scalar _tmp140 = _tmp131 * _tmp28;
  const Scalar _tmp141 = _tmp28 * sqrt_info(4, 2);
  const Scalar _tmp142 = _tmp59 * _tmp74;
  const Scalar _tmp143 = _tmp55 * _tmp74;
  const Scalar _tmp144 = _tmp61 * _tmp74;
  const Scalar _tmp145 = _tmp58 * _tmp74;
  const Scalar _tmp146 = _tmp143 * _tmp28;
  const Scalar _tmp147 = _tmp145 * _tmp4;
  const Scalar _tmp148 = _tmp143 * _tmp4;
  const Scalar _tmp149 = _tmp100 * _tmp58;
  const Scalar _tmp150 = _tmp100 * _tmp61;
  const Scalar _tmp151 = _tmp100 * _tmp59;
  const Scalar _tmp152 = _tmp100 * _tmp55;
  const Scalar _tmp153 = _tmp152 * _tmp4;
  const Scalar _tmp154 = _tmp149 * _tmp4;
  const Scalar _tmp155 = _tmp100 * _tmp77;

  // Output terms (3)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = _tmp16 * sqrt_info(0, 1) + _tmp23 * sqrt_info(0, 0) + _tmp29 * sqrt_info(0, 2) +
               _tmp30 * sqrt_info(0, 3) + _tmp31 * sqrt_info(0, 5) + _tmp32 * sqrt_info(0, 4);
  _res(1, 0) = _tmp16 * sqrt_info(1, 1) + _tmp23 * sqrt_info(1, 0) + _tmp29 * sqrt_info(1, 2) +
               _tmp30 * sqrt_info(1, 3) + _tmp31 * sqrt_info(1, 5) + _tmp32 * sqrt_info(1, 4);
  _res(2, 0) = _tmp16 * sqrt_info(2, 1) + _tmp23 * sqrt_info(2, 0) + _tmp29 * sqrt_info(2, 2) +
               _tmp30 * sqrt_info(2, 3) + _tmp31 * sqrt_info(2, 5) + _tmp32 * sqrt_info(2, 4);
  _res(3, 0) = _tmp15 * _tmp34 + _tmp16 * sqrt_info(3, 1) + _tmp22 * _tmp33 +
               _tmp30 * sqrt_info(3, 3) + _tmp31 * sqrt_info(3, 5) + _tmp32 * sqrt_info(3, 4);
  _res(4, 0) = _tmp16 * sqrt_info(4, 1) + _tmp23 * sqrt_info(4, 0) + _tmp29 * sqrt_info(4, 2) +
               _tmp30 * sqrt_info(4, 3) + _tmp31 * sqrt_info(4, 5) + _tmp32 * sqrt_info(4, 4);
  _res(5, 0) = _tmp16 * sqrt_info(5, 1) + _tmp23 * sqrt_info(5, 0) + _tmp29 * sqrt_info(5, 2) +
               _tmp30 * sqrt_info(5, 3) + _tmp31 * sqrt_info(5, 5) + _tmp32 * sqrt_info(5, 4);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 6, 6>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = _tmp47 * sqrt_info(0, 0) - _tmp48 * _tmp56 + _tmp48 * _tmp63 -
                     _tmp56 * _tmp57 + _tmp57 * _tmp63 + _tmp60 * sqrt_info(0, 0) -
                     _tmp62 * sqrt_info(0, 0) + _tmp76;
    _res_D_x(1, 0) = _tmp47 * sqrt_info(1, 0) - _tmp53 * _tmp78 + _tmp60 * sqrt_info(1, 0) -
                     _tmp62 * sqrt_info(1, 0) - _tmp79 * sqrt_info(1, 1) +
                     _tmp80 * sqrt_info(1, 1) + _tmp81 * sqrt_info(1, 2) + _tmp82;
    _res_D_x(2, 0) = _tmp47 * sqrt_info(2, 0) + _tmp60 * sqrt_info(2, 0) -
                     _tmp62 * sqrt_info(2, 0) + _tmp63 * _tmp84 + _tmp63 * _tmp85 -
                     _tmp79 * sqrt_info(2, 1) - _tmp83 * sqrt_info(2, 2) + _tmp86;
    _res_D_x(3, 0) = -_tmp34 * _tmp56 + _tmp34 * _tmp63 + _tmp39 * _tmp87 +
                     _tmp60 * sqrt_info(3, 0) - _tmp62 * sqrt_info(3, 0) + _tmp63 * _tmp88 -
                     _tmp79 * sqrt_info(3, 1) + _tmp89;
    _res_D_x(4, 0) = _tmp47 * sqrt_info(4, 0) + _tmp60 * sqrt_info(4, 0) -
                     _tmp62 * sqrt_info(4, 0) - _tmp79 * sqrt_info(4, 1) +
                     _tmp80 * sqrt_info(4, 1) + _tmp81 * sqrt_info(4, 2) -
                     _tmp83 * sqrt_info(4, 2) + _tmp90;
    _res_D_x(5, 0) = _tmp47 * sqrt_info(5, 0) - _tmp56 * _tmp91 - _tmp56 * _tmp92 +
                     _tmp60 * sqrt_info(5, 0) - _tmp62 * sqrt_info(5, 0) + _tmp63 * _tmp91 +
                     _tmp63 * _tmp92 + _tmp93;
    _res_D_x(0, 1) = _tmp102 + _tmp47 * sqrt_info(0, 1) + _tmp48 * _tmp95 - _tmp48 * _tmp96 +
                     _tmp57 * _tmp95 - _tmp57 * _tmp96 + _tmp97 * sqrt_info(0, 0) -
                     _tmp98 * sqrt_info(0, 0);
    _res_D_x(1, 1) = _tmp104 * _tmp94 - _tmp105 * sqrt_info(1, 1) + _tmp106 * sqrt_info(1, 1) +
                     _tmp107 + _tmp47 * sqrt_info(1, 1) - _tmp78 * _tmp94 +
                     _tmp97 * sqrt_info(1, 0) - _tmp98 * sqrt_info(1, 0);
    _res_D_x(2, 1) = -_tmp105 * sqrt_info(2, 1) - _tmp108 * sqrt_info(2, 2) + _tmp109 +
                     _tmp47 * sqrt_info(2, 1) + _tmp84 * _tmp95 + _tmp85 * _tmp95 +
                     _tmp97 * sqrt_info(2, 0) - _tmp98 * sqrt_info(2, 0);
    _res_D_x(3, 1) = -_tmp105 * sqrt_info(3, 1) + _tmp110 * _tmp94 + _tmp111 - _tmp34 * _tmp96 +
                     _tmp47 * sqrt_info(3, 1) + _tmp88 * _tmp95 + _tmp97 * sqrt_info(3, 0) -
                     _tmp98 * sqrt_info(3, 0);
    _res_D_x(4, 1) = -_tmp105 * sqrt_info(4, 1) + _tmp106 * sqrt_info(4, 1) -
                     _tmp108 * sqrt_info(4, 2) + _tmp112 * _tmp94 + _tmp113 +
                     _tmp47 * sqrt_info(4, 1) + _tmp97 * sqrt_info(4, 0) - _tmp98 * sqrt_info(4, 0);
    _res_D_x(5, 1) = _tmp114 + _tmp47 * sqrt_info(5, 1) + _tmp91 * _tmp95 - _tmp91 * _tmp96 +
                     _tmp92 * _tmp95 - _tmp92 * _tmp96 + _tmp97 * sqrt_info(5, 0) -
                     _tmp98 * sqrt_info(5, 0);
    _res_D_x(0, 2) = _tmp115 * sqrt_info(0, 0) - _tmp116 * sqrt_info(0, 0) + _tmp117 * _tmp48 +
                     _tmp117 * _tmp57 - _tmp118 * _tmp48 - _tmp118 * _tmp57 + _tmp122 +
                     _tmp47 * sqrt_info(0, 2);
    _res_D_x(1, 2) = _tmp104 * _tmp68 + _tmp115 * sqrt_info(1, 0) - _tmp116 * sqrt_info(1, 0) +
                     _tmp123 * sqrt_info(1, 1) - _tmp124 * sqrt_info(1, 2) -
                     _tmp125 * sqrt_info(1, 1) + _tmp126 + _tmp47 * sqrt_info(1, 2);
    _res_D_x(2, 2) = _tmp115 * sqrt_info(2, 0) - _tmp116 * sqrt_info(2, 0) + _tmp117 * _tmp84 +
                     _tmp117 * _tmp85 - _tmp124 * sqrt_info(2, 2) - _tmp125 * sqrt_info(2, 1) +
                     _tmp127 + _tmp47 * sqrt_info(2, 2);
    _res_D_x(3, 2) = _tmp110 * _tmp68 + _tmp115 * sqrt_info(3, 0) - _tmp116 * sqrt_info(3, 0) +
                     _tmp117 * _tmp88 - _tmp118 * _tmp34 - _tmp125 * sqrt_info(3, 1) + _tmp128 +
                     _tmp47 * sqrt_info(3, 2);
    _res_D_x(4, 2) = _tmp112 * _tmp68 + _tmp115 * sqrt_info(4, 0) - _tmp116 * sqrt_info(4, 0) +
                     _tmp123 * sqrt_info(4, 1) - _tmp124 * sqrt_info(4, 2) -
                     _tmp125 * sqrt_info(4, 1) + _tmp129 + _tmp47 * sqrt_info(4, 2);
    _res_D_x(5, 2) = _tmp115 * sqrt_info(5, 0) - _tmp116 * sqrt_info(5, 0) + _tmp117 * _tmp91 +
                     _tmp117 * _tmp92 - _tmp118 * _tmp91 - _tmp118 * _tmp92 + _tmp130 +
                     _tmp47 * sqrt_info(5, 2);
    _res_D_x(0, 3) = -sqrt_info(0, 3);
    _res_D_x(1, 3) = -sqrt_info(1, 3);
    _res_D_x(2, 3) = -sqrt_info(2, 3);
    _res_D_x(3, 3) = -sqrt_info(3, 3);
    _res_D_x(4, 3) = -sqrt_info(4, 3);
    _res_D_x(5, 3) = -sqrt_info(5, 3);
    _res_D_x(0, 4) = -sqrt_info(0, 4);
    _res_D_x(1, 4) = -sqrt_info(1, 4);
    _res_D_x(2, 4) = -sqrt_info(2, 4);
    _res_D_x(3, 4) = -sqrt_info(3, 4);
    _res_D_x(4, 4) = -sqrt_info(4, 4);
    _res_D_x(5, 4) = -sqrt_info(5, 4);
    _res_D_x(0, 5) = -sqrt_info(0, 5);
    _res_D_x(1, 5) = -sqrt_info(1, 5);
    _res_D_x(2, 5) = -sqrt_info(2, 5);
    _res_D_x(3, 5) = -sqrt_info(3, 5);
    _res_D_x(4, 5) = -sqrt_info(4, 5);
    _res_D_x(5, 5) = -sqrt_info(5, 5);
  }

  if (res_D_y != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 6, 6>> _res_D_y{res_D_y};

    _res_D_y(0, 0) = -_tmp131 * _tmp48 - _tmp131 * _tmp57 + _tmp132 * sqrt_info(0, 0) +
                     _tmp134 * sqrt_info(0, 0) + _tmp135 * _tmp48 + _tmp135 * _tmp57 -
                     _tmp136 * sqrt_info(0, 0) + _tmp76;
    _res_D_y(1, 0) = _tmp132 * sqrt_info(1, 0) + _tmp134 * sqrt_info(1, 0) + _tmp135 * _tmp139 -
                     _tmp136 * sqrt_info(1, 0) - _tmp137 * sqrt_info(1, 1) +
                     _tmp138 * sqrt_info(1, 1) - _tmp140 * sqrt_info(1, 2) + _tmp82;
    _res_D_y(2, 0) = _tmp132 * sqrt_info(2, 0) + _tmp134 * sqrt_info(2, 0) + _tmp135 * _tmp85 -
                     _tmp136 * sqrt_info(2, 0) - _tmp137 * sqrt_info(2, 1) +
                     _tmp138 * sqrt_info(2, 1) - _tmp140 * sqrt_info(2, 2) + _tmp86;
    _res_D_y(3, 0) = -_tmp131 * _tmp34 + _tmp132 * sqrt_info(3, 0) + _tmp133 * _tmp87 +
                     _tmp135 * _tmp34 + _tmp135 * _tmp88 - _tmp136 * sqrt_info(3, 0) -
                     _tmp137 * sqrt_info(3, 1) + _tmp89;
    _res_D_y(4, 0) = _tmp132 * sqrt_info(4, 0) + _tmp134 * sqrt_info(4, 0) + _tmp135 * _tmp141 -
                     _tmp136 * sqrt_info(4, 0) - _tmp137 * sqrt_info(4, 1) +
                     _tmp138 * sqrt_info(4, 1) - _tmp140 * sqrt_info(4, 2) + _tmp90;
    _res_D_y(5, 0) = -_tmp131 * _tmp91 - _tmp131 * _tmp92 + _tmp132 * sqrt_info(5, 0) +
                     _tmp134 * sqrt_info(5, 0) + _tmp135 * _tmp91 + _tmp135 * _tmp92 -
                     _tmp136 * sqrt_info(5, 0) + _tmp93;
    _res_D_y(0, 1) = _tmp102 + _tmp134 * sqrt_info(0, 1) + _tmp142 * sqrt_info(0, 0) -
                     _tmp143 * _tmp48 - _tmp143 * _tmp57 - _tmp144 * sqrt_info(0, 0) +
                     _tmp145 * _tmp48 + _tmp145 * _tmp57;
    _res_D_y(1, 1) = _tmp104 * _tmp74 + _tmp107 + _tmp134 * sqrt_info(1, 1) +
                     _tmp142 * sqrt_info(1, 0) - _tmp144 * sqrt_info(1, 0) -
                     _tmp146 * sqrt_info(1, 2) + _tmp147 * sqrt_info(1, 1) -
                     _tmp148 * sqrt_info(1, 1);
    _res_D_y(2, 1) = _tmp109 + _tmp134 * sqrt_info(2, 1) + _tmp142 * sqrt_info(2, 0) -
                     _tmp144 * sqrt_info(2, 0) + _tmp145 * _tmp85 - _tmp146 * sqrt_info(2, 2) +
                     _tmp147 * sqrt_info(2, 1) - _tmp148 * sqrt_info(2, 1);
    _res_D_y(3, 1) = _tmp110 * _tmp74 + _tmp111 + _tmp134 * sqrt_info(3, 1) +
                     _tmp142 * sqrt_info(3, 0) - _tmp143 * _tmp34 - _tmp144 * sqrt_info(3, 0) +
                     _tmp145 * _tmp88 - _tmp148 * sqrt_info(3, 1);
    _res_D_y(4, 1) = _tmp112 * _tmp74 + _tmp113 + _tmp134 * sqrt_info(4, 1) +
                     _tmp142 * sqrt_info(4, 0) - _tmp144 * sqrt_info(4, 0) -
                     _tmp146 * sqrt_info(4, 2) + _tmp147 * sqrt_info(4, 1) -
                     _tmp148 * sqrt_info(4, 1);
    _res_D_y(5, 1) = _tmp114 + _tmp134 * sqrt_info(5, 1) + _tmp142 * sqrt_info(5, 0) -
                     _tmp143 * _tmp91 - _tmp143 * _tmp92 - _tmp144 * sqrt_info(5, 0) +
                     _tmp145 * _tmp91 + _tmp145 * _tmp92;
    _res_D_y(0, 2) = _tmp122 + _tmp134 * sqrt_info(0, 2) + _tmp149 * _tmp48 + _tmp149 * _tmp57 -
                     _tmp150 * sqrt_info(0, 0) + _tmp151 * sqrt_info(0, 0) - _tmp152 * _tmp48 -
                     _tmp152 * _tmp57;
    _res_D_y(1, 2) = -_tmp100 * _tmp78 + _tmp126 + _tmp134 * sqrt_info(1, 2) + _tmp139 * _tmp149 -
                     _tmp150 * sqrt_info(1, 0) + _tmp151 * sqrt_info(1, 0) -
                     _tmp153 * sqrt_info(1, 1) + _tmp154 * sqrt_info(1, 1);
    _res_D_y(2, 2) = _tmp127 + _tmp134 * sqrt_info(2, 2) + _tmp149 * _tmp85 -
                     _tmp150 * sqrt_info(2, 0) + _tmp151 * sqrt_info(2, 0) -
                     _tmp153 * sqrt_info(2, 1) + _tmp154 * sqrt_info(2, 1) -
                     _tmp155 * sqrt_info(2, 2);
    _res_D_y(3, 2) = _tmp128 + _tmp134 * sqrt_info(3, 2) + _tmp149 * _tmp34 + _tmp149 * _tmp88 -
                     _tmp150 * sqrt_info(3, 0) + _tmp151 * sqrt_info(3, 0) - _tmp152 * _tmp34 -
                     _tmp153 * sqrt_info(3, 1);
    _res_D_y(4, 2) = _tmp129 + _tmp134 * sqrt_info(4, 2) + _tmp141 * _tmp149 -
                     _tmp150 * sqrt_info(4, 0) + _tmp151 * sqrt_info(4, 0) -
                     _tmp153 * sqrt_info(4, 1) + _tmp154 * sqrt_info(4, 1) -
                     _tmp155 * sqrt_info(4, 2);
    _res_D_y(5, 2) = _tmp130 + _tmp134 * sqrt_info(5, 2) + _tmp149 * _tmp91 + _tmp149 * _tmp92 -
                     _tmp150 * sqrt_info(5, 0) + _tmp151 * sqrt_info(5, 0) - _tmp152 * _tmp91 -
                     _tmp152 * _tmp92;
    _res_D_y(0, 3) = sqrt_info(0, 3);
    _res_D_y(1, 3) = sqrt_info(1, 3);
    _res_D_y(2, 3) = sqrt_info(2, 3);
    _res_D_y(3, 3) = sqrt_info(3, 3);
    _res_D_y(4, 3) = sqrt_info(4, 3);
    _res_D_y(5, 3) = sqrt_info(5, 3);
    _res_D_y(0, 4) = sqrt_info(0, 4);
    _res_D_y(1, 4) = sqrt_info(1, 4);
    _res_D_y(2, 4) = sqrt_info(2, 4);
    _res_D_y(3, 4) = sqrt_info(3, 4);
    _res_D_y(4, 4) = sqrt_info(4, 4);
    _res_D_y(5, 4) = sqrt_info(5, 4);
    _res_D_y(0, 5) = sqrt_info(0, 5);
    _res_D_y(1, 5) = sqrt_info(1, 5);
    _res_D_y(2, 5) = sqrt_info(2, 5);
    _res_D_y(3, 5) = sqrt_info(3, 5);
    _res_D_y(4, 5) = sqrt_info(4, 5);
    _res_D_y(5, 5) = sqrt_info(5, 5);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion