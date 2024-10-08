// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline4_rot3_velocity
 *
 * Args:
 *     dt: Scalar
 *     lambdas: Matrix42
 *     x0: Rot3
 *     x1: Rot3
 *     x2: Rot3
 *     x3: Rot3
 *     x4: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline4Rot3VelocityGlobal(
    const Scalar dt, const Eigen::Matrix<Scalar, 4, 2>& lambdas, const sym::Rot3<Scalar>& x0,
    const sym::Rot3<Scalar>& x1, const sym::Rot3<Scalar>& x2, const sym::Rot3<Scalar>& x3,
    const sym::Rot3<Scalar>& x4, const Scalar epsilon) {
  // Total ops: 453

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x4 = x4.Data();

  // Intermediate terms (135)
  const Scalar _tmp0 = Scalar(1.0) / (dt);
  const Scalar _tmp1 = 2 * _x0[3];
  const Scalar _tmp2 = _tmp1 * _x0[2];
  const Scalar _tmp3 = 2 * _x0[0];
  const Scalar _tmp4 = _tmp3 * _x0[1];
  const Scalar _tmp5 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp6 = std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp8 = std::pow(_tmp7, Scalar(2));
  const Scalar _tmp9 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp10 = _x0[3] * _x1[3];
  const Scalar _tmp11 = 1 - epsilon;
  const Scalar _tmp12 = std::min<Scalar>(_tmp11, std::fabs(_tmp10 - _tmp9));
  const Scalar _tmp13 = 1 - std::pow(_tmp12, Scalar(2));
  const Scalar _tmp14 = std::acos(_tmp12);
  const Scalar _tmp15 =
      2 * std::min<Scalar>(0, (((_tmp10 - _tmp9) > 0) - ((_tmp10 - _tmp9) < 0))) + 1;
  const Scalar _tmp16 = std::pow(_tmp14, Scalar(2)) * std::pow(_tmp15, Scalar(2)) *
                        std::pow(lambdas(0, 0), Scalar(2)) / _tmp13;
  const Scalar _tmp17 = 4 * _tmp16;
  const Scalar _tmp18 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp19 = std::pow(_tmp18, Scalar(2));
  const Scalar _tmp20 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp21 = _tmp17 * _tmp19 + _tmp17 * _tmp6 + _tmp17 * _tmp8 + _tmp20;
  const Scalar _tmp22 = std::sqrt(_tmp21);
  const Scalar _tmp23 = (Scalar(1) / Scalar(2)) * _tmp22;
  const Scalar _tmp24 = std::sin(_tmp23);
  const Scalar _tmp25 = 8 * _tmp16 * std::pow(_tmp24, Scalar(2)) / _tmp21;
  const Scalar _tmp26 = -_tmp25 * _tmp6;
  const Scalar _tmp27 = -_tmp19 * _tmp25 + 1;
  const Scalar _tmp28 = -_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2];
  const Scalar _tmp29 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp30 = _x2[3] * _x3[3];
  const Scalar _tmp31 = std::min<Scalar>(_tmp11, std::fabs(_tmp29 - _tmp30));
  const Scalar _tmp32 = std::acos(_tmp31);
  const Scalar _tmp33 =
      2 * std::min<Scalar>(0, (((-_tmp29 + _tmp30) > 0) - ((-_tmp29 + _tmp30) < 0))) + 1;
  const Scalar _tmp34 = 1 - std::pow(_tmp31, Scalar(2));
  const Scalar _tmp35 = _tmp32 * _tmp33 / std::sqrt(_tmp34);
  const Scalar _tmp36 = _tmp28 * _tmp35;
  const Scalar _tmp37 = 2 * lambdas(2, 1);
  const Scalar _tmp38 = -_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0];
  const Scalar _tmp39 = std::pow(_tmp28, Scalar(2));
  const Scalar _tmp40 = std::pow(_tmp32, Scalar(2)) * std::pow(_tmp33, Scalar(2)) *
                        std::pow(lambdas(2, 0), Scalar(2)) / _tmp34;
  const Scalar _tmp41 = 4 * _tmp40;
  const Scalar _tmp42 = _x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1];
  const Scalar _tmp43 = std::pow(_tmp42, Scalar(2));
  const Scalar _tmp44 = std::pow(_tmp38, Scalar(2));
  const Scalar _tmp45 = _tmp20 + _tmp39 * _tmp41 + _tmp41 * _tmp43 + _tmp41 * _tmp44;
  const Scalar _tmp46 = std::sqrt(_tmp45);
  const Scalar _tmp47 = (Scalar(1) / Scalar(2)) * _tmp46;
  const Scalar _tmp48 = std::sin(_tmp47);
  const Scalar _tmp49 = 4 * _tmp48 * lambdas(2, 0) * std::cos(_tmp47) / _tmp46;
  const Scalar _tmp50 = _tmp35 * _tmp49;
  const Scalar _tmp51 = _tmp38 * _tmp50;
  const Scalar _tmp52 = 8 * _tmp40 * std::pow(_tmp48, Scalar(2)) / _tmp45;
  const Scalar _tmp53 = _tmp42 * _tmp52;
  const Scalar _tmp54 = _tmp28 * _tmp53;
  const Scalar _tmp55 = -_x3[0] * _x4[0] - _x3[1] * _x4[1] - _x3[2] * _x4[2];
  const Scalar _tmp56 = _x3[3] * _x4[3];
  const Scalar _tmp57 = std::min<Scalar>(_tmp11, std::fabs(_tmp55 - _tmp56));
  const Scalar _tmp58 =
      2 * lambdas(3, 1) *
      (2 * std::min<Scalar>(0, (((-_tmp55 + _tmp56) > 0) - ((-_tmp55 + _tmp56) < 0))) + 1) *
      std::acos(_tmp57) / std::sqrt(Scalar(1 - std::pow(_tmp57, Scalar(2))));
  const Scalar _tmp59 =
      _tmp58 * (_x3[0] * _x4[2] - _x3[1] * _x4[3] - _x3[2] * _x4[0] + _x3[3] * _x4[1]);
  const Scalar _tmp60 = -_tmp44 * _tmp52;
  const Scalar _tmp61 = -_tmp43 * _tmp52 + 1;
  const Scalar _tmp62 =
      _tmp58 * (-_x3[0] * _x4[1] + _x3[1] * _x4[0] - _x3[2] * _x4[3] + _x3[3] * _x4[2]);
  const Scalar _tmp63 = _tmp28 * _tmp38 * _tmp52;
  const Scalar _tmp64 = _tmp42 * _tmp50;
  const Scalar _tmp65 =
      _tmp58 * (-_x3[0] * _x4[3] - _x3[1] * _x4[2] + _x3[2] * _x4[1] + _x3[3] * _x4[0]);
  const Scalar _tmp66 = _tmp36 * _tmp37 + _tmp59 * (_tmp51 + _tmp54) + _tmp62 * (_tmp60 + _tmp61) +
                        _tmp65 * (_tmp63 - _tmp64);
  const Scalar _tmp67 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp68 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp69 = std::pow(_tmp67, Scalar(2));
  const Scalar _tmp70 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp71 = _x1[3] * _x2[3];
  const Scalar _tmp72 = std::min<Scalar>(_tmp11, std::fabs(_tmp70 - _tmp71));
  const Scalar _tmp73 = 1 - std::pow(_tmp72, Scalar(2));
  const Scalar _tmp74 = std::acos(_tmp72);
  const Scalar _tmp75 =
      2 * std::min<Scalar>(0, (((-_tmp70 + _tmp71) > 0) - ((-_tmp70 + _tmp71) < 0))) + 1;
  const Scalar _tmp76 = std::pow(_tmp74, Scalar(2)) * std::pow(_tmp75, Scalar(2)) *
                        std::pow(lambdas(1, 0), Scalar(2)) / _tmp73;
  const Scalar _tmp77 = 4 * _tmp76;
  const Scalar _tmp78 = std::pow(_tmp68, Scalar(2));
  const Scalar _tmp79 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp80 = std::pow(_tmp79, Scalar(2));
  const Scalar _tmp81 = _tmp20 + _tmp69 * _tmp77 + _tmp77 * _tmp78 + _tmp77 * _tmp80;
  const Scalar _tmp82 = std::sqrt(_tmp81);
  const Scalar _tmp83 = (Scalar(1) / Scalar(2)) * _tmp82;
  const Scalar _tmp84 = std::sin(_tmp83);
  const Scalar _tmp85 = 8 * _tmp76 * std::pow(_tmp84, Scalar(2)) / _tmp81;
  const Scalar _tmp86 = _tmp68 * _tmp85;
  const Scalar _tmp87 = _tmp67 * _tmp86;
  const Scalar _tmp88 = _tmp74 * _tmp75 / std::sqrt(_tmp73);
  const Scalar _tmp89 = 4 * _tmp84 * lambdas(1, 0) * std::cos(_tmp83) / _tmp82;
  const Scalar _tmp90 = _tmp88 * _tmp89;
  const Scalar _tmp91 = _tmp79 * _tmp90;
  const Scalar _tmp92 = _tmp67 * _tmp88;
  const Scalar _tmp93 = 2 * lambdas(1, 1);
  const Scalar _tmp94 = _tmp35 * _tmp37;
  const Scalar _tmp95 = _tmp36 * _tmp49;
  const Scalar _tmp96 = _tmp38 * _tmp53;
  const Scalar _tmp97 = -_tmp39 * _tmp52;
  const Scalar _tmp98 = _tmp38 * _tmp94 + _tmp59 * (-_tmp95 + _tmp96) + _tmp62 * (_tmp63 + _tmp64) +
                        _tmp65 * (_tmp61 + _tmp97);
  const Scalar _tmp99 = _tmp68 * _tmp90;
  const Scalar _tmp100 = _tmp67 * _tmp79 * _tmp85;
  const Scalar _tmp101 = -_tmp78 * _tmp85;
  const Scalar _tmp102 = -_tmp80 * _tmp85 + 1;
  const Scalar _tmp103 = _tmp42 * _tmp94 + _tmp59 * (_tmp60 + _tmp97 + 1) +
                         _tmp62 * (-_tmp51 + _tmp54) + _tmp65 * (_tmp95 + _tmp96);
  const Scalar _tmp104 = _tmp103 * (_tmp101 + _tmp102) + _tmp66 * (_tmp87 - _tmp91) +
                         _tmp92 * _tmp93 + _tmp98 * (_tmp100 + _tmp99);
  const Scalar _tmp105 = _tmp89 * _tmp92;
  const Scalar _tmp106 = _tmp79 * _tmp86;
  const Scalar _tmp107 = -_tmp69 * _tmp85;
  const Scalar _tmp108 = _tmp88 * _tmp93;
  const Scalar _tmp109 = _tmp103 * (_tmp87 + _tmp91) + _tmp108 * _tmp68 +
                         _tmp66 * (_tmp102 + _tmp107) + _tmp98 * (-_tmp105 + _tmp106);
  const Scalar _tmp110 = _tmp25 * _tmp5;
  const Scalar _tmp111 = _tmp110 * _tmp7;
  const Scalar _tmp112 = _tmp14 * _tmp15 / std::sqrt(_tmp13);
  const Scalar _tmp113 = 4 * _tmp24 * lambdas(0, 0) * std::cos(_tmp23) / _tmp22;
  const Scalar _tmp114 = _tmp112 * _tmp113;
  const Scalar _tmp115 = _tmp114 * _tmp18;
  const Scalar _tmp116 = _tmp103 * (_tmp100 - _tmp99) + _tmp108 * _tmp79 +
                         _tmp66 * (_tmp105 + _tmp106) + _tmp98 * (_tmp101 + _tmp107 + 1);
  const Scalar _tmp117 = _tmp112 * _tmp5;
  const Scalar _tmp118 = _tmp113 * _tmp117;
  const Scalar _tmp119 = _tmp18 * _tmp25 * _tmp7;
  const Scalar _tmp120 = 2 * lambdas(0, 1);
  const Scalar _tmp121 = _tmp112 * _tmp120;
  const Scalar _tmp122 = _tmp104 * (_tmp26 + _tmp27) + _tmp109 * (_tmp111 - _tmp115) +
                         _tmp116 * (_tmp118 + _tmp119) + _tmp121 * _tmp7;
  const Scalar _tmp123 = -2 * std::pow(_x0[2], Scalar(2));
  const Scalar _tmp124 = 1 - 2 * std::pow(_x0[1], Scalar(2));
  const Scalar _tmp125 = -_tmp25 * _tmp8;
  const Scalar _tmp126 = _tmp114 * _tmp7;
  const Scalar _tmp127 = _tmp110 * _tmp18;
  const Scalar _tmp128 = _tmp104 * (-_tmp118 + _tmp119) + _tmp109 * (_tmp126 + _tmp127) +
                         _tmp116 * (_tmp125 + _tmp26 + 1) + _tmp121 * _tmp18;
  const Scalar _tmp129 = _tmp1 * _x0[1];
  const Scalar _tmp130 = _tmp3 * _x0[2];
  const Scalar _tmp131 = _tmp104 * (_tmp111 + _tmp115) + _tmp109 * (_tmp125 + _tmp27) +
                         _tmp116 * (-_tmp126 + _tmp127) + _tmp117 * _tmp120;
  const Scalar _tmp132 = -2 * std::pow(_x0[0], Scalar(2));
  const Scalar _tmp133 = _tmp3 * _x0[3];
  const Scalar _tmp134 = 2 * _x0[1] * _x0[2];

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp0 * (_tmp122 * (-_tmp2 + _tmp4) + _tmp128 * (_tmp123 + _tmp124) +
                        _tmp131 * (_tmp129 + _tmp130));
  _res(1, 0) = _tmp0 * (_tmp122 * (_tmp123 + _tmp132 + 1) + _tmp128 * (_tmp2 + _tmp4) +
                        _tmp131 * (-_tmp133 + _tmp134));
  _res(2, 0) = _tmp0 * (_tmp122 * (_tmp133 + _tmp134) + _tmp128 * (-_tmp129 + _tmp130) +
                        _tmp131 * (_tmp124 + _tmp132));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
