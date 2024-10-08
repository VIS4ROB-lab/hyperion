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
 * Symbolic function: spline3_pose3_velocity_factor
 *
 * Args:
 *     dt: Scalar
 *     lambdas: Matrix32
 *     x0: Pose3
 *     x1: Pose3
 *     x2: Pose3
 *     x3: Pose3
 *     velocity: Matrix61
 *     sqrt_info: Matrix66
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix61
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> Spline3Pose3VelocityGlobalFactor(
    const Scalar dt, const Eigen::Matrix<Scalar, 3, 2>& lambdas, const sym::Pose3<Scalar>& x0,
    const sym::Pose3<Scalar>& x1, const sym::Pose3<Scalar>& x2, const sym::Pose3<Scalar>& x3,
    const Eigen::Matrix<Scalar, 6, 1>& velocity, const Eigen::Matrix<Scalar, 6, 6>& sqrt_info,
    const Scalar epsilon) {
  // Total ops: 463

  // Input arrays
  const Eigen::Matrix<Scalar, 7, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 7, 1>& _x3 = x3.Data();

  // Intermediate terms (118)
  const Scalar _tmp0 = Scalar(1.0) / (dt);
  const Scalar _tmp1 = 2 * _x0[0];
  const Scalar _tmp2 = _tmp1 * _x0[1];
  const Scalar _tmp3 = 2 * _x0[3];
  const Scalar _tmp4 = _tmp3 * _x0[2];
  const Scalar _tmp5 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp6 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp7 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp8 = std::pow(_tmp7, Scalar(2));
  const Scalar _tmp9 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp10 = _x0[3] * _x1[3];
  const Scalar _tmp11 =
      2 * std::min<Scalar>(0, (((_tmp10 - _tmp9) > 0) - ((_tmp10 - _tmp9) < 0))) + 1;
  const Scalar _tmp12 = 1 - epsilon;
  const Scalar _tmp13 = std::min<Scalar>(_tmp12, std::fabs(_tmp10 - _tmp9));
  const Scalar _tmp14 = std::acos(_tmp13);
  const Scalar _tmp15 = 1 - std::pow(_tmp13, Scalar(2));
  const Scalar _tmp16 = std::pow(_tmp11, Scalar(2)) * std::pow(_tmp14, Scalar(2)) *
                        std::pow(lambdas(0, 0), Scalar(2)) / _tmp15;
  const Scalar _tmp17 = 4 * _tmp16;
  const Scalar _tmp18 = std::pow(_tmp5, Scalar(2));
  const Scalar _tmp19 = std::pow(_tmp6, Scalar(2));
  const Scalar _tmp20 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp21 = _tmp17 * _tmp18 + _tmp17 * _tmp19 + _tmp17 * _tmp8 + _tmp20;
  const Scalar _tmp22 = std::sqrt(_tmp21);
  const Scalar _tmp23 = (Scalar(1) / Scalar(2)) * _tmp22;
  const Scalar _tmp24 = std::sin(_tmp23);
  const Scalar _tmp25 = 8 * _tmp16 * std::pow(_tmp24, Scalar(2)) / _tmp21;
  const Scalar _tmp26 = _tmp25 * _tmp6;
  const Scalar _tmp27 = _tmp26 * _tmp5;
  const Scalar _tmp28 = _tmp11 * _tmp14 / std::sqrt(_tmp15);
  const Scalar _tmp29 = _tmp28 * _tmp7;
  const Scalar _tmp30 = 4 * _tmp24 * lambdas(0, 0) * std::cos(_tmp23) / _tmp22;
  const Scalar _tmp31 = _tmp29 * _tmp30;
  const Scalar _tmp32 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp33 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp34 = _x1[3] * _x2[3];
  const Scalar _tmp35 = std::min<Scalar>(_tmp12, std::fabs(_tmp33 - _tmp34));
  const Scalar _tmp36 = 1 - std::pow(_tmp35, Scalar(2));
  const Scalar _tmp37 =
      2 * std::min<Scalar>(0, (((-_tmp33 + _tmp34) > 0) - ((-_tmp33 + _tmp34) < 0))) + 1;
  const Scalar _tmp38 = std::acos(_tmp35);
  const Scalar _tmp39 = _tmp37 * _tmp38 / std::sqrt(_tmp36);
  const Scalar _tmp40 = 2 * lambdas(1, 1);
  const Scalar _tmp41 = _tmp39 * _tmp40;
  const Scalar _tmp42 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp43 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp44 = std::pow(_tmp43, Scalar(2));
  const Scalar _tmp45 = std::pow(_tmp37, Scalar(2)) * std::pow(_tmp38, Scalar(2)) *
                        std::pow(lambdas(1, 0), Scalar(2)) / _tmp36;
  const Scalar _tmp46 = 4 * _tmp45;
  const Scalar _tmp47 = std::pow(_tmp32, Scalar(2));
  const Scalar _tmp48 = std::pow(_tmp42, Scalar(2));
  const Scalar _tmp49 = _tmp20 + _tmp44 * _tmp46 + _tmp46 * _tmp47 + _tmp46 * _tmp48;
  const Scalar _tmp50 = std::sqrt(_tmp49);
  const Scalar _tmp51 = (Scalar(1) / Scalar(2)) * _tmp50;
  const Scalar _tmp52 = std::sin(_tmp51);
  const Scalar _tmp53 = 8 * _tmp45 * std::pow(_tmp52, Scalar(2)) / _tmp49;
  const Scalar _tmp54 = _tmp42 * _tmp53;
  const Scalar _tmp55 = _tmp32 * _tmp54;
  const Scalar _tmp56 = _tmp39 * _tmp43;
  const Scalar _tmp57 = 4 * _tmp52 * lambdas(1, 0) * std::cos(_tmp51) / _tmp50;
  const Scalar _tmp58 = _tmp56 * _tmp57;
  const Scalar _tmp59 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp60 = _x2[3] * _x3[3];
  const Scalar _tmp61 = std::min<Scalar>(_tmp12, std::fabs(_tmp59 - _tmp60));
  const Scalar _tmp62 =
      2 * lambdas(2, 1) *
      (2 * std::min<Scalar>(0, (((-_tmp59 + _tmp60) > 0) - ((-_tmp59 + _tmp60) < 0))) + 1) *
      std::acos(_tmp61) / std::sqrt(Scalar(1 - std::pow(_tmp61, Scalar(2))));
  const Scalar _tmp63 =
      _tmp62 * (_x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1]);
  const Scalar _tmp64 = -_tmp48 * _tmp53;
  const Scalar _tmp65 = -_tmp44 * _tmp53;
  const Scalar _tmp66 =
      _tmp62 * (-_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2]);
  const Scalar _tmp67 = _tmp39 * _tmp57;
  const Scalar _tmp68 = _tmp42 * _tmp67;
  const Scalar _tmp69 = _tmp32 * _tmp43 * _tmp53;
  const Scalar _tmp70 =
      _tmp62 * (-_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0]);
  const Scalar _tmp71 = _tmp32 * _tmp41 + _tmp63 * (_tmp55 + _tmp58) +
                        _tmp66 * (_tmp64 + _tmp65 + 1) + _tmp70 * (-_tmp68 + _tmp69);
  const Scalar _tmp72 = _tmp43 * _tmp54;
  const Scalar _tmp73 = _tmp32 * _tmp67;
  const Scalar _tmp74 = -_tmp47 * _tmp53 + 1;
  const Scalar _tmp75 = _tmp40 * _tmp56 + _tmp63 * (_tmp72 - _tmp73) + _tmp66 * (_tmp68 + _tmp69) +
                        _tmp70 * (_tmp64 + _tmp74);
  const Scalar _tmp76 = _tmp25 * _tmp5 * _tmp7;
  const Scalar _tmp77 = _tmp28 * _tmp30;
  const Scalar _tmp78 = _tmp6 * _tmp77;
  const Scalar _tmp79 = _tmp41 * _tmp42 + _tmp63 * (_tmp65 + _tmp74) + _tmp66 * (_tmp55 - _tmp58) +
                        _tmp70 * (_tmp72 + _tmp73);
  const Scalar _tmp80 = -_tmp19 * _tmp25;
  const Scalar _tmp81 = -_tmp25 * _tmp8 + 1;
  const Scalar _tmp82 = 2 * lambdas(0, 1);
  const Scalar _tmp83 = _tmp28 * _tmp82;
  const Scalar _tmp84 = _tmp5 * _tmp83 + _tmp71 * (_tmp27 - _tmp31) + _tmp75 * (_tmp76 + _tmp78) +
                        _tmp79 * (_tmp80 + _tmp81);
  const Scalar _tmp85 = _tmp3 * _x0[1];
  const Scalar _tmp86 = _tmp1 * _x0[2];
  const Scalar _tmp87 = -_tmp18 * _tmp25;
  const Scalar _tmp88 = _tmp5 * _tmp77;
  const Scalar _tmp89 = _tmp26 * _tmp7;
  const Scalar _tmp90 = _tmp6 * _tmp83 + _tmp71 * (_tmp81 + _tmp87) + _tmp75 * (-_tmp88 + _tmp89) +
                        _tmp79 * (_tmp27 + _tmp31);
  const Scalar _tmp91 = -2 * std::pow(_x0[2], Scalar(2));
  const Scalar _tmp92 = 1 - 2 * std::pow(_x0[1], Scalar(2));
  const Scalar _tmp93 = _tmp29 * _tmp82 + _tmp71 * (_tmp88 + _tmp89) +
                        _tmp75 * (_tmp80 + _tmp87 + 1) + _tmp79 * (_tmp76 - _tmp78);
  const Scalar _tmp94 =
      _tmp84 * (_tmp2 - _tmp4) + _tmp90 * (_tmp85 + _tmp86) + _tmp93 * (_tmp91 + _tmp92);
  const Scalar _tmp95 = _tmp0 * _tmp94 - velocity(0, 0);
  const Scalar _tmp96 = _tmp3 * _x0[0];
  const Scalar _tmp97 = 2 * _x0[1] * _x0[2];
  const Scalar _tmp98 = -2 * std::pow(_x0[0], Scalar(2));
  const Scalar _tmp99 =
      _tmp84 * (_tmp96 + _tmp97) + _tmp90 * (_tmp92 + _tmp98) + _tmp93 * (-_tmp85 + _tmp86);
  const Scalar _tmp100 = _tmp0 * _tmp99 - velocity(2, 0);
  const Scalar _tmp101 = -_x1[6] + _x2[6];
  const Scalar _tmp102 = -_x2[6] + _x3[6];
  const Scalar _tmp103 = -_x0[6] + _x1[6];
  const Scalar _tmp104 = -_x1[4] + _x2[4];
  const Scalar _tmp105 = -_x2[4] + _x3[4];
  const Scalar _tmp106 = -_x0[4] + _x1[4];
  const Scalar _tmp107 =
      _tmp104 * lambdas(1, 0) + _tmp105 * lambdas(2, 0) + _tmp106 * lambdas(0, 0) + _x0[4];
  const Scalar _tmp108 =
      _tmp84 * (_tmp91 + _tmp98 + 1) + _tmp90 * (-_tmp96 + _tmp97) + _tmp93 * (_tmp2 + _tmp4);
  const Scalar _tmp109 = -_x1[5] + _x2[5];
  const Scalar _tmp110 = -_x2[5] + _x3[5];
  const Scalar _tmp111 = -_x0[5] + _x1[5];
  const Scalar _tmp112 =
      _tmp109 * lambdas(1, 0) + _tmp110 * lambdas(2, 0) + _tmp111 * lambdas(0, 0) + _x0[5];
  const Scalar _tmp113 = _tmp0 * (_tmp101 * lambdas(1, 1) + _tmp102 * lambdas(2, 1) +
                                  _tmp103 * lambdas(0, 1) + _tmp107 * _tmp108 - _tmp112 * _tmp94) -
                         velocity(5, 0);
  const Scalar _tmp114 =
      _tmp101 * lambdas(1, 0) + _tmp102 * lambdas(2, 0) + _tmp103 * lambdas(0, 0) + _x0[6];
  const Scalar _tmp115 = _tmp0 * (_tmp104 * lambdas(1, 1) + _tmp105 * lambdas(2, 1) +
                                  _tmp106 * lambdas(0, 1) - _tmp108 * _tmp114 + _tmp112 * _tmp99) -
                         velocity(3, 0);
  const Scalar _tmp116 =
      _tmp0 * (-_tmp107 * _tmp99 + _tmp109 * lambdas(1, 1) + _tmp110 * lambdas(2, 1) +
               _tmp111 * lambdas(0, 1) + _tmp114 * _tmp94) -
      velocity(4, 0);
  const Scalar _tmp117 = _tmp0 * _tmp108 - velocity(1, 0);

  // Output terms (1)
  Eigen::Matrix<Scalar, 6, 1> _res;

  _res(0, 0) = _tmp100 * sqrt_info(0, 2) + _tmp113 * sqrt_info(0, 5) + _tmp115 * sqrt_info(0, 3) +
               _tmp116 * sqrt_info(0, 4) + _tmp117 * sqrt_info(0, 1) + _tmp95 * sqrt_info(0, 0);
  _res(1, 0) = _tmp100 * sqrt_info(1, 2) + _tmp113 * sqrt_info(1, 5) + _tmp115 * sqrt_info(1, 3) +
               _tmp116 * sqrt_info(1, 4) + _tmp117 * sqrt_info(1, 1) + _tmp95 * sqrt_info(1, 0);
  _res(2, 0) = _tmp100 * sqrt_info(2, 2) + _tmp113 * sqrt_info(2, 5) + _tmp115 * sqrt_info(2, 3) +
               _tmp116 * sqrt_info(2, 4) + _tmp117 * sqrt_info(2, 1) + _tmp95 * sqrt_info(2, 0);
  _res(3, 0) = _tmp100 * sqrt_info(3, 2) + _tmp113 * sqrt_info(3, 5) + _tmp115 * sqrt_info(3, 3) +
               _tmp116 * sqrt_info(3, 4) + _tmp117 * sqrt_info(3, 1) + _tmp95 * sqrt_info(3, 0);
  _res(4, 0) = _tmp100 * sqrt_info(4, 2) + _tmp113 * sqrt_info(4, 5) + _tmp115 * sqrt_info(4, 3) +
               _tmp116 * sqrt_info(4, 4) + _tmp117 * sqrt_info(4, 1) + _tmp95 * sqrt_info(4, 0);
  _res(5, 0) = _tmp100 * sqrt_info(5, 2) + _tmp113 * sqrt_info(5, 5) + _tmp115 * sqrt_info(5, 3) +
               _tmp116 * sqrt_info(5, 4) + _tmp117 * sqrt_info(5, 1) + _tmp95 * sqrt_info(5, 0);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
