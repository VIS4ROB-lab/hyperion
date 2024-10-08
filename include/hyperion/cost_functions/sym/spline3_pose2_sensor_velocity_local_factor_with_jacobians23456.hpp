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
 * Symbolic function: spline3_pose2_sensor_velocity_factor
 *
 * Args:
 *     dt: Scalar
 *     lambdas: Matrix32
 *     x0: Pose2
 *     x1: Pose2
 *     x2: Pose2
 *     x3: Pose2
 *     x_T_s: Pose2
 *     velocity: Matrix31
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 *     res_D_x0: (3x3) jacobian of res (3) wrt arg x0 (3)
 *     res_D_x1: (3x3) jacobian of res (3) wrt arg x1 (3)
 *     res_D_x2: (3x3) jacobian of res (3) wrt arg x2 (3)
 *     res_D_x3: (3x3) jacobian of res (3) wrt arg x3 (3)
 *     res_D_x_T_s: (3x3) jacobian of res (3) wrt arg x_T_s (3)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline3Pose2SensorVelocityLocalFactorWithJacobians23456(
    const Scalar dt, const Eigen::Matrix<Scalar, 3, 2>& lambdas, const sym::Pose2<Scalar>& x0,
    const sym::Pose2<Scalar>& x1, const sym::Pose2<Scalar>& x2, const sym::Pose2<Scalar>& x3,
    const sym::Pose2<Scalar>& x_T_s, const Eigen::Matrix<Scalar, 3, 1>& velocity,
    const Eigen::Matrix<Scalar, 3, 3>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x0 = nullptr, Scalar* const res_D_x1 = nullptr,
    Scalar* const res_D_x2 = nullptr, Scalar* const res_D_x3 = nullptr,
    Scalar* const res_D_x_T_s = nullptr) {
  // Total ops: 551

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x_T_s = x_T_s.Data();

  // Intermediate terms (174)
  const Scalar _tmp0 = _x_T_s[1] * _x_T_s[3];
  const Scalar _tmp1 = _x_T_s[0] * _x_T_s[2];
  const Scalar _tmp2 = _tmp0 + _tmp1;
  const Scalar _tmp3 = Scalar(1.0) / (dt);
  const Scalar _tmp4 = _x2[1] * _x3[0];
  const Scalar _tmp5 = _x2[0] * _x3[1];
  const Scalar _tmp6 = -_tmp4 + _tmp5;
  const Scalar _tmp7 = _x2[0] * _x3[0];
  const Scalar _tmp8 = _x2[1] * _x3[1];
  const Scalar _tmp9 = _tmp7 + _tmp8;
  const Scalar _tmp10 = _tmp9 + epsilon * ((((_tmp9) > 0) - ((_tmp9) < 0)) + Scalar(0.5));
  const Scalar _tmp11 = std::atan2(_tmp6, _tmp10);
  const Scalar _tmp12 = _x1[0] * _x2[1];
  const Scalar _tmp13 = _x1[1] * _x2[0];
  const Scalar _tmp14 = _tmp12 - _tmp13;
  const Scalar _tmp15 = _x1[0] * _x2[0];
  const Scalar _tmp16 = _x1[1] * _x2[1];
  const Scalar _tmp17 = _tmp15 + _tmp16;
  const Scalar _tmp18 = _tmp17 + epsilon * ((((_tmp17) > 0) - ((_tmp17) < 0)) + Scalar(0.5));
  const Scalar _tmp19 = std::atan2(_tmp14, _tmp18);
  const Scalar _tmp20 = _x0[1] * _x1[0];
  const Scalar _tmp21 = _x0[0] * _x1[1];
  const Scalar _tmp22 = -_tmp20 + _tmp21;
  const Scalar _tmp23 = _x0[1] * _x1[1];
  const Scalar _tmp24 = _x0[0] * _x1[0];
  const Scalar _tmp25 = _tmp23 + _tmp24;
  const Scalar _tmp26 = _tmp25 + epsilon * ((((_tmp25) > 0) - ((_tmp25) < 0)) + Scalar(0.5));
  const Scalar _tmp27 = std::atan2(_tmp22, _tmp26);
  const Scalar _tmp28 =
      _tmp3 * (_tmp11 * lambdas(2, 1) + _tmp19 * lambdas(1, 1) + _tmp27 * lambdas(0, 1));
  const Scalar _tmp29 = _tmp27 * lambdas(0, 0);
  const Scalar _tmp30 = std::cos(_tmp29);
  const Scalar _tmp31 = _tmp30 * _x0[0];
  const Scalar _tmp32 = std::sin(_tmp29);
  const Scalar _tmp33 = _tmp32 * _x0[1];
  const Scalar _tmp34 = _tmp31 - _tmp33;
  const Scalar _tmp35 = _tmp19 * lambdas(1, 0);
  const Scalar _tmp36 = std::sin(_tmp35);
  const Scalar _tmp37 = _tmp34 * _tmp36;
  const Scalar _tmp38 = _tmp32 * _x0[0];
  const Scalar _tmp39 = _tmp30 * _x0[1];
  const Scalar _tmp40 = _tmp38 + _tmp39;
  const Scalar _tmp41 = std::cos(_tmp35);
  const Scalar _tmp42 = _tmp40 * _tmp41;
  const Scalar _tmp43 = _tmp37 + _tmp42;
  const Scalar _tmp44 = _tmp11 * lambdas(2, 0);
  const Scalar _tmp45 = std::sin(_tmp44);
  const Scalar _tmp46 = _tmp43 * _tmp45;
  const Scalar _tmp47 = _tmp34 * _tmp41;
  const Scalar _tmp48 = _tmp36 * _tmp40;
  const Scalar _tmp49 = _tmp47 - _tmp48;
  const Scalar _tmp50 = std::cos(_tmp44);
  const Scalar _tmp51 = _tmp49 * _tmp50;
  const Scalar _tmp52 = -_tmp46 + _tmp51;
  const Scalar _tmp53 = lambdas(0, 1) * (-_x0[2] + _x1[2]) + lambdas(1, 1) * (-_x1[2] + _x2[2]) +
                        lambdas(2, 1) * (-_x2[2] + _x3[2]);
  const Scalar _tmp54 = _tmp43 * _tmp50;
  const Scalar _tmp55 = _tmp45 * _tmp49;
  const Scalar _tmp56 = _tmp54 + _tmp55;
  const Scalar _tmp57 = lambdas(0, 1) * (-_x0[3] + _x1[3]) + lambdas(1, 1) * (-_x1[3] + _x2[3]) +
                        lambdas(2, 1) * (-_x2[3] + _x3[3]);
  const Scalar _tmp58 = _tmp52 * _tmp53 + _tmp56 * _tmp57;
  const Scalar _tmp59 = _tmp3 * _x_T_s[1];
  const Scalar _tmp60 = _tmp52 * _tmp57 - _tmp53 * _tmp56;
  const Scalar _tmp61 = _tmp3 * _x_T_s[0];
  const Scalar _tmp62 = -_tmp58 * _tmp59 + _tmp60 * _tmp61;
  const Scalar _tmp63 = _tmp2 * _tmp28 + _tmp62 - velocity(2, 0);
  const Scalar _tmp64 = _tmp28 - velocity(0, 0);
  const Scalar _tmp65 = _tmp58 * _tmp61;
  const Scalar _tmp66 = _tmp59 * _tmp60;
  const Scalar _tmp67 = _x_T_s[0] * _x_T_s[3] - _x_T_s[1] * _x_T_s[2];
  const Scalar _tmp68 = _tmp28 * _tmp67;
  const Scalar _tmp69 = _tmp65 + _tmp66 - _tmp68 - velocity(1, 0);
  const Scalar _tmp70 = _tmp3 * _tmp67;
  const Scalar _tmp71 = std::pow(_tmp22, Scalar(2));
  const Scalar _tmp72 = std::pow(_tmp26, Scalar(2));
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 = Scalar(1.0) / (_tmp26);
  const Scalar _tmp75 = _tmp72 / (_tmp71 + _tmp72);
  const Scalar _tmp76 = _tmp75 * (-_tmp71 * _tmp73 + _tmp74 * (-_tmp23 - _tmp24));
  const Scalar _tmp77 = _tmp76 * lambdas(0, 1);
  const Scalar _tmp78 = _tmp76 * lambdas(0, 0);
  const Scalar _tmp79 = -_tmp38 * _tmp78 - _tmp38 - _tmp39 * _tmp78 - _tmp39;
  const Scalar _tmp80 = _tmp31 * _tmp78 - _tmp33 * _tmp78 + _tmp34;
  const Scalar _tmp81 = -_tmp36 * _tmp80 + _tmp41 * _tmp79;
  const Scalar _tmp82 = _tmp36 * _tmp79 + _tmp41 * _tmp80;
  const Scalar _tmp83 = _tmp45 * _tmp81 + _tmp50 * _tmp82;
  const Scalar _tmp84 = -_tmp45 * _tmp82 + _tmp50 * _tmp81;
  const Scalar _tmp85 = _tmp3 * (_tmp53 * _tmp84 + _tmp57 * _tmp83);
  const Scalar _tmp86 = _tmp3 * (-_tmp53 * _tmp83 + _tmp57 * _tmp84);
  const Scalar _tmp87 = -_tmp70 * _tmp77 + _tmp85 * _x_T_s[0] + _tmp86 * _x_T_s[1];
  const Scalar _tmp88 = _tmp3 * _tmp77;
  const Scalar _tmp89 = _tmp2 * _tmp88 - _tmp85 * _x_T_s[1] + _tmp86 * _x_T_s[0];
  const Scalar _tmp90 = _tmp56 * _tmp61;
  const Scalar _tmp91 = _tmp90 * lambdas(0, 1);
  const Scalar _tmp92 = _tmp52 * lambdas(0, 1);
  const Scalar _tmp93 = _tmp59 * _tmp92;
  const Scalar _tmp94 = _tmp91 + _tmp93;
  const Scalar _tmp95 = _tmp56 * _tmp59;
  const Scalar _tmp96 = -_tmp61 * _tmp92 + _tmp95 * lambdas(0, 1);
  const Scalar _tmp97 = -_tmp91 - _tmp93;
  const Scalar _tmp98 = std::pow(_tmp14, Scalar(2));
  const Scalar _tmp99 = std::pow(_tmp18, Scalar(2));
  const Scalar _tmp100 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp101 = Scalar(1.0) / (_tmp18);
  const Scalar _tmp102 = -_tmp100 * _tmp98 + _tmp101 * (-_tmp15 - _tmp16);
  const Scalar _tmp103 = _tmp99 / (_tmp98 + _tmp99);
  const Scalar _tmp104 = _tmp103 * lambdas(1, 0);
  const Scalar _tmp105 = _tmp104 * _tmp48;
  const Scalar _tmp106 = _tmp102 * _tmp104;
  const Scalar _tmp107 = _tmp75 * (-_tmp22 * _tmp73 * (_tmp20 - _tmp21) + _tmp25 * _tmp74);
  const Scalar _tmp108 = _tmp107 * lambdas(0, 0);
  const Scalar _tmp109 = _tmp108 * _tmp31 - _tmp108 * _tmp33;
  const Scalar _tmp110 = -_tmp108 * _tmp38 - _tmp108 * _tmp39;
  const Scalar _tmp111 =
      -_tmp102 * _tmp105 + _tmp106 * _tmp47 + _tmp109 * _tmp41 + _tmp110 * _tmp36;
  const Scalar _tmp112 = -_tmp106 * _tmp37 - _tmp106 * _tmp42 - _tmp109 * _tmp36 + _tmp110 * _tmp41;
  const Scalar _tmp113 = _tmp111 * _tmp50 + _tmp112 * _tmp45;
  const Scalar _tmp114 = -_tmp111 * _tmp45 + _tmp112 * _tmp50;
  const Scalar _tmp115 = _tmp113 * _tmp57 + _tmp114 * _tmp53;
  const Scalar _tmp116 = _tmp103 * lambdas(1, 1);
  const Scalar _tmp117 = _tmp3 * (_tmp102 * _tmp116 + _tmp107 * lambdas(0, 1));
  const Scalar _tmp118 = _tmp3 * (-_tmp113 * _tmp53 + _tmp114 * _tmp57);
  const Scalar _tmp119 = -_tmp115 * _tmp59 + _tmp117 * _tmp2 + _tmp118 * _x_T_s[0];
  const Scalar _tmp120 = _tmp115 * _tmp61 - _tmp117 * _tmp67 + _tmp118 * _x_T_s[1];
  const Scalar _tmp121 = lambdas(0, 1) - lambdas(1, 1);
  const Scalar _tmp122 = _tmp121 * _tmp52;
  const Scalar _tmp123 = -_tmp121 * _tmp95 + _tmp122 * _tmp61;
  const Scalar _tmp124 = _tmp122 * _tmp59;
  const Scalar _tmp125 = _tmp121 * _tmp90;
  const Scalar _tmp126 = -_tmp124 - _tmp125;
  const Scalar _tmp127 = _tmp124 + _tmp125;
  const Scalar _tmp128 = std::pow(_tmp6, Scalar(2));
  const Scalar _tmp129 = std::pow(_tmp10, Scalar(2));
  const Scalar _tmp130 = Scalar(1.0) / (_tmp129);
  const Scalar _tmp131 = Scalar(1.0) / (_tmp10);
  const Scalar _tmp132 = _tmp129 / (_tmp128 + _tmp129);
  const Scalar _tmp133 = _tmp132 * (-_tmp128 * _tmp130 + _tmp131 * (-_tmp7 - _tmp8));
  const Scalar _tmp134 = -_tmp100 * _tmp14 * (-_tmp12 + _tmp13) + _tmp101 * _tmp17;
  const Scalar _tmp135 = _tmp3 * (_tmp116 * _tmp134 + _tmp133 * lambdas(2, 1));
  const Scalar _tmp136 = _tmp104 * _tmp134;
  const Scalar _tmp137 = -_tmp105 * _tmp134 + _tmp136 * _tmp47;
  const Scalar _tmp138 = _tmp133 * lambdas(2, 0);
  const Scalar _tmp139 = -_tmp136 * _tmp37 - _tmp136 * _tmp42;
  const Scalar _tmp140 = -_tmp137 * _tmp45 - _tmp138 * _tmp54 - _tmp138 * _tmp55 + _tmp139 * _tmp50;
  const Scalar _tmp141 = _tmp137 * _tmp50 - _tmp138 * _tmp46 + _tmp138 * _tmp51 + _tmp139 * _tmp45;
  const Scalar _tmp142 = _tmp140 * _tmp57 - _tmp141 * _tmp53;
  const Scalar _tmp143 = _tmp140 * _tmp53 + _tmp141 * _tmp57;
  const Scalar _tmp144 = -_tmp135 * _tmp67 + _tmp142 * _tmp59 + _tmp143 * _tmp61;
  const Scalar _tmp145 = _tmp135 * _tmp2 + _tmp142 * _tmp61 - _tmp143 * _tmp59;
  const Scalar _tmp146 = lambdas(1, 1) - lambdas(2, 1);
  const Scalar _tmp147 = _tmp146 * _tmp52;
  const Scalar _tmp148 = _tmp147 * _tmp59;
  const Scalar _tmp149 = _tmp146 * _tmp90;
  const Scalar _tmp150 = -_tmp148 - _tmp149;
  const Scalar _tmp151 = -_tmp146 * _tmp95 + _tmp147 * _tmp61;
  const Scalar _tmp152 = _tmp148 + _tmp149;
  const Scalar _tmp153 = _tmp132 * (-_tmp130 * _tmp6 * (_tmp4 - _tmp5) + _tmp131 * _tmp9);
  const Scalar _tmp154 = _tmp153 * lambdas(2, 0);
  const Scalar _tmp155 = -_tmp154 * _tmp46 + _tmp154 * _tmp51;
  const Scalar _tmp156 = -_tmp154 * _tmp54 - _tmp154 * _tmp55;
  const Scalar _tmp157 = _tmp155 * _tmp57 + _tmp156 * _tmp53;
  const Scalar _tmp158 = _tmp153 * lambdas(2, 1);
  const Scalar _tmp159 = -_tmp155 * _tmp53 + _tmp156 * _tmp57;
  const Scalar _tmp160 = _tmp157 * _tmp61 - _tmp158 * _tmp70 + _tmp159 * _tmp59;
  const Scalar _tmp161 = _tmp158 * _tmp3;
  const Scalar _tmp162 = -_tmp157 * _tmp59 + _tmp159 * _tmp61 + _tmp161 * _tmp2;
  const Scalar _tmp163 = _tmp90 * lambdas(2, 1);
  const Scalar _tmp164 = _tmp52 * lambdas(2, 1);
  const Scalar _tmp165 = _tmp164 * _tmp59;
  const Scalar _tmp166 = -_tmp163 - _tmp165;
  const Scalar _tmp167 = _tmp164 * _tmp61 - _tmp95 * lambdas(2, 1);
  const Scalar _tmp168 = _tmp163 + _tmp165;
  const Scalar _tmp169 = -_tmp28 * (-_tmp0 - _tmp1) + _tmp62;
  const Scalar _tmp170 = -_tmp65 - _tmp66 + _tmp68;
  const Scalar _tmp171 = _tmp28 * _x_T_s[1];
  const Scalar _tmp172 = _tmp28 * _x_T_s[0];
  const Scalar _tmp173 = _tmp28 * sqrt_info(1, 1);

  // Output terms (6)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp63 * sqrt_info(0, 2) + _tmp64 * sqrt_info(0, 0) + _tmp69 * sqrt_info(0, 1);
  _res(1, 0) = _tmp63 * sqrt_info(1, 2) + _tmp64 * sqrt_info(1, 0) + _tmp69 * sqrt_info(1, 1);
  _res(2, 0) = _tmp63 * sqrt_info(2, 2) + _tmp64 * sqrt_info(2, 0) + _tmp69 * sqrt_info(2, 1);

  if (res_D_x0 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x0{res_D_x0};

    _res_D_x0(0, 0) =
        _tmp87 * sqrt_info(0, 1) + _tmp88 * sqrt_info(0, 0) + _tmp89 * sqrt_info(0, 2);
    _res_D_x0(1, 0) =
        _tmp87 * sqrt_info(1, 1) + _tmp88 * sqrt_info(1, 0) + _tmp89 * sqrt_info(1, 2);
    _res_D_x0(2, 0) =
        _tmp87 * sqrt_info(2, 1) + _tmp88 * sqrt_info(2, 0) + _tmp89 * sqrt_info(2, 2);
    _res_D_x0(0, 1) = _tmp94 * sqrt_info(0, 2) + _tmp96 * sqrt_info(0, 1);
    _res_D_x0(1, 1) = _tmp94 * sqrt_info(1, 2) + _tmp96 * sqrt_info(1, 1);
    _res_D_x0(2, 1) = _tmp94 * sqrt_info(2, 2) + _tmp96 * sqrt_info(2, 1);
    _res_D_x0(0, 2) = _tmp96 * sqrt_info(0, 2) + _tmp97 * sqrt_info(0, 1);
    _res_D_x0(1, 2) = _tmp96 * sqrt_info(1, 2) + _tmp97 * sqrt_info(1, 1);
    _res_D_x0(2, 2) = _tmp96 * sqrt_info(2, 2) + _tmp97 * sqrt_info(2, 1);
  }

  if (res_D_x1 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x1{res_D_x1};

    _res_D_x1(0, 0) =
        _tmp117 * sqrt_info(0, 0) + _tmp119 * sqrt_info(0, 2) + _tmp120 * sqrt_info(0, 1);
    _res_D_x1(1, 0) =
        _tmp117 * sqrt_info(1, 0) + _tmp119 * sqrt_info(1, 2) + _tmp120 * sqrt_info(1, 1);
    _res_D_x1(2, 0) =
        _tmp117 * sqrt_info(2, 0) + _tmp119 * sqrt_info(2, 2) + _tmp120 * sqrt_info(2, 1);
    _res_D_x1(0, 1) = _tmp123 * sqrt_info(0, 1) + _tmp126 * sqrt_info(0, 2);
    _res_D_x1(1, 1) = _tmp123 * sqrt_info(1, 1) + _tmp126 * sqrt_info(1, 2);
    _res_D_x1(2, 1) = _tmp123 * sqrt_info(2, 1) + _tmp126 * sqrt_info(2, 2);
    _res_D_x1(0, 2) = _tmp123 * sqrt_info(0, 2) + _tmp127 * sqrt_info(0, 1);
    _res_D_x1(1, 2) = _tmp123 * sqrt_info(1, 2) + _tmp127 * sqrt_info(1, 1);
    _res_D_x1(2, 2) = _tmp123 * sqrt_info(2, 2) + _tmp127 * sqrt_info(2, 1);
  }

  if (res_D_x2 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x2{res_D_x2};

    _res_D_x2(0, 0) =
        _tmp135 * sqrt_info(0, 0) + _tmp144 * sqrt_info(0, 1) + _tmp145 * sqrt_info(0, 2);
    _res_D_x2(1, 0) =
        _tmp135 * sqrt_info(1, 0) + _tmp144 * sqrt_info(1, 1) + _tmp145 * sqrt_info(1, 2);
    _res_D_x2(2, 0) =
        _tmp135 * sqrt_info(2, 0) + _tmp144 * sqrt_info(2, 1) + _tmp145 * sqrt_info(2, 2);
    _res_D_x2(0, 1) = _tmp150 * sqrt_info(0, 2) + _tmp151 * sqrt_info(0, 1);
    _res_D_x2(1, 1) = _tmp150 * sqrt_info(1, 2) + _tmp151 * sqrt_info(1, 1);
    _res_D_x2(2, 1) = _tmp150 * sqrt_info(2, 2) + _tmp151 * sqrt_info(2, 1);
    _res_D_x2(0, 2) = _tmp151 * sqrt_info(0, 2) + _tmp152 * sqrt_info(0, 1);
    _res_D_x2(1, 2) = _tmp151 * sqrt_info(1, 2) + _tmp152 * sqrt_info(1, 1);
    _res_D_x2(2, 2) = _tmp151 * sqrt_info(2, 2) + _tmp152 * sqrt_info(2, 1);
  }

  if (res_D_x3 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x3{res_D_x3};

    _res_D_x3(0, 0) =
        _tmp160 * sqrt_info(0, 1) + _tmp161 * sqrt_info(0, 0) + _tmp162 * sqrt_info(0, 2);
    _res_D_x3(1, 0) =
        _tmp160 * sqrt_info(1, 1) + _tmp161 * sqrt_info(1, 0) + _tmp162 * sqrt_info(1, 2);
    _res_D_x3(2, 0) =
        _tmp160 * sqrt_info(2, 1) + _tmp161 * sqrt_info(2, 0) + _tmp162 * sqrt_info(2, 2);
    _res_D_x3(0, 1) = _tmp166 * sqrt_info(0, 2) + _tmp167 * sqrt_info(0, 1);
    _res_D_x3(1, 1) = _tmp166 * sqrt_info(1, 2) + _tmp167 * sqrt_info(1, 1);
    _res_D_x3(2, 1) = _tmp166 * sqrt_info(2, 2) + _tmp167 * sqrt_info(2, 1);
    _res_D_x3(0, 2) = _tmp167 * sqrt_info(0, 2) + _tmp168 * sqrt_info(0, 1);
    _res_D_x3(1, 2) = _tmp167 * sqrt_info(1, 2) + _tmp168 * sqrt_info(1, 1);
    _res_D_x3(2, 2) = _tmp167 * sqrt_info(2, 2) + _tmp168 * sqrt_info(2, 1);
  }

  if (res_D_x_T_s != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x_T_s{res_D_x_T_s};

    _res_D_x_T_s(0, 0) = _tmp169 * sqrt_info(0, 1) + _tmp170 * sqrt_info(0, 2);
    _res_D_x_T_s(1, 0) = _tmp169 * sqrt_info(1, 1) + _tmp170 * sqrt_info(1, 2);
    _res_D_x_T_s(2, 0) = _tmp169 * sqrt_info(2, 1) + _tmp170 * sqrt_info(2, 2);
    _res_D_x_T_s(0, 1) = _tmp171 * sqrt_info(0, 1) + _tmp172 * sqrt_info(0, 2);
    _res_D_x_T_s(1, 1) = _tmp172 * sqrt_info(1, 2) + _tmp173 * _x_T_s[1];
    _res_D_x_T_s(2, 1) = _tmp171 * sqrt_info(2, 1) + _tmp172 * sqrt_info(2, 2);
    _res_D_x_T_s(0, 2) = _tmp171 * sqrt_info(0, 2) - _tmp172 * sqrt_info(0, 1);
    _res_D_x_T_s(1, 2) = _tmp171 * sqrt_info(1, 2) - _tmp173 * _x_T_s[0];
    _res_D_x_T_s(2, 2) = _tmp171 * sqrt_info(2, 2) - _tmp172 * sqrt_info(2, 1);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
