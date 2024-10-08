// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/rot3.h>

namespace sym_hyperion {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline4_rot3_delta_factor
 *
 * Args:
 *     lambdas: Matrix41
 *     x0: Rot3
 *     x1: Rot3
 *     x2: Rot3
 *     x3: Rot3
 *     x4: Rot3
 *     y: Rot3
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline4Rot3DeltaFactor(
    const Eigen::Matrix<Scalar, 4, 1>& lambdas, const sym::Rot3<Scalar>& x0,
    const sym::Rot3<Scalar>& x1, const sym::Rot3<Scalar>& x2, const sym::Rot3<Scalar>& x3,
    const sym::Rot3<Scalar>& x4, const sym::Rot3<Scalar>& y,
    const Eigen::Matrix<Scalar, 3, 3>& sqrt_info, const Scalar epsilon) {
  // Total ops: 456

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x4 = x4.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y = y.Data();

  // Intermediate terms (100)
  const Scalar _tmp0 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp1 = _x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1];
  const Scalar _tmp2 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp3 = _x2[3] * _x3[3];
  const Scalar _tmp4 = 1 - epsilon;
  const Scalar _tmp5 = std::min<Scalar>(_tmp4, std::fabs(_tmp2 - _tmp3));
  const Scalar _tmp6 = std::acos(_tmp5);
  const Scalar _tmp7 =
      2 * std::min<Scalar>(0, (((-_tmp2 + _tmp3) > 0) - ((-_tmp2 + _tmp3) < 0))) + 1;
  const Scalar _tmp8 = 1 - std::pow(_tmp5, Scalar(2));
  const Scalar _tmp9 = 4 * std::pow(_tmp6, Scalar(2)) * std::pow(_tmp7, Scalar(2)) *
                       std::pow(lambdas(2, 0), Scalar(2)) / _tmp8;
  const Scalar _tmp10 = -_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2];
  const Scalar _tmp11 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp12 = -_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0];
  const Scalar _tmp13 =
      std::sqrt(Scalar(std::pow(_tmp1, Scalar(2)) * _tmp9 + std::pow(_tmp10, Scalar(2)) * _tmp9 +
                       _tmp11 + std::pow(_tmp12, Scalar(2)) * _tmp9));
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp15 =
      _tmp6 * _tmp7 * lambdas(2, 0) * std::sin(_tmp14) / (_tmp13 * std::sqrt(_tmp8));
  const Scalar _tmp16 = _tmp1 * _tmp15;
  const Scalar _tmp17 = -_x3[0] * _x4[3] - _x3[1] * _x4[2] + _x3[2] * _x4[1] + _x3[3] * _x4[0];
  const Scalar _tmp18 = -_x3[0] * _x4[0] - _x3[1] * _x4[1] - _x3[2] * _x4[2];
  const Scalar _tmp19 = _x3[3] * _x4[3];
  const Scalar _tmp20 = std::min<Scalar>(_tmp4, std::fabs(_tmp18 - _tmp19));
  const Scalar _tmp21 = std::acos(_tmp20);
  const Scalar _tmp22 =
      2 * std::min<Scalar>(0, (((-_tmp18 + _tmp19) > 0) - ((-_tmp18 + _tmp19) < 0))) + 1;
  const Scalar _tmp23 = 1 - std::pow(_tmp20, Scalar(2));
  const Scalar _tmp24 = 4 * std::pow(_tmp21, Scalar(2)) * std::pow(_tmp22, Scalar(2)) *
                        std::pow(lambdas(3, 0), Scalar(2)) / _tmp23;
  const Scalar _tmp25 = -_x3[0] * _x4[1] + _x3[1] * _x4[0] - _x3[2] * _x4[3] + _x3[3] * _x4[2];
  const Scalar _tmp26 = _x3[0] * _x4[2] - _x3[1] * _x4[3] - _x3[2] * _x4[0] + _x3[3] * _x4[1];
  const Scalar _tmp27 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp17, Scalar(2)) * _tmp24 +
                                         _tmp24 * std::pow(_tmp25, Scalar(2)) +
                                         _tmp24 * std::pow(_tmp26, Scalar(2))));
  const Scalar _tmp28 = (Scalar(1) / Scalar(2)) * _tmp27;
  const Scalar _tmp29 = std::cos(_tmp28);
  const Scalar _tmp30 = 2 * _tmp29;
  const Scalar _tmp31 =
      _tmp21 * _tmp22 * lambdas(3, 0) * std::sin(_tmp28) / (std::sqrt(_tmp23) * _tmp27);
  const Scalar _tmp32 = _tmp26 * _tmp31;
  const Scalar _tmp33 = std::cos(_tmp14);
  const Scalar _tmp34 = 2 * _tmp33;
  const Scalar _tmp35 = 4 * _tmp31;
  const Scalar _tmp36 = _tmp17 * _tmp35;
  const Scalar _tmp37 = _tmp10 * _tmp15;
  const Scalar _tmp38 = _tmp12 * _tmp15;
  const Scalar _tmp39 = _tmp25 * _tmp35;
  const Scalar _tmp40 = _tmp16 * _tmp30 + _tmp32 * _tmp34 + _tmp36 * _tmp37 - _tmp38 * _tmp39;
  const Scalar _tmp41 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp42 = _x1[3] * _x2[3];
  const Scalar _tmp43 = std::min<Scalar>(_tmp4, std::fabs(_tmp41 - _tmp42));
  const Scalar _tmp44 = 1 - std::pow(_tmp43, Scalar(2));
  const Scalar _tmp45 = std::acos(_tmp43);
  const Scalar _tmp46 =
      2 * std::min<Scalar>(0, (((-_tmp41 + _tmp42) > 0) - ((-_tmp41 + _tmp42) < 0))) + 1;
  const Scalar _tmp47 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp48 = 4 * std::pow(_tmp45, Scalar(2)) * std::pow(_tmp46, Scalar(2)) *
                        std::pow(lambdas(1, 0), Scalar(2)) / _tmp44;
  const Scalar _tmp49 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp50 = std::sqrt(Scalar(std::pow(_tmp0, Scalar(2)) * _tmp48 + _tmp11 +
                                         std::pow(_tmp47, Scalar(2)) * _tmp48 +
                                         _tmp48 * std::pow(_tmp49, Scalar(2))));
  const Scalar _tmp51 = (Scalar(1) / Scalar(2)) * _tmp50;
  const Scalar _tmp52 =
      2 * _tmp45 * _tmp46 * lambdas(1, 0) * std::sin(_tmp51) / (std::sqrt(_tmp44) * _tmp50);
  const Scalar _tmp53 = _tmp40 * _tmp52;
  const Scalar _tmp54 = _tmp15 * _tmp30;
  const Scalar _tmp55 = 4 * _tmp32;
  const Scalar _tmp56 = _tmp31 * _tmp34;
  const Scalar _tmp57 = _tmp10 * _tmp54 - _tmp16 * _tmp36 + _tmp25 * _tmp56 + _tmp38 * _tmp55;
  const Scalar _tmp58 = std::cos(_tmp51);
  const Scalar _tmp59 = _tmp12 * _tmp54 + _tmp16 * _tmp39 + _tmp17 * _tmp56 - _tmp37 * _tmp55;
  const Scalar _tmp60 = _tmp52 * _tmp59;
  const Scalar _tmp61 = -_tmp16 * _tmp55 + _tmp29 * _tmp33 - _tmp36 * _tmp38 - _tmp37 * _tmp39;
  const Scalar _tmp62 = _tmp52 * _tmp61;
  const Scalar _tmp63 = _tmp0 * _tmp53 - _tmp47 * _tmp60 + _tmp49 * _tmp62 + _tmp57 * _tmp58;
  const Scalar _tmp64 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp65 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp66 = _x0[3] * _x1[3];
  const Scalar _tmp67 = std::min<Scalar>(_tmp4, std::fabs(_tmp65 - _tmp66));
  const Scalar _tmp68 = 1 - std::pow(_tmp67, Scalar(2));
  const Scalar _tmp69 = std::acos(_tmp67);
  const Scalar _tmp70 =
      2 * std::min<Scalar>(0, (((-_tmp65 + _tmp66) > 0) - ((-_tmp65 + _tmp66) < 0))) + 1;
  const Scalar _tmp71 = 4 * std::pow(_tmp69, Scalar(2)) * std::pow(_tmp70, Scalar(2)) *
                        std::pow(lambdas(0, 0), Scalar(2)) / _tmp68;
  const Scalar _tmp72 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp73 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp74 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp64, Scalar(2)) * _tmp71 +
                                         _tmp71 * std::pow(_tmp72, Scalar(2)) +
                                         _tmp71 * std::pow(_tmp73, Scalar(2))));
  const Scalar _tmp75 = (Scalar(1) / Scalar(2)) * _tmp74;
  const Scalar _tmp76 = std::cos(_tmp75);
  const Scalar _tmp77 = _tmp52 * _tmp57;
  const Scalar _tmp78 = -_tmp0 * _tmp60 - _tmp47 * _tmp53 - _tmp49 * _tmp77 + _tmp58 * _tmp61;
  const Scalar _tmp79 =
      2 * _tmp69 * _tmp70 * lambdas(0, 0) * std::sin(_tmp75) / (std::sqrt(_tmp68) * _tmp74);
  const Scalar _tmp80 = _tmp73 * _tmp79;
  const Scalar _tmp81 = -_tmp0 * _tmp77 + _tmp40 * _tmp58 + _tmp47 * _tmp62 + _tmp49 * _tmp60;
  const Scalar _tmp82 = _tmp64 * _tmp79;
  const Scalar _tmp83 = _tmp0 * _tmp62 + _tmp47 * _tmp77 - _tmp49 * _tmp53 + _tmp58 * _tmp59;
  const Scalar _tmp84 = _tmp72 * _tmp79;
  const Scalar _tmp85 = _tmp63 * _tmp76 + _tmp78 * _tmp80 + _tmp81 * _tmp82 - _tmp83 * _tmp84;
  const Scalar _tmp86 = -_tmp63 * _tmp82 + _tmp76 * _tmp81 + _tmp78 * _tmp84 + _tmp80 * _tmp83;
  const Scalar _tmp87 = -_tmp63 * _tmp80 + _tmp76 * _tmp78 - _tmp81 * _tmp84 - _tmp82 * _tmp83;
  const Scalar _tmp88 = _tmp63 * _tmp84 + _tmp76 * _tmp83 + _tmp78 * _tmp82 - _tmp80 * _tmp81;
  const Scalar _tmp89 = _tmp85 * _x0[1] - _tmp86 * _x0[2] + _tmp87 * _x0[0] + _tmp88 * _x0[3];
  const Scalar _tmp90 = -_tmp85 * _x0[0] + _tmp86 * _x0[3] + _tmp87 * _x0[1] + _tmp88 * _x0[2];
  const Scalar _tmp91 = _tmp85 * _x0[3] + _tmp86 * _x0[0] + _tmp87 * _x0[2] - _tmp88 * _x0[1];
  const Scalar _tmp92 = -_tmp85 * _x0[2] - _tmp86 * _x0[1] + _tmp87 * _x0[3] - _tmp88 * _x0[0];
  const Scalar _tmp93 = -_tmp89 * _y[0] - _tmp90 * _y[1] - _tmp91 * _y[2];
  const Scalar _tmp94 = _tmp92 * _y[3];
  const Scalar _tmp95 = std::min<Scalar>(_tmp4, std::fabs(_tmp93 - _tmp94));
  const Scalar _tmp96 =
      2 * (2 * std::min<Scalar>(0, (((-_tmp93 + _tmp94) > 0) - ((-_tmp93 + _tmp94) < 0))) + 1) *
      std::acos(_tmp95) / std::sqrt(Scalar(1 - std::pow(_tmp95, Scalar(2))));
  const Scalar _tmp97 =
      _tmp96 * (_tmp89 * _y[3] + _tmp90 * _y[2] - _tmp91 * _y[1] - _tmp92 * _y[0]);
  const Scalar _tmp98 =
      _tmp96 * (_tmp89 * _y[1] - _tmp90 * _y[0] + _tmp91 * _y[3] - _tmp92 * _y[2]);
  const Scalar _tmp99 =
      _tmp96 * (-_tmp89 * _y[2] + _tmp90 * _y[3] + _tmp91 * _y[0] - _tmp92 * _y[1]);

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp97 * sqrt_info(0, 0) + _tmp98 * sqrt_info(0, 2) + _tmp99 * sqrt_info(0, 1);
  _res(1, 0) = _tmp97 * sqrt_info(1, 0) + _tmp98 * sqrt_info(1, 2) + _tmp99 * sqrt_info(1, 1);
  _res(2, 0) = _tmp97 * sqrt_info(2, 0) + _tmp98 * sqrt_info(2, 2) + _tmp99 * sqrt_info(2, 1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
