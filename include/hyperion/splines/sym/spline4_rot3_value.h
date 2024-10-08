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
 * Symbolic function: spline4_rot3_value
 *
 * Args:
 *     lambdas: Matrix41
 *     x0: Rot3
 *     x1: Rot3
 *     x2: Rot3
 *     x3: Rot3
 *     x4: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Rot3
 */
template <typename Scalar>
sym::Rot3<Scalar> Spline4Rot3Value(const Eigen::Matrix<Scalar, 4, 1>& lambdas,
                                   const sym::Rot3<Scalar>& x0, const sym::Rot3<Scalar>& x1,
                                   const sym::Rot3<Scalar>& x2, const sym::Rot3<Scalar>& x3,
                                   const sym::Rot3<Scalar>& x4, const Scalar epsilon) {
  // Total ops: 398

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x4 = x4.Data();

  // Intermediate terms (92)
  const Scalar _tmp0 = _x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1];
  const Scalar _tmp1 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp2 = _x2[3] * _x3[3];
  const Scalar _tmp3 = 1 - epsilon;
  const Scalar _tmp4 = std::min<Scalar>(_tmp3, std::fabs(_tmp1 - _tmp2));
  const Scalar _tmp5 = std::acos(_tmp4);
  const Scalar _tmp6 =
      2 * std::min<Scalar>(0, (((-_tmp1 + _tmp2) > 0) - ((-_tmp1 + _tmp2) < 0))) + 1;
  const Scalar _tmp7 = 1 - std::pow(_tmp4, Scalar(2));
  const Scalar _tmp8 = 4 * std::pow(_tmp5, Scalar(2)) * std::pow(_tmp6, Scalar(2)) *
                       std::pow(lambdas(2, 0), Scalar(2)) / _tmp7;
  const Scalar _tmp9 = -_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2];
  const Scalar _tmp10 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp11 = -_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0];
  const Scalar _tmp12 =
      std::sqrt(Scalar(std::pow(_tmp0, Scalar(2)) * _tmp8 + _tmp10 +
                       std::pow(_tmp11, Scalar(2)) * _tmp8 + _tmp8 * std::pow(_tmp9, Scalar(2))));
  const Scalar _tmp13 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp14 =
      _tmp5 * _tmp6 * lambdas(2, 0) * std::sin(_tmp13) / (_tmp12 * std::sqrt(_tmp7));
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = -_x3[0] * _x4[3] - _x3[1] * _x4[2] + _x3[2] * _x4[1] + _x3[3] * _x4[0];
  const Scalar _tmp17 = -_x3[0] * _x4[0] - _x3[1] * _x4[1] - _x3[2] * _x4[2];
  const Scalar _tmp18 = _x3[3] * _x4[3];
  const Scalar _tmp19 = std::min<Scalar>(_tmp3, std::fabs(_tmp17 - _tmp18));
  const Scalar _tmp20 = std::acos(_tmp19);
  const Scalar _tmp21 =
      2 * std::min<Scalar>(0, (((-_tmp17 + _tmp18) > 0) - ((-_tmp17 + _tmp18) < 0))) + 1;
  const Scalar _tmp22 = 1 - std::pow(_tmp19, Scalar(2));
  const Scalar _tmp23 = 4 * std::pow(_tmp20, Scalar(2)) * std::pow(_tmp21, Scalar(2)) *
                        std::pow(lambdas(3, 0), Scalar(2)) / _tmp22;
  const Scalar _tmp24 = -_x3[0] * _x4[1] + _x3[1] * _x4[0] - _x3[2] * _x4[3] + _x3[3] * _x4[2];
  const Scalar _tmp25 = _x3[0] * _x4[2] - _x3[1] * _x4[3] - _x3[2] * _x4[0] + _x3[3] * _x4[1];
  const Scalar _tmp26 = std::sqrt(Scalar(_tmp10 + std::pow(_tmp16, Scalar(2)) * _tmp23 +
                                         _tmp23 * std::pow(_tmp24, Scalar(2)) +
                                         _tmp23 * std::pow(_tmp25, Scalar(2))));
  const Scalar _tmp27 = (Scalar(1) / Scalar(2)) * _tmp26;
  const Scalar _tmp28 = std::cos(_tmp27);
  const Scalar _tmp29 = 2 * _tmp28;
  const Scalar _tmp30 =
      _tmp20 * _tmp21 * lambdas(3, 0) * std::sin(_tmp27) / (std::sqrt(_tmp22) * _tmp26);
  const Scalar _tmp31 = std::cos(_tmp13);
  const Scalar _tmp32 = 2 * _tmp31;
  const Scalar _tmp33 = _tmp30 * _tmp32;
  const Scalar _tmp34 = 4 * _tmp30;
  const Scalar _tmp35 = _tmp16 * _tmp34;
  const Scalar _tmp36 = _tmp14 * _tmp9;
  const Scalar _tmp37 = _tmp24 * _tmp30;
  const Scalar _tmp38 = 4 * _tmp37;
  const Scalar _tmp39 = _tmp14 * _tmp38;
  const Scalar _tmp40 = -_tmp11 * _tmp39 + _tmp15 * _tmp29 + _tmp25 * _tmp33 + _tmp35 * _tmp36;
  const Scalar _tmp41 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp42 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp43 = _x1[3] * _x2[3];
  const Scalar _tmp44 = std::min<Scalar>(_tmp3, std::fabs(_tmp42 - _tmp43));
  const Scalar _tmp45 = 1 - std::pow(_tmp44, Scalar(2));
  const Scalar _tmp46 = std::acos(_tmp44);
  const Scalar _tmp47 =
      2 * std::min<Scalar>(0, (((-_tmp42 + _tmp43) > 0) - ((-_tmp42 + _tmp43) < 0))) + 1;
  const Scalar _tmp48 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp49 = 4 * std::pow(_tmp46, Scalar(2)) * std::pow(_tmp47, Scalar(2)) *
                        std::pow(lambdas(1, 0), Scalar(2)) / _tmp45;
  const Scalar _tmp50 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp51 = std::sqrt(Scalar(_tmp10 + std::pow(_tmp41, Scalar(2)) * _tmp49 +
                                         std::pow(_tmp48, Scalar(2)) * _tmp49 +
                                         _tmp49 * std::pow(_tmp50, Scalar(2))));
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) * _tmp51;
  const Scalar _tmp53 =
      2 * _tmp46 * _tmp47 * lambdas(1, 0) * std::sin(_tmp52) / (std::sqrt(_tmp45) * _tmp51);
  const Scalar _tmp54 = _tmp41 * _tmp53;
  const Scalar _tmp55 = _tmp14 * _tmp29;
  const Scalar _tmp56 = _tmp25 * _tmp34;
  const Scalar _tmp57 = _tmp11 * _tmp14;
  const Scalar _tmp58 = -_tmp15 * _tmp35 + _tmp32 * _tmp37 + _tmp55 * _tmp9 + _tmp56 * _tmp57;
  const Scalar _tmp59 = std::cos(_tmp52);
  const Scalar _tmp60 = _tmp11 * _tmp55 + _tmp15 * _tmp38 + _tmp16 * _tmp33 - _tmp36 * _tmp56;
  const Scalar _tmp61 = _tmp53 * _tmp60;
  const Scalar _tmp62 = -_tmp15 * _tmp56 + _tmp28 * _tmp31 - _tmp35 * _tmp57 - _tmp39 * _tmp9;
  const Scalar _tmp63 = _tmp53 * _tmp62;
  const Scalar _tmp64 = _tmp40 * _tmp54 - _tmp48 * _tmp61 + _tmp50 * _tmp63 + _tmp58 * _tmp59;
  const Scalar _tmp65 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp66 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp67 = _x0[3] * _x1[3];
  const Scalar _tmp68 = std::min<Scalar>(_tmp3, std::fabs(_tmp66 - _tmp67));
  const Scalar _tmp69 = 1 - std::pow(_tmp68, Scalar(2));
  const Scalar _tmp70 = std::acos(_tmp68);
  const Scalar _tmp71 =
      2 * std::min<Scalar>(0, (((-_tmp66 + _tmp67) > 0) - ((-_tmp66 + _tmp67) < 0))) + 1;
  const Scalar _tmp72 = 4 * std::pow(_tmp70, Scalar(2)) * std::pow(_tmp71, Scalar(2)) *
                        std::pow(lambdas(0, 0), Scalar(2)) / _tmp69;
  const Scalar _tmp73 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp74 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp75 = std::sqrt(Scalar(_tmp10 + std::pow(_tmp65, Scalar(2)) * _tmp72 +
                                         _tmp72 * std::pow(_tmp73, Scalar(2)) +
                                         _tmp72 * std::pow(_tmp74, Scalar(2))));
  const Scalar _tmp76 = (Scalar(1) / Scalar(2)) * _tmp75;
  const Scalar _tmp77 = std::cos(_tmp76);
  const Scalar _tmp78 = _tmp53 * _tmp58;
  const Scalar _tmp79 = _tmp40 * _tmp53;
  const Scalar _tmp80 = -_tmp48 * _tmp79 - _tmp50 * _tmp78 - _tmp54 * _tmp60 + _tmp59 * _tmp62;
  const Scalar _tmp81 =
      2 * _tmp70 * _tmp71 * lambdas(0, 0) * std::sin(_tmp76) / (std::sqrt(_tmp69) * _tmp75);
  const Scalar _tmp82 = _tmp80 * _tmp81;
  const Scalar _tmp83 = _tmp40 * _tmp59 + _tmp48 * _tmp63 + _tmp50 * _tmp61 - _tmp54 * _tmp58;
  const Scalar _tmp84 = _tmp65 * _tmp81;
  const Scalar _tmp85 = _tmp48 * _tmp78 - _tmp50 * _tmp79 + _tmp54 * _tmp62 + _tmp59 * _tmp60;
  const Scalar _tmp86 = _tmp73 * _tmp81;
  const Scalar _tmp87 = _tmp64 * _tmp77 + _tmp74 * _tmp82 + _tmp83 * _tmp84 - _tmp85 * _tmp86;
  const Scalar _tmp88 = _tmp74 * _tmp81;
  const Scalar _tmp89 = -_tmp64 * _tmp84 + _tmp73 * _tmp82 + _tmp77 * _tmp83 + _tmp85 * _tmp88;
  const Scalar _tmp90 = -_tmp64 * _tmp88 + _tmp77 * _tmp80 - _tmp83 * _tmp86 - _tmp84 * _tmp85;
  const Scalar _tmp91 = _tmp64 * _tmp86 + _tmp65 * _tmp82 + _tmp77 * _tmp85 - _tmp83 * _tmp88;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res[0] = _tmp87 * _x0[1] - _tmp89 * _x0[2] + _tmp90 * _x0[0] + _tmp91 * _x0[3];
  _res[1] = -_tmp87 * _x0[0] + _tmp89 * _x0[3] + _tmp90 * _x0[1] + _tmp91 * _x0[2];
  _res[2] = _tmp87 * _x0[3] + _tmp89 * _x0[0] + _tmp90 * _x0[2] - _tmp91 * _x0[1];
  _res[3] = -_tmp87 * _x0[2] - _tmp89 * _x0[1] + _tmp90 * _x0[3] - _tmp91 * _x0[0];

  return sym::Rot3<Scalar>(_res);
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
