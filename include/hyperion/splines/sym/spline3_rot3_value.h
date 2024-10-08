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
 * Symbolic function: spline3_rot3_value
 *
 * Args:
 *     lambdas: Matrix31
 *     x0: Rot3
 *     x1: Rot3
 *     x2: Rot3
 *     x3: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Rot3
 */
template <typename Scalar>
sym::Rot3<Scalar> Spline3Rot3Value(const Eigen::Matrix<Scalar, 3, 1>& lambdas,
                                   const sym::Rot3<Scalar>& x0, const sym::Rot3<Scalar>& x1,
                                   const sym::Rot3<Scalar>& x2, const sym::Rot3<Scalar>& x3,
                                   const Scalar epsilon) {
  // Total ops: 297

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();

  // Intermediate terms (67)
  const Scalar _tmp0 = _x1[0] * _x2[2] - _x1[1] * _x2[3] - _x1[2] * _x2[0] + _x1[3] * _x2[1];
  const Scalar _tmp1 = -_x1[0] * _x2[0] - _x1[1] * _x2[1] - _x1[2] * _x2[2];
  const Scalar _tmp2 = _x1[3] * _x2[3];
  const Scalar _tmp3 = 1 - epsilon;
  const Scalar _tmp4 = std::min<Scalar>(_tmp3, std::fabs(_tmp1 - _tmp2));
  const Scalar _tmp5 = 1 - std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = std::acos(_tmp4);
  const Scalar _tmp7 =
      2 * std::min<Scalar>(0, (((-_tmp1 + _tmp2) > 0) - ((-_tmp1 + _tmp2) < 0))) + 1;
  const Scalar _tmp8 = 4 * std::pow(_tmp6, Scalar(2)) * std::pow(_tmp7, Scalar(2)) *
                       std::pow(lambdas(1, 0), Scalar(2)) / _tmp5;
  const Scalar _tmp9 = -_x1[0] * _x2[3] - _x1[1] * _x2[2] + _x1[2] * _x2[1] + _x1[3] * _x2[0];
  const Scalar _tmp10 = -_x1[0] * _x2[1] + _x1[1] * _x2[0] - _x1[2] * _x2[3] + _x1[3] * _x2[2];
  const Scalar _tmp11 = std::pow(epsilon, Scalar(2));
  const Scalar _tmp12 =
      std::sqrt(Scalar(std::pow(_tmp0, Scalar(2)) * _tmp8 + std::pow(_tmp10, Scalar(2)) * _tmp8 +
                       _tmp11 + _tmp8 * std::pow(_tmp9, Scalar(2))));
  const Scalar _tmp13 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp14 =
      _tmp6 * _tmp7 * lambdas(1, 0) * std::sin(_tmp13) / (_tmp12 * std::sqrt(_tmp5));
  const Scalar _tmp15 = _tmp0 * _tmp14;
  const Scalar _tmp16 = -_x2[0] * _x3[1] + _x2[1] * _x3[0] - _x2[2] * _x3[3] + _x2[3] * _x3[2];
  const Scalar _tmp17 = -_x2[0] * _x3[0] - _x2[1] * _x3[1] - _x2[2] * _x3[2];
  const Scalar _tmp18 = _x2[3] * _x3[3];
  const Scalar _tmp19 = std::min<Scalar>(_tmp3, std::fabs(_tmp17 - _tmp18));
  const Scalar _tmp20 = std::acos(_tmp19);
  const Scalar _tmp21 =
      2 * std::min<Scalar>(0, (((-_tmp17 + _tmp18) > 0) - ((-_tmp17 + _tmp18) < 0))) + 1;
  const Scalar _tmp22 = _x2[0] * _x3[2] - _x2[1] * _x3[3] - _x2[2] * _x3[0] + _x2[3] * _x3[1];
  const Scalar _tmp23 = 1 - std::pow(_tmp19, Scalar(2));
  const Scalar _tmp24 = 4 * std::pow(_tmp20, Scalar(2)) * std::pow(_tmp21, Scalar(2)) *
                        std::pow(lambdas(2, 0), Scalar(2)) / _tmp23;
  const Scalar _tmp25 = -_x2[0] * _x3[3] - _x2[1] * _x3[2] + _x2[2] * _x3[1] + _x2[3] * _x3[0];
  const Scalar _tmp26 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp16, Scalar(2)) * _tmp24 +
                                         std::pow(_tmp22, Scalar(2)) * _tmp24 +
                                         _tmp24 * std::pow(_tmp25, Scalar(2))));
  const Scalar _tmp27 = (Scalar(1) / Scalar(2)) * _tmp26;
  const Scalar _tmp28 =
      _tmp20 * _tmp21 * lambdas(2, 0) * std::sin(_tmp27) / (std::sqrt(_tmp23) * _tmp26);
  const Scalar _tmp29 = 4 * _tmp28;
  const Scalar _tmp30 = _tmp16 * _tmp29;
  const Scalar _tmp31 = _tmp14 * _tmp9;
  const Scalar _tmp32 = std::cos(_tmp27);
  const Scalar _tmp33 = 2 * _tmp32;
  const Scalar _tmp34 = _tmp22 * _tmp29;
  const Scalar _tmp35 = _tmp10 * _tmp14;
  const Scalar _tmp36 = _tmp25 * _tmp28;
  const Scalar _tmp37 = std::cos(_tmp13);
  const Scalar _tmp38 = 2 * _tmp37;
  const Scalar _tmp39 = _tmp15 * _tmp30 + _tmp31 * _tmp33 - _tmp34 * _tmp35 + _tmp36 * _tmp38;
  const Scalar _tmp40 = -_x0[0] * _x1[1] + _x0[1] * _x1[0] - _x0[2] * _x1[3] + _x0[3] * _x1[2];
  const Scalar _tmp41 = -_x0[0] * _x1[0] - _x0[1] * _x1[1] - _x0[2] * _x1[2];
  const Scalar _tmp42 = _x0[3] * _x1[3];
  const Scalar _tmp43 = std::min<Scalar>(_tmp3, std::fabs(_tmp41 - _tmp42));
  const Scalar _tmp44 = 1 - std::pow(_tmp43, Scalar(2));
  const Scalar _tmp45 = std::acos(_tmp43);
  const Scalar _tmp46 =
      2 * std::min<Scalar>(0, (((-_tmp41 + _tmp42) > 0) - ((-_tmp41 + _tmp42) < 0))) + 1;
  const Scalar _tmp47 = -_x0[0] * _x1[3] - _x0[1] * _x1[2] + _x0[2] * _x1[1] + _x0[3] * _x1[0];
  const Scalar _tmp48 = 4 * std::pow(_tmp45, Scalar(2)) * std::pow(_tmp46, Scalar(2)) *
                        std::pow(lambdas(0, 0), Scalar(2)) / _tmp44;
  const Scalar _tmp49 = _x0[0] * _x1[2] - _x0[1] * _x1[3] - _x0[2] * _x1[0] + _x0[3] * _x1[1];
  const Scalar _tmp50 = std::sqrt(Scalar(_tmp11 + std::pow(_tmp40, Scalar(2)) * _tmp48 +
                                         std::pow(_tmp47, Scalar(2)) * _tmp48 +
                                         _tmp48 * std::pow(_tmp49, Scalar(2))));
  const Scalar _tmp51 = (Scalar(1) / Scalar(2)) * _tmp50;
  const Scalar _tmp52 =
      2 * _tmp45 * _tmp46 * lambdas(0, 0) * std::sin(_tmp51) / (std::sqrt(_tmp44) * _tmp50);
  const Scalar _tmp53 = _tmp40 * _tmp52;
  const Scalar _tmp54 = _tmp14 * _tmp33;
  const Scalar _tmp55 = _tmp28 * _tmp38;
  const Scalar _tmp56 = 4 * _tmp36;
  const Scalar _tmp57 = _tmp10 * _tmp54 - _tmp15 * _tmp56 + _tmp16 * _tmp55 + _tmp31 * _tmp34;
  const Scalar _tmp58 = _tmp47 * _tmp52;
  const Scalar _tmp59 = -_tmp15 * _tmp34 - _tmp30 * _tmp35 - _tmp31 * _tmp56 + _tmp32 * _tmp37;
  const Scalar _tmp60 = _tmp49 * _tmp52;
  const Scalar _tmp61 = _tmp0 * _tmp54 + _tmp22 * _tmp55 - _tmp30 * _tmp31 + _tmp35 * _tmp56;
  const Scalar _tmp62 = std::cos(_tmp51);
  const Scalar _tmp63 = _tmp39 * _tmp53 - _tmp57 * _tmp58 + _tmp59 * _tmp60 + _tmp61 * _tmp62;
  const Scalar _tmp64 = _tmp39 * _tmp62 - _tmp53 * _tmp61 + _tmp57 * _tmp60 + _tmp58 * _tmp59;
  const Scalar _tmp65 = -_tmp39 * _tmp58 - _tmp53 * _tmp57 + _tmp59 * _tmp62 - _tmp60 * _tmp61;
  const Scalar _tmp66 = -_tmp39 * _tmp60 + _tmp53 * _tmp59 + _tmp57 * _tmp62 + _tmp58 * _tmp61;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res[0] = -_tmp63 * _x0[2] + _tmp64 * _x0[3] + _tmp65 * _x0[0] + _tmp66 * _x0[1];
  _res[1] = _tmp63 * _x0[3] + _tmp64 * _x0[2] + _tmp65 * _x0[1] - _tmp66 * _x0[0];
  _res[2] = _tmp63 * _x0[0] - _tmp64 * _x0[1] + _tmp65 * _x0[2] + _tmp66 * _x0[3];
  _res[3] = -_tmp63 * _x0[1] - _tmp64 * _x0[0] + _tmp65 * _x0[3] - _tmp66 * _x0[2];

  return sym::Rot3<Scalar>(_res);
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
