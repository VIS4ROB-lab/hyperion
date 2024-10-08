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
 * Symbolic function: rot3_delta_factor
 *
 * Args:
 *     x: Rot3
 *     y: Rot3
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 *     res_D_x: (3x3) jacobian of res (3) wrt arg x (3)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Rot3DeltaFactorWithJacobian0(
    const sym::Rot3<Scalar>& x, const sym::Rot3<Scalar>& y,
    const Eigen::Matrix<Scalar, 3, 3>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr) {
  // Total ops: 313

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y = y.Data();

  // Intermediate terms (93)
  const Scalar _tmp0 = _x[3] * _y[3];
  const Scalar _tmp1 = _x[1] * _y[1];
  const Scalar _tmp2 = _x[0] * _y[0];
  const Scalar _tmp3 = _x[2] * _y[2];
  const Scalar _tmp4 = _tmp0 + _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp5 = 2 * std::min<Scalar>(0, (((_tmp4) > 0) - ((_tmp4) < 0))) + 1;
  const Scalar _tmp6 = 2 * _tmp5;
  const Scalar _tmp7 = 1 - epsilon;
  const Scalar _tmp8 = std::min<Scalar>(_tmp7, std::fabs(_tmp4));
  const Scalar _tmp9 = std::acos(_tmp8) / std::sqrt(Scalar(1 - std::pow(_tmp8, Scalar(2))));
  const Scalar _tmp10 = _tmp6 * _tmp9;
  const Scalar _tmp11 = _x[3] * _y[2];
  const Scalar _tmp12 = _x[1] * _y[0];
  const Scalar _tmp13 = _x[0] * _y[1];
  const Scalar _tmp14 = _x[2] * _y[3];
  const Scalar _tmp15 = _tmp11 + _tmp12 - _tmp13 - _tmp14;
  const Scalar _tmp16 = _tmp15 * sqrt_info(0, 2);
  const Scalar _tmp17 = _x[3] * _y[1];
  const Scalar _tmp18 = _x[1] * _y[3];
  const Scalar _tmp19 = _x[0] * _y[2];
  const Scalar _tmp20 = _x[2] * _y[0];
  const Scalar _tmp21 = _tmp17 - _tmp18 + _tmp19 - _tmp20;
  const Scalar _tmp22 = _tmp10 * _tmp21;
  const Scalar _tmp23 = _x[3] * _y[0];
  const Scalar _tmp24 = _x[1] * _y[2];
  const Scalar _tmp25 = _x[0] * _y[3];
  const Scalar _tmp26 = _x[2] * _y[1];
  const Scalar _tmp27 = _tmp23 - _tmp24 - _tmp25 + _tmp26;
  const Scalar _tmp28 = _tmp27 * sqrt_info(0, 0);
  const Scalar _tmp29 = _tmp6 * sqrt_info(1, 2);
  const Scalar _tmp30 = _tmp15 * _tmp9;
  const Scalar _tmp31 = _tmp10 * _tmp27;
  const Scalar _tmp32 = _tmp6 * sqrt_info(2, 2);
  const Scalar _tmp33 = (Scalar(1) / Scalar(2)) * _tmp23;
  const Scalar _tmp34 = (Scalar(1) / Scalar(2)) * _tmp24;
  const Scalar _tmp35 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp36 = (Scalar(1) / Scalar(2)) * _tmp26;
  const Scalar _tmp37 = _tmp33 - _tmp34 - _tmp35 + _tmp36;
  const Scalar _tmp38 = _tmp0 + _tmp1 + _tmp2 + _tmp3;
  const Scalar _tmp39 = std::fabs(_tmp38);
  const Scalar _tmp40 = std::min<Scalar>(_tmp39, _tmp7);
  const Scalar _tmp41 = std::acos(_tmp40);
  const Scalar _tmp42 = 1 - std::pow(_tmp40, Scalar(2));
  const Scalar _tmp43 = _tmp5 * ((((-_tmp39 + _tmp7) > 0) - ((-_tmp39 + _tmp7) < 0)) + 1) *
                        (((_tmp38) > 0) - ((_tmp38) < 0));
  const Scalar _tmp44 = _tmp40 * _tmp41 * _tmp43 / (_tmp42 * std::sqrt(_tmp42));
  const Scalar _tmp45 = _tmp37 * _tmp44;
  const Scalar _tmp46 = _tmp43 / _tmp42;
  const Scalar _tmp47 = _tmp37 * _tmp46;
  const Scalar _tmp48 = _tmp21 * _tmp46;
  const Scalar _tmp49 = _tmp37 * _tmp48;
  const Scalar _tmp50 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp51 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp52 = (Scalar(1) / Scalar(2)) * _tmp19;
  const Scalar _tmp53 = (Scalar(1) / Scalar(2)) * _tmp20;
  const Scalar _tmp54 = _tmp41 / std::sqrt(_tmp42);
  const Scalar _tmp55 = _tmp54 * (-_tmp50 + _tmp51 - _tmp52 + _tmp53);
  const Scalar _tmp56 = (Scalar(1) / Scalar(2)) * _tmp11;
  const Scalar _tmp57 = (Scalar(1) / Scalar(2)) * _tmp12;
  const Scalar _tmp58 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp59 = (Scalar(1) / Scalar(2)) * _tmp14;
  const Scalar _tmp60 = _tmp56 + _tmp57 - _tmp58 - _tmp59;
  const Scalar _tmp61 = _tmp54 * _tmp6;
  const Scalar _tmp62 = _tmp60 * _tmp61;
  const Scalar _tmp63 = _tmp21 * _tmp45;
  const Scalar _tmp64 = -Scalar(1) / Scalar(2) * _tmp0 - Scalar(1) / Scalar(2) * _tmp1 -
                        Scalar(1) / Scalar(2) * _tmp2 - Scalar(1) / Scalar(2) * _tmp3;
  const Scalar _tmp65 = _tmp61 * _tmp64;
  const Scalar _tmp66 = _tmp28 * _tmp46;
  const Scalar _tmp67 = _tmp15 * sqrt_info(1, 2);
  const Scalar _tmp68 = _tmp15 * _tmp47;
  const Scalar _tmp69 = _tmp27 * _tmp45;
  const Scalar _tmp70 = _tmp60 * sqrt_info(1, 1);
  const Scalar _tmp71 = _tmp27 * _tmp46;
  const Scalar _tmp72 = _tmp71 * sqrt_info(1, 0);
  const Scalar _tmp73 = _tmp15 * sqrt_info(2, 2);
  const Scalar _tmp74 = _tmp71 * sqrt_info(2, 0);
  const Scalar _tmp75 = _tmp50 - _tmp51 + _tmp52 - _tmp53;
  const Scalar _tmp76 = _tmp44 * _tmp75;
  const Scalar _tmp77 = _tmp46 * _tmp75;
  const Scalar _tmp78 = _tmp21 * _tmp77;
  const Scalar _tmp79 = _tmp61 * (-_tmp56 - _tmp57 + _tmp58 + _tmp59);
  const Scalar _tmp80 = _tmp21 * _tmp76;
  const Scalar _tmp81 = _tmp29 * _tmp54;
  const Scalar _tmp82 = _tmp27 * _tmp76;
  const Scalar _tmp83 = _tmp15 * _tmp77;
  const Scalar _tmp84 = _tmp32 * _tmp54;
  const Scalar _tmp85 = _tmp21 * _tmp44;
  const Scalar _tmp86 = _tmp60 * _tmp85;
  const Scalar _tmp87 = _tmp44 * _tmp60;
  const Scalar _tmp88 = _tmp46 * _tmp60;
  const Scalar _tmp89 = _tmp61 * (-_tmp33 + _tmp34 + _tmp35 - _tmp36);
  const Scalar _tmp90 = _tmp48 * _tmp60;
  const Scalar _tmp91 = _tmp61 * _tmp75;
  const Scalar _tmp92 = _tmp27 * _tmp87;

  // Output terms (2)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp10 * _tmp16 + _tmp10 * _tmp28 + _tmp22 * sqrt_info(0, 1);
  _res(1, 0) = _tmp22 * sqrt_info(1, 1) + _tmp29 * _tmp30 + _tmp31 * sqrt_info(1, 0);
  _res(2, 0) = _tmp22 * sqrt_info(2, 1) + _tmp30 * _tmp32 + _tmp31 * sqrt_info(2, 0);

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 3, 3>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = _tmp16 * _tmp45 - _tmp16 * _tmp47 + _tmp28 * _tmp45 - _tmp37 * _tmp66 -
                     _tmp49 * sqrt_info(0, 1) + _tmp55 * _tmp6 * sqrt_info(0, 2) +
                     _tmp62 * sqrt_info(0, 1) + _tmp63 * sqrt_info(0, 1) + _tmp65 * sqrt_info(0, 0);
    _res_D_x(1, 0) = _tmp29 * _tmp55 - _tmp37 * _tmp72 + _tmp45 * _tmp67 -
                     _tmp49 * sqrt_info(1, 1) + _tmp61 * _tmp70 + _tmp63 * sqrt_info(1, 1) +
                     _tmp65 * sqrt_info(1, 0) - _tmp68 * sqrt_info(1, 2) + _tmp69 * sqrt_info(1, 0);
    _res_D_x(2, 0) = _tmp32 * _tmp55 - _tmp37 * _tmp74 + _tmp45 * _tmp73 -
                     _tmp49 * sqrt_info(2, 1) + _tmp62 * sqrt_info(2, 1) +
                     _tmp63 * sqrt_info(2, 1) + _tmp65 * sqrt_info(2, 0) -
                     _tmp68 * sqrt_info(2, 2) + _tmp69 * sqrt_info(2, 0);
    _res_D_x(0, 1) = _tmp16 * _tmp76 - _tmp16 * _tmp77 + _tmp28 * _tmp76 +
                     _tmp37 * _tmp61 * sqrt_info(0, 2) + _tmp65 * sqrt_info(0, 1) -
                     _tmp66 * _tmp75 - _tmp78 * sqrt_info(0, 1) + _tmp79 * sqrt_info(0, 0) +
                     _tmp80 * sqrt_info(0, 1);
    _res_D_x(1, 1) = _tmp37 * _tmp81 + _tmp65 * sqrt_info(1, 1) + _tmp67 * _tmp76 -
                     _tmp72 * _tmp75 - _tmp78 * sqrt_info(1, 1) + _tmp79 * sqrt_info(1, 0) +
                     _tmp80 * sqrt_info(1, 1) + _tmp82 * sqrt_info(1, 0) - _tmp83 * sqrt_info(1, 2);
    _res_D_x(2, 1) = _tmp37 * _tmp84 + _tmp65 * sqrt_info(2, 1) + _tmp73 * _tmp76 -
                     _tmp74 * _tmp75 - _tmp78 * sqrt_info(2, 1) + _tmp79 * sqrt_info(2, 0) +
                     _tmp80 * sqrt_info(2, 1) + _tmp82 * sqrt_info(2, 0) - _tmp83 * sqrt_info(2, 2);
    _res_D_x(0, 2) = _tmp16 * _tmp87 - _tmp16 * _tmp88 + _tmp28 * _tmp87 - _tmp60 * _tmp66 +
                     _tmp65 * sqrt_info(0, 2) + _tmp86 * sqrt_info(0, 1) +
                     _tmp89 * sqrt_info(0, 1) - _tmp90 * sqrt_info(0, 1) + _tmp91 * sqrt_info(0, 0);
    _res_D_x(1, 2) = -_tmp48 * _tmp70 - _tmp60 * _tmp72 + _tmp64 * _tmp81 + _tmp67 * _tmp87 -
                     _tmp67 * _tmp88 + _tmp70 * _tmp85 + _tmp89 * sqrt_info(1, 1) +
                     _tmp91 * sqrt_info(1, 0) + _tmp92 * sqrt_info(1, 0);
    _res_D_x(2, 2) = -_tmp60 * _tmp74 + _tmp64 * _tmp84 + _tmp73 * _tmp87 - _tmp73 * _tmp88 +
                     _tmp86 * sqrt_info(2, 1) + _tmp89 * sqrt_info(2, 1) -
                     _tmp90 * sqrt_info(2, 1) + _tmp91 * sqrt_info(2, 0) + _tmp92 * sqrt_info(2, 0);
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_hyperion
