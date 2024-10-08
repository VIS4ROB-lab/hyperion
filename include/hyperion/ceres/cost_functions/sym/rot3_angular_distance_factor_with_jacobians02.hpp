// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/rot3.h>

namespace sym_ceres {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: rot3_angular_distance_factor
 *
 * Args:
 *     x: Rot3
 *     x_d_y: Matrix11
 *     y: Rot3
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 *     res_D_x: (1x4) jacobian (result_dim x storage_dim) of res (1) wrt arg x (4) (row-major)
 *     res_D_y: (1x4) jacobian (result_dim x storage_dim) of res (1) wrt arg y (4) (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Rot3AngularDistanceFactorWithJacobians02(
    const sym::Rot3<Scalar>& x, const Eigen::Matrix<Scalar, 1, 1>& x_d_y,
    const sym::Rot3<Scalar>& y, const Eigen::Matrix<Scalar, 1, 1>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x = nullptr, Scalar* const res_D_y = nullptr) {
  // Total ops: 283

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x = x.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _y = y.Data();

  // Intermediate terms (90)
  const Scalar _tmp0 = _x[3] * _y[0];
  const Scalar _tmp1 = _x[1] * _y[2];
  const Scalar _tmp2 = _x[0] * _y[3];
  const Scalar _tmp3 = _x[2] * _y[1];
  const Scalar _tmp4 = _tmp0 - _tmp1 - _tmp2 + _tmp3;
  const Scalar _tmp5 = _x[3] * _y[3];
  const Scalar _tmp6 = _x[1] * _y[1];
  const Scalar _tmp7 = _x[0] * _y[0];
  const Scalar _tmp8 = _x[2] * _y[2];
  const Scalar _tmp9 = _tmp5 + _tmp6 + _tmp7 + _tmp8;
  const Scalar _tmp10 =
      std::pow(Scalar(2 * std::min<Scalar>(0, (((_tmp9) > 0) - ((_tmp9) < 0))) + 1), Scalar(2));
  const Scalar _tmp11 = 4 * _tmp10;
  const Scalar _tmp12 = _tmp11 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp13 = 1 - epsilon;
  const Scalar _tmp14 = std::min<Scalar>(_tmp13, std::fabs(_tmp9));
  const Scalar _tmp15 =
      std::pow(Scalar(std::acos(_tmp14)), Scalar(2)) / (1 - std::pow(_tmp14, Scalar(2)));
  const Scalar _tmp16 = _x[3] * _y[1];
  const Scalar _tmp17 = _x[1] * _y[3];
  const Scalar _tmp18 = _x[0] * _y[2];
  const Scalar _tmp19 = _x[2] * _y[0];
  const Scalar _tmp20 = _tmp16 - _tmp17 + _tmp18 - _tmp19;
  const Scalar _tmp21 = _tmp11 * std::pow(_tmp20, Scalar(2));
  const Scalar _tmp22 = _x[3] * _y[2];
  const Scalar _tmp23 = _x[1] * _y[0];
  const Scalar _tmp24 = _x[0] * _y[1];
  const Scalar _tmp25 = _x[2] * _y[3];
  const Scalar _tmp26 = _tmp22 + _tmp23 - _tmp24 - _tmp25;
  const Scalar _tmp27 = _tmp11 * std::pow(_tmp26, Scalar(2));
  const Scalar _tmp28 = (Scalar(1) / Scalar(2)) * _tmp16;
  const Scalar _tmp29 = (Scalar(1) / Scalar(2)) * _tmp17;
  const Scalar _tmp30 = (Scalar(1) / Scalar(2)) * _tmp18;
  const Scalar _tmp31 = (Scalar(1) / Scalar(2)) * _tmp19;
  const Scalar _tmp32 = _tmp28 - _tmp29 + _tmp30 - _tmp31;
  const Scalar _tmp33 = _tmp5 + _tmp6 + _tmp7 + _tmp8;
  const Scalar _tmp34 = std::fabs(_tmp33);
  const Scalar _tmp35 =
      ((((_tmp13 - _tmp34) > 0) - ((_tmp13 - _tmp34) < 0)) + 1) * (((_tmp33) > 0) - ((_tmp33) < 0));
  const Scalar _tmp36 = std::min<Scalar>(_tmp13, _tmp34);
  const Scalar _tmp37 = std::acos(_tmp36);
  const Scalar _tmp38 = std::pow(_tmp37, Scalar(2));
  const Scalar _tmp39 = 1 - std::pow(_tmp36, Scalar(2));
  const Scalar _tmp40 = _tmp36 * _tmp38 / std::pow(_tmp39, Scalar(2));
  const Scalar _tmp41 = _tmp35 * _tmp40;
  const Scalar _tmp42 = _tmp27 * _tmp41;
  const Scalar _tmp43 = _tmp37 / (_tmp39 * std::sqrt(_tmp39));
  const Scalar _tmp44 = _tmp35 * _tmp43;
  const Scalar _tmp45 = _tmp12 * _tmp44;
  const Scalar _tmp46 = _tmp21 * _tmp35;
  const Scalar _tmp47 = _tmp40 * _tmp46;
  const Scalar _tmp48 = (Scalar(1) / Scalar(2)) * _tmp5;
  const Scalar _tmp49 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp50 = (Scalar(1) / Scalar(2)) * _tmp7;
  const Scalar _tmp51 = (Scalar(1) / Scalar(2)) * _tmp8;
  const Scalar _tmp52 = -_tmp48 - _tmp49 - _tmp50 - _tmp51;
  const Scalar _tmp53 = _tmp38 / _tmp39;
  const Scalar _tmp54 = 8 * _tmp10 * _tmp53;
  const Scalar _tmp55 = _tmp20 * _tmp54;
  const Scalar _tmp56 = _tmp12 * _tmp41;
  const Scalar _tmp57 = _tmp43 * _tmp46;
  const Scalar _tmp58 = _tmp27 * _tmp44;
  const Scalar _tmp59 = (Scalar(1) / Scalar(2)) * _tmp0;
  const Scalar _tmp60 = (Scalar(1) / Scalar(2)) * _tmp1;
  const Scalar _tmp61 = (Scalar(1) / Scalar(2)) * _tmp2;
  const Scalar _tmp62 = (Scalar(1) / Scalar(2)) * _tmp3;
  const Scalar _tmp63 = _tmp59 - _tmp60 - _tmp61 + _tmp62;
  const Scalar _tmp64 = _tmp26 * _tmp54;
  const Scalar _tmp65 = (Scalar(1) / Scalar(2)) * _tmp22;
  const Scalar _tmp66 = (Scalar(1) / Scalar(2)) * _tmp23;
  const Scalar _tmp67 = (Scalar(1) / Scalar(2)) * _tmp24;
  const Scalar _tmp68 = (Scalar(1) / Scalar(2)) * _tmp25;
  const Scalar _tmp69 = -_tmp65 - _tmp66 + _tmp67 + _tmp68;
  const Scalar _tmp70 = _tmp4 * _tmp54;
  const Scalar _tmp71 = _tmp63 * _tmp64 + _tmp69 * _tmp70;
  const Scalar _tmp72 = _tmp32 * _tmp42 - _tmp32 * _tmp45 + _tmp32 * _tmp47 + _tmp32 * _tmp56 -
                        _tmp32 * _tmp57 - _tmp32 * _tmp58 + _tmp52 * _tmp55 + _tmp71;
  const Scalar _tmp73 =
      sqrt_info(0, 0) /
      std::sqrt(Scalar(_tmp12 * _tmp53 + _tmp21 * _tmp53 + _tmp27 * _tmp53 + epsilon));
  const Scalar _tmp74 = _tmp72 * _tmp73;
  const Scalar _tmp75 = _tmp65 + _tmp66 - _tmp67 - _tmp68;
  const Scalar _tmp76 = -_tmp59 + _tmp60 + _tmp61 - _tmp62;
  const Scalar _tmp77 = _tmp32 * _tmp70 + _tmp55 * _tmp76;
  const Scalar _tmp78 = _tmp42 * _tmp75 - _tmp45 * _tmp75 + _tmp47 * _tmp75 + _tmp52 * _tmp64 +
                        _tmp56 * _tmp75 - _tmp57 * _tmp75 - _tmp58 * _tmp75 + _tmp77;
  const Scalar _tmp79 = _tmp73 * _tmp78;
  const Scalar _tmp80 = _tmp46 * _tmp63;
  const Scalar _tmp81 = -_tmp28 + _tmp29 - _tmp30 + _tmp31;
  const Scalar _tmp82 = _tmp55 * _tmp75 + _tmp64 * _tmp81;
  const Scalar _tmp83 = _tmp40 * _tmp80 + _tmp42 * _tmp63 - _tmp43 * _tmp80 - _tmp45 * _tmp63 +
                        _tmp52 * _tmp70 + _tmp56 * _tmp63 - _tmp58 * _tmp63 + _tmp82;
  const Scalar _tmp84 = _tmp73 * _tmp83;
  const Scalar _tmp85 = _tmp73 * _x[0];
  const Scalar _tmp86 = _tmp48 + _tmp49 + _tmp50 + _tmp51;
  const Scalar _tmp87 =
      _tmp73 * (_tmp42 * _tmp76 - _tmp45 * _tmp76 + _tmp47 * _tmp76 + _tmp56 * _tmp76 -
                _tmp57 * _tmp76 - _tmp58 * _tmp76 + _tmp70 * _tmp86 + _tmp82);
  const Scalar _tmp88 =
      _tmp73 * (_tmp42 * _tmp81 - _tmp45 * _tmp81 + _tmp47 * _tmp81 + _tmp55 * _tmp86 +
                _tmp56 * _tmp81 - _tmp57 * _tmp81 - _tmp58 * _tmp81 + _tmp71);
  const Scalar _tmp89 =
      _tmp73 * (_tmp42 * _tmp69 - _tmp45 * _tmp69 + _tmp47 * _tmp69 + _tmp56 * _tmp69 -
                _tmp57 * _tmp69 - _tmp58 * _tmp69 + _tmp64 * _tmp86 + _tmp77);

  // Output terms (3)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) *
               (-x_d_y(0, 0) +
                std::sqrt(Scalar(_tmp12 * _tmp15 + _tmp15 * _tmp21 + _tmp15 * _tmp27 + epsilon)));

  if (res_D_x != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 4>> _res_D_x{res_D_x};

    _res_D_x(0, 0) = -_tmp74 * _x[2] + _tmp79 * _x[1] + _tmp84 * _x[3];
    _res_D_x(0, 1) = _tmp74 * _x[3] - _tmp78 * _tmp85 + _tmp84 * _x[2];
    _res_D_x(0, 2) = _tmp72 * _tmp85 + _tmp79 * _x[3] - _tmp84 * _x[1];
    _res_D_x(0, 3) = -_tmp74 * _x[1] - _tmp79 * _x[2] - _tmp83 * _tmp85;
  }

  if (res_D_y != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 4>> _res_D_y{res_D_y};

    _res_D_y(0, 0) = _tmp87 * _y[3] - _tmp88 * _y[2] + _tmp89 * _y[1];
    _res_D_y(0, 1) = _tmp87 * _y[2] + _tmp88 * _y[3] - _tmp89 * _y[0];
    _res_D_y(0, 2) = -_tmp87 * _y[1] + _tmp88 * _y[0] + _tmp89 * _y[3];
    _res_D_y(0, 3) = -_tmp87 * _y[0] - _tmp88 * _y[1] - _tmp89 * _y[2];
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
