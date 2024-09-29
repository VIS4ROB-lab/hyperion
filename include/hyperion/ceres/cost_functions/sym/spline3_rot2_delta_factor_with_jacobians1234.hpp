// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/rot2.h>

namespace sym_ceres {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline3_rot2_delta_factor
 *
 * Args:
 *     lambdas: Matrix31
 *     x0: Rot2
 *     x1: Rot2
 *     x2: Rot2
 *     x3: Rot2
 *     y: Rot2
 *     sqrt_info: Matrix11
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix11
 *     res_D_x0: (1x2) jacobian (result_dim x storage_dim) of res (1) wrt arg x0 (2) (row-major)
 *     res_D_x1: (1x2) jacobian (result_dim x storage_dim) of res (1) wrt arg x1 (2) (row-major)
 *     res_D_x2: (1x2) jacobian (result_dim x storage_dim) of res (1) wrt arg x2 (2) (row-major)
 *     res_D_x3: (1x2) jacobian (result_dim x storage_dim) of res (1) wrt arg x3 (2) (row-major)
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 1, 1> Spline3Rot2DeltaFactorWithJacobians1234(
    const Eigen::Matrix<Scalar, 3, 1>& lambdas, const sym::Rot2<Scalar>& x0,
    const sym::Rot2<Scalar>& x1, const sym::Rot2<Scalar>& x2, const sym::Rot2<Scalar>& x3,
    const sym::Rot2<Scalar>& y, const Eigen::Matrix<Scalar, 1, 1>& sqrt_info, const Scalar epsilon,
    Scalar* const res_D_x0 = nullptr, Scalar* const res_D_x1 = nullptr,
    Scalar* const res_D_x2 = nullptr, Scalar* const res_D_x3 = nullptr) {
  // Total ops: 279

  // Input arrays
  const Eigen::Matrix<Scalar, 2, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 2, 1>& _y = y.Data();

  // Intermediate terms (106)
  const Scalar _tmp0 = _x2[1] * _x3[0];
  const Scalar _tmp1 = _x2[0] * _x3[1];
  const Scalar _tmp2 = -_tmp0 + _tmp1;
  const Scalar _tmp3 = _x2[0] * _x3[0];
  const Scalar _tmp4 = _x2[1] * _x3[1];
  const Scalar _tmp5 = _tmp3 + _tmp4;
  const Scalar _tmp6 = _tmp5 + epsilon * ((((_tmp5) > 0) - ((_tmp5) < 0)) + Scalar(0.5));
  const Scalar _tmp7 = lambdas(2, 0) * std::atan2(_tmp2, _tmp6);
  const Scalar _tmp8 = std::sin(_tmp7);
  const Scalar _tmp9 = _x1[0] * _x2[1];
  const Scalar _tmp10 = _x1[1] * _x2[0];
  const Scalar _tmp11 = -_tmp10 + _tmp9;
  const Scalar _tmp12 = _x1[0] * _x2[0];
  const Scalar _tmp13 = _x1[1] * _x2[1];
  const Scalar _tmp14 = _tmp12 + _tmp13;
  const Scalar _tmp15 = _tmp14 + epsilon * ((((_tmp14) > 0) - ((_tmp14) < 0)) + Scalar(0.5));
  const Scalar _tmp16 = lambdas(1, 0) * std::atan2(_tmp11, _tmp15);
  const Scalar _tmp17 = std::sin(_tmp16);
  const Scalar _tmp18 = _tmp17 * _tmp8;
  const Scalar _tmp19 = std::cos(_tmp7);
  const Scalar _tmp20 = std::cos(_tmp16);
  const Scalar _tmp21 = _tmp19 * _tmp20;
  const Scalar _tmp22 = -_tmp18 + _tmp21;
  const Scalar _tmp23 = _x0[1] * _x1[0];
  const Scalar _tmp24 = _x0[0] * _x1[1];
  const Scalar _tmp25 = -_tmp23 + _tmp24;
  const Scalar _tmp26 = _x0[1] * _x1[1];
  const Scalar _tmp27 = _x0[0] * _x1[0];
  const Scalar _tmp28 = _tmp26 + _tmp27;
  const Scalar _tmp29 = _tmp28 + epsilon * ((((_tmp28) > 0) - ((_tmp28) < 0)) + Scalar(0.5));
  const Scalar _tmp30 = lambdas(0, 0) * std::atan2(_tmp25, _tmp29);
  const Scalar _tmp31 = std::sin(_tmp30);
  const Scalar _tmp32 = _tmp22 * _tmp31;
  const Scalar _tmp33 = _tmp20 * _tmp8;
  const Scalar _tmp34 = _tmp17 * _tmp19;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = std::cos(_tmp30);
  const Scalar _tmp37 = _tmp35 * _tmp36;
  const Scalar _tmp38 = _tmp32 + _tmp37;
  const Scalar _tmp39 = _tmp22 * _tmp36;
  const Scalar _tmp40 = _tmp31 * _tmp35;
  const Scalar _tmp41 = _tmp39 - _tmp40;
  const Scalar _tmp42 = -_tmp38 * _x0[1] + _tmp41 * _x0[0];
  const Scalar _tmp43 = _tmp38 * _x0[0];
  const Scalar _tmp44 = _tmp41 * _x0[1];
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -_tmp42 * _y[1] + _tmp45 * _y[0];
  const Scalar _tmp47 = _tmp42 * _y[0] + _tmp45 * _y[1];
  const Scalar _tmp48 = _tmp47 + epsilon * ((((_tmp47) > 0) - ((_tmp47) < 0)) + Scalar(0.5));
  const Scalar _tmp49 = std::pow(_tmp25, Scalar(2));
  const Scalar _tmp50 = std::pow(_tmp29, Scalar(2));
  const Scalar _tmp51 = Scalar(1.0) / (_tmp50);
  const Scalar _tmp52 = Scalar(1.0) / (_tmp29);
  const Scalar _tmp53 = -_tmp49 * _tmp51 + _tmp52 * (-_tmp26 - _tmp27);
  const Scalar _tmp54 = _tmp50 * lambdas(0, 0) / (_tmp49 + _tmp50);
  const Scalar _tmp55 = _tmp53 * _tmp54;
  const Scalar _tmp56 = _tmp32 * _tmp54;
  const Scalar _tmp57 = -_tmp37 * _tmp55 - _tmp53 * _tmp56;
  const Scalar _tmp58 = _tmp39 * _tmp55 - _tmp40 * _tmp55;
  const Scalar _tmp59 = -_tmp43 - _tmp44 + _tmp57 * _x0[0] - _tmp58 * _x0[1];
  const Scalar _tmp60 = _tmp42 + _tmp57 * _x0[1] + _tmp58 * _x0[0];
  const Scalar _tmp61 = Scalar(1.0) / (_tmp48);
  const Scalar _tmp62 = std::pow(_tmp48, Scalar(2));
  const Scalar _tmp63 = _tmp46 / _tmp62;
  const Scalar _tmp64 = _tmp62 * sqrt_info(0, 0) / (std::pow(_tmp46, Scalar(2)) + _tmp62);
  const Scalar _tmp65 = _tmp64 * (_tmp61 * (-_tmp59 * _y[1] + _tmp60 * _y[0]) -
                                  _tmp63 * (_tmp59 * _y[0] + _tmp60 * _y[1]));
  const Scalar _tmp66 = Scalar(1.0) / (_tmp15);
  const Scalar _tmp67 = std::pow(_tmp11, Scalar(2));
  const Scalar _tmp68 = std::pow(_tmp15, Scalar(2));
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = _tmp66 * (-_tmp12 - _tmp13) - _tmp67 * _tmp69;
  const Scalar _tmp71 = _tmp68 * lambdas(1, 0) / (_tmp67 + _tmp68);
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp18 * _tmp71;
  const Scalar _tmp74 = _tmp21 * _tmp72 - _tmp70 * _tmp73;
  const Scalar _tmp75 = -_tmp33 * _tmp72 - _tmp34 * _tmp72;
  const Scalar _tmp76 = -_tmp25 * _tmp51 * (_tmp23 - _tmp24) + _tmp28 * _tmp52;
  const Scalar _tmp77 = _tmp54 * _tmp76;
  const Scalar _tmp78 = _tmp31 * _tmp75 + _tmp36 * _tmp74 + _tmp39 * _tmp77 - _tmp40 * _tmp77;
  const Scalar _tmp79 = -_tmp31 * _tmp74 + _tmp36 * _tmp75 - _tmp37 * _tmp77 - _tmp56 * _tmp76;
  const Scalar _tmp80 = -_tmp78 * _x0[1] + _tmp79 * _x0[0];
  const Scalar _tmp81 = _tmp78 * _x0[0] + _tmp79 * _x0[1];
  const Scalar _tmp82 = _tmp64 * (_tmp61 * (-_tmp80 * _y[1] + _tmp81 * _y[0]) -
                                  _tmp63 * (_tmp80 * _y[0] + _tmp81 * _y[1]));
  const Scalar _tmp83 = Scalar(1.0) / (_tmp6);
  const Scalar _tmp84 = std::pow(_tmp2, Scalar(2));
  const Scalar _tmp85 = std::pow(_tmp6, Scalar(2));
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = _tmp85 * lambdas(2, 0) / (_tmp84 + _tmp85);
  const Scalar _tmp88 = _tmp87 * (_tmp83 * (-_tmp3 - _tmp4) - _tmp84 * _tmp86);
  const Scalar _tmp89 = -_tmp11 * _tmp69 * (_tmp10 - _tmp9) + _tmp14 * _tmp66;
  const Scalar _tmp90 = _tmp71 * _tmp89;
  const Scalar _tmp91 = -_tmp33 * _tmp88 - _tmp33 * _tmp90 - _tmp34 * _tmp88 - _tmp34 * _tmp90;
  const Scalar _tmp92 = -_tmp18 * _tmp88 + _tmp21 * _tmp88 + _tmp21 * _tmp90 - _tmp73 * _tmp89;
  const Scalar _tmp93 = -_tmp31 * _tmp92 + _tmp36 * _tmp91;
  const Scalar _tmp94 = _tmp31 * _tmp91 + _tmp36 * _tmp92;
  const Scalar _tmp95 = _tmp93 * _x0[1] + _tmp94 * _x0[0];
  const Scalar _tmp96 = _tmp93 * _x0[0] - _tmp94 * _x0[1];
  const Scalar _tmp97 = _tmp64 * (_tmp61 * (_tmp95 * _y[0] - _tmp96 * _y[1]) -
                                  _tmp63 * (_tmp95 * _y[1] + _tmp96 * _y[0]));
  const Scalar _tmp98 = _tmp87 * (-_tmp2 * _tmp86 * (_tmp0 - _tmp1) + _tmp5 * _tmp83);
  const Scalar _tmp99 = -_tmp18 * _tmp98 + _tmp21 * _tmp98;
  const Scalar _tmp100 = -_tmp33 * _tmp98 - _tmp34 * _tmp98;
  const Scalar _tmp101 = _tmp100 * _tmp36 - _tmp31 * _tmp99;
  const Scalar _tmp102 = _tmp100 * _tmp31 + _tmp36 * _tmp99;
  const Scalar _tmp103 = _tmp101 * _x0[0] - _tmp102 * _x0[1];
  const Scalar _tmp104 = _tmp101 * _x0[1] + _tmp102 * _x0[0];
  const Scalar _tmp105 = _tmp64 * (_tmp61 * (-_tmp103 * _y[1] + _tmp104 * _y[0]) -
                                   _tmp63 * (_tmp103 * _y[0] + _tmp104 * _y[1]));

  // Output terms (5)
  Eigen::Matrix<Scalar, 1, 1> _res;

  _res(0, 0) = sqrt_info(0, 0) * std::atan2(_tmp46, _tmp48);

  if (res_D_x0 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 2>> _res_D_x0{res_D_x0};

    _res_D_x0(0, 0) = -_tmp65 * _x0[1];
    _res_D_x0(0, 1) = _tmp65 * _x0[0];
  }

  if (res_D_x1 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 2>> _res_D_x1{res_D_x1};

    _res_D_x1(0, 0) = -_tmp82 * _x1[1];
    _res_D_x1(0, 1) = _tmp82 * _x1[0];
  }

  if (res_D_x2 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 2>> _res_D_x2{res_D_x2};

    _res_D_x2(0, 0) = -_tmp97 * _x2[1];
    _res_D_x2(0, 1) = _tmp97 * _x2[0];
  }

  if (res_D_x3 != nullptr) {
    Eigen::Map<Eigen::Matrix<Scalar, 1, 2>> _res_D_x3{res_D_x3};

    _res_D_x3(0, 0) = -_tmp105 * _x3[1];
    _res_D_x3(0, 1) = _tmp105 * _x3[0];
  }

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres