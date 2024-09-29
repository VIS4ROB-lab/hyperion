// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/pose2.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline5_pose2_acceleration
 *
 * Args:
 *     dt: Scalar
 *     lambdas: Matrix53
 *     x0: Pose2
 *     x1: Pose2
 *     x2: Pose2
 *     x3: Pose2
 *     x4: Pose2
 *     x5: Pose2
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline5Pose2AccelerationLocal(
    const Scalar dt, const Eigen::Matrix<Scalar, 5, 3>& lambdas, const sym::Pose2<Scalar>& x0,
    const sym::Pose2<Scalar>& x1, const sym::Pose2<Scalar>& x2, const sym::Pose2<Scalar>& x3,
    const sym::Pose2<Scalar>& x4, const sym::Pose2<Scalar>& x5, const Scalar epsilon) {
  // Total ops: 184

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x4 = x4.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x5 = x5.Data();

  // Intermediate terms (51)
  const Scalar _tmp0 = std::pow(dt, Scalar(-2));
  const Scalar _tmp1 = _x2[0] * _x3[0] + _x2[1] * _x3[1];
  const Scalar _tmp2 =
      std::atan2(_x2[0] * _x3[1] - _x2[1] * _x3[0],
                 _tmp1 + epsilon * ((((_tmp1) > 0) - ((_tmp1) < 0)) + Scalar(0.5)));
  const Scalar _tmp3 = _x3[0] * _x4[0] + _x3[1] * _x4[1];
  const Scalar _tmp4 =
      std::atan2(_x3[0] * _x4[1] - _x3[1] * _x4[0],
                 _tmp3 + epsilon * ((((_tmp3) > 0) - ((_tmp3) < 0)) + Scalar(0.5)));
  const Scalar _tmp5 = _x4[0] * _x5[0] + _x4[1] * _x5[1];
  const Scalar _tmp6 =
      std::atan2(_x4[0] * _x5[1] - _x4[1] * _x5[0],
                 _tmp5 + epsilon * ((((_tmp5) > 0) - ((_tmp5) < 0)) + Scalar(0.5)));
  const Scalar _tmp7 = _x1[0] * _x2[0] + _x1[1] * _x2[1];
  const Scalar _tmp8 =
      std::atan2(_x1[0] * _x2[1] - _x1[1] * _x2[0],
                 _tmp7 + epsilon * ((((_tmp7) > 0) - ((_tmp7) < 0)) + Scalar(0.5)));
  const Scalar _tmp9 = _x0[0] * _x1[0] + _x0[1] * _x1[1];
  const Scalar _tmp10 =
      std::atan2(_x0[0] * _x1[1] - _x0[1] * _x1[0],
                 _tmp9 + epsilon * ((((_tmp9) > 0) - ((_tmp9) < 0)) + Scalar(0.5)));
  const Scalar _tmp11 = _tmp10 * lambdas(0, 0);
  const Scalar _tmp12 = std::cos(_tmp11);
  const Scalar _tmp13 = std::sin(_tmp11);
  const Scalar _tmp14 = _tmp12 * _x0[0] - _tmp13 * _x0[1];
  const Scalar _tmp15 = _tmp8 * lambdas(1, 0);
  const Scalar _tmp16 = std::sin(_tmp15);
  const Scalar _tmp17 = _tmp12 * _x0[1] + _tmp13 * _x0[0];
  const Scalar _tmp18 = std::cos(_tmp15);
  const Scalar _tmp19 = _tmp14 * _tmp16 + _tmp17 * _tmp18;
  const Scalar _tmp20 = _tmp2 * lambdas(2, 0);
  const Scalar _tmp21 = std::sin(_tmp20);
  const Scalar _tmp22 = _tmp14 * _tmp18 - _tmp16 * _tmp17;
  const Scalar _tmp23 = std::cos(_tmp20);
  const Scalar _tmp24 = -_tmp19 * _tmp21 + _tmp22 * _tmp23;
  const Scalar _tmp25 = _tmp4 * lambdas(3, 0);
  const Scalar _tmp26 = std::sin(_tmp25);
  const Scalar _tmp27 = _tmp19 * _tmp23 + _tmp21 * _tmp22;
  const Scalar _tmp28 = std::cos(_tmp25);
  const Scalar _tmp29 = _tmp24 * _tmp26 + _tmp27 * _tmp28;
  const Scalar _tmp30 = _tmp6 * lambdas(4, 0);
  const Scalar _tmp31 = std::sin(_tmp30);
  const Scalar _tmp32 = _tmp24 * _tmp28 - _tmp26 * _tmp27;
  const Scalar _tmp33 = std::cos(_tmp30);
  const Scalar _tmp34 = -_tmp29 * _tmp31 + _tmp32 * _tmp33;
  const Scalar _tmp35 = -_x2[2] + _x3[2];
  const Scalar _tmp36 = -_x1[2] + _x2[2];
  const Scalar _tmp37 = -_x4[2] + _x5[2];
  const Scalar _tmp38 = -_x0[2] + _x1[2];
  const Scalar _tmp39 = -_x3[2] + _x4[2];
  const Scalar _tmp40 = _tmp35 * lambdas(2, 2) + _tmp36 * lambdas(1, 2) + _tmp37 * lambdas(4, 2) +
                        _tmp38 * lambdas(0, 2) + _tmp39 * lambdas(3, 2);
  const Scalar _tmp41 = _tmp29 * _tmp33 + _tmp31 * _tmp32;
  const Scalar _tmp42 = -_x2[3] + _x3[3];
  const Scalar _tmp43 = -_x1[3] + _x2[3];
  const Scalar _tmp44 = -_x4[3] + _x5[3];
  const Scalar _tmp45 = -_x0[3] + _x1[3];
  const Scalar _tmp46 = -_x3[3] + _x4[3];
  const Scalar _tmp47 = _tmp42 * lambdas(2, 2) + _tmp43 * lambdas(1, 2) + _tmp44 * lambdas(4, 2) +
                        _tmp45 * lambdas(0, 2) + _tmp46 * lambdas(3, 2);
  const Scalar _tmp48 = _tmp35 * lambdas(2, 1) + _tmp36 * lambdas(1, 1) + _tmp37 * lambdas(4, 1) +
                        _tmp38 * lambdas(0, 1) + _tmp39 * lambdas(3, 1);
  const Scalar _tmp49 = _tmp42 * lambdas(2, 1) + _tmp43 * lambdas(1, 1) + _tmp44 * lambdas(4, 1) +
                        _tmp45 * lambdas(0, 1) + _tmp46 * lambdas(3, 1);
  const Scalar _tmp50 = _tmp10 * lambdas(0, 1) + _tmp2 * lambdas(2, 1) + _tmp4 * lambdas(3, 1) +
                        _tmp6 * lambdas(4, 1) + _tmp8 * lambdas(1, 1);

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp0 * (_tmp10 * lambdas(0, 2) + _tmp2 * lambdas(2, 2) + _tmp4 * lambdas(3, 2) +
                        _tmp6 * lambdas(4, 2) + _tmp8 * lambdas(1, 2));
  _res(1, 0) =
      _tmp0 * (_tmp34 * _tmp40 + _tmp41 * _tmp47 + _tmp50 * (_tmp34 * _tmp49 - _tmp41 * _tmp48));
  _res(2, 0) =
      _tmp0 * (_tmp34 * _tmp47 - _tmp40 * _tmp41 - _tmp50 * (_tmp34 * _tmp48 + _tmp41 * _tmp49));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym