// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     FACTOR.hpp.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include <sym/pose2.h>

namespace sym_ceres {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: spline3_pose2_velocity_factor
 *
 * Args:
 *     dt: Scalar
 *     lambdas: Matrix32
 *     x0: Pose2
 *     x1: Pose2
 *     x2: Pose2
 *     x3: Pose2
 *     velocity: Matrix31
 *     sqrt_info: Matrix33
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix31
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> Spline3Pose2VelocityGlobalFactor(
    const Scalar dt, const Eigen::Matrix<Scalar, 3, 2>& lambdas, const sym::Pose2<Scalar>& x0,
    const sym::Pose2<Scalar>& x1, const sym::Pose2<Scalar>& x2, const sym::Pose2<Scalar>& x3,
    const Eigen::Matrix<Scalar, 3, 1>& velocity, const Eigen::Matrix<Scalar, 3, 3>& sqrt_info,
    const Scalar epsilon) {
  // Total ops: 92

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _x0 = x0.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x1 = x1.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x2 = x2.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _x3 = x3.Data();

  // Intermediate terms (14)
  const Scalar _tmp0 = Scalar(1.0) / (dt);
  const Scalar _tmp1 = -_x2[2] + _x3[2];
  const Scalar _tmp2 = -_x1[2] + _x2[2];
  const Scalar _tmp3 = -_x0[2] + _x1[2];
  const Scalar _tmp4 = _x2[0] * _x3[0] + _x2[1] * _x3[1];
  const Scalar _tmp5 = _x1[0] * _x2[0] + _x1[1] * _x2[1];
  const Scalar _tmp6 = _x0[0] * _x1[0] + _x0[1] * _x1[1];
  const Scalar _tmp7 =
      lambdas(0, 1) *
          std::atan2(_x0[0] * _x1[1] - _x0[1] * _x1[0],
                     _tmp6 + epsilon * ((((_tmp6) > 0) - ((_tmp6) < 0)) + Scalar(0.5))) +
      lambdas(1, 1) *
          std::atan2(_x1[0] * _x2[1] - _x1[1] * _x2[0],
                     _tmp5 + epsilon * ((((_tmp5) > 0) - ((_tmp5) < 0)) + Scalar(0.5))) +
      lambdas(2, 1) * std::atan2(_x2[0] * _x3[1] - _x2[1] * _x3[0],
                                 _tmp4 + epsilon * ((((_tmp4) > 0) - ((_tmp4) < 0)) + Scalar(0.5)));
  const Scalar _tmp8 = -_x2[3] + _x3[3];
  const Scalar _tmp9 = -_x1[3] + _x2[3];
  const Scalar _tmp10 = -_x0[3] + _x1[3];
  const Scalar _tmp11 =
      _tmp0 * (_tmp1 * lambdas(2, 1) + _tmp2 * lambdas(1, 1) + _tmp3 * lambdas(0, 1) +
               _tmp7 * (_tmp10 * lambdas(0, 0) + _tmp8 * lambdas(2, 0) + _tmp9 * lambdas(1, 0) +
                        _x0[3])) -
      velocity(1, 0);
  const Scalar _tmp12 = _tmp0 * _tmp7 - velocity(0, 0);
  const Scalar _tmp13 = _tmp0 * (_tmp10 * lambdas(0, 1) -
                                 _tmp7 * (_tmp1 * lambdas(2, 0) + _tmp2 * lambdas(1, 0) +
                                          _tmp3 * lambdas(0, 0) + _x0[2]) +
                                 _tmp8 * lambdas(2, 1) + _tmp9 * lambdas(1, 1)) -
                        velocity(2, 0);

  // Output terms (1)
  Eigen::Matrix<Scalar, 3, 1> _res;

  _res(0, 0) = _tmp11 * sqrt_info(0, 1) + _tmp12 * sqrt_info(0, 0) + _tmp13 * sqrt_info(0, 2);
  _res(1, 0) = _tmp11 * sqrt_info(1, 1) + _tmp12 * sqrt_info(1, 0) + _tmp13 * sqrt_info(1, 2);
  _res(2, 0) = _tmp11 * sqrt_info(2, 1) + _tmp12 * sqrt_info(2, 0) + _tmp13 * sqrt_info(2, 2);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym_ceres
