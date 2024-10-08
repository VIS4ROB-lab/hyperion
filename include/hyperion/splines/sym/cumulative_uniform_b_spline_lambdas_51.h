// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: cumulative_uniform_b_spline_lambdas
 *
 * Args:
 *     ut: Scalar
 *
 * Outputs:
 *     res: Matrix42
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 2> CumulativeUniformBSplineLambdas51(const Scalar ut) {
  // Total ops: 31

  // Input arrays

  // Intermediate terms (11)
  const Scalar _tmp0 = std::pow(ut, Scalar(2));
  const Scalar _tmp1 = (Scalar(1) / Scalar(4)) * _tmp0;
  const Scalar _tmp2 = std::pow(ut, Scalar(4));
  const Scalar _tmp3 = (Scalar(1) / Scalar(24)) * _tmp2;
  const Scalar _tmp4 = [&]() {
    const Scalar base = ut;
    return base * base * base;
  }();
  const Scalar _tmp5 = (Scalar(1) / Scalar(6)) * _tmp4;
  const Scalar _tmp6 = _tmp5 + (Scalar(1) / Scalar(6)) * ut;
  const Scalar _tmp7 = (Scalar(1) / Scalar(8)) * _tmp2;
  const Scalar _tmp8 = (Scalar(1) / Scalar(2)) * ut;
  const Scalar _tmp9 = (Scalar(1) / Scalar(2)) * _tmp0 + Scalar(1) / Scalar(6);
  const Scalar _tmp10 = (Scalar(1) / Scalar(2)) * _tmp4;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 2> _res;

  _res(0, 0) = -_tmp1 - _tmp3 + _tmp6 + Scalar(23) / Scalar(24);
  _res(1, 0) =
      -Scalar(1) / Scalar(3) * _tmp4 + _tmp7 + (Scalar(2) / Scalar(3)) * ut + Scalar(1) / Scalar(2);
  _res(2, 0) = _tmp1 + _tmp6 - _tmp7 + Scalar(1) / Scalar(24);
  _res(3, 0) = _tmp3;
  _res(0, 1) = -_tmp5 - _tmp8 + _tmp9;
  _res(1, 1) = -_tmp0 + _tmp10 + Scalar(2) / Scalar(3);
  _res(2, 1) = -_tmp10 + _tmp8 + _tmp9;
  _res(3, 1) = _tmp5;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
